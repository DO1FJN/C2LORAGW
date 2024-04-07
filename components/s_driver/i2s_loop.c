
/*
 * i2s_loop.c
 *
 *  Created on: 01.02.2024
 *     Project: C2 Lora Gateway
 *      Author: Jan Alte, DO1FJN
 * 
 * A copy'n'pasted I2S driver that WORKS!
 * TX outputs from a looping simple buffer (residing in DMA capable memory) w/o memcpy().
 * User simply must fill the buffer in the same speed while output is active.
 * 
 * ToDo
 * - tx gap handling (modes silence, insert user defined frame, repeat the last frame)
 * - rx buffer overflow handling (not present)
 * - exakt DMA frame-stop
 * - duplex mode with common BCLK and WS lines
 * 
 */

#include "i2s_loop.h"

#include "s_buffer.h"

#include "esp_err.h"
#include "esp_check.h"

#include "driver/gpio.h"

#include "soc/soc_caps.h"
#include "soc/i2s_struct.h" 
#include "soc/io_mux_reg.h"
#include "soc/i2s_periph.h"
#include "soc/gpio_periph.h"

#include "hal/gpio_hal.h"
#include "hal/i2s_hal.h"
#include "hal/i2s_ll.h"
#if SOC_GDMA_SUPPORTED
#include "soc/gdma_channel.h"
#include "hal/gdma_hal.h"
#include "esp_private/gdma.h"
#include "esp_private/periph_ctrl.h"
#endif

#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_clk_tree.h"
#include "esp_rom_lldesc.h"
#include "esp_heap_caps.h"
#include "esp_dma_utils.h"
#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
#include "esp_cache.h"
#endif


#define I2S_INTR_ALLOC_FLAGS    (ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_SHARED)
#define I2S_DMA_ALLOC_CAPS      (MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA)

#define I2S_STD_TOTAL_SLOTS     2       // in std mode only left + right are supported

#if SOC_GDMA_SUPPORTED
#define tDMAchan_handle         gdma_channel_handle_t
#else
#define tDMAchan_handle         intr_handle_t
#endif

#define I2SLOOP_NO_OF_DESC      2

typedef enum {
  I2SLOOP_UNDEF = 0, I2SLOOP_IDLE, I2SLOOP_ACTIVE, I2SLOOP_FINISH_BUFFER, I2SLOOP_STOP
} ti2s_run_mode;

typedef enum {
  TXGAP_INSERT_BUFFER, TXGAP_REPEAT_FRAME
} tTXgap_mode;

typedef struct {
  i2s_dev_t *           dev;
  ti2s_run_mode         tx_mode;
  ti2s_run_mode         rx_mode;
  tDMAchan_handle       tx_dma_chan;
  tDMAchan_handle       rx_dma_chan;
  tsimple_buffer *      tx_buf;
  tsimple_buffer *      rx_buf;
  lldesc_t *            tx_dma_desc;
  lldesc_t *            rx_dma_desc;
  // Todo gap frame buffer API to set silence / comfort-noise
  tTXgap_mode           tx_gap_mode;
  void *                tx_gap_frame_buf;
  unsigned short        tx_gap_size;

  void *                rx_user_data;
  thandle_received_frame_funct rx_handler;
#ifdef CONFIG_PM_ENABLE
  esp_pm_lock_handle_t  pm_lock;       ///< Power management lock
#endif  
} ti2sloop_def;


static const char *TAG = "i2s_loop";
static const char *I2SLOOP_DRIVER_NAME = "I2S-Loop";

static i2s_dev_t *I2S[SOC_I2S_NUM] = { 
  &I2S0
#if (SOC_I2S_NUM > 1)
  , &I2S1
#endif  
};

static ti2sloop_def I2Sdef[SOC_I2S_NUM];

#ifndef I2SLOOP_DONT_REGISTER_DRIVER
esp_err_t i2s_platform_acquire_occupation(int id, const char *comp_name); // forward from i2s_platform.c
#endif


#if SOC_GDMA_SUPPORTED
static bool i2sloop_dma_tx_callback(gdma_channel_handle_t dma_chan, gdma_event_data_t *event_data, void *user_data);
static bool i2sloop_dma_rx_callback(gdma_channel_handle_t dma_chan, gdma_event_data_t *event_data, void *user_data);
#else
static void i2sloop_intr_tx_callback(void *arg);
static void i2sloop_intr_rx_callback(void *arg);
#endif

__attribute__((always_inline))
inline void *i2sloop_dma_calloc(size_t num, size_t size, uint32_t caps, size_t *actual_size) {
    void *ptr = NULL;
    esp_dma_calloc(num, size, caps, &ptr, actual_size);
    return ptr;
}


static uint32_t i2sloop_get_source_clk_freq(i2s_clock_src_t clk_src, uint32_t mclk_freq_hz) {
  uint32_t clk_freq = 0;
#if SOC_I2S_SUPPORTS_APLL
  if (clk_src == I2S_CLK_SRC_APLL) {
     return i2s_set_get_apll_freq(mclk_freq_hz);
  }
#endif
  esp_clk_tree_src_get_freq_hz(clk_src, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &clk_freq);
  return clk_freq;
}


static esp_err_t i2sloop_calc_clock(i2s_hal_clock_info_t *clock_info, i2s_clock_src_t clock_src, unsigned int sample_rate, unsigned char bit_width, uint32_t ext_clk_freq_hz) {
  uint32_t mclk_multiple = bit_width == 24 ? 384 : 256;

  clock_info->bclk = sample_rate * I2S_STD_TOTAL_SLOTS * bit_width;
  clock_info->mclk = sample_rate * mclk_multiple;
  clock_info->bclk_div = clock_info->mclk / clock_info->bclk;
  clock_info->sclk = i2sloop_get_source_clk_freq(clock_src, clock_info->mclk);

#if SOC_I2S_HW_VERSION_2
  if (clock_src == I2S_CLK_SRC_EXTERNAL) {
    clock_info->sclk = ext_clk_freq_hz;
  }
#endif

  clock_info->mclk_div = clock_info->sclk / clock_info->mclk;
  // Check if the configuration is correct
  ESP_RETURN_ON_FALSE(clock_info->mclk_div, ESP_ERR_INVALID_ARG, TAG, "sample rate is too large for the current clock source");

#if SOC_I2S_SUPPORTS_APLL
      // Enable APLL and acquire its lock when the clock source is APLL
  if (clock_src == I2S_CLK_SRC_APLL) {
    periph_rtc_apll_acquire();
//    handle->apll_en = true;
  }
#endif
  ESP_LOGD(TAG, "Clock division info: [sclk] %"PRIu32" Hz [mdiv] %d [mclk] %"PRIu32" Hz [bdiv] %d [bclk] %"PRIu32" Hz",
    clock_info->sclk, clock_info->mclk_div, clock_info->mclk, clock_info->bclk_div, clock_info->bclk);
  return ESP_OK;
}


static esp_err_t i2sloop_set_txclock(i2s_dev_t *dev, i2s_clock_src_t clock_src, unsigned int sample_rate, unsigned char bit_width, uint32_t ext_clk_freq_hz) {
  i2s_hal_context_t hal_ctx = {.dev = dev };
  i2s_hal_clock_info_t clock_info = { };
  esp_err_t res = i2sloop_calc_clock(&clock_info, clock_src, sample_rate, bit_width, ext_clk_freq_hz);
  if (res != ESP_OK) return res;
  i2s_hal_set_tx_clock(&hal_ctx, &clock_info, clock_src);
  return ESP_OK;
}


static esp_err_t i2sloop_set_rxclock(i2s_dev_t *dev, i2s_clock_src_t clock_src, unsigned int sample_rate, unsigned char bit_width, uint32_t ext_clk_freq_hz) {
  i2s_hal_context_t hal_ctx = {.dev = dev };
  i2s_hal_clock_info_t clock_info = { };
  esp_err_t res = i2sloop_calc_clock(&clock_info, clock_src, sample_rate, bit_width, ext_clk_freq_hz);
  if (res != ESP_OK) return res;
  i2s_hal_set_rx_clock(&hal_ctx, &clock_info, clock_src);
  return ESP_OK;
}



/* *** GPIO SECTION ***
 *
 *
 */

static void i2sloop_gpio_check_and_set(int gpio, uint32_t signal_idx, bool is_input, bool is_invert) {
  // Ignore the pin if pin = UNUSED
  if (gpio != (int)GPIO_NUM_NC) {
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    if (is_input) {
      /* Set direction, for some GPIOs, the input function are not enabled as default */
      gpio_set_direction(gpio, GPIO_MODE_INPUT);
      esp_rom_gpio_connect_in_signal(gpio, signal_idx, is_invert);
    } else {
      gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
      esp_rom_gpio_connect_out_signal(gpio, signal_idx, is_invert, 0);
    }
  }
}

static void i2sloop_gpio_loopback_set(int gpio, uint32_t out_sig_idx, uint32_t in_sig_idx) {
  if (gpio != (int)GPIO_NUM_NC) {
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    gpio_set_direction(gpio, GPIO_MODE_INPUT_OUTPUT);
    esp_rom_gpio_connect_out_signal(gpio, out_sig_idx, 0, 0);
    esp_rom_gpio_connect_in_signal(gpio, in_sig_idx, 0);
  }
}


static esp_err_t i2sloop_check_set_mclk(int id, int gpio_num, i2s_clock_src_t clk_src, bool is_invert) {
  if (gpio_num == (int)GPIO_NUM_NC) {
    return ESP_OK;
  }
#if CONFIG_IDF_TARGET_ESP32
  bool is_i2s0 = id == I2S_NUM_0;
  bool is_apll = clk_src == I2S_CLK_SRC_APLL;
  if (g_i2s.controller[id]->mclk_out_hdl == NULL) {
    soc_clkout_sig_id_t clkout_sig = is_apll ? CLKOUT_SIG_APLL : (is_i2s0 ? CLKOUT_SIG_I2S0 : CLKOUT_SIG_I2S1);
    ESP_RETURN_ON_ERROR(esp_clock_output_start(clkout_sig, gpio_num, &(g_i2s.controller[id]->mclk_out_hdl)), TAG, "mclk configure failed");
  }
#else
  ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(gpio_num), ESP_ERR_INVALID_ARG, TAG, "mck_io_num invalid");
#if SOC_I2S_HW_VERSION_2
  if (clk_src == I2S_CLK_SRC_EXTERNAL) {
    i2sloop_gpio_check_and_set(gpio_num, i2s_periph_signal[id].mck_in_sig, true, is_invert);
  } else
#endif  // SOC_I2S_HW_VERSION_2
  {
    i2sloop_gpio_check_and_set(gpio_num, i2s_periph_signal[id].mck_out_sig, false, is_invert);
  }
#endif  // CONFIG_IDF_TARGET_ESP32
  ESP_LOGD(TAG, "MCLK is pinned to GPIO%d on I2S%d", gpio_num, id);
  return ESP_OK;
}


static esp_err_t i2sloop_set_ds_mclk_gpio(int id, gpio_num_t dout_pin, gpio_num_t din_pin, const ti2sloop_gpio *mclk_gpio, i2s_clock_src_t clock_src) {
  ESP_RETURN_ON_FALSE((id >= 0) && (id < SOC_I2S_NUM), ESP_ERR_INVALID_ARG, TAG, "invalid I2S id");
  ESP_RETURN_ON_FALSE(mclk_gpio != NULL, ESP_ERR_INVALID_ARG, TAG, "no MCLK config!");
  if (dout_pin != GPIO_NUM_NC) {    // Set data output GPIO
    if (dout_pin == din_pin) {      // Loopback if dout = din
      i2sloop_gpio_loopback_set(dout_pin, i2s_periph_signal[id].data_out_sig, i2s_periph_signal[id].data_in_sig);
    } else {
      // Set data output GPIO
      i2sloop_gpio_check_and_set(dout_pin, i2s_periph_signal[id].data_out_sig, false, false);
    }
  }
  if ((din_pin != GPIO_NUM_NC) && (din_pin != dout_pin)) {    // Set data input GPIO
    i2sloop_gpio_check_and_set(din_pin, i2s_periph_signal[id].data_in_sig, true, false);
  }
  // Set mclk pin
  ESP_RETURN_ON_ERROR(i2sloop_check_set_mclk(id, mclk_gpio->mclk, clock_src, mclk_gpio->invert_flags.mclk_inv), TAG, "mclk config failed");
  return ESP_OK;
}


static esp_err_t i2sloop_set_txgpio(int id, const ti2sloop_gpio *gpio_cfg) {
  ESP_RETURN_ON_FALSE((id >= 0) && (id < SOC_I2S_NUM), ESP_ERR_INVALID_ARG, TAG, "invalid I2S id");
  // Check validity of selected pins  
  ESP_RETURN_ON_FALSE((gpio_cfg->bclk == GPIO_NUM_NC) || GPIO_IS_VALID_GPIO(gpio_cfg->bclk), ESP_ERR_INVALID_ARG, TAG, "bclk invalid");
  ESP_RETURN_ON_FALSE((gpio_cfg->ws == GPIO_NUM_NC) || GPIO_IS_VALID_GPIO(gpio_cfg->ws), ESP_ERR_INVALID_ARG, TAG, "ws invalid");
/*
  if (gpio_cfg->dout != -1) {    // Set data output GPIO
    if (gpio_cfg->dout == gpio_cfg->din) {      // Loopback if dout = din
      i2sloop_gpio_loopback_set(gpio_cfg->dout, i2s_periph_signal[id].data_out_sig, i2s_periph_signal[id].data_in_sig);
    } else {
      // Set data output GPIO
      i2sloop_gpio_check_and_set(gpio_cfg->dout, i2s_periph_signal[id].data_out_sig, false, false);
    }
  }
  if ((gpio_cfg->din != -1) && (gpio_cfg->din != gpio_cfg->dout)) {    // Set data input GPIO
    i2sloop_gpio_check_and_set(gpio_cfg->din, i2s_periph_signal[id].data_in_sig, true, false);
  }
  // Set mclk pin
  ESP_RETURN_ON_ERROR(i2sloop_check_set_mclk(id, gpio_cfg->mclk, clock_src, gpio_cfg->invert_flags.mclk_inv), TAG, "mclk config failed");
*/  
  // For "rx + master" mode, select RX signal index for ws and bck
  i2sloop_gpio_check_and_set(gpio_cfg->ws,   i2s_periph_signal[id].m_tx_ws_sig,  false, gpio_cfg->invert_flags.ws_inv);
  i2sloop_gpio_check_and_set(gpio_cfg->bclk, i2s_periph_signal[id].m_tx_bck_sig, false, gpio_cfg->invert_flags.bclk_inv);
  return ESP_OK;
}


static esp_err_t i2sloop_set_rxgpio(int id, const ti2sloop_gpio *rx_gpio_cfg, const ti2sloop_gpio *tx_gpio_cfg) {
  ESP_RETURN_ON_FALSE((id >= 0) && (id < SOC_I2S_NUM), ESP_ERR_INVALID_ARG, TAG, "invalid I2S id");
  // Check validity of selected pins 
  ESP_RETURN_ON_FALSE((rx_gpio_cfg->bclk == GPIO_NUM_NC) || GPIO_IS_VALID_GPIO(rx_gpio_cfg->bclk), ESP_ERR_INVALID_ARG, TAG, "bclk invalid");
  ESP_RETURN_ON_FALSE((rx_gpio_cfg->ws == GPIO_NUM_NC) || GPIO_IS_VALID_GPIO(rx_gpio_cfg->ws), ESP_ERR_INVALID_ARG, TAG, "ws invalid");  
  
  // For "rx + master" mode, select RX signal index for ws and bck
  // ToDo
  //if ((tx_gpio_cfg == NULL) || (tx_gpio_cfg->ws != rx_gpio_cfg->ws)) {
    i2sloop_gpio_check_and_set(rx_gpio_cfg->ws,   i2s_periph_signal[id].m_rx_ws_sig,  false, rx_gpio_cfg->invert_flags.ws_inv);
  //}
  //if ((tx_gpio_cfg == NULL) || (tx_gpio_cfg->bclk != rx_gpio_cfg->bclk)) {
    i2sloop_gpio_check_and_set(rx_gpio_cfg->bclk, i2s_periph_signal[id].m_rx_bck_sig, false, rx_gpio_cfg->invert_flags.bclk_inv);
  //}
  return ESP_OK;
}

// *** GPIO SECTION END ***




/**
 * @brief   I2S DMA interrupt initialization
 * @note    I2S will use GDMA if chip supports, and the interrupt is triggered by GDMA.
 *
 * @param   port_id     I2S hardware 0 or 1
 * @param   intr_flag   Interrupt allocation flag
 * @return
 *      - ESP_OK                    I2S DMA interrupt initialize success
 *      - ESP_ERR_NOT_FOUND         GDMA channel not found
 *      - ESP_ERR_INVALID_ARG       Invalid arguments
 *      - ESP_ERR_INVALID_STATE     GDMA state error
 */
#if SOC_GDMA_SUPPORTED

static esp_err_t i2sloop_init_dma_intr(int port_id, gdma_channel_handle_t *dma_chan) {
  ti2sloop_def *device;  
  gdma_channel_alloc_config_t dma_cfg = { };
  // Set GDMA trigger module
  gdma_trigger_t trig = { .periph = GDMA_TRIG_PERIPH_I2S };
  switch (port_id) {
  case 0:
    trig.instance_id = SOC_GDMA_TRIG_PERIPH_I2S0;
    break;
#if (SOC_I2S_NUM > 1)
  case 1:
    trig.instance_id = SOC_GDMA_TRIG_PERIPH_I2S1;
    break;
#endif
  default:
    ESP_RETURN_ON_ERROR(ESP_ERR_INVALID_ARG, TAG, "invalid handle");
  } // hctiws

  device = &I2Sdef[port_id];

  // Register a new GDMA tx channel
  if (dma_chan == &device->tx_dma_chan) {
    gdma_tx_event_callbacks_t cb = {
      .on_trans_eof = i2sloop_dma_tx_callback
    };
    dma_cfg.direction = GDMA_CHANNEL_DIRECTION_TX;
    ESP_LOGD(TAG, "create new tx ahb_channel");
    ESP_RETURN_ON_ERROR(gdma_new_ahb_channel(&dma_cfg, dma_chan), TAG, "Register tx dma channel error");     
    ESP_LOGD(TAG, "chan @ %8p, connect dma_chan", *dma_chan);
    ESP_RETURN_ON_ERROR(gdma_connect(*dma_chan, trig), TAG, "Connect tx dma channel error");
    ESP_LOGD(TAG, "register event callbacks");
    // Set callback function for GDMA, the interrupt is triggered by GDMA, then the GDMA ISR will call the  callback function
    gdma_register_tx_event_callbacks(*dma_chan, &cb, device);

  } else if (dma_chan == &device->rx_dma_chan) {

    gdma_rx_event_callbacks_t cb = {
      .on_recv_eof = i2sloop_dma_rx_callback
    };
    dma_cfg.direction = GDMA_CHANNEL_DIRECTION_RX;
    ESP_LOGD(TAG, "create new rx ahb_channel");
    ESP_RETURN_ON_ERROR(gdma_new_ahb_channel(&dma_cfg, dma_chan), TAG, "Register tx dma channel error");     
    ESP_LOGD(TAG, "chan @ %8p, connect dma_chan", *dma_chan);
    ESP_RETURN_ON_ERROR(gdma_connect(*dma_chan, trig), TAG, "Connect tx dma channel error");
    ESP_LOGD(TAG, "register event callbacks");
    gdma_register_rx_event_callbacks(*dma_chan, &cb, device);

  } else {
    ESP_RETURN_ON_ERROR(ESP_ERR_INVALID_ARG, TAG, "invalid dma_channel");
  }
  return ESP_OK;
}


#else

static esp_err_t i2sloop_init_dma_intr(int port_id, intr_handle_t dma_chan, void *user_data)  {
  i2s_dev_t *dev;
  ESP_RETURN_ON_FALSE((port_id >= 0)&&(port_id < SOC_I2S_NUM),  ESP_ERR_INVALID_ARG, TAG, "invalid port_id");
  uint32_t interrupt_status_reg = (uint32_t) i2s_ll_get_interrupt_status_reg(dev);
  uint32_t intr_flag = I2S_INTR_ALLOC_FLAGS | handle->intr_prio_flags;
  // Initialize I2S module interrupt
  if (dma_chan == device->tx_dma_chan) {
    esp_intr_alloc_intrstatus(i2s_periph_signal[port_id].irq, intr_flag, interrupt_status_reg, I2S_LL_TX_EVENT_MASK, i2sloop_intr_tx_callback, user_data, &device->tx_dma_chan);
  } else if (dma_chan == device->rx_dma_chan) {
    esp_intr_alloc_intrstatus(i2s_periph_signal[port_id].irq, intr_flag, interrupt_status_reg, I2S_LL_TX_EVENT_MASK, i2sloop_intr_rx_callback, user_data, &device->rx_dma_chan);
  }
  // Start DMA
  i2s_ll_enable_dma(dev, true);
  return ESP_OK;
}

#endif  // SOC_GDMA_SUPPORTED



static esp_err_t i2sloop_alloc_dma_desc(lldesc_t **dma_desc, U8 no_of_dma_desc, const tsimple_buffer *s_buf) {
  size_t desc_size = 0;
  ESP_RETURN_ON_FALSE((s_buf!= NULL) && (s_buf->start_ptr != NULL), ESP_ERR_INVALID_ARG, TAG, "no or unallocated rtp buffer");
  *dma_desc = (lldesc_t *) i2sloop_dma_calloc(no_of_dma_desc, sizeof(lldesc_t), I2S_DMA_ALLOC_CAPS, &desc_size);
  //*dma_desc = (lldesc_t *) calloc(no_of_dma_desc, sizeof(lldesc_t));
  ESP_RETURN_ON_FALSE((dma_desc != NULL), ESP_ERR_NO_MEM, TAG, "allocate DMA description failed");

  ESP_RETURN_ON_FALSE((s_buf->frame_size < 4096), ESP_ERR_INVALID_ARG, TAG, "buffer framesize is lager than dma can handle");

  lldesc_t *dma_desc_ptr = *dma_desc;
  uint8_t  *frame_sptr   = (uint8_t *) s_buf->start_ptr;

  for (int i = 0; i < no_of_dma_desc; i++, frame_sptr += s_buf->frame_size, dma_desc_ptr++) {   
    if (((void *)frame_sptr - s_buf->start_ptr) > (s_buf->size - s_buf->frame_size)) {    // more frames than real buffer memory - not good      
      frame_sptr = (uint8_t *) s_buf->start_ptr;
      ESP_LOGW(TAG, "rtp buffer smaller than 2 frames!");
    }
    // Allocate DMA descriptor    
    dma_desc_ptr->owner  = 1;  // HW is owner
    dma_desc_ptr->eof    = 1;  // ToDo test
    dma_desc_ptr->sosf   = 0;
    dma_desc_ptr->offset = 0;
    dma_desc_ptr->length = s_buf->frame_size;
    dma_desc_ptr->size   = s_buf->frame_size;      
    dma_desc_ptr->buf    = frame_sptr;

    STAILQ_NEXT(dma_desc_ptr, qe) = (i < (no_of_dma_desc - 1))? &dma_desc_ptr[1]: *dma_desc; // Connect DMA descriptor as a circle

    ESP_LOGD(TAG, "desc addr: %8p\tbuffer addr:%8p\t next %8p", dma_desc_ptr, dma_desc_ptr->buf, STAILQ_NEXT(dma_desc_ptr, qe) );

  } // rof all 

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  esp_cache_msync(*dma_desc, desc_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
#endif
  return ESP_OK;
}


uint32_t i2sloop_set_tx_slot(i2s_dev_t *dev, const ti2sloop_slotcfg *config) {
  uint32_t slot_bit_width = config->bitwidth < config->databits ? config->databits : config->bitwidth;
  i2s_ll_tx_reset(dev);
  i2s_ll_tx_set_slave_mod(dev, false);  // TX Slave
  i2s_ll_tx_set_sample_bit(dev, slot_bit_width, config->databits);
  i2s_ll_tx_enable_msb_shift(dev, config->bit_shift);
  i2s_ll_tx_set_ws_width(dev, config->ws_width);
#if SOC_I2S_HW_VERSION_1
  i2s_ll_tx_enable_mono_mode(dev, config->mode == SLOT_MODE_MONO);
  i2s_ll_tx_select_std_slot(dev, config->mask, config->mode == SLOT_MODE_MONO);
  // According to the test, the behavior of tx_msb_right is opposite with TRM, TRM is wrong?  
  i2s_ll_tx_enable_msb_right(dev, config->msb_right);
  i2s_ll_tx_enable_right_first(dev, config->ws_pol);
  // Should always enable fifo 
  i2s_ll_tx_force_enable_fifo_mod(dev, true);
#elif SOC_I2S_HW_VERSION_2
  bool is_copy_mono = (config->mode == SLOT_MODE_MONO) && (config->mask == 3);
  i2s_ll_tx_enable_mono_mode(dev, is_copy_mono);
  i2s_ll_tx_select_std_slot(dev, is_copy_mono? I2S_STD_SLOT_LEFT: config->mask);
  i2s_ll_tx_set_skip_mask(dev, (config->mask != 3) && (config->mode == SLOT_MODE_STEREO));
  i2s_ll_tx_set_half_sample_bit(dev, slot_bit_width);
  i2s_ll_tx_set_ws_idle_pol(dev, config->ws_pol);
  i2s_ll_tx_set_bit_order(dev, config->bit_order_lsb);
  i2s_ll_tx_enable_left_align(dev, config->left_align);
  i2s_ll_tx_enable_big_endian(dev, config->big_endian);
#endif
  return slot_bit_width;
}


uint32_t i2sloop_set_rx_slot(i2s_dev_t *dev, const ti2sloop_slotcfg *config) {
  uint32_t slot_bit_width = config->bitwidth < config->databits ? config->databits : config->bitwidth;
  i2s_ll_rx_reset(dev);
  i2s_ll_rx_set_slave_mod(dev, false);  // TX Slave
  i2s_ll_rx_set_sample_bit(dev, slot_bit_width, config->databits);
  i2s_ll_rx_enable_msb_shift(dev, config->bit_shift);
  i2s_ll_rx_set_ws_width(dev, config->ws_width);
#if SOC_I2S_HW_VERSION_1
  i2s_ll_rx_enable_mono_mode(dev, config->mode == SLOT_MODE_MONO);
  i2s_ll_rx_select_std_slot(dev, config->mask, config->mode == SLOT_MODE_MONO);
  // According to the test, the behavior of tx_msb_right is opposite with TRM, TRM is wrong?  
  i2s_ll_rx_enable_msb_right(dev, config->msb_right);
  i2s_ll_rx_enable_right_first(dev, config->ws_pol);
  // Should always enable fifo 
  i2s_ll_rx_force_enable_fifo_mod(dev, true);
#elif SOC_I2S_HW_VERSION_2
  bool is_copy_mono = (config->mode == SLOT_MODE_MONO) && (config->mask == 3);
  i2s_ll_rx_enable_mono_mode(dev, is_copy_mono);
  i2s_ll_rx_select_std_slot(dev, is_copy_mono? I2S_STD_SLOT_LEFT: config->mask);
  i2s_ll_rx_set_half_sample_bit(dev, slot_bit_width);
  i2s_ll_rx_set_ws_idle_pol(dev, config->ws_pol);
  i2s_ll_rx_set_bit_order(dev, config->bit_order_lsb);
  i2s_ll_rx_enable_left_align(dev, config->left_align);
  i2s_ll_rx_enable_big_endian(dev, config->big_endian);
#endif
  return slot_bit_width;
}



/**
 * @brief   I2S loop initialization
 * @note    it will initialize the hardware
 *
 * @param   config     I2S hardware configuration
 * @param   tx_buf     TX buffer struct with DMA capable buffer (start_ptr)
 * @param   rx_buf     RX buffer struct with DMA capable buffer (start_ptr)
 * @return
 *      - ESP_OK                    I2S successfully configured
 *      - ESP_ERR_NOT_FOUND         occupied by another driver
 *      - ESP_ERR_INVALID_ARG       Invalid arguments
 *      - ...
 */
esp_err_t i2sloop_init(int num, const ti2sloop_config *tx_conf, tsimple_buffer *tx_buf, const ti2sloop_config *rx_conf, tsimple_buffer *rx_buf) {
  i2s_dev_t *dev;
  i2s_clock_src_t clock_src = I2S_CLK_SRC_DEFAULT;

  const ti2sloop_config *config = (tx_conf != NULL) && (tx_buf != NULL)? tx_conf: rx_conf;

  ESP_RETURN_ON_FALSE((num >= 0) && (num < SOC_I2S_NUM), ESP_ERR_INVALID_ARG, TAG, "invalid I2S id");

  if ((config == NULL) || (num >= SOC_I2S_NUM) || (num < 0)) {
    return ESP_ERR_INVALID_ARG;
  }
  dev = I2S[num];
  ESP_LOGD(TAG, "init");
  memset(&I2Sdef[num], 0, sizeof(ti2sloop_def));
  I2Sdef[num].dev = dev;
#ifndef I2SLOOP_DONT_REGISTER_DRIVER
  // register within espressif "driver" so other drivers see this I2S as 'occupied'
  ESP_RETURN_ON_ERROR(i2s_platform_acquire_occupation(num, I2SLOOP_DRIVER_NAME), TAG, "I2S is occupied");
#else
  // alternative w/o care abount other I2S usage:
  PERIPH_RCC_ATOMIC() {
    i2s_ll_enable_bus_clock(num, true);
    i2s_ll_reset_register(num);
    i2s_ll_enable_core_clock(dev, true);
  }
#endif

  i2sloop_set_ds_mclk_gpio(num, tx_conf? tx_conf->gpio.dio: GPIO_NUM_NC, rx_conf? rx_conf->gpio.dio: GPIO_NUM_NC, &config->gpio, clock_src);

  if ((tx_buf != NULL) && (tx_conf != NULL)) {
    size_t gap_size;
    uint32_t real_slot_width = i2sloop_set_tx_slot(dev, &tx_conf->slot);
    I2Sdef[num].tx_mode = I2SLOOP_UNDEF;    
    ESP_RETURN_ON_ERROR(i2sloop_set_txclock(dev, clock_src, tx_conf->sample_rate_hz, real_slot_width, 0), TAG, "initialize channel failed while setting clock");
    ESP_RETURN_ON_ERROR(i2sloop_set_txgpio(num, &tx_conf->gpio), TAG, "initialize tx channel failed while setting gpio pins");
    ESP_RETURN_ON_ERROR(i2sloop_init_dma_intr(num, &I2Sdef[num].tx_dma_chan), TAG, "initialize tx dma interrupt failed");    
    I2Sdef[num].tx_buf = tx_buf;
    i2sloop_alloc_dma_desc(&I2Sdef[num].tx_dma_desc, I2SLOOP_NO_OF_DESC, tx_buf);
    I2Sdef[num].tx_gap_frame_buf = i2sloop_dma_calloc(1, tx_buf->frame_size, I2S_DMA_ALLOC_CAPS, &gap_size);
    if (gap_size != tx_buf->frame_size) {
      ESP_LOGW(TAG, "tx gab size don't match (got %d, %d requested)", gap_size, tx_buf->size);
    }
    I2Sdef[num].tx_gap_size = (gap_size > tx_buf->frame_size)? tx_buf->frame_size: gap_size;
    ESP_LOGD(TAG, "enable I2S tx...");
    i2s_ll_tx_enable_std(dev);
    I2Sdef[num].tx_mode = I2SLOOP_IDLE;
  }
  if ((rx_buf != NULL) && (rx_conf != NULL))  {
    uint32_t rx_slot_width = i2sloop_set_rx_slot(dev, &rx_conf->slot);
    I2Sdef[num].rx_mode = I2SLOOP_UNDEF;
    ESP_RETURN_ON_ERROR(i2sloop_set_rxclock(dev, clock_src, rx_conf->sample_rate_hz, rx_slot_width, 0), TAG, "initialize channel failed while setting clock");
    ESP_RETURN_ON_ERROR(i2sloop_set_rxgpio(num, &rx_conf->gpio, tx_conf? &tx_conf->gpio: NULL), TAG, "initialize rx channel failed while setting gpio pins");
    ESP_RETURN_ON_ERROR(i2sloop_init_dma_intr(num, &I2Sdef[num].rx_dma_chan), TAG, "initialize rx dma interrupt failed");    
    I2Sdef[num].rx_buf = rx_buf;
    i2sloop_alloc_dma_desc(&I2Sdef[num].rx_dma_desc, I2SLOOP_NO_OF_DESC, rx_buf);
    i2s_ll_rx_set_eof_num(dev, rx_buf->frame_size);
    ESP_LOGD(TAG, "enable I2S rx...");
    i2s_ll_rx_enable_std(dev);
    I2Sdef[num].rx_mode = I2SLOOP_IDLE;
  }

/*
#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_type_t pm_type = ESP_PM_APB_FREQ_MAX;
#if SOC_I2S_SUPPORTS_APLL
    if (std_cfg->clk_cfg.clk_src == I2S_CLK_SRC_APLL) {
        pm_type = ESP_PM_NO_LIGHT_SLEEP;
    }
#endif // SOC_I2S_SUPPORTS_APLL
    ESP_RETURN_ON_ERROR(esp_pm_lock_create(pm_type, 0, "i2s_loop_dev", &I2Sdef[num].pm_lock), TAG, "I2S pm lock create failed");
#endif
*/
  ESP_LOGD(TAG, "I2S%d has been initialized to STD-LOOP mode successfully", num);   // Initialization finished
  return ESP_OK;
}




esp_err_t i2sloop_create_loop(tsimple_buffer *loop_buffer, U16 frame_cnt, U16 frame_size, U16 steps_per_frame) {
  size_t real_buf_size;
  ESP_RETURN_ON_FALSE((loop_buffer != NULL), ESP_ERR_INVALID_ARG, TAG, "invalid arguments");
  ESP_LOGD(TAG, "i2s_create_loop: %d frames a %d bytes (%d steps)", frame_cnt, frame_size, steps_per_frame);

  sbuf_setparameter(loop_buffer, frame_cnt, frame_size, steps_per_frame);

  loop_buffer->start_ptr = i2sloop_dma_calloc(frame_cnt, frame_size, I2S_DMA_ALLOC_CAPS, &real_buf_size);
  if (real_buf_size != loop_buffer->size) {
    ESP_LOGW(TAG, "i2sloop_create_txloop size mismatch: %d wanted, got %d byte", loop_buffer->size, real_buf_size);
  }
  loop_buffer->unsend_ptr = (void *) loop_buffer->start_ptr;
  loop_buffer->write_ptr  = (void *) loop_buffer->start_ptr;
  loop_buffer->size       = (loop_buffer->start_ptr == NULL)? 0: real_buf_size;
  ESP_LOGD(TAG, "loop_buffer %8p created (%d bytes @ %8p)", loop_buffer, real_buf_size, loop_buffer->start_ptr);
  return ESP_OK;
}



void i2sloop_prepare_dma_desc(tsimple_buffer *loop_buffer, lldesc_t *desc) {
  const uint8_t *frame_sptr = loop_buffer->start_ptr;
  lldesc_t *dma_desc_ptr = desc;
  uint32_t frame_size    = loop_buffer->frame_size;
  for (int i = 0; i < I2SLOOP_NO_OF_DESC; i++, frame_sptr += frame_size, dma_desc_ptr++) {   
    dma_desc_ptr->owner  = 1;
    dma_desc_ptr->length = frame_size;
    dma_desc_ptr->size   = frame_size;      
    dma_desc_ptr->buf    = frame_sptr;
    ESP_LOGD(TAG, "prep desc addr: %8p\tbuffer addr:%8p\t next %8p", dma_desc_ptr, dma_desc_ptr->buf, STAILQ_NEXT(dma_desc_ptr, qe) );
  } // rof all
  // todo check for preloaded length
}


esp_err_t i2sloop_reconfig_rx(int num, U16 frame_cnt, U16 frame_size, U16 steps_per_frame) {
  ti2sloop_def   *device;
  tsimple_buffer *rx_buf;
  ESP_RETURN_ON_FALSE((num >= 0) && (num < SOC_I2S_NUM) && (frame_cnt >= 2) && (frame_size < 4096), ESP_ERR_INVALID_ARG, TAG, "invalid arguments");
  device = &I2Sdef[num];
  ESP_RETURN_ON_FALSE(device->rx_mode != I2SLOOP_ACTIVE, ESP_FAIL, TAG, "receiver is active");
  rx_buf = device->rx_buf;
  if ((rx_buf != NULL)) {    
    sbuf_destroy(rx_buf);
    i2sloop_create_loop(rx_buf, frame_cnt, frame_size, steps_per_frame);
    i2sloop_prepare_dma_desc(rx_buf, device->rx_dma_desc);
    i2s_ll_rx_set_eof_num(device->dev, rx_buf->frame_size);
  }
  return ESP_OK;
}



esp_err_t i2sloop_set_rx_handler(int num, thandle_received_frame_funct handler, void *user_data) {
  ESP_RETURN_ON_FALSE((num >= 0) && (num < SOC_I2S_NUM), ESP_ERR_INVALID_ARG, TAG, "invalid I2S id");  
  I2Sdef[num].rx_handler   = handler;
  I2Sdef[num].rx_user_data = user_data;
  return ESP_OK;
}





/* *** Start / Stop and Shutdown functions

*/


static void i2sloop_tx_channel_start(ti2sloop_def *device) {
  i2s_ll_tx_reset(device->dev);
#if SOC_GDMA_SUPPORTED
  gdma_reset(device->tx_dma_chan);
#else
  i2s_ll_tx_reset_dma((device->dev);
#endif
  i2s_ll_tx_reset_fifo(device->dev);
  i2sloop_prepare_dma_desc(device->tx_buf, device->tx_dma_desc);
#if SOC_GDMA_SUPPORTED
  gdma_start(device->tx_dma_chan, (uint32_t)device->tx_dma_desc);
#else
  esp_intr_enable(device->tx_dma_chan);
  i2s_ll_tx_enable_intr(device->dev);
  i2s_ll_enable_dma(device->dev, true);
  i2s_ll_tx_start_link(device->dev, (uint32_t)device->tx_dma_desc);
#endif
  ESP_LOGD(TAG, "tx chan start");
  i2s_ll_tx_start(device->dev);
}


static void i2sloop_rx_channel_start(ti2sloop_def *device) {
  i2s_ll_rx_reset(device->dev);
#if SOC_GDMA_SUPPORTED
  gdma_reset(device->rx_dma_chan);
#else
  i2s_ll_rx_reset_dma((device->dev);  
#endif
  i2s_ll_rx_reset_fifo(device->dev);
  i2sloop_prepare_dma_desc(device->rx_buf, device->rx_dma_desc);
#if SOC_GDMA_SUPPORTED
  gdma_start(device->rx_dma_chan, (uint32_t)device->rx_dma_desc);
#else
  esp_intr_enable(device->rx_dma_chan);
  i2s_ll_rx_enable_intr(device->dev);
  i2s_ll_enable_dma(device->dev, true);
  i2s_ll_rx_start_link(device->dev, (uint32_t)device->rx_dma_desc);
#endif
  i2s_ll_rx_start(device->dev);
}


static void i2sloop_tx_channel_stop(ti2sloop_def *device) {
  i2s_ll_tx_stop(device->dev);
#if SOC_GDMA_SUPPORTED
  gdma_stop(device->tx_dma_chan);
#else
  i2s_ll_tx_stop_link(device->dev);
  i2s_ll_tx_disable_intr(device->dev);
  i2s_ll_enable_dma(device->dev, false);
  esp_intr_disable(device->tx_dma_chan);
#endif
  device->tx_mode = I2SLOOP_IDLE;
}


static void i2sloop_rx_channel_stop(ti2sloop_def *device) {
  i2s_ll_rx_stop(device->dev);
#if SOC_GDMA_SUPPORTED
  gdma_stop(device->rx_dma_chan);
#else
  i2s_ll_rx_stop_link(device->dev);
  i2s_ll_rx_disable_intr(device->dev);
  i2s_ll_enable_dma(device->dev, false);
  esp_intr_disable(device->rx_dma_chan);
#endif
  device->rx_mode = I2SLOOP_IDLE;
}



esp_err_t i2sloop_enable(int num) {
  ti2sloop_def *device;
  if ((num >= SOC_I2S_NUM)) {
    return ESP_ERR_INVALID_ARG;
  }
  device = &I2Sdef[num];
  if (device->tx_buf) {
    i2sloop_tx_channel_start(device);
    device->tx_mode = I2SLOOP_ACTIVE;
  }
  if (device->rx_buf) {
    i2sloop_rx_channel_start(device);
    device->rx_mode = I2SLOOP_ACTIVE;
  }
  return ESP_OK;
}


esp_err_t i2sloop_enable_tx(int num) {
  ti2sloop_def *device;
  if ((num >= SOC_I2S_NUM)) {
    return ESP_ERR_INVALID_ARG;
  }
  device = &I2Sdef[num];
  if (device->tx_buf) {
    i2sloop_tx_channel_start(device);
    device->tx_mode = I2SLOOP_ACTIVE;
  }
  return ESP_OK;
}


esp_err_t i2sloop_enable_rx(int num) {
  ti2sloop_def *device;
  if ((num >= SOC_I2S_NUM)) {
    return ESP_ERR_INVALID_ARG;
  }
  device = &I2Sdef[num];
  if (device->rx_buf) {
    i2sloop_rx_channel_start(device);
    device->rx_mode = I2SLOOP_ACTIVE;
  }
  return ESP_OK;
}


esp_err_t i2sloop_disable(int num) {
  ti2sloop_def *device;
  if ((num >= SOC_I2S_NUM)) {
    return ESP_ERR_INVALID_ARG;
  }
  device = &I2Sdef[num];
  if (device->tx_buf) {
    i2sloop_tx_channel_stop(device);
  }
  if (device->rx_buf) {    
    i2sloop_rx_channel_stop(device);
  }
  return ESP_OK;     
}

esp_err_t i2sloop_disable_tx(int num) {
  ti2sloop_def *device;
  if ((num >= SOC_I2S_NUM)) {
    return ESP_ERR_INVALID_ARG;
  }
  device = &I2Sdef[num];
  if (device->tx_buf) {
    i2sloop_tx_channel_stop(device);
  }
  return ESP_OK;     
}

esp_err_t i2sloop_disable_rx(int num) {
  ti2sloop_def *device;
  if ((num >= SOC_I2S_NUM)) {
    return ESP_ERR_INVALID_ARG;
  }
  device = &I2Sdef[num];
  if (device->rx_buf) {    
    i2sloop_rx_channel_stop(device);
  }
  return ESP_OK;     
}


esp_err_t i2sloop_shutdown_both(int num) {
  ti2sloop_def *device;
  if ((num >= SOC_I2S_NUM)) {
    return ESP_ERR_INVALID_ARG;
  }
  device = &I2Sdef[num];
  if ((device->tx_buf) && (device->tx_mode == I2SLOOP_ACTIVE)) {
    device->tx_mode = I2SLOOP_FINISH_BUFFER;
  }
  if ((device->rx_buf) && (device->rx_mode == I2SLOOP_ACTIVE)) {
    device->rx_mode = I2SLOOP_FINISH_BUFFER;
  }
  return ESP_OK;
}


esp_err_t i2sloop_shutdown_tx(int num) {
  ti2sloop_def *device;
  if ((num >= SOC_I2S_NUM)) {
    return ESP_ERR_INVALID_ARG;
  }
  device = &I2Sdef[num];
  if ((device->tx_buf) && (device->tx_mode == I2SLOOP_ACTIVE)) {
    device->tx_mode = I2SLOOP_FINISH_BUFFER;
  }
  return ESP_OK;
}


esp_err_t i2sloop_shutdown_rx(int num) {
  ti2sloop_def *device;
  if ((num >= SOC_I2S_NUM)) {
    return ESP_ERR_INVALID_ARG;
  }
  device = &I2Sdef[num];
  if ((device->rx_buf) && (device->rx_mode == I2SLOOP_ACTIVE)) {
    device->rx_mode = I2SLOOP_FINISH_BUFFER;
  }
  return ESP_OK;
}


tsimple_buffer *i2sloop_get_tx_buffer(int num) {
  if ((num < 0) || (num >= SOC_I2C_NUM)) return NULL;
  return I2Sdef[num].tx_buf;
}


tsimple_buffer *i2sloop_get_rx_buffer(int num) {
  if ((num < 0) || (num >= SOC_I2C_NUM)) return NULL;
  return I2Sdef[num].rx_buf;
}















#if SOC_GDMA_SUPPORTED

int gap_counter = 0;

static bool IRAM_ATTR i2sloop_dma_tx_callback(gdma_channel_handle_t dma_chan, gdma_event_data_t *event_data, void *user_data) {
  uint32_t bytes_to_send = 0;
  ti2sloop_def *device   = (ti2sloop_def *) user_data;
  lldesc_t *finish_desc  = (lldesc_t *) event_data->tx_eof_desc_addr;  

  const lldesc_t *curr_desc = (lldesc_t *) finish_desc->qe.stqe_next; // this one is active now
  void *sending_ptr    = (void *) curr_desc->buf;
  uint32_t sending_len = curr_desc->length;
  
  tsimple_buffer *tx_buf = device->tx_buf;
  const void *end_ptr  = tx_buf->start_ptr + tx_buf->size;
  void *just_send_ptr  = sending_ptr + sending_len;
  
  if (device->tx_mode == I2SLOOP_STOP) {
    i2sloop_tx_channel_stop(device);
    device->tx_mode = I2SLOOP_IDLE;
  }

  //finish_desc->owner = 0;
  //taskENTER_CRITICAL_ISR(&tx_buf->mutex);
  if ((just_send_ptr >= tx_buf->start_ptr) && (just_send_ptr <= end_ptr)) {
    if (just_send_ptr == end_ptr) just_send_ptr = (void *)tx_buf->start_ptr;

    bytes_to_send = (just_send_ptr < tx_buf->write_ptr)? tx_buf->write_ptr - just_send_ptr: end_ptr - just_send_ptr;

    if (bytes_to_send > tx_buf->frame_size) bytes_to_send = tx_buf->frame_size; // artificial - not needed
    if (bytes_to_send >= 16) {
      finish_desc->buf    = just_send_ptr;
      finish_desc->length = bytes_to_send;
      just_send_ptr += bytes_to_send;
      if (just_send_ptr >= end_ptr) just_send_ptr = (void *)tx_buf->start_ptr; 
      tx_buf->unsend_ptr = (void *) just_send_ptr;      
    } else switch (device->tx_gap_mode) {
      case TXGAP_INSERT_BUFFER:
        finish_desc->buf    = device->tx_gap_frame_buf;
        finish_desc->length = device->tx_gap_size;
        gap_counter++;
        break;
      case TXGAP_REPEAT_FRAME:
        finish_desc->buf    = sending_ptr;
        finish_desc->length = sending_len;
        gap_counter++;
        break;
    } // hctiws

  } else { // fi sending from tx_buf (ELSE sending gap insertion buffer)

    bytes_to_send = (tx_buf->unsend_ptr < tx_buf->write_ptr)? tx_buf->write_ptr - tx_buf->unsend_ptr: end_ptr - tx_buf->unsend_ptr;
    
    if (bytes_to_send > tx_buf->frame_size) bytes_to_send = tx_buf->frame_size; // artificial - not needed
    if (bytes_to_send >= 16) {      
      finish_desc->buf    = tx_buf->unsend_ptr;
      finish_desc->length = bytes_to_send;
    } else {
      finish_desc->buf    = device->tx_gap_frame_buf;
      finish_desc->length = device->tx_gap_size;
      gap_counter++;
    }

  }
  //taskEXIT_CRITICAL_ISR(&tx_buf->mutex);
  //finish_desc->owner = 1;

  if ((device->tx_mode == I2SLOOP_FINISH_BUFFER) && (bytes_to_send == 0)) {
    finish_desc->owner = 0;
    device->tx_mode = I2SLOOP_STOP;
  }
  return false;
}


static bool IRAM_ATTR i2sloop_dma_rx_callback(gdma_channel_handle_t dma_chan, gdma_event_data_t *event_data, void *user_data) {
  bool wakywaky_thready = false;

  ti2sloop_def *device  = (ti2sloop_def *) user_data;
  
  lldesc_t *finish_desc = (lldesc_t *) event_data->rx_eof_desc_addr;

  tsimple_buffer *rx_buf = device->rx_buf;

  const lldesc_t *curr_desc = (lldesc_t *) finish_desc->qe.stqe_next; // this one is active now

  if ((device->rx_mode == I2SLOOP_STOP) || (event_data->flags.abnormal_eof)) {
    i2sloop_rx_channel_stop(device);
    device->rx_mode = I2SLOOP_IDLE;
  }
  
  // if (finish_desc->length == 0) return false;

  // 1. update rx_buffer:
  //taskENTER_CRITICAL_ISR(&rx_buf->mutex);

  rx_buf->write_ptr = (void *) finish_desc->buf + finish_desc->length;

  // 2. update desc
  void *next_buf = (void *)curr_desc->buf + curr_desc->size;
  if ((void *)next_buf >= (rx_buf->start_ptr + rx_buf->size)) {
    next_buf = (void *) rx_buf->start_ptr;
  }
  finish_desc->buf  = next_buf;
  finish_desc->size = rx_buf->frame_size;  
  finish_desc->length = 0;
  finish_desc->owner  = 1; // retrigger it.
  //taskEXIT_CRITICAL_ISR(&rx_buf->mutex);

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  esp_cache_msync((void *)finish_desc->buf, finish_desc->size, ESP_CACHE_MSYNC_FLAG_INVALIDATE);
#endif

  // 3. inform user task
  if (device->rx_handler != NULL) {
    wakywaky_thready = device->rx_handler(rx_buf, device->rx_user_data);
  } // fi user thread call 

  return wakywaky_thready;
}


#else

static void IRAM_ATTR i2sloop_intr_tx_callback(void *arg) {
/*        
  BaseType_t need_yield1 = 0;
  BaseType_t need_yield2 = 0;
  BaseType_t user_need_yield = 0;
  lldesc_t *finish_desc = NULL;
  i2s_event_data_t evt;
  i2s_chan_handle_t handle = (i2s_chan_handle_t)arg;
  uint32_t dummy;

  uint32_t status = i2s_hal_get_intr_status(&(handle->controller->hal));
  i2s_hal_clear_intr_status(&(handle->controller->hal), status);
  if (!status) {
    return;
  }

  if (handle && (status & I2S_LL_EVENT_TX_EOF)) {
    i2s_hal_get_out_eof_des_addr(&(handle->controller->hal), (uint32_t *)&finish_desc);
    evt.data = &(finish_desc->buf);
    evt.size = handle->dma.buf_size;
    if (handle->callbacks.on_sent) {
      user_need_yield |= handle->callbacks.on_sent(handle, &evt, handle->user_data);
    }
    if (xQueueIsQueueFullFromISR(handle->msg_queue)) {
      xQueueReceiveFromISR(handle->msg_queue, &dummy, &need_yield1);
      if (handle->callbacks.on_send_q_ovf) {
        evt.data = NULL;
        user_need_yield |= handle->callbacks.on_send_q_ovf(handle, &evt, handle->user_data);
      }
    }
    // Auto clear the dma buffer after data sent
    if (handle->dma.auto_clear) {
      uint8_t *buff = (uint8_t *)finish_desc->buf;
      memset(buff, 0, handle->dma.buf_size);
    }
    xQueueSendFromISR(handle->msg_queue, &(finish_desc->buf), &need_yield2);
  }

  if (need_yield1 || need_yield2 || user_need_yield) {
    portYIELD_FROM_ISR();
  }
  */
}

static void IRAM_ATTR i2sloop_intr_rx_callback(void *arg) {
/*        
  BaseType_t need_yield1 = 0;
  BaseType_t need_yield2 = 0;
  BaseType_t user_need_yield = 0;
  lldesc_t *finish_desc = NULL;
  i2s_event_data_t evt;
  i2s_chan_handle_t handle = (i2s_chan_handle_t)arg;
  uint32_t dummy;

  uint32_t status = i2s_hal_get_intr_status(&(handle->controller->hal));
  i2s_hal_clear_intr_status(&(handle->controller->hal), status);
  if (!status) {
    return;
  }

  if (handle && (status & I2S_LL_EVENT_RX_EOF)) {
    i2s_hal_get_in_eof_des_addr(&(handle->controller->hal), (uint32_t *)&finish_desc);
    evt.data = &(finish_desc->buf);
    evt.size = handle->dma.buf_size;
    if (handle->callbacks.on_recv) {
      user_need_yield |= handle->callbacks.on_recv(handle, &evt, handle->user_data);
    }
    if (xQueueIsQueueFullFromISR(handle->msg_queue)) {
      xQueueReceiveFromISR(handle->msg_queue, &dummy, &need_yield1);
      if (handle->callbacks.on_recv_q_ovf) {
        evt.data = NULL;
        user_need_yield |= handle->callbacks.on_recv_q_ovf(handle, &evt, handle->user_data);
      }
    }
    xQueueSendFromISR(handle->msg_queue, &(finish_desc->buf), &need_yield2);
  }

  if (need_yield1 || need_yield2 || user_need_yield) {
    portYIELD_FROM_ISR();
  }
  */
}


#endif
