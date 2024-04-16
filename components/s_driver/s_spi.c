/*
This source file contain to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

*/
#include "s_spi.h"

#include "sx126x_hal.h"

#include "esp_err.h"
#include "esp_check.h"

#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_ipc.h"
#include "esp_clk_tree.h"
#include "esp_rom_lldesc.h"
#include "esp_heap_caps.h"
#include "esp_dma_utils.h"

#include "soc/spi_struct.h"
#include "soc/soc_caps.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_periph.h"

#include "driver/gpio.h"

#include "hal/spi_hal.h"
#include "hal/spi_ll.h"
#include "hal/gpio_ll.h"

#if SOC_GDMA_SUPPORTED
#include "hal/gdma_hal.h"
#include "hal/gdma_ll.h"
#include "soc/gdma_channel.h"
#include "esp_private/gdma.h"
#include "esp_private/periph_ctrl.h"
#endif

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
#include "esp_cache.h"
#endif

#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define SSPI_GDMA_DESC_CNT        8     // we can only tranmit/receive up to 32K per single SPI transfers (0x40000 max bitlength)

#ifndef SSPI_DONT_REGISTER_DRIVER
bool spicommon_periph_claim(spi_host_device_t host, const char* source);
#endif

#define SSPI_BUSFLAG_SLAVE         0          ///< Initialize I/O in slave mode
#define SSPI_BUSFLAG_MASTER        (1<<0)     ///< Initialize I/O in master mode
#define SSPI_BUSFLAG_IOMUX_PINS    (1<<1)     ///< Check using iomux pins. Or indicates the pins are configured through the IO mux rather than GPIO matrix.
#define SSPI_BUSFLAG_GPIO_PINS     (1<<2)     ///< Force the signals to be routed through GPIO matrix. Or indicates the pins are routed through the GPIO matrix.
#define SSPI_BUSFLAG_SCLK          (1<<3)     ///< Check existing of SCLK pin. Or indicates CLK line initialized.
#define SSPI_BUSFLAG_MISO          (1<<4)     ///< Check existing of MISO pin. Or indicates MISO line initialized.
#define SSPI_BUSFLAG_MOSI          (1<<5)     ///< Check existing of MOSI pin. Or indicates MOSI line initialized.
#define SSPI_BUSFLAG_DUAL          (1<<6)     ///< Check MOSI and MISO pins can output. Or indicates bus able to work under DIO mode.
#define SSPI_BUSFLAG_WPHD          (1<<7)     ///< Check existing of WP and HD pins. Or indicates WP & HD pins initialized.
#define SSPI_BUSFLAG_QUAD          (SSPI_BUSFLAG_DUAL|SSPI_BUSFLAG_WPHD)     ///< Check existing of MOSI/MISO/WP/HD pins as output. Or indicates bus able to work under QIO mode.

#define SSPI_NOTIFY_INDEX          (CONFIG_FREERTOS_TASK_NOTIFICATION_ARRAY_ENTRIES - 1)
#define SSPI_TRANSFER_DONE_BIT     0x10000000L    // task notify bit




typedef struct {
  gpio_num_t            dc_pin;
  uint32_t              user_reg, user1_reg, user2_reg, misc_reg;
  TaskHandle_t          waiting_task;
} tsspi_dev;


typedef struct {
  int                   id;
  spi_dev_t *           dev;
  uint32_t              flags, intr_flags;
  
  intr_handle_t         intr;

  lldesc_t *            tx_dma_desc;
  lldesc_t *            rx_dma_desc;

#if SOC_GDMA_SUPPORTED
  gdma_channel_handle_t tx_dma_chan;
  gdma_channel_handle_t rx_dma_chan;
#endif
  uint32_t              tx_dma_chno;
  uint32_t              rx_dma_chno;

  const void *          tx_transfer_buf;
  void *                rx_transfer_buf;
  uint32_t              remaining_tx_bytes;
  uint32_t              remaining_rx_bytes;
  //uint32_t              tx_loop_buf_size;

#ifdef CONFIG_PM_ENABLE
  esp_pm_lock_handle_t  pm_lock;       ///< Power management lock
#endif

  tsspi_dev *           active_device;
  tsspi_dev             devices[SSPI_MAX_DEVICES];

} tsspi_def;

static const char *TAG = "S_SPI";


static tsspi_def S_SPI[SOC_SPI_PERIPH_NUM];


#if SOC_PERIPH_CLK_CTRL_SHARED
#define SSPI_MASTER_PERI_CLOCK_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#define SSPI_MASTER_PERI_CLOCK_ATOMIC()
#endif


#define sspi_check_pin(pin_num, pin_name, check_output) if (check_output) { \
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(pin_num), ESP_ERR_INVALID_ARG, TAG, pin_name" not valid"); \
  } else { \
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(pin_num), ESP_ERR_INVALID_ARG, TAG, pin_name" not valid"); \
  }



static inline bool is_valid_spi(int spi_num) {
#if (SOC_SPI_PERIPH_NUM == 2)
  return spi_num >= 0 && spi_num <= 1;
#elif (SOC_SPI_PERIPH_NUM == 3)
  return spi_num >= 0 && spi_num <= 2;
#endif
}

__attribute__((always_inline))
inline void *sspi_dma_calloc(size_t num, size_t size, uint32_t caps, size_t *actual_size) {
    void *ptr = NULL;
    esp_dma_calloc(num, size, caps, &ptr, actual_size);
    return ptr;
}



static bool sbus_uses_iomux_pins(spi_host_device_t host, const tsspi_gpio *gpio_conf) {
  if (gpio_conf->sclk >= 0 && gpio_conf->sclk != spi_periph_signal[host].spiclk_iomux_pin) return false;
  if (gpio_conf->quadwp >= 0 && gpio_conf->quadwp != spi_periph_signal[host].spiwp_iomux_pin) return false;  
  if (gpio_conf->quadhd >= 0 && gpio_conf->quadhd != spi_periph_signal[host].spihd_iomux_pin) return false;
  if (gpio_conf->mosi >= 0 && gpio_conf->mosi != spi_periph_signal[host].spid_iomux_pin) return false;  
  if (gpio_conf->miso >= 0 && gpio_conf->miso != spi_periph_signal[host].spiq_iomux_pin) return false; 
  return true;
}


static void sbus_iomux_pins_set(spi_host_device_t host, const tsspi_gpio *gpio_conf) {
  if (gpio_conf->mosi >= 0) {
    gpio_iomux_in(gpio_conf->mosi, spi_periph_signal[host].spid_in);
    gpio_iomux_out(gpio_conf->mosi, spi_periph_signal[host].func, false);
  }
  if (gpio_conf->miso >= 0) {
    gpio_iomux_in(gpio_conf->miso, spi_periph_signal[host].spiq_in);
    gpio_iomux_out(gpio_conf->miso, spi_periph_signal[host].func, false);
  }
  if (gpio_conf->quadwp >= 0) {
    gpio_iomux_in(gpio_conf->quadwp, spi_periph_signal[host].spiwp_in);
    gpio_iomux_out(gpio_conf->quadwp, spi_periph_signal[host].func, false);
  }
  if (gpio_conf->quadhd >= 0) {
    gpio_iomux_in(gpio_conf->quadhd, spi_periph_signal[host].spihd_in);
    gpio_iomux_out(gpio_conf->quadhd, spi_periph_signal[host].func, false);
  }
  if (gpio_conf->sclk >= 0) {
    gpio_iomux_in(gpio_conf->sclk, spi_periph_signal[host].spiclk_in);
    gpio_iomux_out(gpio_conf->sclk, spi_periph_signal[host].func, false);
  }
}




/*
Do the common stuff to hook up a SPI host to a bus defined by a bunch of GPIO pins. Feed it a host number and a
bus bus_config struct and it'll set up the GPIO matrix and enable the device. If a pin is set to non-negative value,
it should be able to be initialized.
*/
static esp_err_t sspi_init_bus_io(spi_host_device_t host, const tsspi_gpio *gpio_conf, uint32_t flags, uint32_t *flags_o) {

  uint32_t temp_flag = 0;

  bool miso_need_output;
  bool mosi_need_output;
  bool sclk_need_output;
  if ((flags & SSPI_BUSFLAG_MASTER) != 0) {
    // initial for master
    miso_need_output = ((flags & SSPI_BUSFLAG_DUAL) != 0) ? true : false;
    mosi_need_output = true;
    sclk_need_output = true;
    temp_flag |= SSPI_BUSFLAG_MASTER;
  } else {
    // initial for slave
    miso_need_output = true;
    mosi_need_output = ((flags & SSPI_BUSFLAG_DUAL) != 0) ? true : false;
    sclk_need_output = false;
  }

  const bool wp_need_output = true;
  const bool hd_need_output = true;

  // check pin capabilities
  if (gpio_conf->sclk != GPIO_NUM_NC) {
    temp_flag |= SSPI_BUSFLAG_SCLK;
    sspi_check_pin(gpio_conf->sclk, "sclk", sclk_need_output);
  }
  if (gpio_conf->quadwp != GPIO_NUM_NC) {
    sspi_check_pin(gpio_conf->quadwp, "wp", wp_need_output);
  }
  if (gpio_conf->quadhd != GPIO_NUM_NC) {
    sspi_check_pin(gpio_conf->quadhd, "hd", hd_need_output);
  }

  // set flags for QUAD mode according to the existence of wp and hd
  if ((gpio_conf->quadhd != GPIO_NUM_NC) && (gpio_conf->quadwp != GPIO_NUM_NC)) temp_flag |= SSPI_BUSFLAG_WPHD;
  if (gpio_conf->mosi != GPIO_NUM_NC) {
    temp_flag |= SSPI_BUSFLAG_MOSI;
    sspi_check_pin(gpio_conf->mosi, "mosi", mosi_need_output);
  }
  if (gpio_conf->miso != GPIO_NUM_NC) {
    temp_flag |= SSPI_BUSFLAG_MISO;
    sspi_check_pin(gpio_conf->miso, "miso", miso_need_output);
  }
  // set flags for DUAL mode according to output-capability of MOSI and MISO pins.
  if ((gpio_conf->mosi < 0 || GPIO_IS_VALID_OUTPUT_GPIO(gpio_conf->mosi)) && (gpio_conf->miso < 0 || GPIO_IS_VALID_OUTPUT_GPIO(gpio_conf->miso))) {
    temp_flag |= SSPI_BUSFLAG_DUAL;
  }

  // check if the selected pins correspond to the iomux pins of the peripheral
  bool use_iomux = !(flags & SSPI_BUSFLAG_GPIO_PINS) && sbus_uses_iomux_pins(host, gpio_conf);
  if (use_iomux) {
    temp_flag |= SSPI_BUSFLAG_IOMUX_PINS;
  } else {
    temp_flag |= SSPI_BUSFLAG_GPIO_PINS;
  }

  uint32_t missing_flag = flags & ~temp_flag;
  missing_flag &= ~SSPI_BUSFLAG_MASTER;  // don't check this flag

  if (missing_flag != 0) {
    // check pins existence
    if (missing_flag & SSPI_BUSFLAG_SCLK) ESP_LOGE(TAG, "sclk pin required.");    
    if (missing_flag & SSPI_BUSFLAG_MOSI) ESP_LOGE(TAG, "mosi pin required.");
    if (missing_flag & SSPI_BUSFLAG_MISO) ESP_LOGE(TAG, "miso pin required.");
    if (missing_flag & SSPI_BUSFLAG_DUAL) ESP_LOGE(TAG, "not both mosi and miso output capable");
    if (missing_flag & SSPI_BUSFLAG_WPHD) ESP_LOGE(TAG, "both wp and hd required.");
    if (missing_flag & SSPI_BUSFLAG_IOMUX_PINS) ESP_LOGE(TAG, "not using iomux pins");    
  }
  ESP_RETURN_ON_FALSE(missing_flag == 0, ESP_ERR_INVALID_ARG, TAG, "not all required capabilities satisfied.");

  if (use_iomux) {
    // All SPI iomux pin selections resolve to 1, so we put that here instead of trying to figure
    // out which FUNC_GPIOx_xSPIxx to grab; they all are defined to 1 anyway.
    ESP_LOGD(TAG, "SPI%d use iomux pins.", host + 1);
    sbus_iomux_pins_set(host, gpio_conf);
  } else {
    // Use GPIO matrix
    ESP_LOGD(TAG, "SPI%d use gpio matrix.", host + 1);
    if (gpio_conf->mosi >= 0) {
      if (mosi_need_output || (temp_flag & SSPI_BUSFLAG_DUAL)) {
        gpio_set_direction(gpio_conf->mosi, GPIO_MODE_INPUT_OUTPUT);
        esp_rom_gpio_connect_out_signal(gpio_conf->mosi, spi_periph_signal[host].spid_out, false, false);
      } else {
        gpio_set_direction(gpio_conf->mosi, GPIO_MODE_INPUT);
      }
      esp_rom_gpio_connect_in_signal(gpio_conf->mosi, spi_periph_signal[host].spid_in, false);
#if CONFIG_IDF_TARGET_ESP32S2
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[bus_config->mosi]);
#endif
      gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[gpio_conf->mosi], PIN_FUNC_GPIO);
    }
    if (gpio_conf->miso >= 0) {
      if (miso_need_output || (temp_flag & SSPI_BUSFLAG_DUAL)) {
        gpio_set_direction(gpio_conf->miso, GPIO_MODE_INPUT_OUTPUT);
        esp_rom_gpio_connect_out_signal(gpio_conf->miso, spi_periph_signal[host].spiq_out, false, false);
      } else {
        gpio_set_direction(gpio_conf->miso, GPIO_MODE_INPUT);
      }
      esp_rom_gpio_connect_in_signal(gpio_conf->miso, spi_periph_signal[host].spiq_in, false);
#if CONFIG_IDF_TARGET_ESP32S2
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[bus_config->miso]);
#endif
      gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[gpio_conf->miso], PIN_FUNC_GPIO);
    }
    if (gpio_conf->quadwp >= 0) {
      gpio_set_direction(gpio_conf->quadwp, GPIO_MODE_INPUT_OUTPUT);
      esp_rom_gpio_connect_out_signal(gpio_conf->quadwp, spi_periph_signal[host].spiwp_out, false, false);
      esp_rom_gpio_connect_in_signal(gpio_conf->quadwp, spi_periph_signal[host].spiwp_in, false);
#if CONFIG_IDF_TARGET_ESP32S2
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[bus_config->quadwp]);
#endif
      gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[gpio_conf->quadwp], PIN_FUNC_GPIO);
    }
    if (gpio_conf->quadhd >= 0) {
      gpio_set_direction(gpio_conf->quadhd, GPIO_MODE_INPUT_OUTPUT);
      esp_rom_gpio_connect_out_signal(gpio_conf->quadhd, spi_periph_signal[host].spihd_out, false, false);
      esp_rom_gpio_connect_in_signal(gpio_conf->quadhd, spi_periph_signal[host].spihd_in, false);
#if CONFIG_IDF_TARGET_ESP32S2
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[bus_config->quadhd]);
#endif
      gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[gpio_conf->quadhd], PIN_FUNC_GPIO);
    }
    if (gpio_conf->sclk >= 0) {
      if (sclk_need_output) {
        gpio_set_direction(gpio_conf->sclk, GPIO_MODE_INPUT_OUTPUT);
        esp_rom_gpio_connect_out_signal(gpio_conf->sclk, spi_periph_signal[host].spiclk_out, false, false);
      } else {
        gpio_set_direction(gpio_conf->sclk, GPIO_MODE_INPUT);
      }
      esp_rom_gpio_connect_in_signal(gpio_conf->sclk, spi_periph_signal[host].spiclk_in, false);
#if CONFIG_IDF_TARGET_ESP32S2
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[bus_config->sclk]);
#endif
      gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[gpio_conf->sclk], PIN_FUNC_GPIO);
    }
  }
  ESP_LOGD(TAG, "flags out: %08lxh", temp_flag);
  if (flags_o) *flags_o = temp_flag;
  return ESP_OK;
}



static void sspi_cs_init(spi_host_device_t host, gpio_num_t cs_io_pin, int cs_num, bool force_iomux) {
  if (force_iomux && (cs_io_pin == spi_periph_signal[host].spics0_iomux_pin) && (cs_num == 0)) {
    // The cs0s for all SPI peripherals map to pin mux source 1, so we use that instead of a define.
    //gpio_iomux_in(cs_io_pin, spi_periph_signal[host].spics_in);
    gpio_ll_iomux_in(&GPIO, cs_io_pin, spi_periph_signal[host].spics_in);
    //gpio_iomux_out(cs_io_pin, spi_periph_signal[host].func, false);
    gpio_ll_iomux_out(&GPIO, cs_io_pin, spi_periph_signal[host].func, false);
  } else {
    // Use GPIO matrix
    if (GPIO_IS_VALID_OUTPUT_GPIO(cs_io_pin)) {
      gpio_set_direction(cs_io_pin, GPIO_MODE_INPUT_OUTPUT);
      esp_rom_gpio_connect_out_signal(cs_io_pin, spi_periph_signal[host].spics_out[cs_num], false, false);
    } else {
      gpio_set_direction(cs_io_pin, GPIO_MODE_INPUT);
    }
    if (cs_num == 0) {
      esp_rom_gpio_connect_in_signal(cs_io_pin, spi_periph_signal[host].spics_in, false);
    }
    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[cs_io_pin]);
    gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[cs_io_pin], PIN_FUNC_GPIO);
  }
}


static void sspi_cs_disconnect_out(spi_host_device_t host, gpio_num_t cs_io_pin, int cs_num, bool force_iomux) {
  if (force_iomux && (cs_io_pin == spi_periph_signal[host].spics0_iomux_pin) && (cs_num == 0)) {
    gpio_ll_iomux_out(&GPIO, cs_io_pin, SIG_GPIO_OUT_IDX, false);
  } else {
    esp_rom_gpio_connect_out_signal(cs_io_pin, SIG_GPIO_OUT_IDX, false, false);
  }
}


static void sspi_cs_reconnect_out(spi_host_device_t host, gpio_num_t cs_io_pin, int cs_num, bool force_iomux) {
  if (force_iomux && (cs_io_pin == spi_periph_signal[host].spics0_iomux_pin) && (cs_num == 0)) {
    gpio_ll_iomux_out(&GPIO, cs_io_pin, spi_periph_signal[host].func, false);
  } else {
    esp_rom_gpio_connect_out_signal(cs_io_pin, spi_periph_signal[host].spics_out[cs_num], false, false);
  }
}



#if SOC_GDMA_SUPPORTED

static esp_err_t sspi_init_dma_intr(int host_id, gdma_channel_handle_t *dma_chan) {
  gdma_trigger_t trigger;
  gdma_channel_alloc_config_t dma_cfg = { };
  uint32_t *channel_no;
  switch (host_id) {    
  case SPI2_NUM:
    trigger = GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_SPI, 2);
    break;
  case SPI3_NUM:
    trigger = GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_SPI, 3);
    break;  
  default:
    ESP_RETURN_ON_ERROR(ESP_ERR_INVALID_ARG, TAG, "invalid SPI host id");
  } // hctiws

  tsspi_def *sspi = &S_SPI[host_id];

  // Register a new GDMA tx channel
  if (dma_chan == &sspi->tx_dma_chan) {
    dma_cfg.direction = GDMA_CHANNEL_DIRECTION_TX;
    dma_cfg.flags.reserve_sibling = sspi->flags & SSPI_BUSFLAG_MISO? 1: 0;
    ESP_LOGD(TAG, "create new tx ahb_channel");
    ESP_RETURN_ON_ERROR(gdma_new_ahb_channel(&dma_cfg, dma_chan), TAG, "Register tx dma channel error");     
    ESP_LOGD(TAG, "chan @ %8p, connect dma_chan", *dma_chan);
    ESP_RETURN_ON_ERROR(gdma_connect(*dma_chan, trigger), TAG, "Connect tx dma channel error");
    channel_no = &sspi->tx_dma_chno;
  } else if (dma_chan == &sspi->rx_dma_chan) {
    dma_cfg.direction = GDMA_CHANNEL_DIRECTION_RX;
    dma_cfg.sibling_chan = sspi->tx_dma_chan;
    ESP_LOGD(TAG, "create new rx ahb_channel");
    ESP_RETURN_ON_ERROR(gdma_new_ahb_channel(&dma_cfg, dma_chan), TAG, "Register tx dma channel error");     
    ESP_LOGD(TAG, "chan @ %8p, connect dma_chan", *dma_chan);
    ESP_RETURN_ON_ERROR(gdma_connect(*dma_chan, trigger), TAG, "Connect tx dma channel error");
    channel_no = &sspi->rx_dma_chno;
  } else {
    ESP_RETURN_ON_ERROR(ESP_ERR_INVALID_ARG, TAG, "invalid dma_channel");
  }

  gdma_get_channel_id(*dma_chan, (int *)channel_no);
  return ESP_OK;
}


static esp_err_t sspi_alloc_dma_desc(lldesc_t **dma_desc, U8 no_of_dma_desc) {
  size_t desc_size = 0;
  *dma_desc = (lldesc_t *) sspi_dma_calloc(no_of_dma_desc, sizeof(lldesc_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA, &desc_size);
  ESP_RETURN_ON_FALSE((dma_desc != NULL), ESP_ERR_NO_MEM, TAG, "allocate DMA description failed");
#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  esp_cache_msync(*dma_desc, desc_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
#endif
  return ESP_OK;
}


#else

static esp_err_t sspi_init_dma_intr(int host_id, intr_handle_t dma_chan)  {

  assert(is_valid_host(host_id));

  esp_err_t ret = ESP_OK;
  bool success = false;
  uint32_t actual_dma_chan = 0;

  tsspi_def *sspi = &S_SPI[host_id];

#if CONFIG_IDF_TARGET_ESP32
  for (int i = 1; i < SOC_SPI_DMA_CHAN_NUM + 1; i++) {
    success = claim_dma_chan(i, &actual_dma_chan);
    if (success) {
      break;
    }
  }
#elif CONFIG_IDF_TARGET_ESP32S2
  // On ESP32S2, each SPI controller has its own DMA channel
  success = claim_dma_chan(host_id, &actual_dma_chan);
#endif  // #if CONFIG_IDF_TARGET_XXX

  // On ESP32 and ESP32S2, actual_tx_dma_chan and actual_rx_dma_chan are always same

  sspi->rx_dma_chno = actual_dma_chan;
  sspi->tx_dma_chno = actual_dma_chan;

  if (!success) {
    SPI_CHECK(false, "no available dma channel", ESP_ERR_NOT_FOUND);
  }

  connect_spi_and_dma(host_id, actual_dma_chan);

  return ret;

}

#endif  // SOC_GDMA_SUPPORTED





static void sspi_interrupt(void *arg);

#define alloc_sspi_interrupt(sspi, isr_funct) esp_intr_alloc(spi_periph_signal[sspi->id].irq, sspi->intr_flags | ESP_INTR_FLAG_INTRDISABLED, isr_funct, sspi, &sspi->intr)

#if (SOC_CPU_CORES_NUM > 1) && (!CONFIG_FREERTOS_UNICORE)
typedef struct {
    tsspi_def *   sspi;
    esp_err_t *   err;
} tsspi_ipc_param;

static void ipc_sspi_isr_reg_to_core(void *args) {
  tsspi_def * sspi = ((tsspi_ipc_param *)args)->sspi;
  esp_err_t * rerr = ((tsspi_ipc_param *)args)->err;
  *rerr = alloc_sspi_interrupt(sspi, sspi_interrupt);
}
#endif





esp_err_t sspi_init(int num, const tsspi_bus_config *bus_config) {
  esp_err_t err;
  soc_periph_spi_clk_src_t clk_src;
  uint32_t clock_source_hz, eff_spi_clock, spi_clock_reg;

  ESP_RETURN_ON_FALSE(is_valid_spi(num), ESP_ERR_INVALID_ARG, TAG, "invalid spi num");
  // ESP_RETURN_ON_FALSE(bus_ctx[num] == NULL, ESP_ERR_INVALID_STATE, "SPI bus already initialized."); defined static - sad
#ifndef SSPI_DONT_REGISTER_DRIVER  
  bool spi_chan_claimed = spicommon_periph_claim(num, "spi master");
  ESP_RETURN_ON_FALSE(spi_chan_claimed, ESP_ERR_INVALID_STATE, TAG, "host_id already in use");
#else
   PERIPH_RCC_ATOMIC() {
     spi_ll_enable_bus_clock(num, true);
     spi_ll_reset_register(num);
   }
#endif
   ESP_LOGD(TAG, "SPI%d init...", num+1);
   tsspi_def *sspi = &S_SPI[num];

   memset(sspi, 0, sizeof(tsspi_def));
   sspi->id  = num;
   sspi->dev = SPI_LL_GET_HW(num);


#ifdef CONFIG_PM_ENABLE
   err = esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "spi_master", &sspi->pm_lock);
    if (err != ESP_OK) {
        goto cleanup;
    }
#endif //CONFIG_PM_ENABLE

  err = sspi_init_bus_io(num, &bus_config->gpio, SSPI_BUSFLAG_MASTER | bus_config->flags, &sspi->flags);
  if (err != ESP_OK) {
    return err;
  }

  if (num != SPI1_NUM) {    // interrupts are not allowed on SPI1 bus
#if (SOC_CPU_CORES_NUM > 1) && (!CONFIG_FREERTOS_UNICORE)
    const esp_intr_cpu_affinity_t isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;
    if (isr_cpu_id > ESP_INTR_CPU_AFFINITY_AUTO) {
        ESP_RETURN_ON_FALSE(isr_cpu_id <= ESP_INTR_CPU_AFFINITY_1, ESP_ERR_INVALID_ARG, TAG, "invalid core id");
        tsspi_ipc_param ipc_arg = {
          .sspi = sspi, .err = &err
        };
        esp_ipc_call_blocking(ESP_INTR_CPU_AFFINITY_TO_CORE_ID(isr_cpu_id), ipc_sspi_isr_reg_to_core, (void *) &ipc_arg);
    } else
#endif
    {
        err = alloc_sspi_interrupt(sspi, sspi_interrupt);
    }
    ESP_RETURN_ON_ERROR(err, TAG, "can't alloc SPI interrupt");

  } // fi install ISR

// ToDO
#if SOC_SPI_SUPPORT_CLK_RC_FAST
  if (bus_config->clock_source == SPI_CLK_SRC_RC_FAST) {
    ESP_RETURN_ON_FALSE(periph_rtc_dig_clk8m_enable(), ESP_ERR_INVALID_STATE, TAG, "the selected clock not available");
  }
#endif

  clk_src = SPI_CLK_SRC_DEFAULT;
  clock_source_hz = 0;
 /* if (bus_config->clock_source) {
    clk_src = dev_config->clock_source;
  }*/
  esp_clk_tree_src_get_freq_hz(clk_src, ESP_CLK_TREE_SRC_FREQ_PRECISION_APPROX, &clock_source_hz);
  ESP_LOGD(TAG, "clock source = %luHz", clock_source_hz);
  ESP_RETURN_ON_FALSE((bus_config->speed_hz > 0) && (bus_config->speed_hz <= clock_source_hz), ESP_ERR_INVALID_ARG, TAG, "invalid sclk speed");

// ToDo
#ifdef CONFIG_IDF_TARGET_ESP32
  //The hardware looks like it would support this, but actually setting cs_ena_pretrans when transferring in full
  //duplex mode does absolutely nothing on the ESP32.
  ESP_RETURN_ON_FALSE(bus_config->param.cs_ena_pretrans <= 1 || (bus_config->param.addr_bits == 0 && bus_config->param.cmd_bits == 0) ||
       (bus_config->param.flags & SPI_DEVICE_HALFDUPLEX), ESP_ERR_INVALID_ARG, TAG, "In full-duplex mode, only support cs pretrans delay = 1 and without address_bits and command_bits");
#endif

  // ToDo more config:
  //  int half_duplex   = 0; //dev_config->flags & SPI_DEVICE_HALFDUPLEX ? 1 : 0;
  //  int no_compensate = 0; //dev_config->flags & SPI_DEVICE_NO_DUMMY ? 1 : 0;
  int duty_cycle     = 128; //(dev_config->duty_cycle_pos == 0) ? 128 : dev_config->duty_cycle_pos;
  int input_delay_ns = 0; //dev_config->input_delay_ns
  int timing_dummy_n = 0, timing_miso_delay = 0;

  // get clock timing
  spi_clock_reg = 0;
  eff_spi_clock = spi_ll_master_cal_clock(clock_source_hz, bus_config->speed_hz, duty_cycle, &spi_clock_reg);
  spi_clock_reg &= 0x803FFFFF; // mask reserved (the bits are undefined from spi_ll_master_cal_clock()

  // When the speed is too fast, we may need to use dummy cycles to compensate the reading. But these don't work for full-duplex connections.
  spi_hal_cal_timing(clock_source_hz, eff_spi_clock, !(sspi->flags & SSPI_BUSFLAG_IOMUX_PINS), input_delay_ns, &timing_dummy_n, &timing_miso_delay);

#ifdef CONFIG_IDF_TARGET_ESP32
  const int freq_limit = spi_hal_get_freq_limit(timing_param->use_gpio, input_delay_ns);
  SPI_HAL_CHECK(timing_param->half_duplex || timing_dummy == 0 || timing_param->no_compensate,
                  "When work in full-duplex mode at frequency > %.1fMHz, device cannot read correct data.\n\
Try to use IOMUX pins to increase the frequency limit, or use the half duplex mode.\n\
Please note the SPI master can only work at divisors of 80MHz, and the driver always tries to find the closest frequency to your configuration.\n\
Specify ``SPI_DEVICE_NO_DUMMY`` to ignore this checking. Then you can output data at higher speed, or read data at your own risk.",
                  ESP_ERR_NOT_SUPPORTED, freq_limit / 1000. / 1000 );
#endif

  SSPI_MASTER_PERI_CLOCK_ATOMIC() {
    spi_ll_enable_clock(num, true);
  }
#if SPI_LL_MOSI_FREE_LEVEL
    // Change default data line level to low which same as esp32
    spi_ll_set_mosi_free_level(sspi->dev, 0);
#endif
    spi_ll_master_init(sspi->dev);
/*    
    if (config->dma_enabled) {
        s_spi_hal_dma_init_config(hal);
    }
*/
  ESP_RETURN_ON_ERROR(sspi_init_dma_intr(num, &sspi->tx_dma_chan), TAG, "connect TX DMA failed");
  ESP_LOGD(TAG, "TX DMA connected, channel no = %lu", sspi->tx_dma_chno);
  if (sspi->flags & SSPI_BUSFLAG_MISO) {
    ESP_RETURN_ON_ERROR(sspi_init_dma_intr(num, &sspi->rx_dma_chan), TAG, "connect RX DMA failed");
    ESP_LOGD(TAG, "RX DMA connected, channel no = %lu", sspi->rx_dma_chno);
  }

#if SOC_GDMA_SUPPORTED    
  ESP_RETURN_ON_ERROR(sspi_alloc_dma_desc(&sspi->tx_dma_desc, SSPI_GDMA_DESC_CNT), TAG, "alloc of RX GDMA desc failed");
  if (sspi->flags & SSPI_BUSFLAG_MISO) {
    ESP_RETURN_ON_ERROR(sspi_alloc_dma_desc(&sspi->rx_dma_desc, SSPI_GDMA_DESC_CNT), TAG, "alloc of RX GDMA desc failed");
  }
#endif

  //Force a transaction done interrupt. This interrupt won't fire yet because
  //we initialized the SPI interrupt as disabled. This way, we can just
  //enable the SPI interrupt and the interrupt handler will kick in, handling
  //any transactions that are queued.
  spi_ll_enable_int(sspi->dev);
  spi_ll_set_int_stat(sspi->dev);
  spi_ll_set_mosi_delay(sspi->dev, 0, 0);
  spi_ll_master_set_clock_by_reg(sspi->dev, &spi_clock_reg);
  spi_ll_master_select_cs(sspi->dev, -1); // diable all CS
  spi_ll_apply_config(sspi->dev);

  sspi->active_device = NULL;

  ESP_LOGD(TAG, "SPI%d ready, clock=%luHz.", num+1, eff_spi_clock);
  return err;
}




esp_err_t sspi_device_init(int num, int device_num, const tsspi_device_cfg *dev_config) {
  ESP_RETURN_ON_FALSE(is_valid_spi(num), ESP_ERR_INVALID_ARG, TAG, "invalid spi num");
  ESP_RETURN_ON_FALSE((device_num >= 0) && (device_num < SSPI_MAX_DEVICES), ESP_ERR_INVALID_ARG, TAG, "invalid spi device");
  ESP_RETURN_ON_FALSE((dev_config->cs_pin == GPIO_NUM_NC) || GPIO_IS_VALID_OUTPUT_GPIO(dev_config->cs_pin), ESP_ERR_INVALID_ARG, TAG, "cs pin invalid");
  ESP_RETURN_ON_FALSE((dev_config->dc_pin == GPIO_NUM_NC) || GPIO_IS_VALID_OUTPUT_GPIO(dev_config->dc_pin), ESP_ERR_INVALID_ARG, TAG, "dc pin invalid");
  spi_dev_t shadow_builder;

  tsspi_def *sspi   = &S_SPI[num];
  tsspi_dev *device = &sspi->devices[device_num];
  
  memset(device, 0, sizeof(tsspi_dev));
  memset((void *) &shadow_builder, 0, sizeof(spi_dev_t));

  // Set CS pin, CS options
  shadow_builder.misc.val = 0x3F; // diable all CSx lines by default
  if (dev_config->cs_pin != GPIO_NUM_NC) {
    sspi_cs_init(num, dev_config->cs_pin, device_num, sspi->flags & SSPI_BUSFLAG_IOMUX_PINS);
    shadow_builder.misc.val &= ~(1 << device_num);    
  }  
  
  device->dc_pin = dev_config->dc_pin;
  if (device->dc_pin != GPIO_NUM_NC) {
    gpio_set_direction(device->dc_pin, GPIO_MODE_OUTPUT);
  }

  shadow_builder.user.usr_mosi = sspi->flags & SSPI_BUSFLAG_MOSI? 1: 0;
  shadow_builder.user.usr_miso = sspi->flags & SSPI_BUSFLAG_MISO? 1: 0;
  if (shadow_builder.user.val == 0) {
    ESP_LOGW(TAG, "no MOSI nor MISO enabled");
  }
  
  shadow_builder.user.ck_out_edge   = dev_config->mode.cpol;
  shadow_builder.misc.master_cs_pol = dev_config->mode.cpha;
  
  shadow_builder.user.doutdin  = !dev_config->conf.half_duplex;
  shadow_builder.user.sio      = dev_config->conf.sio_mode; // "1" if MOSI & MISO share same pin
  shadow_builder.user.cs_setup = dev_config->conf.cs_setup;
  shadow_builder.user.cs_hold  = dev_config->conf.cs_hold;

  if (sspi->dev == NULL) {
    ESP_LOGE(TAG, "spi hardware not initialized!");
  } else {
    shadow_builder.user1.val = sspi->dev->user1.val;
    shadow_builder.user2.val = sspi->dev->user2.val;
  }
  if (dev_config->cmd_bits) {
    shadow_builder.user.usr_command = 1;
    shadow_builder.user2.usr_command_bitlen = dev_config->cmd_bits - 1;
  }
  if (dev_config->addr_bits) {
    shadow_builder.user.usr_addr = 1;
    shadow_builder.user1.usr_addr_bitlen = dev_config->addr_bits - 1;
  }
  shadow_builder.user1.cs_setup_time = dev_config->cs_setup_time? dev_config->cs_setup_time - 1: 0;
  shadow_builder.user1.cs_hold_time = dev_config->cs_hold_time;

  device->user_reg  = shadow_builder.user.val;
  device->user1_reg = shadow_builder.user1.val;
  device->user2_reg = shadow_builder.user2.val;
  device->misc_reg  = shadow_builder.misc.val;
  return ESP_OK;
}



static void IRAM_ATTR sspi_select_device(tsspi_def *sspi, tsspi_dev *device) {
  sspi->dev->user.val  = device->user_reg;
  sspi->dev->user1.val = device->user1_reg;
  sspi->dev->user2.val = device->user2_reg;
  sspi->dev->misc.val  = device->misc_reg;
  spi_ll_apply_config(sspi->dev);
  sspi->active_device = device;  
}


esp_err_t sspi_device_select(int num, int device_num) {
  ESP_RETURN_ON_FALSE(is_valid_spi(num), ESP_ERR_INVALID_ARG, TAG, "invalid spi num");
  ESP_RETURN_ON_FALSE((device_num >= 0) && (device_num < SSPI_MAX_DEVICES), ESP_ERR_INVALID_ARG, TAG, "invalid spi device");
  tsspi_def *sspi   = &S_SPI[num];
  sspi_select_device(sspi, &sspi->devices[device_num]);
  ESP_LOGD(TAG, "device %d selected", device_num);
  return ESP_OK;
}




#if SOC_NON_CACHEABLE_OFFSET  // only define in P4 target!
#define ADDR_DMA_2_CPU(addr)   ((typeof(addr))((uint32_t)(addr) + SOC_NON_CACHEABLE_OFFSET))
#define ADDR_CPU_2_DMA(addr)   ((typeof(addr))((uint32_t)(addr) - SOC_NON_CACHEABLE_OFFSET))
#else
#define ADDR_DMA_2_CPU(addr)   (addr)
#define ADDR_CPU_2_DMA(addr)   (addr)
#endif

static uint32_t IRAM_ATTR sspi_dma_desc_setup_link(lldesc_t *dmadesc, const void *data, uint32_t len, bool is_rx, bool is_loop) {
  lldesc_t *prev_dmadesc = NULL;
  lldesc_t *dmadesc_ptr  = ADDR_DMA_2_CPU(dmadesc);
  uint32_t dmachunklen = 0;
  uint32_t loaded_len  = 0;
  for (int n = 0; (len > 0) && (n < SSPI_GDMA_DESC_CNT); n++, dmadesc_ptr++, len -= dmachunklen, data += dmachunklen, loaded_len += dmachunklen) {
    if (prev_dmadesc != NULL) {
      STAILQ_NEXT(prev_dmadesc, qe) = ADDR_CPU_2_DMA(dmadesc_ptr);
    } // fi prev
    dmachunklen = len;
    if (dmachunklen > DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED) {
      dmachunklen = DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED;
    }
    /*
    if (is_rx) {      // bullshit: Receive needs DMA length rounded to next 32-bit boundary      
      dmadesc_ptr->size   = (dmachunklen + 3) & (~3);
      dmadesc_ptr->length = (dmachunklen + 3) & (~3);
    } else*/ {
      dmadesc_ptr->size   = dmachunklen;
      dmadesc_ptr->length = dmachunklen;
    }    
    dmadesc_ptr->buf = (const uint8_t *)data;
    dmadesc_ptr->owner = 1;
    dmadesc_ptr->eof   = 0;
    //ESP_EARLY:LOGD(TAG, "dmadesc %d: @ %8p len=%d", n, dmadesc->buf, dmadesc->length);
    prev_dmadesc = dmadesc_ptr;    
  } // rof
  STAILQ_NEXT(prev_dmadesc, qe) = is_loop? dmadesc: NULL; // link list ends here (or not)
  prev_dmadesc->eof = !is_loop;
  //ESP_EARLY_LOGD(TAG, "dmadesc leftover len=%d", len); 
  return loaded_len; // this length is left over...
}


static void IRAM_ATTR sspi_dma_desc_setup_2buf(lldesc_t *dmadesc, const void *data1, uint16_t len1, const void *data2, uint32_t len2, bool is_rx) {
  bool data2_used = (data2 != NULL) && (len2 > 0);
  lldesc_t *dmadesc_ptr = ADDR_DMA_2_CPU(dmadesc);
  dmadesc_ptr[0].size   = len1;
  dmadesc_ptr[0].length = len1;
  dmadesc_ptr[0].buf    = (const uint8_t *)data1;
  dmadesc_ptr[0].owner  = 1;
  dmadesc_ptr[0].eof    = data2_used? 0: 1;
  STAILQ_NEXT(&dmadesc_ptr[0], qe) = data2_used? ADDR_CPU_2_DMA(&dmadesc_ptr[1]): NULL;
  if (data2_used) {
    sspi_dma_desc_setup_link(&dmadesc[1], data2, len2, is_rx, false);
  } // fi
  /*
  dmadesc_ptr[1].size   = len2;
  dmadesc_ptr[1].length = len2;      
  dmadesc_ptr[1].buf    = (const uint8_t *)data2;
  dmadesc_ptr[1].owner  = 1;
  dmadesc_ptr[1].eof    = 1;
  STAILQ_NEXT(&dmadesc_ptr[1], qe) = NULL;*/
}



//This GDMA related part will be introduced by GDMA dedicated APIs in the future. Here we temporarily use macros.
#if SOC_GDMA_SUPPORTED
#if (SOC_GDMA_TRIG_PERIPH_SPI2_BUS == SOC_GDMA_BUS_AHB) && (SOC_AHB_GDMA_VERSION == 1)
#include "soc/gdma_struct.h"
#include "hal/gdma_ll.h"

#define sspi_dma_ll_rx_reset(chan)                             gdma_ll_rx_reset_channel(&GDMA, chan)
#define sspi_dma_ll_tx_reset(chan)                             gdma_ll_tx_reset_channel(&GDMA, chan);
#define sspi_dma_ll_rx_start(chan, addr) do {\
            gdma_ll_rx_set_desc_addr(&GDMA, chan, (uint32_t)addr); gdma_ll_rx_start(&GDMA, chan);\
        } while (0)
#define sspi_dma_ll_tx_start(chan, addr) do {\
            gdma_ll_tx_set_desc_addr(&GDMA, chan, (uint32_t)addr); gdma_ll_tx_start(&GDMA, chan);\
        } while (0)

#elif (SOC_GDMA_TRIG_PERIPH_SPI2_BUS == SOC_GDMA_BUS_AXI)   //TODO: IDF-6152, refactor spi hal layer
#include "hal/axi_dma_ll.h"
#define sspi_dma_ll_rx_reset(chan)                             axi_dma_ll_rx_reset_channel(&AXI_DMA, chan)
#define sspi_dma_ll_tx_reset(chan)                             axi_dma_ll_tx_reset_channel(&AXI_DMA, chan);
#define sspi_dma_ll_rx_start(chan, addr) do {\
            axi_dma_ll_rx_set_desc_addr(&AXI_DMA, chan, (uint32_t)addr);\
            axi_dma_ll_rx_start(&AXI_DMA, chan);\
        } while (0)
#define sspi_dma_ll_tx_start(chan, addr) do {\
            axi_dma_ll_tx_set_desc_addr(&AXI_DMA, chan, (uint32_t)addr);\
            axi_dma_ll_tx_start(&AXI_DMA, chan);\
        } while (0)
#endif
#endif  //SOC_GDMA_SUPPORTED



static int32_t IRAM_ATTR sspi_setup_tx_dma(tsspi_def *sspi, const void *chunk_data, uint32_t length, bool is_loop) {
  uint32_t loaded_len = 0;
  spi_dev_t *spihw = sspi->dev;
#if SOC_GDMA_SUPPORTED
  loaded_len = sspi_dma_desc_setup_link(sspi->tx_dma_desc, chunk_data, length, false, is_loop);
  sspi_dma_ll_tx_reset(sspi->tx_dma_chno);
  //gdma_reset(sspi->tx_dma_chan); <- maybe not int safe?
  spi_ll_dma_tx_fifo_reset(spihw);
  spi_ll_outfifo_empty_clr(spihw);
  sspi_dma_ll_tx_start(sspi->tx_dma_chno, sspi->tx_dma_desc);
  //gdma_start(sspi->tx_dma_chan, (uint32_t)sspi->tx_dma_desc);  
#else
  // ToDo
  spi_ll_dma_tx_fifo_reset(spihw);
  spi_ll_outfifo_empty_clr(spihw);

#endif
  return loaded_len;
}


static void sspi_setup_tx_2buf(tsspi_def *sspi, const void *data1, uint16_t length1, const void *data2, uint32_t length2) {
  spi_dev_t *spihw = sspi->dev;
#if SOC_GDMA_SUPPORTED
  sspi_dma_desc_setup_2buf(sspi->tx_dma_desc, data1, length1, data2, length2, false);
  sspi_dma_ll_tx_reset(sspi->tx_dma_chno);
  spi_ll_dma_tx_fifo_reset(spihw);
  spi_ll_outfifo_empty_clr(spihw);
  sspi_dma_ll_tx_start(sspi->tx_dma_chno, sspi->tx_dma_desc);
#else
  // ToDo
  spi_ll_dma_tx_fifo_reset(spihw);
  spi_ll_outfifo_empty_clr(spihw);
#endif
}


static void sspi_setup_rx_2buf(tsspi_def *sspi, const void *data1, uint16_t length1, const void *data2, uint32_t length2) {
  spi_dev_t *spihw = sspi->dev;
#if SOC_GDMA_SUPPORTED
  sspi_dma_desc_setup_2buf(sspi->rx_dma_desc, data1, length1, data2, length2, false);
  sspi_dma_ll_rx_reset(sspi->rx_dma_chno);
  spi_ll_dma_rx_fifo_reset(spihw);
  spi_ll_infifo_full_clr(spihw);
  sspi_dma_ll_rx_start(sspi->rx_dma_chno, sspi->rx_dma_desc);
#else
  // ToDo
  spi_ll_dma_rx_fifo_reset(spihw);
  spi_ll_infifo_full_clr(spihw);
#endif
}



static int32_t IRAM_ATTR sspi_transfer_chunk(tsspi_def *sspi, const void *chunk_data, uint32_t length) {
  spi_dev_t *spihw = sspi->dev;
  uint32_t loaded_len = 0;
  if (length == 0) return 0;
  if (chunk_data == NULL) { // do noting exept retrigger DMA transfer

    // loop buffers or transfer predefined DMA
    loaded_len = (length > 32736)? 32736: length;
    spi_ll_dma_tx_enable(spihw, 1);

  } else if (length <= 64) {

    spi_ll_dma_tx_enable(spihw, 0);
    //ESP_EARLY_LOGD(TAG, "spi fifo only, %u bytes", length);
    spi_ll_write_buffer(spihw, chunk_data, length << 3); 
    loaded_len = length;

  } else {

    loaded_len = sspi_setup_tx_dma(sspi, chunk_data, length, false);
    spi_ll_dma_tx_enable(spihw, 1);

  } // fi DMA xfer

  spihw->ms_dlen.ms_data_bitlen = (loaded_len << 3) - 1;
  spihw->misc.cs_keep_active    = loaded_len < length;
  spi_ll_apply_config(spihw);
  spi_ll_user_start(spihw);  
  return length - loaded_len;
}


static void IRAM_ATTR sspi_interrupt(void *arg) {
  BaseType_t do_yield = pdFALSE;
  tsspi_def *sspi  = (tsspi_def *) arg;
  spi_dev_t *spihw = sspi->dev;
  tsspi_dev *dev   = sspi->active_device;

  assert(spi_ll_usr_is_done(spihw));

  esp_intr_disable(sspi->intr);

  spi_ll_dma_rx_enable(spihw, 0);
  spi_ll_dma_tx_enable(spihw, 0);

#if CONFIG_IDF_TARGET_ESP32
  //This workaround is only for esp32, where tx_dma_chan and rx_dma_chan are always same
  spicommon_dmaworkaround_idle(sspi->tx_dma_chan);
#endif  //#if CONFIG_IDF_TARGET_ESP32

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE   //invalidate here to let user access rx data in post_cb if possible

#endif

#ifdef CONFIG_PM_ENABLE
  //Release APB frequency lock
  esp_pm_lock_release(sspi->pm_lock);
#endif
  if (sspi->remaining_rx_bytes > 0) {

  }

  if (sspi->remaining_tx_bytes > 0) {
    int32_t leftover_len = 0;

    spi_ll_clear_int_stat(spihw); // new transmision - waiting of the tx_done_int, no other ints after enabling

#if CONFIG_IDF_TARGET_ESP32
    if (sspi->tx_transfer_buf || sspi->rx_transfer_buf) {
      //mark channel as active, so that the DMA will not be reset by the slave
      //This workaround is only for esp32, where tx_dma_chan and rx_dma_chan are always same
      spicommon_dmaworkaround_transfer_active(sspi->tx_dma_chan);
    }
#endif  //#if CONFIG_IDF_TARGET_ESP32

    // Todo setup RX
    leftover_len = sspi_transfer_chunk(sspi, sspi->tx_transfer_buf, sspi->remaining_tx_bytes);

    if (leftover_len == 0) {
      sspi->tx_transfer_buf    = NULL;
      sspi->remaining_tx_bytes = 0;
    } else { // fi next
      if (sspi->tx_transfer_buf != NULL) {
        sspi->tx_transfer_buf += sspi->remaining_tx_bytes - leftover_len;
      }
      sspi->remaining_tx_bytes = leftover_len;
    }
    esp_intr_enable(sspi->intr);

  } else if (dev && dev->waiting_task) {
    xTaskNotifyIndexedFromISR(dev->waiting_task, SSPI_NOTIFY_INDEX, SSPI_TRANSFER_DONE_BIT, eSetBits, &do_yield);
  }

  if (do_yield) portYIELD_FROM_ISR();
}



static inline void sspi_set_dc_pin(const tsspi_def *sspi, uint32_t level) {
  gpio_num_t dc_pin = sspi->active_device->dc_pin;
  if (dc_pin != GPIO_NUM_NC) {
    gpio_ll_set_level(&GPIO, dc_pin, level);
  }  
}




bool sspi_wait_transfer(tsspi_dev *dev, unsigned int timeout_ms) {
  uint32_t return_bits;
  BaseType_t wait_res = xTaskNotifyWaitIndexed(SSPI_NOTIFY_INDEX, SSPI_TRANSFER_DONE_BIT, SSPI_TRANSFER_DONE_BIT, &return_bits, pdMS_TO_TICKS(timeout_ms));
  dev->waiting_task = NULL;
  if (wait_res) {
    ESP_LOGV(TAG, "tx finished");
  } else {
    ESP_LOGD(TAG, "tx timeout");
  }
  return wait_res;
}


bool sspi_wait_finish(int num, unsigned int timeout_ms) {
  ESP_RETURN_ON_FALSE(is_valid_spi(num), false, TAG, "invalid spi num");
  tsspi_def *sspi = &S_SPI[num];
  if (sspi->active_device->waiting_task != xTaskGetCurrentTaskHandle()) {
    while (sspi->dev->cmd.usr); // ToDo
    return true;
  }
  return sspi_wait_transfer(sspi->active_device, timeout_ms);
}




esp_err_t sspi_send_cmd_w_arg(int num, U8 cmd, U8 arg_size, U32 arg) {

  ESP_RETURN_ON_FALSE(is_valid_spi(num), ESP_ERR_INVALID_ARG, TAG, "invalid spi num");
  tsspi_def *sspi = &S_SPI[num];
  spi_dev_t *spihw = sspi->dev;
  ESP_RETURN_ON_FALSE(sspi->active_device != NULL, ESP_ERR_NOT_FOUND, TAG, "no device selected");
  
  spi_ll_dma_tx_enable(spihw, 0);

  spihw->data_buf[8] = (uint32_t)cmd;

  ESP_LOGV(TAG, "send cmd %02xh, arg %08lxh", cmd, arg);

  spihw->misc.cs_keep_active = (arg_size > 0);
  spihw->user.usr_mosi_highpart = 1;
  spihw->ms_dlen.ms_data_bitlen = 7;

  spihw->cmd.update = 1;    
  sspi_set_dc_pin(sspi, 0);     // cmd mode
  while (spihw->cmd.update);    // waiting config applied

  spihw->cmd.usr = 1;
  while (spihw->cmd.usr);       // wait till finish
  sspi_set_dc_pin(sspi, 1);     // data mode

  if (arg_size == 0) return ESP_OK;

  spihw->data_buf[8] = arg;  
  spihw->misc.cs_keep_active = 0;
  spihw->ms_dlen.ms_data_bitlen = (arg_size << 3) - 1;

  spihw->cmd.update = 1;
  while (spihw->cmd.update);    // waiting config applied

  spihw->cmd.usr = 1;
  while (spihw->cmd.usr);       // wait till finish

  //ESP_LOGD(TAG, "spi out: %08lxh; in: %08lxh, len: %dbits", spihw->data_buf[8], spihw->data_buf[0], spihw->ms_dlen.ms_data_bitlen + 1);
  return ESP_OK;
}



esp_err_t sspi_transfer_data(int num, const void *data_to_send, void *receive_buffer, unsigned int length, int wait_ms) {
  ESP_RETURN_ON_FALSE(is_valid_spi(num), ESP_ERR_INVALID_ARG, TAG, "invalid spi num");
  tsspi_def *sspi = &S_SPI[num];

  ESP_RETURN_ON_FALSE(sspi->dev->cmd.usr == 0, ESP_FAIL, TAG, "SPI is busy");
  ESP_RETURN_ON_FALSE(sspi->active_device != NULL, ESP_ERR_NOT_FOUND, TAG, "no device selected");

  tsspi_dev *dev = sspi->active_device;
  
  dev->waiting_task = xTaskGetCurrentTaskHandle();

  sspi->tx_transfer_buf    = data_to_send;  
  sspi->remaining_tx_bytes = length;
  //sspi->tx_loop_buf_size   = 0;

  if (sspi->rx_dma_desc && receive_buffer) {
    sspi->rx_transfer_buf    = receive_buffer;
    sspi->remaining_rx_bytes = length;
  }

  ESP_LOGV(TAG, "send %u bytes", length);
  //tx_call_cnt = 0;
  esp_intr_enable(sspi->intr);
  if (wait_ms == 0) return ESP_OK;
  return !sspi_wait_transfer(dev, wait_ms)? ESP_ERR_TIMEOUT: ESP_OK;
}


esp_err_t sspi_loop_data(int num, const void *loop_buffer_to_send, unsigned int buffer_size, unsigned int length, int wait_ms) {
  ESP_RETURN_ON_FALSE(is_valid_spi(num), ESP_ERR_INVALID_ARG, TAG, "invalid spi num");
  tsspi_def *sspi  = &S_SPI[num];
  spi_dev_t *spihw = sspi->dev;
  tsspi_dev *dev   = sspi->active_device;

  ESP_RETURN_ON_FALSE(spihw->cmd.usr == 0, ESP_FAIL, TAG, "SPI is busy");
  ESP_RETURN_ON_FALSE(sspi->active_device != NULL, ESP_ERR_NOT_FOUND, TAG, "no device selected");
  ESP_RETURN_ON_FALSE(buffer_size <= 32736, ESP_ERR_INVALID_ARG, TAG, "loop buffer size to large (>32K-32)");

  dev->waiting_task = xTaskGetCurrentTaskHandle();
  sspi->tx_transfer_buf    = NULL;
  sspi->remaining_tx_bytes = length;

  sspi_setup_tx_dma(sspi, loop_buffer_to_send, buffer_size, true); 

  ESP_LOGV(TAG, "fill-loop %u bytes (buf=%d bytes)", length, buffer_size);
  //tx_call_cnt = 0;
  esp_intr_enable(sspi->intr);
  if (wait_ms == 0) return ESP_OK;
  return sspi_wait_transfer(dev, wait_ms)? ESP_OK: ESP_ERR_TIMEOUT;
}




/*
 * SX126x driver HAL
 */
#define MAX_STATUSBUF_SIZE            16
#define SX126X_MAX_TRANSFER_TIME_MS   50

#define WAIT_SPI_LOOP_CNT             1000000L


struct sx126x_ctx_struct {
  tsspi_def *  sspi;
  uint8_t      cs_num;
  gpio_num_t   nss_pin;
  gpio_num_t   busy_pin;
  gpio_num_t   reset_pin;
  uint8_t      status_buf[MAX_STATUSBUF_SIZE];
};



esp_err_t sx126x_init_hal(tsx126x_ctx **context, const tsx126x_config *sx126x_config) {
  esp_err_t err;
  tsx126x_ctx *mycontext;
  if ((sx126x_config == NULL) || (context == NULL)) return ESP_ERR_INVALID_ARG;
  tsspi_device_cfg dev_cfg = { 
    .cs_pin    = sx126x_config->nss_pin,
    .dc_pin    = GPIO_NUM_NC,
    .cmd_bits  = 8,
    .addr_bits = 8,
    .cs_setup_time = 0,
    .cs_hold_time  = 0,
    .conf = { .cs_hold = 0, .cs_setup = 0 }
  };
  *context = NULL;

  if (sx126x_config->nss_pin != GPIO_NUM_NC) {
    gpio_set_level(sx126x_config->nss_pin, 1);
    gpio_set_direction(sx126x_config->nss_pin, GPIO_MODE_OUTPUT);
  }
  err = sspi_device_init(sx126x_config->spi_num, sx126x_config->spi_device, &dev_cfg);
  if (err != ESP_OK) return err;
  size_t context_size = 0;
  mycontext = sspi_dma_calloc(1, sizeof(struct sx126x_ctx_struct), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA, &context_size);
  ESP_RETURN_ON_FALSE(mycontext != NULL, ESP_ERR_NO_MEM, TAG, "no memory for S126x context");

  mycontext->sspi   = &S_SPI[sx126x_config->spi_num];
  mycontext->cs_num = sx126x_config->spi_device;

  mycontext->nss_pin   = sx126x_config->nss_pin;
  mycontext->reset_pin = sx126x_config->reset_pin;
  mycontext->busy_pin  = sx126x_config->busy_pin;
  // Todo
  // gpio init BUSY, Reset 

  // sspi_check_pin(pin_num, pin_name, check_output)

  if (mycontext->reset_pin != GPIO_NUM_NC) {
    sspi_check_pin(mycontext->reset_pin, "SX126x Reset", true);
    gpio_set_level(mycontext->reset_pin, 0);
    gpio_set_direction(mycontext->reset_pin, GPIO_MODE_OUTPUT);
  } // fi
  int64_t reset_delay_to = esp_timer_get_time() + 120;

  if (mycontext->busy_pin != GPIO_NUM_NC) {
    sspi_check_pin(mycontext->busy_pin, "SX126x BUSY", false);
    gpio_set_direction(mycontext->busy_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(mycontext->busy_pin, GPIO_PULLDOWN_ONLY);
  } // fi

  *context = mycontext;
  // last action: get SX126x out of reset. Pin must be held low for min 100µs
  // TODo insert some µwait
  if (mycontext->reset_pin != GPIO_NUM_NC) {
    while (esp_timer_get_time() < reset_delay_to); // wait the test
    gpio_set_level(mycontext->reset_pin, 1);
  }
  return ESP_OK;
}




inline esp_err_t sx126x_hal_wait_spi(const tsx126x_ctx *ctx) {
  ESP_RETURN_ON_FALSE_ISR(ctx != NULL, ESP_ERR_INVALID_ARG, TAG, "context is NULL");
  spi_dev_t *spihw = ctx->sspi->dev;
  for (int i=WAIT_SPI_LOOP_CNT; (spihw->cmd.usr) && (i > 0); i--);
  return spihw->cmd.usr? ESP_FAIL: ESP_OK;
}



//bool sx126x_is_busy(const tsx126x_ctx *ctx);
inline bool sx126x_is_busy(const tsx126x_ctx *ctx) {
  return (ctx==NULL) || gpio_get_level(ctx->busy_pin);
}


bool sx126x_hal_wait_busy(const tsx126x_ctx* context) {
  int64_t max_wakeup_delay_to;
  if (sx126x_hal_wait_spi(context) != ESP_OK) return false;  
  max_wakeup_delay_to = esp_timer_get_time() + 4;
  while (esp_timer_get_time() < max_wakeup_delay_to); // wait the first 3-4µs (BUSY goes high 600ns after NSS goes high!)
  max_wakeup_delay_to += 3500;
  while (sx126x_is_busy(context) && (esp_timer_get_time() < max_wakeup_delay_to)) {
    taskYIELD();
  } // ehliw
  return !sx126x_is_busy(context);
}


static void sx126x_disconnect_nss(const tsx126x_ctx *ctx) {
  sspi_cs_disconnect_out(ctx->sspi->id, ctx->nss_pin, ctx->cs_num, ctx->sspi->flags & SSPI_BUSFLAG_IOMUX_PINS);
}


static void sx126x_reconnect_nss(const tsx126x_ctx *ctx) {
  sspi_cs_reconnect_out(ctx->sspi->id, ctx->nss_pin, ctx->cs_num, ctx->sspi->flags & SSPI_BUSFLAG_IOMUX_PINS);
}



static sx126x_hal_status_t sx126x_hal_io(const tsx126x_ctx *ctx, const uint8_t* command, const uint16_t command_length, const uint8_t* data, const uint16_t data_length, bool rx) {
  ESP_RETURN_ON_FALSE(ctx != NULL, SX126X_HAL_STATUS_ERROR, TAG, "context is NULL");
  ESP_RETURN_ON_FALSE((command_length <= MAX_STATUSBUF_SIZE), SX126X_HAL_STATUS_ERROR, TAG, "command_length invalid");
  tsspi_def *sspi  = ctx->sspi; 
  spi_dev_t *spihw = sspi->dev;
  tsspi_dev *dev;
  for (int i=WAIT_SPI_LOOP_CNT; (spihw->cmd.usr) && (i > 0); i--);
  ESP_RETURN_ON_FALSE(spihw->cmd.usr == 0, SX126X_HAL_STATUS_ERROR, TAG, "SPI is busy");  
  if (sspi->active_device != &sspi->devices[ctx->cs_num]) {
    sspi_select_device(sspi, &sspi->devices[ctx->cs_num]);
    ESP_LOGV(TAG, "select SX126x device #%d", ctx->cs_num);
  }
  dev = sspi->active_device;  
  sspi->remaining_tx_bytes = 0;
#if CONFIG_LOG_DEFAULT_LEVEL > 2
  memset((void *)ctx->status_buf, 0xFF, sizeof(ctx->status_buf));
#endif
  if (rx) {
    if (data_length == 0) return SX126X_HAL_STATUS_OK; // no need to read...
    sspi_setup_tx_2buf(sspi, command, command_length, NULL, 0);
    sspi_setup_rx_2buf(sspi, ctx->status_buf, command_length, data, data_length);
  } else {
    sspi_setup_tx_2buf(sspi, command, command_length, data, data_length);
    sspi_setup_rx_2buf(sspi, ctx->status_buf, command_length, NULL, 0);
  }  
  spihw->ms_dlen.ms_data_bitlen = ((command_length + data_length) << 3) - 1;
  spihw->user.usr_command = 0;
  spihw->user.usr_addr = 0;
  spi_ll_clear_int_stat(spihw);
  spi_ll_dma_tx_enable(spihw, 1);
  spi_ll_dma_rx_enable(spihw, 1);
  spi_ll_apply_config(spihw);
  spi_ll_user_start(spihw);

  ESP_LOGV(TAG, "sx126x_hal_%s(%u cmd, %u data)", rx? "read": "write", command_length, data_length);
  dev->waiting_task = xTaskGetCurrentTaskHandle();
  xTaskNotifyStateClear(dev->waiting_task);
  esp_intr_enable(sspi->intr);
  return sspi_wait_transfer(dev, SX126X_MAX_TRANSFER_TIME_MS)? SX126X_HAL_STATUS_OK: SX126X_HAL_STATUS_ERROR;
}


sx126x_hal_status_t sx126x_hal_write(const void* context, const uint8_t* command, const uint16_t command_length, const uint8_t* data, const uint16_t data_length) {
  sx126x_hal_status_t wr_s = sx126x_hal_io((const tsx126x_ctx *) context, command, command_length, data, data_length, false);
  sx126x_hal_wait_busy((const tsx126x_ctx *) context);
  return wr_s;
}


sx126x_hal_status_t sx126x_hal_read(const void* context, const uint8_t* command, const uint16_t command_length, uint8_t* data, const uint16_t data_length) {
  return sx126x_hal_io((const tsx126x_ctx *) context, command, command_length, data, data_length, true);
}


esp_err_t sx126x_hal_fast_cmd(const tsx126x_ctx *ctx, const uint8_t* command, uint16_t command_length) {
  ESP_RETURN_ON_FALSE_ISR(ctx != NULL, ESP_ERR_INVALID_ARG, TAG, "context is NULL");
  tsspi_def *sspi  = ctx->sspi; 
  spi_dev_t *spihw = sspi->dev;
  for (int i=WAIT_SPI_LOOP_CNT; (spihw->cmd.usr) && (i > 0); i--);
  ESP_RETURN_ON_FALSE_ISR(spihw->cmd.usr == 0, ESP_ERR_NOT_FINISHED, TAG, "SPI is busy");
  if (sspi->active_device != &sspi->devices[ctx->cs_num]) {
    sspi_select_device(sspi, &sspi->devices[ctx->cs_num]);
    ESP_EARLY_LOGV(TAG, "select SX126x device #%d", ctx->cs_num);
  }
  spihw->user.usr_command = 0;
  spihw->user.usr_addr    = 0;
  sspi_transfer_chunk(sspi, command, command_length);
  return ESP_OK;
}


esp_err_t sx126x_hal_fast_bufferwrite(const tsx126x_ctx *ctx, uint8_t offset, const uint8_t* data, uint8_t length) {
  ESP_RETURN_ON_FALSE_ISR(ctx != NULL, ESP_ERR_INVALID_ARG, TAG, "context is NULL");
  tsspi_def *sspi  = ctx->sspi; 
  spi_dev_t *spihw = sspi->dev;
  for (int i=WAIT_SPI_LOOP_CNT; (spihw->cmd.usr) && (i > 0); i--);
  ESP_RETURN_ON_FALSE_ISR(spihw->cmd.usr == 0, ESP_ERR_NOT_FINISHED, TAG, "SPI is busy");
  if (sspi->active_device != &sspi->devices[ctx->cs_num]) {
    sspi_select_device(sspi, &sspi->devices[ctx->cs_num]);
    ESP_EARLY_LOGV(TAG, "select SX126x device #%d", ctx->cs_num);
  }
  spihw->user.usr_command = 1;
  spihw->user.usr_addr    = 1;
  spihw->user2.usr_command_value = 0x0E; // 0x0E = OPcode SX126x for buffer write operation
  spihw->addr = (uint32_t)offset << 24;
  sspi_transfer_chunk(sspi, data, length);
  return ESP_OK;
}


sx126x_hal_status_t sx126x_hal_reset(const void *context) {
  if (context == NULL) return SX126X_HAL_STATUS_ERROR;
  const tsx126x_ctx *ctx = (const tsx126x_ctx *) context;
  if (ctx->reset_pin != GPIO_NUM_NC) {
    gpio_set_level(ctx->reset_pin, 0);
    int64_t reset_delay_to = esp_timer_get_time() + 120;
    while (esp_timer_get_time() < reset_delay_to); // wait the test
    gpio_set_level(ctx->reset_pin, 1);
    return SX126X_HAL_STATUS_OK;
  }
  return SX126X_HAL_STATUS_ERROR;
}


sx126x_hal_status_t sx126x_hal_wakeup(const void* context) {  
  if (context == NULL) return SX126X_HAL_STATUS_ERROR;
  const tsx126x_ctx *ctx = (const tsx126x_ctx *) context;
  const uint8_t standby_rc_cmd[2] = { 0x80, 0x00 };

  gpio_set_level(ctx->nss_pin, 0);
  sx126x_disconnect_nss(ctx);
  int64_t wakeup_delay_to = esp_timer_get_time() + 120;
  while (esp_timer_get_time() < wakeup_delay_to); // wait the test
  sx126x_hal_wait_busy(context);
  sx126x_hal_fast_cmd(context, standby_rc_cmd, 2);
  sx126x_hal_wait_spi(context);
  gpio_set_level(ctx->nss_pin, 1);
  sx126x_reconnect_nss(ctx);
  return sx126x_hal_wait_busy(context)? SX126X_HAL_STATUS_OK: SX126X_HAL_STATUS_ERROR;
}


unsigned char * sx126x_get_status_buffer(const tsx126x_ctx *context) {
  return (context==NULL)? NULL: (unsigned char *) context->status_buf;
}
