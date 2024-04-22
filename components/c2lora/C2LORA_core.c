/*

This source file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

*/

#include "C2LORA_core.h"
#include "C2LORA_modes.h"

#include "hardware.h"

#include "sx126x_hal.h"
#include "sx126x_regs.h"


#include "esp_err.h"
#include "esp_check.h"
#include "esp_attr.h"
#include "esp_timer.h"


#include "driver/gpio.h"

#include <string.h>

static const char *TAG = "C2LORAcore";

#define C2LORA_TIMEOUT_580            ((C2LORA_WHEADER_FRAMELEN_MS+24)*64)

#define C2LORA_MAX_JITTER_US          22500   // because of the freeRTOS 10ms tick there are some 10ms gaps that can occur... 
                                              // w/o there is only a jitter of max 2 symbol length approx. 4000µs


static const uint8_t sx126x_start_hdr[4] = { SX126X_CMD_START_TX, 0, (uint8_t)(C2LORA_TIMEOUT_580 >> 8), (uint8_t)(C2LORA_TIMEOUT_580 >> 0) };
static const uint8_t sx126x_clear_sta[3] = { SX126X_CMD_CLEAR_IRQSTA, (uint8_t)(SX126X_IRQ_ALL >> 8), (uint8_t)SX126X_IRQ_ALL };



esp_err_t C2LORA_init_spi(void) {
  esp_err_t err = ESP_OK;  
  const tsspi_bus_config bus_cfg = {
    .speed_hz = SX126X_SPISPEED_MHZ * 1000000L,
    .gpio = {
      .sclk   = SX126X_SCK_Pin,
      .mosi   = SX126X_MOSI_Pin,
      .miso   = SX126X_MISO_Pin,
      .cd_sel = GPIO_NUM_NC,
      .quadhd = GPIO_NUM_NC,
      .quadwp = GPIO_NUM_NC
    }
  };
  ESP_LOGD(TAG, "C2LORA_init_spi");
  err = sspi_init(SX126X_HOST, &bus_cfg);
  return err;
}



esp_err_t C2LORA_prepare_streamstruct(tLoraStream *lorastream, tC2LORA_mode mode) {
  ESP_RETURN_ON_FALSE(lorastream != NULL, ESP_ERR_INVALID_ARG, TAG, "arument is NULL");

  const uint8_t pkt_params[8] = {
    SX126X_PKT_PARAMS_CMD, 0, 12, SX126X_LORA_PKT_IMPLICIT, 32, 0, 0
  };
  memset(lorastream, 0, sizeof(struct sLoraStream));
  lorastream->def = C2LORA_get_parameter4mode(mode);
  memcpy(lorastream->pkt_params, pkt_params, sizeof(lorastream->pkt_params));  // prepare pkt params pkt
  if (lorastream->def == NULL) {
    ESP_LOGE(TAG, "C2LORA mode not defined (mode #%d)", mode);
    return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}



void C2LORA_prepare_cydata(uint8_t *cydata, uint8_t d_length) {
  for (int i = 0; i < d_length; i++) {
    cydata[i] = 0xA0 + i;
  }
  memset(cydata + d_length, 0x55, 32 - d_length); // fille unused
}


bool C2LORA_is_header(bool last_was_header, uint8_t first_byte, int32_t interpkt_dur_us) {
  uint8_t header_byte = first_byte & ~C2LORA_FRAME_STARTBYTE;
  switch (first_byte) {            // header byte w/o error: no doubt its a '33h' or 'CCh'
  case C2LORA_FRAME_STARTBYTE:
    return false;
  case (uint8_t)(~C2LORA_FRAME_STARTBYTE):
    return true;
  } // hctiws
  // if the last packet was a header or the packet begins at teh expected time -> return false
  // RX task must set "with_header" to false after a stream ends, to get the first header of a new transmission
  if ( (last_was_header) || (abs(interpkt_dur_us - (C2LORA_DVOICE_FRAMELEN_MS * 1000L) < C2LORA_MAX_JITTER_US)) ) {
    return false;
  }
  first_byte &= C2LORA_FRAME_STARTBYTE;
  int header_bits = __builtin_popcount(header_byte);
  return (header_bits == 4)  || (header_bits > __builtin_popcount(first_byte));
}


sx126x_errors_mask_t SX126x_GetDeviceError(const tsx126x_ctx *ctx) {
  sx126x_errors_mask_t sx_errs;
  sx126x_get_device_errors(ctx, &sx_errs); // no BUSY
  ESP_LOG_LEVEL_LOCAL((sx_errs & 0x1FF? ESP_LOG_ERROR: ESP_LOG_INFO), TAG, 
    "device errors: cali [RC64K=%c, RC13M=%c, PLL=%c, ADC=%c IMG=%c] | XOSC start=%c | PLL lock=%c | PA ramp=%c", 
    sx_errs & SX126X_ERRORS_RC64K_CALIBRATION? 'Y':'n', sx_errs & SX126X_ERRORS_RC13M_CALIBRATION? 'Y':'n', 
    sx_errs & SX126X_ERRORS_PLL_CALIBRATION? 'Y':'n', sx_errs & SX126X_ERRORS_ADC_CALIBRATION? 'Y':'n', 
    sx_errs & SX126X_ERRORS_IMG_CALIBRATION? 'Y':'n', sx_errs & SX126X_ERRORS_XOSC_START? 'Y':'n', 
    sx_errs & SX126X_ERRORS_PLL_LOCK? 'Y':'n', sx_errs & SX126X_ERRORS_PA_RAMP? 'Y':'n'
  );
  if (sx_errs & 0xFE00) {
    ESP_LOGD(TAG, "additional RFU bit's set: %04xh", sx_errs);
  }
  return sx_errs & 0x1FF;
}


static void SX126x_print_status(uint8_t status) {
  static const char *chip_modes[8] = { 
    "0/unused", "1(RFU)", "STBY_RC", "STBY_XOSC", "FS", "RX", "TX", "7/inval" 
  };
  static const char *cmd_status[8] = { 
    "0/resvd", "1(RFU)", "data available", "cmd timeout", "cmd processing err", "fail to execute cmd", "cmd TX done", "7/inval" 
  };
  ESP_LOGI(TAG, "STATUS mode: %s | status: %s", chip_modes[(status >> 4) & 7], cmd_status[(status >> 1) & 7]);
}


void SX126x_print_statuscode_deverrs(const void *context) {
  const tsx126x_ctx *ctx = (const tsx126x_ctx *) context;
  uint8_t last_status = 0xFF;
  const uint8_t *last_status_buffer = sx126x_get_status_buffer(ctx);  
  if (last_status_buffer[1] == 0xFF) { // there is no status code (after a single byte cmd)  
    //sx126x_get_status(ctx, &struct_type); <- we got a struct back here, using hal cmd instead
    last_status = 0xC0; // get status cmd
    sx126x_hal_read(ctx, &last_status, 1, &last_status, 1); // read status
    ESP_LOGD(TAG, "got status: %02xh", last_status);
    SX126x_print_status(last_status);
  } else {
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, last_status_buffer + 1, 4, ESP_LOG_VERBOSE);
    for (int pos = 1; (pos < 16) && (last_status_buffer[pos] != 0xFF); pos++) {      
      if (last_status_buffer[pos] != last_status) {
        last_status = last_status_buffer[pos];
        SX126x_print_status(last_status);
      }
    } // rof status in change
  }
  SX126x_GetDeviceError(ctx);
}



esp_err_t C2LORA_init_sx126x(tLoraStream *lora, const tsx126x_config *cfg) {
  esp_err_t err;
  sx126x_status_t s;

  ESP_RETURN_ON_FALSE((lora != NULL) && (cfg != NULL), ESP_ERR_INVALID_ARG, TAG, "config is NULL");

  err = sx126x_init_hal(&lora->ctx, cfg); // this funbction do the device reset too...
  if (err != ESP_OK) {
    return err;
  }
  ESP_LOGD(TAG, "wakeup");
  sx126x_wakeup(lora->ctx);               // wakeup uses the got standby cmd with the long NSS=low begin for waking up the SX126x

  lora->events   = xEventGroupCreate();

  lora->intr_pin = cfg->dio_pin[0];
  lora->rxen_pin = cfg->ext_rxsw_pin;
  lora->txfb_pin = cfg->dio_pin[1];

  if (lora->intr_pin != GPIO_NUM_NC) {
    gpio_set_direction(lora->intr_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(lora->intr_pin, GPIO_PULLDOWN_ONLY);
    ESP_LOGD(TAG, "IO%d is interrupt/DIO1 pin", lora->intr_pin);    
  } else {
    ESP_LOGW(TAG, "no intr pin (DIO1) defined!");
  }
  if (lora->txfb_pin != GPIO_NUM_NC) {
    gpio_set_direction(lora->intr_pin, GPIO_MODE_INPUT);
    ESP_LOGD(TAG, "IO%d is DIO2 (tx feedback) pin", lora->txfb_pin);
  }
  if (lora->rxen_pin != GPIO_NUM_NC) {
    gpio_set_direction(lora->rxen_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(lora->rxen_pin, 0);
    ESP_LOGD(TAG, "IO%d defined for RX enable (RF switch)", lora->rxen_pin);
  }

#if SX126X_ENABLE_DCDC
  ESP_LOGD(TAG, "turn on DC-DC reculator");
  s = sx126x_set_reg_mode(lora->ctx, SX126X_REG_MODE_DCDC);
  if (s) ESP_LOGE(TAG, "no DC-DC turned on");
#endif

#ifdef SX126X_TCXO_VOLTAGE
  ESP_LOGD(TAG, "set TCXO voltage");
  s = sx126x_set_dio3_as_tcxo_ctrl(lora->ctx, SX126X_TCXO_VOLTAGE, SX126X_TCXO_STARTTIME << 6); // 5ms
  if (s) ESP_LOGE(TAG, "set TCXO voltage");  
#endif

  sx126x_clear_device_errors(lora->ctx);    // don't set BUSY

  ESP_LOGD(TAG, "set packet type");
  s = sx126x_set_pkt_type(lora->ctx, SX126X_PKT_TYPE_LORA); // ~16µs BUSY
  if (s) ESP_LOGE(TAG, "set packet type");

  ESP_LOGD(TAG, "cfg tx clamp");
  s = sx126x_cfg_tx_clamp(lora->ctx);  // optimize PA clamping threshold (workaround for antena mismatch)
  if (s) ESP_LOGE(TAG, "cfg tx clamp");

  ESP_LOGD(TAG, "set freq to %luHz", lora->frequency + lora->freq_offset);
  s = sx126x_set_rf_freq(lora->ctx, lora->frequency + lora->freq_offset);  // needs ~40µs BUSY
  if (s) ESP_LOGE(TAG, "set freq");

  C2LORA_set_tx_power(lora, lora->tx_power_dBm);

  ESP_LOGD(TAG, "set DIO2 as RF switch control");
  s = sx126x_set_dio2_as_rf_sw_ctrl(lora->ctx, true); // ~8µs BUSY
  if (s) ESP_LOGE(TAG, "set DIO2 as RF switch control");

  ESP_LOGD(TAG, "set IQ workarround");

  uint8_t sx126x_reg;
  s = sx126x_read_register(lora->ctx, SX126X_REG_IQ_POLARITY, &sx126x_reg, 1);
  if (s == SX126X_STATUS_OK) {
    ESP_LOGD(TAG, "register %03xh=%02xh", SX126X_REG_IQ_POLARITY, sx126x_reg); 
    sx126x_reg |= 0x04;                   // workarround for standard IQ polarity
    s = sx126x_write_register(lora->ctx, SX126X_REG_IQ_POLARITY, &sx126x_reg, 1);
  } // fi read
  if (s) ESP_LOGE(TAG, "set IQ workarround");

  ESP_LOGD(TAG, "set lora sync word");
  uint16_t c2lora_sync_word = C2LORA_SYNC_WORD;
  s = sx126x_write_register(lora->ctx, SX126X_REG_LR_SYNCWORD, (uint8_t *) &c2lora_sync_word, 2);
  if (s) ESP_LOGE(TAG, "set lora sync word");

  ESP_LOGD(TAG, "set lora mod params");
  s = sx126x_set_lora_mod_params(lora->ctx, &lora->def->mod); // -> these are 3 separate I/Os (hal_write, read+write register within BUSY period)
  if (s) ESP_LOGE(TAG, "set lora mod params");

  ESP_LOGD(TAG, "specific image calibration (430-440MHz)");
  //sx126x_cal(sx126x_ctx, SX126X_CAL_ALL & ~SX126X_CAL_IMAGE);
  sx126x_cal_img(lora->ctx, 0x6B, 0x6F); // 430-440MHz
  if (s) ESP_LOGE(TAG, "calibration fails");

  DEBUG_SX126X_INFOS(lora->ctx);

  lora->state = LS_IDLE;
  return ESP_OK;

  sx126x_errors_mask_t device_errs = SX126x_GetDeviceError(lora->ctx);
  lora->state = device_errs? LS_INACTIVE: LS_IDLE;
  return device_errs == 0? ESP_OK: ESP_FAIL;
}



esp_err_t C2LORA_set_standby(tLoraStream *lora, sx126x_standby_cfgs_t mode) {
  sx126x_status_t s;
  gpio_set_intr_type(lora->intr_pin, GPIO_INTR_DISABLE);
  if (lora->rxen_pin != GPIO_NUM_NC) {
    gpio_set_level(lora->rxen_pin, 0);
  } // fi
  s = sx126x_set_standby(lora->ctx, mode);
  return s? ESP_FAIL: ESP_OK;
}


esp_err_t C2LORA_set_tx_power(tLoraStream *lora, signed char pwr_dBm) {
  const int8_t  max_dBm_sel[]      = { 12, 15, 18, 21 };
  const uint8_t pa_duties_sx1268[] = { 0, 4, 2, 3, 4 };
  const uint8_t hp_maxsel_sx1268[] = { 3, 6, 3, 5, 7 };
  sx126x_pa_cfg_params_t pa_params = {    // default values for up to +14dBm (all SX126x devices)
    .device_sel    = 0x00,      // is always a SX1262 or SX1268
    .pa_lut        = 0x01
  };
  sx126x_status_t s;
  int pa_param_sel;

  ESP_RETURN_ON_FALSE(lora != NULL, ESP_ERR_INVALID_ARG, TAG, "LoraStram is NULL");  
  for (pa_param_sel = 0; pa_param_sel < 4; pa_param_sel++) {
    if (pwr_dBm <= max_dBm_sel[pa_param_sel]) break;
  } // rof
  pa_params.pa_duty_cycle = pa_duties_sx1268[pa_param_sel];
  pa_params.hp_max        = hp_maxsel_sx1268[pa_param_sel];
  if (pwr_dBm > 9) {
    pwr_dBm = pa_param_sel > 1? 22: 15;
  } else if (pwr_dBm < -9) {
    pwr_dBm = -9;
  }

  ESP_LOGD(TAG, "set PA parameter: pa_duty=%d,hp_max=%d", pa_params.pa_duty_cycle, pa_params.hp_max);
  s = sx126x_set_pa_cfg(lora->ctx, &pa_params);   // ~12µs BUSY
  if (s) ESP_LOGE(TAG, "set PA parameter");
#if (defined SX126X_PA_GAIN_DBM) && (SX126X_PA_GAIN_DBM > 0)
  ESP_LOGD(TAG, "set TX parameter: power=%ddBm + %ddB PA-gain", pwr_dBm, SX126X_PA_GAIN_DB);
#else
  ESP_LOGD(TAG, "set TX parameter: power=%ddBm", pwr_dBm);
#endif  
  s = sx126x_set_tx_params(lora->ctx, pwr_dBm, SX126X_RAMP_800_US); // ~ 8µs BUSY // max-ramp 3400µs
  if (s) ESP_LOGE(TAG, "set TX parameter");
  return s;
}


sx126x_status_t C2LORA_handle_device_errors(tLoraStream *lora) {
  sx126x_status_t      s = SX126X_STATUS_OK;
  sx126x_errors_mask_t dev_errors = SX126x_GetDeviceError(lora->ctx);
  if (dev_errors == 0) {
    return SX126X_STATUS_OK;
  }
  ESP_LOGW(TAG, "have device errors...");
  if (dev_errors & (SX126X_ERRORS_RC64K_CALIBRATION|SX126X_ERRORS_RC13M_CALIBRATION|SX126X_ERRORS_PLL_CALIBRATION|SX126X_ERRORS_ADC_CALIBRATION)) {
    ESP_LOGD(TAG, "calibrate all");
    s |= sx126x_cal(lora->ctx, SX126X_CAL_ALL);
  }
  if (dev_errors & SX126X_ERRORS_IMG_CALIBRATION) {
    ESP_LOGD(TAG, "calibrate image reject");
    s |= sx126x_cal_img(lora->ctx, 0x6B, 0x6F); // 430-440MHz
  }
  s |= sx126x_clear_device_errors(lora->ctx);
  return s==SX126X_STATUS_OK? (SX126x_GetDeviceError(lora->ctx)==0? SX126X_STATUS_OK: SX126X_STATUS_UNSUPPORTED_FEATURE): s;
}


static void IRAM_ATTR C2LORA_TXdone_gpio_handler(void *arg) {
  tLoraStream *lora = (tLoraStream *) arg;
  xEventGroupSetBitsFromISR(lora->events, C2LORA_EVENT_SX126X_INT, NULL);
  ESP_EARLY_LOGV(TAG, "TXdone irq (%d)", lora->state);
}


inline esp_err_t C2LORA_clear_irq_mask(const tLoraStream *lora) {
  return sx126x_hal_fast_cmd(lora->ctx, sx126x_clear_sta, sizeof(sx126x_clear_sta));
}


esp_err_t C2LORA_prepare_transmit(tLoraStream *lora, bool preamble_for_rt_speech) {
  sx126x_hal_status_t s;
  uint8_t start_byte = ~C2LORA_FRAME_STARTBYTE; // 0xCC for header start

  ESP_RETURN_ON_FALSE(lora != NULL, ESP_ERR_INVALID_ARG, TAG, "LoraStram is NULL");

  if (lora->state == LS_INACTIVE) {
    ESP_LOGW(TAG, "lora is not active");
    return ESP_ERR_INVALID_STATE;
  }

  s = C2LORA_handle_device_errors(lora);
  if (s) {
    ESP_LOGE(TAG, "can't transmit, device have errors!");
    return ESP_FAIL;
  }
  // update packet parameter
  lora->pkt_params[SX126X_PKT_PRE_BYTEPOS] = preamble_for_rt_speech? lora->def->firstDV_preamble: lora->def->default_preamble;
  lora->pkt_params[SX126X_PKT_LEN_BYTEPOS] = lora->def->bytes_per_header + lora->def->bytes_per_packet;
  sx126x_hal_fast_cmd(lora->ctx, lora->pkt_params, SX126X_SIZE_PKT_PARAMS);

  lora->p.min_pkt_end_time = (C2LORA_DVOICE_FRAMELEN_MS - lora->def->min_finish_ms) * 1000;
  lora->p.max_pkt_end_time = (C2LORA_DVOICE_FRAMELEN_MS - lora->def->max_finish_ms) * 1000;

  lora->p.bits_header = lora->def->bytes_per_header << 3;
  lora->p.bits_dvoice = lora->def->dv_frames_per_packet * lora->dva->bits_per_frame;
  lora->p.bits_cydata = ((lora->def->bytes_per_packet - 1) << 3) - lora->p.bits_dvoice; // minus 1 für START

  lora->state        = TX_SEND_HEADER;
  lora->frame_cnt    = 0;
  lora->wr_offset    = lora->p.bits_header + 8;
  lora->wr_cy_ofs    = 0;
  lora->pkt_framecnt = 0;
  lora->pkt_offset   = 0;

  int bytes_space  = 256 - 1 - lora->def->bytes_per_header;
  int packets_left = bytes_space / lora->def->bytes_per_packet;
  //int frame_space  = (bytes_space - (packets_left * lora->def->bytes_per_packet) - (int)(lora->p.bits_cydata >> 3)) / lora->dva->bytes_per_frame;
  int frame_space  = (bytes_space % lora->def->bytes_per_packet) / lora->dva->bytes_per_frame;
  ESP_LOGD(TAG, "%dbytes = %dp + %df", bytes_space, packets_left, frame_space);

  lora->frame_himark = packets_left * lora->def->dv_frames_per_packet + frame_space;

  C2LORA_prepare_cydata(lora->cydata, (lora->p.bits_cydata + 7) >> 3);

  sx126x_hal_wait_busy(lora->ctx);

  s |= sx126x_set_buffer_base_address(lora->ctx, lora->pkt_offset, 0);
  s |= sx126x_set_dio_irq_params(lora->ctx, SX126X_IRQ_TX_DONE, SX126X_IRQ_TX_DONE, 0x00, 0x00);
  s |= sx126x_set_rx_tx_fallback_mode(lora->ctx, SX126X_FALLBACK_FS);

  s |= sx126x_write_buffer(lora->ctx, lora->pkt_offset, &start_byte, 1);
  s |= sx126x_write_buffer(lora->ctx, lora->pkt_offset + 1, lora->header, lora->def->bytes_per_header);

  if (lora->freq_shift != 0) {
    ESP_LOGD(TAG, "set TX freq");
    s |= sx126x_set_rf_freq(lora->ctx, lora->frequency + lora->freq_shift);  // needs ~40µs BUSY
  }

  s |= sx126x_clear_irq_status(lora->ctx, SX126X_IRQ_ALL);
  if (s) {
    ESP_LOGE(TAG, "C2LORA failed to init 4 start");
    return ESP_FAIL;
  }
  ESP_LOGD(TAG, "set intr pin %d handler", lora->intr_pin);
  gpio_isr_handler_add(lora->intr_pin, C2LORA_TXdone_gpio_handler, (void*) lora);
  gpio_set_intr_type(lora->intr_pin, GPIO_INTR_POSEDGE); 
  return s? ESP_FAIL: ESP_OK;
}


inline esp_err_t C2LORA_begin_transmit(tLoraStream *lora) {
  esp_err_t err = sx126x_hal_fast_cmd(lora->ctx, sx126x_start_hdr, 4);
  if (err == ESP_OK) {
    lora->pkt_start_time = esp_timer_get_time();
    // reset packet length (updated just after TXdone IRQ+Event)
    lora->update_pkt_params = lora->pkt_params[SX126X_PKT_LEN_BYTEPOS] != lora->def->bytes_per_packet;
    lora->pkt_params[SX126X_PKT_LEN_BYTEPOS] = lora->def->bytes_per_packet;
    lora->pkt_params[SX126X_PKT_PRE_BYTEPOS] = lora->def->default_preamble;
  }
  return err;
}


esp_err_t C2LORA_finish_transmit(tLoraStream *lora) {
  sx126x_status_t s = 0;
  lora->state = LS_IDLE;
  lora->dva   = NULL;
  lora->finish_tx = NULL;
  gpio_set_intr_type(lora->intr_pin, GPIO_INTR_DISABLE);
  gpio_isr_handler_remove(lora->intr_pin);

  s = sx126x_set_rx_tx_fallback_mode(lora->ctx, SX126X_FALLBACK_STDBY_RC);
  s |= sx126x_set_dio_irq_params(lora->ctx, SX126X_IRQ_NONE, SX126X_IRQ_NONE, 0x00, 0x00);
  s |= C2LORA_handle_device_errors(lora);

  if (lora->freq_shift != 0) {
    ESP_LOGD(TAG, "set RX freq");
    s |= sx126x_set_rf_freq(lora->ctx, lora->frequency);  // needs ~40µs BUSY
  } // fi shift
  return s? ESP_FAIL: ESP_OK;
}



/*

Functions For Reciving

*/



static void IRAM_ATTR gpio_rx_handler(void *arg) {
  tLoraStream *lora = (tLoraStream *) arg;
  lora->pkt_start_time = esp_timer_get_time(); // invalid if the SX126x INT was a RXdone, doesn't matter
  xEventGroupSetBitsFromISR(lora->events, C2LORA_EVENT_SX126X_INT, NULL);
  ESP_EARLY_LOGV(TAG, "RX isr");
}


#define C2LORARX_SYMBOL_TIMEOUT 12

esp_err_t C2LORA_prepare_receive(tLoraStream *lora, uint32_t *symbol_time, uint32_t *preamble_us, uint32_t *firstdata_us, uint32_t *cydata_us) {
  sx126x_hal_status_t s;
  ESP_RETURN_ON_FALSE((lora != NULL) && (lora->dva != NULL), ESP_ERR_INVALID_ARG, TAG, "LoraStram or DVA is NULL");

  if (lora->state == LS_INACTIVE) {
    ESP_LOGW(TAG, "lora is not active");
    return ESP_ERR_INVALID_STATE;
  }
  if (lora->dva->bits_per_frame == 0) {
    ESP_LOGW(TAG, "no speech data within a frame!");
  }

  lora->p.bits_header = lora->def->bytes_per_header << 3;
  lora->p.bits_dvoice = lora->def->dv_frames_per_packet * lora->dva->bits_per_frame;
  lora->p.bits_cydata = ((lora->def->bytes_per_packet-1) << 3) - lora->p.bits_dvoice; // minus 1 für START
  // set packet params (max expected preamble plus packet length)
  lora->pkt_params[SX126X_PKT_PRE_BYTEPOS] = lora->def->default_preamble + 1;   // should be the max possible preamble
  if (lora->def->firstDV_preamble > lora->pkt_params[SX126X_PKT_PRE_BYTEPOS]) { // it the first tries there was sync problems if preamble length>22
    lora->pkt_params[SX126X_PKT_PRE_BYTEPOS] = lora->def->firstDV_preamble;
  } // fi longer preamble used
  lora->pkt_params[SX126X_PKT_LEN_BYTEPOS] = lora->def->bytes_per_header + lora->def->bytes_per_packet;
  // update packet params:
  sx126x_hal_fast_cmd(lora->ctx, lora->pkt_params, SX126X_SIZE_PKT_PARAMS);

  uint32_t symbol_time_us   = c2lora_calc_symboltime(lora->def);
  uint32_t preamble_time_us = c2lora_calc_firstprefetch(lora->def, symbol_time_us, 0, false);
  uint32_t firstdat_time_us = c2lora_calc_firstprefetch(lora->def, symbol_time_us, lora->dva->bytes_per_frame, false);
  uint32_t cyclic_data_us   = c2lora_calc_rxlength(lora->def, symbol_time_us, (lora->p.bits_cydata + 7) >> 3, 0);

  if (symbol_time) symbol_time[0] = symbol_time_us;
  if (preamble_us) preamble_us[0] = preamble_time_us;
  if (firstdata_us) firstdata_us[0] = firstdat_time_us;
  if (cydata_us) cydata_us[0] = cyclic_data_us;
  //ESP_LOGD(TAG, "sym %luµs pre %luµs first %luµs cy %luµs", symbol_time_us, preamble_time_us, firstdat_time_us, cyclic_data_us);

  if (!sbuf_create(lora->dva->buf, lora->def->dv_frames_per_packet, lora->dva->bytes_per_frame, lora->dva->samples_per_frame)) {
    ESP_LOGW(TAG, "no speech/voice buffer created!");
  } // fi create

  sx126x_hal_wait_busy(lora->ctx);    // wait for packet-parameter command to be processed

  lora->pkt_offset = 0;
  ESP_LOGD(TAG, "set rx packet offset to %d", lora->pkt_offset);
  s = sx126x_set_buffer_base_address(lora->ctx, 0, lora->pkt_offset);
  if (s) ESP_LOGE(TAG, "set rx packet offset");

  ESP_LOGD(TAG, "set dio irq params");
  s = sx126x_set_dio_irq_params(lora->ctx, SX126X_IRQ_PREAMBLE_DETECTED, C2LORARX_INTMASK, 0x00, 0x00);
  if (s) ESP_LOGE(TAG, "set dio irq params");

  ESP_LOGD(TAG, "set rx/tx fallback mode");
  s = sx126x_set_rx_tx_fallback_mode(lora->ctx, SX126X_FALLBACK_FS);
  if (s) ESP_LOGE(TAG, "set rx/tx fallback mode");

#ifdef SX126X_RX_BOOST_GAIN
  ESP_LOGD(TAG, "boost gain");
  uint8_t gain_value = SX126X_RX_BOOST_GAIN? 0x96: 0x94;
  s = sx126x_write_register(lora->ctx, SX126X_REG_RXGAIN, &gain_value, 1);
  if (s) ESP_LOGE(TAG, "no boosted gain");
#endif

  ESP_LOGD(TAG, "set rx continuous");
  s = sx126x_set_rx_with_timeout_in_rtc_step(lora->ctx, SX126X_RX_CONTINUOUS);
  if (s) ESP_LOGE(TAG, "set rx continuous");

  //sx126x_set_rx_with_timeout_in_rtc_step(lora->ctx, 0);
  //sx126x_set_lora_symb_nb_timeout(lora->ctx, C2LORARX_SYMBOL_TIMEOUT); // setting this results ONLY in non-continous mode

  ESP_LOGD(TAG, "clear irq status");
  s = sx126x_clear_irq_status(lora->ctx, C2LORARX_INTMASK);
  if (s) ESP_LOGE(TAG, "clear irq status");

  esp_err_t err = gpio_isr_handler_add(lora->intr_pin, gpio_rx_handler, (void*) lora);
  ESP_RETURN_ON_ERROR(err, TAG, "init GPIO ISR failed (%s)", esp_err_to_name(err));

  err = gpio_set_intr_type(lora->intr_pin, GPIO_INTR_POSEDGE);
  if ((lora->rxen_pin != GPIO_NUM_NC) && (err == ESP_OK)) {
    gpio_set_level(lora->rxen_pin, 1);
  }

  lora->state     = RX_NOSYNC;
  lora->frame_cnt = 0; 
  return err;
}


/*

Functions For Debugging

*/


esp_err_t C2LORA_read_registers_at(tLoraStream *lora, unsigned short addr) {
  uint8_t register_dump[32];
  const uint8_t reg_read_cmd[4] = {
    0x1D, addr >> 8, addr & 0xFF, 0x00
  };
  esp_err_t err;
  err = sx126x_hal_read(lora->ctx, reg_read_cmd, 4, register_dump, sizeof(register_dump));
  //err = sx126x_read_registerlora->ctx, addr, register_dump, sizeof(register_dump));
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "register content @%04xh:", addr);    
    ESP_LOG_BUFFER_HEX(TAG, register_dump, sizeof(register_dump));    
    printf("ASCII: ");
    for (int i=0; i < sizeof(register_dump); i++) {
      printf("%c", (register_dump[i] > 31)&&(register_dump[i] < 128)? register_dump[i]: '.');
    }
    printf("|\n\n");
  } else {
    ESP_LOGW(TAG, "read registers @%04xh fails (%s)", addr, esp_err_to_name(err));
  }
  return err;
}
