/*

This source file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.

*/

#include "c2lora_uhf.h"

#include "sdkconfig.h"

#include "hardware.h"

#include "C2LORA.h"
#include "C2LORA_modes.h"
#include "C2LORA_core.h"
#include "C2LORA_header.h"
#include "C2LORA_hamdlnk.h"


#include "sx126x.h"
#include "sx126x_hal.h"
#include "sx126x_regs.h"
#include "s_spi.h"

#include "localaudio.h"
#include "subframedef.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "hal/gpio_ll.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include <nvs_flash.h>

#include <string.h>

#define C2LORA_MAX_RXGAP_MS           40

#define C2LORA_SHIFT_REG_CNT          1  // number of 64bit buffers for misaligned data

#define C2LORA_UHF_CMD_QUEUE_SIZE     4

#define C2LORARX_TASK_STACK_SIZE      12000
#define C2LORARX_TASK_PRIORITY        10

#define C2LORA_TX_STREAM_ID           0x0D1A0CFA  // some != zero value

#if (defined SX126X_2_NSS_Pin) && (SX126X_2_NSS_Pin != GPIO_NUM_NC)
#define C2LORA_2ND_STREAM
#endif

typedef struct {
  tLoraCmdFunc    cmd_fct;
  void *          cmd_arg;
} tLoraCmdQItem;

static const char *TAG = "C2LORA";

static const char *C2LORA_namespace = "C2LORA";
static const char *C2LORA_Mode	    = "OPmode";
static const char *C2LORA_Freq      = "frequency";
static const char *C2LORA_FreqShift = "freqshift";
static const char *C2LORA_Offset    = "f_cal_hz";

#define C2LORA_TIMEOUT_480            ((C2LORA_DVOICE_FRAMELEN_MS+16)*64)

static const uint8_t sx126x_start_txd[4] = { SX126X_CMD_START_TX, 0, (uint8_t)(C2LORA_TIMEOUT_480 >> 8), (uint8_t)C2LORA_TIMEOUT_480 };


static uint8_t            lora_queue_item_buf[C2LORA_UHF_CMD_QUEUE_SIZE * sizeof(tLoraCmdQItem)];
static StaticQueue_t      lora_queue_struct_buf;

static tLoraStream        lora_stream;

static unsigned char      local_header[C2LORA_MAX_HEADER_SIZE]; // keeps a copy of the local header (for PTT on this device)

int64_t                   audio_start_us; // ToDo better


static uint8_t DMA_ATTR   rx_packet[256];



static void C2LORA_control_task(void *thread_data);




#ifdef C2LORA_2ND_STREAM

// ToDo 2-SPI Version!
#define sx126x_init_mutex()      sx_mutex = xSemaphoreCreateMutex()
#define sx126x_lock()            xSemaphoreTake(sx_mutex, portMAX_DELAY)
#define sx126x_unlock()          xSemaphoreGive(sx_mutex)
#define sx126x_delete_mutex()    vSemaphoreDelete(sx_mutex)

static const char *C2LORA_Offs_2  = "f2_cal_hz";

static uint8_t            lora_2_queue_item_buf[C2LORA_UHF_CMD_QUEUE_SIZE * sizeof(tLoraCmdQItem)];
static StaticQueue_t      lora_2_queue_struct_buf;
static tLoraStream        lora_2nd_stream;
static SemaphoreHandle_t  sx_mutex;

tLoraStream *C2LORA_get_2nd_stream(void) {
  return &lora_2nd_stream;
}

#else

#define sx126x_init_mutex()
#define sx126x_lock()
#define sx126x_unlock()
#define sx126x_delete_mutex()

tLoraStream *C2LORA_get_2nd_stream(void) {
  return NULL;
}

#endif



tLoraStream *C2LORA_get_primary_stream(void) {
  return &lora_stream;
}


tLoraStream *C2LORA_get_stream_4_transmit(void) {
#ifdef C2LORA_2ND_STREAM
  if ((lora_2nd_stream.ctx != NULL) & (lora_2nd_stream.state > LS_INACTIVE) && (lora_2nd_stream.state < RX_NOSYNC)) {
    return &lora_2nd_stream;
  }
#endif
  return &lora_stream;
}


esp_err_t C2LORA_init_donglemode(tC2LORA_mode default_mode) {

  nvs_handle hnd;
  esp_err_t err = C2LORA_prepare_streamstruct(&lora_stream, default_mode);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "init tranceiver fails");
     return err; 
  }

  tLoraStream *lora = &lora_stream;

  err = nvs_open(C2LORA_namespace, NVS_READONLY, &hnd);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "no parameter set found in NVS");    
  } else {
    uint8_t  mode_nvs;
    uint32_t freq_nvs;
    int32_t  freq_shift;
    int32_t  offset_nvs;
    const tC2LORAdef *def_nvs;
    err  = nvs_get_u8(hnd, C2LORA_Mode, &mode_nvs);
    err |= nvs_get_u32(hnd, C2LORA_Freq, &freq_nvs);
    err |= nvs_get_i32(hnd, C2LORA_FreqShift, &freq_shift);
    err |= nvs_get_i32(hnd, C2LORA_Offset, &offset_nvs);
    if ((err == ESP_OK) && (mode_nvs < C2LORA_NO_OF_MODES) && (freq_nvs >= C2LORA_MIN_FREQ_HZ) && (freq_nvs <= C2LORA_MAX_FREQ_HZ)) {
      lora->freq_offset = offset_nvs;
      lora->frequency   = freq_nvs;      
      lora->freq_shift  = freq_shift;   // ToDo range check
      def_nvs = C2LORA_get_parameter4mode(mode_nvs);
      if (def_nvs != NULL) {
        lora->def = def_nvs;
        default_mode = mode_nvs;
      } else {
        ESP_LOGW(TAG, "invalid C2LORA mode, using default.");
      }
    } else {
      ESP_LOGW(TAG, "incomplete or invalid parameter found, using defaults.");
    }
    nvs_close(hnd);
  } // else namespace defined
  ESP_LOGI(TAG, "init c2lora 4 dongle-mode done, mode: %s", C2LORA_get_mode_name(default_mode));
  return ESP_OK;
}



esp_err_t C2LORA_init_tranceiver(int spi_device_no, unsigned int default_frequency_hz, tC2LORA_mode default_mode, int freq_offset_hz) {
  const tsx126x_config cfg = {
    .spi_num    = SX126X_HOST,
    .spi_device = spi_device_no,
    .nss_pin    = SX126X_NSS_Pin,
    .reset_pin  = SX126X_RST_Pin,
    .busy_pin   = SX126X_BUSY_Pin,
    .dio_pin    = { SX126X_DIO1_Pin,
#ifdef SX126X_DIO2_Pin
    SX126X_DIO2_Pin,
#else
    GPIO_NUM_NC,
#endif
    GPIO_NUM_NC },
#ifdef SX126X_RXSWITCH_Pin
    .ext_rxsw_pin = SX126X_RXSWITCH_Pin,
#else
    .ext_rxsw_pin = GPIO_NUM_NC,
#endif
  };
  nvs_handle hnd;

  sx126x_init_mutex();

  esp_err_t err = C2LORA_prepare_streamstruct(&lora_stream, default_mode);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "init tranceiver fails");
     return err; 
  }


  tLoraStream *lora = &lora_stream;

  lora->cmd_queue   = xQueueCreateStatic(C2LORA_UHF_CMD_QUEUE_SIZE, sizeof(tLoraCmdQItem), lora_queue_item_buf, &lora_queue_struct_buf);
  lora->freq_offset = freq_offset_hz;
  lora->frequency   = default_frequency_hz;

  err = nvs_open(C2LORA_namespace, NVS_READONLY, &hnd);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "no parameter set found in NVS");    
  } else {
    uint8_t  mode_nvs;
    uint32_t freq_nvs;
    int32_t  freq_shift;
    int32_t  offset_nvs;
    const tC2LORAdef *def_nvs;
    err  = nvs_get_u8(hnd, C2LORA_Mode, &mode_nvs);
    err |= nvs_get_u32(hnd, C2LORA_Freq, &freq_nvs);
    err |= nvs_get_i32(hnd, C2LORA_FreqShift, &freq_shift);
    err |= nvs_get_i32(hnd, C2LORA_Offset, &offset_nvs);
    if ((err == ESP_OK) && (mode_nvs < C2LORA_NO_OF_MODES) && (freq_nvs >= C2LORA_MIN_FREQ_HZ) && (freq_nvs <= C2LORA_MAX_FREQ_HZ)) {
      lora->freq_offset = offset_nvs;
      lora->frequency   = freq_nvs;      
      lora->freq_shift  = freq_shift;   // ToDo range check
      def_nvs = C2LORA_get_parameter4mode(mode_nvs);
      if (def_nvs != NULL) {
        lora->def = def_nvs;
        default_mode = mode_nvs;
      } else {
        ESP_LOGW(TAG, "invalid C2LORA mode, using default.");
      }
    } else {
      ESP_LOGW(TAG, "incomplete or invalid parameter found, using defaults.");
    }
    nvs_close(hnd);
  } // else namespace defined
  
  ESP_RETURN_ON_ERROR(C2LORA_init_sx126x(&lora_stream, &cfg), TAG, "SX126x init fails");

  err = xTaskCreate(C2LORA_control_task, "C2LORA_CTL", C2LORARX_TASK_STACK_SIZE, lora, C2LORARX_TASK_PRIORITY, NULL)? ESP_OK: ESP_FAIL;

  ESP_LOGI(TAG, "init tranceiver done, tuned to %8.4fMHz, mode: %s", (double)lora->frequency / 1000000, C2LORA_get_mode_name(default_mode));
  return ESP_OK;
}


#ifdef C2LORA_2ND_STREAM
esp_err_t C2LORA_init_tranceiver_2(int spi_device_no, int freq_offset_hz) {
  const tsx126x_config cfg = {
    .spi_num    = SPI3_NUM,
    .spi_device = spi_device_no,
    .nss_pin    = SX126X_2_NSS_Pin,
    .reset_pin  = GPIO_NUM_NC,
    .busy_pin   = SX126X_2_BUSY_Pin,
    .dio_pin    = { SX126X_2_DIO1_Pin, 
#ifdef SX126X_2_DIO2_Pin
    SX126X_DIO2_Pin,
#else
    GPIO_NUM_NC,
#endif
    GPIO_NUM_NC },
    .ext_rxsw_pin = GPIO_NUM_NC,
  };
  nvs_handle hnd;


  tLoraStream *lora_prim = C2LORA_get_primary_stream();
  tLoraStream *lora      = C2LORA_get_2nd_stream();
  tC2LORA_mode c2_mode   = C2LORA_get_mode_by_parameter(lora_prim->def);

  esp_err_t err = C2LORA_prepare_streamstruct(lora, c2_mode);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "init tranceiver fails");
     return err; 
  }
  ESP_LOGD(TAG, "...init 2nd SX126x tranceiver module...");

  lora->cmd_queue   = xQueueCreateStatic(C2LORA_UHF_CMD_QUEUE_SIZE, sizeof(tLoraCmdQItem), lora_2_queue_item_buf, &lora_2_queue_struct_buf);
  lora->freq_offset = freq_offset_hz;
  lora->frequency   = lora_prim->frequency + lora_prim->freq_shift; // wen don't neet to switch frequencies
  lora->freq_shift  = 0;

  err = nvs_open(C2LORA_namespace, NVS_READONLY, &hnd);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "no parameter set found in NVS");    
  } else {
    int32_t  offset_nvs;
    err = nvs_get_i32(hnd, C2LORA_Offs_2, &offset_nvs);
    if ((err == ESP_OK)) {
      lora->freq_offset = offset_nvs;
    } else {
      ESP_LOGW(TAG, "no frequency offset (calibration value) found, using defaults.");
    }
    nvs_close(hnd);
  } // else namespace defined
  ESP_RETURN_ON_ERROR(C2LORA_init_sx126x(lora, &cfg), TAG, "2nd SX126x init fails");

  err = xTaskCreate(C2LORA_control_task, "C2LORA_CTL2", C2LORARX_TASK_STACK_SIZE, lora, C2LORARX_TASK_PRIORITY+1, NULL)? ESP_OK: ESP_FAIL;

  ESP_LOGI(TAG, "init 2nd tranceiver done, tuned to %8.4fMHz, mode: %s", (double)lora->frequency / 1000000, C2LORA_get_mode_name(c2_mode));
  return ESP_OK;
}

#else // no 2nd module

esp_err_t C2LORA_init_tranceiver_2(int spi_device_no, int freq_offset_hz) { 
  return ESP_FAIL;
}

#endif



esp_err_t C2LORA_save_config(void) {
  nvs_handle hnd;
  esp_err_t err = nvs_open(C2LORA_namespace, NVS_READWRITE, &hnd);
  ESP_RETURN_ON_FALSE(err == ESP_OK, err, TAG, "can't open NVS");
  tLoraStream *lora = C2LORA_get_primary_stream();
  err  = nvs_set_u8( hnd, C2LORA_Mode,   lora->def->mode);
  err |= nvs_set_u32(hnd, C2LORA_Freq,   lora->frequency);
  err |= nvs_set_i32(hnd, C2LORA_FreqShift, lora->freq_shift);
  err |= nvs_set_i32(hnd, C2LORA_Offset, lora->freq_offset);
#ifdef C2LORA_2ND_STREAM
  err |= nvs_set_i32(hnd, C2LORA_Offs_2, C2LORA_get_2nd_stream()->freq_offset);
#endif
  if (err) {
    ESP_LOGD(TAG, "config save: have errors (%s)", esp_err_to_name(err));  
  } else {
    ESP_LOGD(TAG, "config saved.");
  }
  nvs_close(hnd);
  return err;
}



esp_err_t C2LORA_task_run_cmd(tLoraStream *lora, tLoraCmdFunc cmd_funct, void *cmd_arg, bool no_wait) {
  uint32_t notify_value;
  bool notify_task_event;
  tLoraCmdQItem q_item = {
    .cmd_fct = cmd_funct,
    .cmd_arg = cmd_arg
  };
  ESP_RETURN_ON_FALSE((lora != NULL) && (lora->cmd_queue != NULL), ESP_ERR_INVALID_ARG, TAG, "LoraStream is NULL");
  lora->task_to_notify = no_wait? NULL: xTaskGetCurrentTaskHandle();  
  notify_task_event = lora->state >= TX_SEND_HEADER;
  if (lora->task_to_notify) {
    xTaskNotifyStateClear(lora->task_to_notify);
  }
  if (!xQueueSendToBack(lora->cmd_queue, &q_item, no_wait? 0: pdMS_TO_TICKS(50))) {
    ESP_LOGE(TAG, "cmd queue is full");
    return ESP_ERR_NO_MEM;
  }
  if (notify_task_event) {
    xEventGroupSetBits(lora->events, C2LORA_EVENT_RECONFIGURE);
  } // fi 
  if (no_wait) return ESP_OK;
  if (xTaskNotifyWait(-1, -1, &notify_value, pdMS_TO_TICKS(500))) {
    ESP_LOGD(TAG, "cmd executed, result %s", esp_err_to_name(notify_value));
    return (esp_err_t) notify_value;
  } // fi got response
  return ESP_ERR_TIMEOUT;
}




inline unsigned int C2LORA_get_frequency(void) {
  return lora_stream.frequency;
}

inline signed int C2LORA_get_freqshift(void) {
  return lora_stream.freq_shift;
}


static uint32_t c2lora_update_frequency(tLoraStream *lora, void * arg) {
  sx126x_status_t s;
  lora->frequency = (uint32_t) arg;
  sx126x_lock();
  ESP_LOGD(TAG, "set freq");
  s = sx126x_set_rf_freq(lora->ctx, lora->frequency + lora->freq_offset);  // needs ~40µs BUSY
  if (s) {
    ESP_LOGE(TAG, "set freq");
  }
  DEBUG_SX126X_INFOS(lora->ctx);
  sx126x_unlock();
  return s? ESP_FAIL: ESP_OK;
}


esp_err_t C2LORA_set_frequency(unsigned int frequency_hz, signed int tx_shift_hz) {
  esp_err_t err;
  tLoraStream *lora = C2LORA_get_primary_stream();
  ESP_RETURN_ON_FALSE(lora != NULL, ESP_ERR_INVALID_STATE, TAG, "no primary stream defined");
  ESP_RETURN_ON_FALSE((lora->state == LS_IDLE)||(lora->state==RX_NOSYNC), ESP_ERR_INVALID_STATE, TAG, "denied, is not idle");

  lora->frequency  = frequency_hz;
  lora->freq_shift = tx_shift_hz;
  err = C2LORA_task_run_cmd(lora, c2lora_update_frequency, (void *)frequency_hz, false);
#ifdef C2LORA_2ND_STREAM
  lora = C2LORA_get_2nd_stream();
  lora->frequency = frequency_hz + tx_shift_hz;
  err = C2LORA_task_run_cmd(lora, c2lora_update_frequency, (void *)frequency_hz, false);
#endif
  return err;
}


inline tC2LORA_mode C2LORA_get_mode(void) {
  return lora_stream.def->mode;
}

unsigned char C2LORA_get_codec_type(void) {
  if (lora_stream.def == NULL) return 255;
  return lora_stream.def->codec_type;
}


static uint32_t c2lora_set_c2mode(tLoraStream *lora, void * arg) {
  sx126x_status_t s;
  ESP_LOGD(TAG, "set lora mod params");
  lora->def = (const tC2LORAdef *) arg;
  sx126x_lock();
  s = sx126x_set_lora_mod_params(lora->ctx, &lora->def->mod); // -> these are 3 separate I/Os (hal_write, read+write register within BUSY period)
  sx126x_unlock();
  if (s) ESP_LOGE(TAG, "set lora mod params");        
  return s? ESP_FAIL: ESP_OK;
}


esp_err_t C2LORA_set_mode(tC2LORA_mode mode) {
  esp_err_t err = ESP_OK;;
  tLoraStream *lora = C2LORA_get_primary_stream();
  ESP_RETURN_ON_FALSE((lora->state == LS_IDLE) || (lora->state == RX_NOSYNC), ESP_ERR_INVALID_STATE, TAG, "C2LORA is not idle");  
  const tC2LORAdef *new_def = C2LORA_get_parameter4mode(mode);
  if (new_def == NULL) {
    ESP_LOGE(TAG, "no such mode (%d)", mode);
    return ESP_FAIL;
  }
  if (lora->def != new_def) {
    tC2LORA_mode old_mode = lora->def->mode;
    err = C2LORA_task_run_cmd(lora, c2lora_set_c2mode, (void *) new_def, false);
#ifdef C2LORA_2ND_STREAM
    lora = C2LORA_get_2nd_stream();
    err = C2LORA_task_run_cmd(lora, c2lora_set_c2mode, (void *) new_def, false);
#endif
    C2LORA_update_header(local_header, old_mode, mode);
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "new mode: %s", C2LORA_get_mode_name(mode));
    } else {
      ESP_LOGW(TAG, "mode not correctly set.");
    }
  } else {
    ESP_LOGI(TAG, "still the same mode: %s", C2LORA_get_mode_name(mode));
  }
  return err;
}


esp_err_t C2LORA_set_channel(unsigned int frequency_hz, tC2LORA_mode mode) {
  tLoraStream *lora = C2LORA_get_primary_stream();
  ESP_RETURN_ON_FALSE((lora->state == LS_IDLE) || (lora->state == RX_NOSYNC), ESP_ERR_INVALID_STATE, TAG, "C2LORA is not idle");

  C2LORA_set_frequency(frequency_hz, lora->freq_shift);
  C2LORA_set_mode(mode);
  return ESP_OK;
}


esp_err_t C2LORA_set_txpower(signed char power_dBm) {
  tLoraStream *lora = C2LORA_get_stream_4_transmit();
  return C2LORA_set_tx_power(lora, power_dBm);
}


esp_err_t C2LORA_set_local_header(const char *callsign, const char *destination, tKindOfSender kos) {
  ESP_RETURN_ON_FALSE((callsign != NULL) && (destination!=NULL), ESP_ERR_INVALID_ARG, TAG, "NULL arguments");
  return C2LORA_upd_header(local_header, callsign, 7, destination, 7, kos, false);
}


esp_err_t C2LORA_set_local_header_additionals(unsigned short area_code, const char *locator) {
   return C2LORA_add_to_header(local_header, area_code, locator);
}



static uint32_t c2lora_enable_calibration(tLoraStream *lora, void * arg) {
  sx126x_status_t s;
  sx126x_lock();
  if ((bool) arg) {
    s = sx126x_set_tx_cw(lora->ctx);
  } else {
    s = sx126x_set_standby(lora->ctx, SX126X_STANDBY_CFG_RC);
  }
  sx126x_unlock();
  if (s == SX126X_STATUS_OK) lora->state = (bool) arg? LS_CALIBRATE: LS_IDLE;
  if ((bool) arg) {
    ESP_LOG_LEVEL_LOCAL((s? ESP_LOG_ERROR: ESP_LOG_INFO), TAG, "set lora continuous wave %s", s? "fails": "on");
  } else if (s) {
    ESP_LOGE(TAG, "disable continuous wave fails");
  }
  return s;
}



esp_err_t C2LORA_set_calibration(bool active, bool rx_module) {
  tLoraStream *lora = rx_module? C2LORA_get_primary_stream(): C2LORA_get_stream_4_transmit();
  // not strictly necessary, but for calibration its better to not interrupt other activities (RX, TX)...
  ESP_RETURN_ON_FALSE((lora->state == LS_IDLE)||(lora->state == LS_CALIBRATE), ESP_ERR_INVALID_STATE, TAG, "C2LORA is not idle");
  esp_err_t result = C2LORA_task_run_cmd(lora, c2lora_enable_calibration, (void *) active, false);
  if (active) {
    if (result != ESP_OK) {
      printf("continuous wave not active!\n");
    } else {
      printf("transmitting continuous wave, actual frequency offset is %+ldHz\n" , lora->freq_offset);
    }
  } else {
    printf("continuous disabled. standby.\n");
  }
  return result;
}


esp_err_t C2LORA_set_freq_offset(int freq_offset_hz) {
  ESP_RETURN_ON_FALSE((freq_offset_hz > -1000000) && (freq_offset_hz < 1000000), ESP_ERR_INVALID_ARG, TAG, "frequency offset is to high");
  tLoraStream *lora = C2LORA_get_primary_stream();
  lora->freq_offset = freq_offset_hz;
  return C2LORA_task_run_cmd(lora, c2lora_update_frequency, (void *) lora->frequency, false);
}


esp_err_t C2LORA_get_local_header(char *callsign, char *destination, tKindOfSender *kos) {
  ESP_RETURN_ON_FALSE((callsign != NULL) && (destination != NULL), ESP_ERR_INVALID_ARG, TAG, "callsign and destination must be defined");
  if (C2LORA_check_header(local_header, lora_stream.def->bytes_per_header)) {
    tKindOfSender kos_int = C2LORA_decode_header(local_header, callsign, destination, NULL);
    if (kos != NULL) kos[0] = kos_int;
    return ESP_OK;
  }
  return ESP_FAIL;
}

esp_err_t C2LORA_get_local_header_additionals(unsigned short *area_code, char *locator) {
  ESP_RETURN_ON_FALSE((area_code != NULL) && (locator != NULL), ESP_ERR_INVALID_ARG, TAG, "area_code ptr and locator must be defined");
  return C2LORA_get_from_header(lora_stream.def->mode, local_header, area_code, locator, true);
}


uint32_t c2lora_start_countinuous_rx(tLoraStream *lora, void *arg) {
  bool enable = (bool) arg;
  lora->state = enable? RX_NOSYNC: LS_IDLE;  // if this state is set, the control task enters a receiving section.
  // task leaves rx by C2LORA_EVENT_xxx and a state is set outside RX_xxx states.
  return 0;
}


esp_err_t C2LORA_start_continuous_rx(bool enable) {
  tLoraStream *lora = C2LORA_get_primary_stream();
  if (enable) {
    xEventGroupSetBits(lora->events, C2LORA_EVENT_RECEIVE_ACTI);
  } else {
    xEventGroupClearBits(lora->events, C2LORA_EVENT_RECEIVE_ACTI);
  }
  esp_err_t err = C2LORA_task_run_cmd(lora, c2lora_start_countinuous_rx, (void *) enable, false);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "reciver %s active: %s", enable? "not": "still", esp_err_to_name(err));
  }
  return err;
}



#if CONFIG_LOCAL_PTT_ENABLED

static uint32_t c2lora_finish_audio_tx(tLoraStream *lora, void *arg) {
  if (lora == NULL) {
    ESP_LOGE(TAG, "no argument for finisher call");
    return ESP_ERR_INVALID_ARG;
  }
  if (lora->dva == NULL) {
    ESP_LOGD(TAG, "no audio stream connected");
  } else {
    localaudio_end_stream(lora->dva, true);
  }
  // Bug: condition if end_stream is very slow... esp. in Modes with low or none cydata!
  ESP_LOGD(TAG, "wait TXdone...");
  for (int max_delay = 5; max_delay > 0; max_delay--) {
    vTaskDelay(3);
    if (lora->dva->streamid == -1) break;
  } // rof
  ESP_LOGD(TAG, "TXdone.");
  return (uint32_t) ESP_OK;
}


uint32_t c2lora_start_tx_microphone(tLoraStream *lora, void *arg) {
  int frame_len_ms;  
  tdvstream *new_dva;
  // ToDo cydata
  if (lora->state >= RX_NOSYNC) {
    ESP_LOGW(TAG, "C2LORA is in receiving mode...");
    return ESP_ERR_INVALID_STATE; // is in use
  }
  if (lora->dva != NULL) {
    ESP_LOGW(TAG, "a input stream is already in use for C2LORA");
    return ESP_ERR_INVALID_STATE; // is in use
  }
  new_dva = (tdvstream *) malloc(sizeof(tdvstream));
  if (new_dva == NULL) {
    ESP_LOGE(TAG, "no memory for stream");
    return ESP_ERR_NO_MEM;
  }

  memset(new_dva, 0, sizeof(tdvstream));
  new_dva->streamid   = -1;    
  new_dva->codec_type = 255;

  frame_len_ms = C2LORA_DVOICE_FRAMELEN_MS / lora->def->dv_frames_per_packet;

  if (isSF_CODEC2(lora->def->codec_type)) {
    int codec2_mode = SF_CODEC2_get_mode(lora->def->codec_type);
    localaudio_create_codec2_inputstream(new_dva, codec2_mode, frame_len_ms, C2LORA_handle_encoded, lora);
    audio_start_us = esp_timer_get_time();
  } else {
    ESP_LOGE(TAG, "config error: no valid codec2 selected");
    free(new_dva);
    return ESP_ERR_INVALID_ARG;
  }
  if (new_dva->streamid < 0) {
    ESP_LOGE(TAG, "create_audiostream: localaudio denied");
    free(new_dva);
    return ESP_FAIL;
  }
  new_dva->codec_type = lora->def->codec_type;
  if (frame_len_ms != (U32)new_dva->samples_per_frame * 1000 / new_dva->srate) {
    ESP_LOGW(TAG, "frame size mismatch. garbled output!");
  } // fi something is wrong!

  lora->dva = new_dva;
  lora->finish_tx = c2lora_finish_audio_tx;

  memcpy(lora->header, local_header, lora_stream.def->bytes_per_header);

  sx126x_lock();
  esp_err_t err = C2LORA_prepare_transmit(lora, true);
  sx126x_unlock();

#if CONFIG_USE_TEST_BITPATTERN
  uint8_t trx_packet[256];
  for (int i=0; i < 256; i++) {
    trx_packet[i] = 0x10 + i;
  }
  sx126x_write_buffer(lora->ctx, lora->pkt_offset + 1 + lora->def->bytes_per_header, trx_packet, sizeof(trx_packet) - lora->def->bytes_per_header - 1);
#endif

  vTaskDelay(pdMS_TO_TICKS(c2lora_calc_txdelay_4_encoded(lora->def, esp_timer_get_time() - audio_start_us)));
  return (uint32_t) err;
}



esp_err_t C2LORA_start_tx_microphone(void) {
  tLoraStream *lora = C2LORA_get_stream_4_transmit();
  return C2LORA_task_run_cmd(lora, c2lora_start_tx_microphone, NULL, false);
}


esp_err_t C2LORA_end_tx_microphone(void) {
  tLoraStream *lora = C2LORA_get_stream_4_transmit();
  if (lora->dva == NULL) {
    ESP_LOGW(TAG, "C2LORA not in use.");
    return ESP_OK;
  }
  if ((lora->state < TX_SEND_HEADER) || (lora->state > TX_FRAME_GAP)) {
    ESP_LOGW(TAG, "C2LORA not transmitting");
    return ESP_OK;
  }
  ESP_LOGD(TAG, "ending audio transmission...");
  xEventGroupSetBits(lora->events, C2LORA_EVENT_TRANSMIT_FINI);
  return ESP_OK;
}

#endif  // local PTT 



esp_err_t C2LORA_start_rx_speaker(void) {
  return ESP_OK;
}


esp_err_t C2LORA_end_rx_speaker(void) {
  return ESP_OK;
}


inline void C2LORA_print_status(void) {
  static const char *state_names[] = {
    "INACTIVE", "IDLE",
    "SEND_HEADER", "SEND_FRAME", "FRAME_DONE", "FRAME_GAP", "LAST_FRAME",
    "RX:NOSYNC", "RX:PREAMBLE", "RX:FIRST_FRAME", "RECIVING", "RX:DONE"
  };
  if (lora_stream.state <= RX_DONE) {
    printf("C2LORA state: %s.\n", state_names[lora_stream.state]);
  } else {
    printf("unknown / invalid state (%d).\n", lora_stream.state);
  }
  SX126x_print_statuscode_deverrs(lora_stream.ctx);
}


#if CONFIG_USE_TEST_BITPATTERN
static uint8_t test_frame_4_tx[64];

static uint8_t *C2LORA_get_testpattern_frame(int pkt_no, uint8_t frame_cnt, uint8_t frame_bytes) {
  uint8_t patter_value = (pkt_no << 4) + frame_cnt;
  memset(test_frame_4_tx, patter_value, frame_bytes);
  test_frame_4_tx[frame_bytes - 1] = frame_cnt << 4;
  return test_frame_4_tx;
}

static void C2LORA_log_testpattern_from_buffer(tdvstream *dva) {
  U32 unsend_enc_bytes = 0;
  const uint8_t *frame;
  do {
    frame = sbuf_get_unsend_block(dva->buf, &unsend_enc_bytes);
    if (unsend_enc_bytes == 0) continue;
    ESP_LOG_BUFFER_HEX(TAG, frame, unsend_enc_bytes);
    sbuf_unsend_block_processed(dva->buf, unsend_enc_bytes);
  } while (unsend_enc_bytes > 0);
}

#else

static void C2LORA_handle_rx_cydata(tLoraStream *lora, const uint8_t *cydata) {

  ESP_LOGD(TAG, "cyclic data (%dbytes):", (lora->p.bits_cydata + 7) >> 3);
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, cydata, (lora->p.bits_cydata + 7) >> 3, ESP_LOG_DEBUG);

}

#endif



static inline bool C2LORA_adjust_preamble(tLoraStream *lora, int32_t done_after_start_us) {
  if (lora->pkt_params[SX126X_PKT_LEN_BYTEPOS] != lora->def->bytes_per_packet) {    // longer / header packet?
    lora->pkt_params[SX126X_PKT_LEN_BYTEPOS] = lora->def->bytes_per_packet;
    lora->pkt_params[SX126X_PKT_PRE_BYTEPOS] = lora->def->default_preamble;
    lora->update_pkt_params = true;
  } else {
    uint8_t corrected_preabmle = lora->def->default_preamble;
    // Todo more filter jitter
    if (done_after_start_us < lora->p.min_pkt_end_time) {
      corrected_preabmle--;
    } else if (done_after_start_us > lora->p.max_pkt_end_time) {
      corrected_preabmle++;
    }
    lora->update_pkt_params = lora->pkt_params[SX126X_PKT_PRE_BYTEPOS] != corrected_preabmle;
    lora->pkt_params[SX126X_PKT_PRE_BYTEPOS] = corrected_preabmle;
  }
  ESP_LOGD(TAG, "TX #%02lx done after %6ldµs, p=%d, ofs=%02xh", lora->frame_cnt / lora->def->dv_frames_per_packet, done_after_start_us, lora->pkt_params[SX126X_PKT_PRE_BYTEPOS], lora->pkt_offset);
  return lora->update_pkt_params;
}



void C2LORA_handle_encoded(tdvstream *dva, void *userdata) {
  Union64 frame_shifted[C2LORA_SHIFT_REG_CNT];
  uint8_t last_frame_buffer[((SF_MAX_BITS_PER_FRAME+15) >> 3) + C2LORA_MAX_CYDATA_LENGTH];    //s build the last frame witdh cydata and startbyte for a single SPI buffer write
  tLoraStream *lora   = (tLoraStream *) userdata; //&lora_stream;
  tLoraTxState state  = lora->state; // local copy because GPIO ISR can alter this
  uint8_t frame_bytes = dva->bytes_per_frame;
  uint8_t frame_bits  = dva->bits_per_frame;
  uint8_t bit_shift;  
  bool first_pktframe;
  bool tx_enabled = true;
  int32_t done_after_start_us = -1; // we got an packet full excact after this duration (from start)
  
  esp_err_t err = ESP_OK;

  U32 unsend_enc_bytes = 0;
  const uint8_t *frame = sbuf_get_unsend_block(dva->buf, &unsend_enc_bytes);  
  const uint8_t *frame_ptr = frame;

  U16 frame_cnt = unsend_enc_bytes / dva->bytes_per_frame;

  ESP_LOGV(TAG, "%lu / %lu (+ %uf)", lora->frame_cnt, lora->frame_himark, frame_cnt);

  if ((lora->frame_himark - lora->frame_cnt) < frame_cnt) {
    frame_cnt = lora->frame_himark - lora->frame_cnt;
    ESP_LOGW(TAG, "SX126x buffer runs full");
  }

  if (frame_cnt == 0) {    
    ESP_LOGW(TAG, "empty encoded data"); 
    return;
  }

  if (frame_bytes > sizeof(frame_shifted)) {
    frame_bytes = sizeof(frame_shifted);
    ESP_LOGW(TAG, "odd frame_len (to big, %dbits)", frame_bits);
  } // fi

  if (state == TX_FRAME_GAP) {    // we not sending because no buffered digital voice was present at pkt end
    sx126x_lock();
    C2LORA_clear_irq_mask(lora);
    tx_enabled = C2LORA_begin_transmit(lora) == ESP_OK;
    sx126x_unlock();
    ESP_LOGD(TAG, "TX gap");
  } // fi gap

  first_pktframe = lora->pkt_framecnt == 0;
  if ((state != TX_LAST_FRAME) && tx_enabled) lora->state = TX_SEND_FRAME; // set new state (we are sending speech frames now)
  if (first_pktframe) {
    ESP_LOGD(TAG, "TX #%02lx encoded data after %ldµs", lora->frame_cnt / lora->def->dv_frames_per_packet, (int32_t)(esp_timer_get_time() - lora->pkt_start_time));
  } // fi

  for (int frame_no = frame_cnt; frame_no > 0; frame_no--, frame_ptr += dva->bytes_per_frame) {
    const uint8_t *frame_ptr_i = frame_ptr;
    uint8_t frame_pkt_bytes_i  = frame_bytes;
    bit_shift = lora->wr_offset & 7;
#if CONFIG_USE_TEST_BITPATTERN
    frame_ptr_i = C2LORA_get_testpattern_frame(lora->frame_cnt / lora->def->dv_frames_per_packet, lora->pkt_framecnt, frame_bytes);
#endif
    if (bit_shift) { 
      uint8_t bit_mask = 0xFF << (8 - bit_shift);
      memcpy(frame_shifted->u8, frame_ptr_i, frame_pkt_bytes_i);
#if C2LORA_SHIFT_REG_CNT > 1
      uint8_t upr_byte = frame_shifted->u8[7];
#endif
      frame_shifted[0].u64 = __builtin_bswap64(frame_shifted[0].u64) >> bit_shift;
      frame_shifted[0].u8[7] |= lora->wr_incomplete_byte & bit_mask;
      frame_shifted[0].u64 = __builtin_bswap64(frame_shifted[0].u64);
#if C2LORA_SHIFT_REG_CNT > 1
      if (dva->bytes_per_frame > 8) {
        frame_shifted[1].u64 = __bswap64(frame_shifted[1].u64) >> bit_shift;
        frame_shifted[1].u8[7] |= upr_byte & bit_mask;
        frame_shifted[1].u64 = __bswap64(frame_shifted[1].u64);
      } // fi using more ttan 64bits...
#endif
      frame_ptr_i = frame_shifted->u8;
    } // fi frame is bit-shifted

    lora->pkt_framecnt++;
    lora->wr_incomplete_byte = frame_ptr_i[frame_bits >> 3];
    // test frame ist the last one
    if (lora->pkt_framecnt == lora->def->dv_frames_per_packet) {  // we got the last DV-frame, build big packet
      memcpy(last_frame_buffer, frame_ptr_i, frame_pkt_bytes_i);
      if (lora->p.bits_cydata > 7) {
        memcpy(last_frame_buffer + frame_pkt_bytes_i, lora->cydata, (lora->p.bits_cydata >> 3)); //sizeof(last_frame_buffer);
        frame_pkt_bytes_i += lora->p.bits_cydata >> 3;
      }
      last_frame_buffer[frame_pkt_bytes_i] = C2LORA_FRAME_STARTBYTE;
      frame_pkt_bytes_i++;
      frame_ptr_i = last_frame_buffer;
    } // fi last frame

    sx126x_lock();
    err = sx126x_hal_fast_bufferwrite(lora->ctx, lora->pkt_offset + (lora->wr_offset >> 3), frame_ptr_i, frame_pkt_bytes_i);
    sx126x_unlock();
    lora->wr_offset += frame_bits;

    // test packet is now complete
    if (lora->pkt_framecnt >= lora->def->dv_frames_per_packet) {    // a packet is completed written with speech data
      if (done_after_start_us == -1) {
        done_after_start_us = esp_timer_get_time() - lora->pkt_start_time;
      } // fi set once
      first_pktframe    |= frame_no > 1; // there are more frames
      lora->pkt_offset  += lora->def->bytes_per_packet;
      if (lora->wr_offset > (lora->p.bits_dvoice + 8)) { // this packet starts with a header
        lora->pkt_offset += lora->def->bytes_per_header;
      } // fi last frame has a header
      lora->wr_offset    = 8;
      lora->pkt_framecnt = 0;
      //sx126x_read_buffer(lora->ctx, 0, tx_packet, 64);
      //ESP_LOG_BUFFER_HEX_LEVEL(TAG, tx_packet, p_len, ESP_LOG_DEBUG);
    } // fi 
#if CONFIG_USE_TEST_BITPATTERN
    lora->frame_cnt++;
#endif
  } // rof multiple frames

  if (lora->rec != NULL) {
    record_append_dvframe(lora->rec, frame, frame_cnt);
  }
  sbuf_unsend_block_processed(dva->buf, frame_cnt * dva->bytes_per_frame);
#ifndef CONFIG_USE_TEST_BITPATTERN
  lora->frame_cnt += frame_cnt;
#endif

  if (done_after_start_us != -1) {
    C2LORA_adjust_preamble(lora, done_after_start_us); // todo get returned value
    if (done_after_start_us > ((C2LORA_WHEADER_FRAMELEN_MS+24) * 1000L )) {
      lora->state = TX_LAST_FRAME;
      ESP_LOGW(TAG, "TX timeout!");
    }
  } // fi set once

  if (err) {
    ESP_LOGE(TAG, "can't write to SX126X buffer");
  }

  if (first_pktframe) {

    ESP_LOGD(TAG, "TX first %lldµs", esp_timer_get_time() - lora->pkt_start_time);
   
  } // fi first frame processed

  if (lora->pkt_framecnt >= lora->def->dv_frames_per_packet) {    // we ending exact after a completed packet
    if ((state >= TX_SEND_HEADER) && (state != TX_LAST_FRAME)) {
      lora->state = TX_FRAME_DONE;
    } else {
      ESP_LOGD(TAG, "TX finished");
    }
  }

} // end handle encoded speech data




static bool C2LORA_add_frame_2_stream(tdvstream *dva, const uint8_t *frame, uint8_t bit_shift, trtp_data *rtp) {
  Union64 frame_shifted[C2LORA_SHIFT_REG_CNT];
  const uint8_t *frame_ptr = frame;
  if ((dva == NULL) || (dva->buf == NULL)) return false;

  if (bit_shift > 0) {  
    uint32_t frame_size = dva->bytes_per_frame;
    if (frame_size > sizeof(frame_shifted)) {
      ESP_LOGW(TAG, "odd frame_len (to big, %lubytes)", frame_size);
      frame_size = sizeof(frame_shifted); 
    }
    memcpy(frame_shifted->u8, frame, frame_size);
#if C2LORA_SHIFT_REG_CNT > 1
    uint8_t bit_mask = (0x01 << bit_shift) - 1;
    uint8_t lwr_byte = frame_shifted[1].u8[0];
    if (dva->bytes_per_frame > 8) {
      frame_shifted[1].u64 = __bswap64(frame_shifted[1].u64) << bit_shift;      
      frame_shifted[1].u64 = __bswap64(frame_shifted[1].u64);
    }
#endif
    frame_shifted[0].u64 = __builtin_bswap64(frame_shifted[0].u64) << bit_shift;
#if C2LORA_SHIFT_REG_CNT > 1
    frame_shifted[1].u8[0] |= lwr_byte & bit_mask;
#endif
    frame_shifted[0].u64 = __builtin_bswap64(frame_shifted[0].u64);
    frame_ptr = frame_shifted->u8;
  } // fi

  sbuf_fillnext(dva->buf, frame_ptr, 1);
#if CONFIG_USE_TEST_BITPATTERN
  return false;
#endif
  return rtp != NULL? UTX_handle_encoded_dvframe(frame_ptr, rtp): false;
}



static void c2lora_rx_finisher(tdvstream *dva) {
  ESP_LOGD(TAG, "RX DVstream #%d ends now", dva->streamid);
  dva->streamid = -1;
}


static esp_err_t c2lora_create_audioout_stream(tdvstream *dva) {            
  ESP_RETURN_ON_FALSE(dva != NULL, ESP_ERR_INVALID_ARG, TAG, "can't create audio-out: DVstream is null");
  int codec2_mode = SF_CODEC2_get_mode(dva->codec_type);
  localaudio_create_codec2_outputstream(dva, codec2_mode, dva->bytes_per_frame, (tfinished_stream_fct)c2lora_rx_finisher);
  if (dva->streamid >= 0) {
      sbuf_update_time(dva->buf, xTaskGetTickCount());
      sbuf_set_steppos(dva->buf, 0);
  } else {
    //sx126x_set_dio_irq_params(lora->ctx, SX126X_IRQ_PREAMBLE_DETECTED, C2LORARX_INTMASK, 0x00, 0x00);
    ESP_LOGE(TAG, "create_audiostream: localaudio denied");    
  }
  return dva->streamid < 0? ESP_FAIL: ESP_OK;
}





static inline void c2lora_calc_no_frames_fit(tLoraStream *lora) {
  uint32_t new_highmark;
  int packets_left = 256 / lora->def->bytes_per_packet;
  int bytes_left   = 256 % lora->def->bytes_per_packet;
  int frame_space  = (bytes_left - (lora->p.bits_cydata >> 3)) / lora->dva->bytes_per_frame;
  new_highmark = packets_left * lora->def->dv_frames_per_packet + frame_space;
  if (lora->frame_himark < new_highmark) {
    lora->frame_himark = new_highmark;  
  }
}


static inline esp_err_t c2lora_transmit_funct(tLoraStream *lora) {
  esp_err_t  err;
  int64_t    start_time;
  int32_t    done_after_start_us;  
  TickType_t wait_packet_timeout = pdMS_TO_TICKS(C2LORA_WHEADER_FRAMELEN_MS+30);  // first timeout is 4 header...

  uint8_t    sx126x_set_baseadr[3] = { SX126X_CMD_SET_BASEADR, 0, 0 };
  uint32_t   packet_transmitted = 0;

  sx126x_lock();
  err = C2LORA_begin_transmit(lora);
  sx126x_unlock();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "can't enable transmitter (%s)", esp_err_to_name(err));
    return err;
  }
  ESP_LOGI(TAG, "C2LORA TX start, %ubits header, %ubits of %dtotal, %ubits cyclic data", lora->p.bits_header, lora->p.bits_dvoice, lora->def->bytes_per_packet << 3, lora->p.bits_cydata);

  do {  // loop until the TX_LAST_FRAME

    EventBits_t tx_bits = xEventGroupWaitBits(lora->events, C2LORA_ALL_EVENTS, pdTRUE, pdFALSE, wait_packet_timeout);

    ESP_LOGV(TAG, "tx event bits: %06lxh", tx_bits);

    if (tx_bits & C2LORA_EVENT_TRANSMIT_FINI) { // ending transmission request...
      lora->state = TX_LAST_FRAME;
    }

    if (tx_bits & C2LORA_EVENT_SX126X_INT) {
      sx126x_set_baseadr[1] = lora->pkt_offset; // update to a new base adress (the next packet is written to)
      sx126x_lock();
      C2LORA_clear_irq_mask(lora);
      sx126x_hal_fast_cmd(lora->ctx, sx126x_set_baseadr, 3);

      if (lora->update_pkt_params) {    // update packet parameters esp. packet-lenghth or preamble-count
        lora->update_pkt_params = sx126x_hal_fast_cmd(lora->ctx, lora->pkt_params, SX126X_SIZE_PKT_PARAMS) != ESP_OK;   
      } // fi
      switch (lora->state) {
      case TX_SEND_FRAME:
        sx126x_hal_fast_cmd(lora->ctx, sx126x_start_txd, sizeof(sx126x_start_txd));
        break;
      case TX_FRAME_DONE:    // fi continue with next packet transmission
        lora->state = TX_FRAME_GAP;
        break;
      case TX_LAST_FRAME:
        break;
      default:
        break;
      } // hctiws
      start_time = esp_timer_get_time();
      done_after_start_us  = start_time - lora->pkt_start_time;
      lora->pkt_start_time = start_time;    
      sx126x_hal_wait_busy(lora->ctx);
      sx126x_unlock();

      if (packet_transmitted == 0) {
        c2lora_calc_no_frames_fit(lora); // recalculate no of frams that will fit within the SX126x buffer
        wait_packet_timeout = pdMS_TO_TICKS(C2LORA_DVOICE_FRAMELEN_MS+20);
      } // fi the very-first packet
      packet_transmitted++;

      lora->frame_himark += lora->def->dv_frames_per_packet;
      if ((lora->dva != NULL) && (sbuf_get_unsend_size(lora->dva->buf) >= lora->dva->bytes_per_frame)) {
        C2LORA_handle_encoded(lora->dva, lora);      
      }
      ESP_LOGW(TAG, "TXdone event after %luµs", done_after_start_us);
    }

    if (tx_bits & (C2LORA_EVENT_RECONFIGURE|C2LORA_EVENT_TERMINATE)) {
      lora->state = TX_LAST_FRAME;
      err = ESP_OK;
    }

    if ((tx_bits & C2LORA_ALL_EVENTS) == 0) {      
      ESP_LOGW(TAG, "TX timeout, INTR pin = %d", gpio_get_level(lora->intr_pin));
      lora->state = TX_LAST_FRAME;
      err = ESP_ERR_TIMEOUT;
    }

  } while (lora->state != TX_LAST_FRAME);

  if (lora->finish_tx != NULL) {
    err = lora->finish_tx(lora, NULL); // cleanup and stop localaudio... whatever      
  }
  sx126x_lock();
  err = C2LORA_finish_transmit(lora);
  sx126x_unlock();
  xEventGroupClearBits(lora->events, C2LORA_EVENT_TRANSMIT_FINI);
  ESP_LOGI(TAG, "%lu packets transmitted", packet_transmitted);
  return err;
}



static inline esp_err_t c2lora_receive_funct(tLoraStream *lora, trtp_data * FWDrtp) {

  bool was_repeated;
  tKindOfSender kos;
  char callsign[8];
  char recipient[8];

  esp_err_t   err;
  TickType_t  task_timeout   = portMAX_DELAY;
  uint32_t    rx_timeout_us  = (C2LORA_DVOICE_FRAMELEN_MS + C2LORA_MAX_RXGAP_MS) * 1000;
  uint32_t    symbol_time_us;
  uint32_t    preamble_time_us;
  uint32_t    firstdat_time_us;
  uint32_t    cydata_time_us;
  TickType_t  cyclic_data_to;
  EventBits_t rx_bits        = 0;        
  uint8_t *   rxpacket_ptr   = rx_packet;
  uint8_t     read_ahead_ofs = 0;
  bool        with_header    = false;
  bool        udp_fwd_enable = false;
  bool udp_packet_ready4tx   = false;

  sx126x_hal_status_t s;

  if (lora->def == NULL) {
    ESP_LOGE(TAG, "no mode defined. no rx!");      
    return ESP_FAIL;
  }

#if CONFIG_USE_TEST_BITPATTERN
  // clear it 4 test 
  memset(rx_packet, 0xA5, sizeof(rx_packet));
  sx126x_write_buffer(lora->ctx, 1, rx_packet, 255);
#endif
  memset(callsign,  0, sizeof(callsign));
  memset(recipient, 0, sizeof(recipient));


  err = C2LORA_prepare_receive(lora, &symbol_time_us, &preamble_time_us, &firstdat_time_us, &cydata_time_us);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "preparing RX fails: %s", esp_err_to_name(err));
    lora->dva = NULL;
    return err;
  }
  read_ahead_ofs = lora->pkt_offset;

#if CONFIG_USE_TEST_BITPATTERN
  for (int i=0; i < sizeof(rx_packet); i++) {
    rx_packet[i] = 0x20 + i;
  }
  sx126x_write_buffer(lora->ctx, lora->pkt_offset, rx_packet, sizeof(rx_packet) - 1);
#endif

  ESP_LOGD(TAG, "calculated preamble = %luµs, symboltime %luµs, firstdat airtime %luµs, cyclic data %luµs", preamble_time_us, symbol_time_us, firstdat_time_us, cydata_time_us);

  cyclic_data_to = (cydata_time_us + 10000) / (configTICK_RATE_HZ * 100);

  xEventGroupClearBits(lora->events, C2LORA_EVENT_RECONFIGURE|C2LORA_EVENT_SX126X_INT);
  
  ESP_LOGD(TAG, "receiving %s (%d)", (lora->state >= RX_NOSYNC? "start": "canceled"), lora->state);

  while (lora->state >= RX_NOSYNC) {

    rx_bits = xEventGroupWaitBits(lora->events, C2LORA_ALL_EVENTS, pdTRUE, pdFALSE, task_timeout);

    if (rx_bits & (C2LORA_EVENT_RECONFIGURE|C2LORA_EVENT_TERMINATE)) break; // break RX-while loop!

    if (rx_bits & C2LORA_EVENT_SX126X_INT) { // DIO1 ISR
      sx126x_irq_mask_t irq_mask;
      
      sx126x_lock();
      sx126x_set_dio_irq_params(lora->ctx,  SX126X_IRQ_RX_DONE, C2LORARX_INTMASK, 0x00, 0x00);
      sx126x_get_and_clear_irq_status(lora->ctx, &irq_mask);

      // *** packet receive finished (must processed before PREAMBLE) ***
      if (irq_mask & SX126X_IRQ_RX_DONE) {
        sx126x_irq_mask_t         irq_mask_refresh;
        sx126x_rx_buffer_status_t rxbuffer_status = { 0, 0 };
        sx126x_pkt_status_lora_t  rxpacket_status;
        int rx_len;
        uint8_t last_dv_frames = lora->def->dv_frames_per_packet - lora->pkt_framecnt;
        uint8_t cydata_ofs     = ((with_header? 8 + lora->p.bits_header: 8) + lora->p.bits_dvoice + 7) >> 3; // Todo check 8nit clipping
        lora->state  = RX_DONE;
        task_timeout = pdMS_TO_TICKS(C2LORA_MAX_RXGAP_MS);
        sx126x_set_dio_irq_params(lora->ctx, SX126X_IRQ_PREAMBLE_DETECTED, C2LORARX_INTMASK, 0x00, 0x00);
        sx126x_get_rx_buffer_status(lora->ctx, &rxbuffer_status);

        rx_len = rx_packet + rxbuffer_status.pld_len_in_bytes - rxpacket_ptr;
        if (rx_len > 0) {
          ESP_LOGD(TAG, "last %d bytes of %d", rx_len, rxbuffer_status.pld_len_in_bytes);
          sx126x_read_buffer(lora->ctx, read_ahead_ofs, rxpacket_ptr, rx_len);
        } // fi
        read_ahead_ofs = rxbuffer_status.buffer_start_pointer + rxbuffer_status.pld_len_in_bytes; // set new offset for next packet

#if CONFIG_USE_TEST_BITPATTERN
//          ESP_LOG_BUFFER_HEX_LEVEL(TAG, rx_packet, rxbuffer_status.pld_len_in_bytes, ESP_LOG_DEBUG);
#endif
        for (; lora->pkt_framecnt < lora->def->dv_frames_per_packet; lora->pkt_framecnt++) {
          udp_packet_ready4tx |= C2LORA_add_frame_2_stream(lora->dva, rx_packet + (lora->rd_offset >> 3), lora->rd_offset & 7, FWDrtp);
          lora->rd_offset += lora->dva->bits_per_frame;
          lora->frame_cnt++;
        } // fi

        if (last_dv_frames > 0) {
          sbuf_update_time(lora->dva->buf, xTaskGetTickCount());
#if CONFIG_USE_TEST_BITPATTERN
          C2LORA_log_testpattern_from_buffer(lora->dva);
          ESP_LOGI(TAG, "(%d frames complete after RXdone)", last_dv_frames);
#else
          localaudio_handle_stream(lora->dva->streamid);   // trigger audio task to process this output stream
#endif
        } // fi get leftover dv_frames

#ifdef CONFIG_USE_TEST_BITPATTERN
        if (lora->p.bits_cydata > 0) { // mode 6 don't have cyclic data!
          ESP_LOGD(TAG, "cyclic data (%dbytes):", (lora->p.bits_cydata + 7) >> 3);
          ESP_LOG_BUFFER_HEX_LEVEL(TAG, rx_packet + cydata_ofs, (lora->p.bits_cydata + 7) >> 3, ESP_LOG_DEBUG);
        } // fi
#else
        C2LORA_handle_rx_cydata(lora, rx_packet + cydata_ofs);
#endif
        ESP_LOGD(TAG, "RX done, buffer:%ubytes @ %02xh: %02xh | %02xh", rxbuffer_status.pld_len_in_bytes, rxbuffer_status.buffer_start_pointer, 
          rx_packet[0], rx_packet[rxbuffer_status.pld_len_in_bytes - (lora->p.bits_cydata >> 3) - 1]);

        s = sx126x_get_lora_pkt_status(lora->ctx, &rxpacket_status);

        s |= sx126x_get_and_clear_irq_status(lora->ctx, &irq_mask_refresh);
        irq_mask |= irq_mask_refresh; // add a preample event, if this occured in the RXdone handling time.
        if (s) {
          ESP_LOGE(TAG, "error get packet status");
        } else {
#if (defined SX126X_LNA_RSSI_SHIFT) && (SX126X_LNA_RSSI_SHIFT > 0)
          ESP_LOGD(TAG, "RX rssi %ddBm sig %ddBm, SNR:%ddBm, DVframes:%lu IRQmask:%04xh", 
            rxpacket_status.rssi_pkt_in_dbm - SX126X_LNA_RSSI_SHIFT,
            rxpacket_status.signal_rssi_pkt_in_dbm - SX126X_LNA_RSSI_SHIFT,
            rxpacket_status.snr_pkt_in_db, lora->frame_cnt, irq_mask);
#else
          ESP_LOGD(TAG, "RX rssi %ddBm sig %ddBm, SNR:%ddBm, DVframes:%lu IRQmask:%04xh", 
            rxpacket_status.rssi_pkt_in_dbm,
            rxpacket_status.signal_rssi_pkt_in_dbm,
            rxpacket_status.snr_pkt_in_db, lora->frame_cnt, irq_mask);
#endif              
        }
        //if (rxbuffer_status.pld_len_in_bytes > 0) {
        //  ESP_LOG_BUFFER_HEX(TAG, rx_packet, rxbuffer_status.pld_len_in_bytes);
        //}
        //ESP_LOGI(TAG, "high stack water mark is %u", uxTaskGetStackHighWaterMark(NULL));

        // refresh IRQ-Status (new preable?)
        
        // ??? maskiert? check
#ifndef CONFIG_USE_TEST_BITPATTERN
        if (udp_packet_ready4tx) {
          if (udp_fwd_enable) UTX_transmit(FWDrtp);
          udp_packet_ready4tx = false;
        }
#endif
      } // fi RXdone

      // *** PREAMBLE detected ***
      if (irq_mask & SX126X_IRQ_PREAMBLE_DETECTED) {
        int32_t process_time_us;
        lora->state         = RX_PREAMBLE;
        lora->rd_offset     = 0;
        lora->pkt_framecnt  = 0;
        lora->udp_stream_id = C2LORA_TX_STREAM_ID;
        rxpacket_ptr   = rx_packet;  // reset packet position
#if CONFIG_USE_TEST_BITPATTERN
        memset(rx_packet, 0x22, sizeof(rx_packet)); 
#endif

// ToDo in Funktionen auslagern.  
        if ((lora->dva->streamid == -1) && (lora->dva->codec_type != 255)) {
          c2lora_create_audioout_stream(lora->dva);
        } // fi

        process_time_us = esp_timer_get_time() - lora->pkt_start_time;
        task_timeout = (firstdat_time_us - process_time_us + 10000) / (configTICK_RATE_HZ * 100) + 1;  // run task after a initial delay
        ESP_LOGD(TAG, "RX preamble, %ldµs since INT, to=%lu", process_time_us, task_timeout);
      } // fi preamble

      sx126x_unlock();

    } else // fi DIO1 INT

    if ((rx_bits & C2LORA_ALL_EVENTS) == 0) {  // do cyclic checking (event timeouts)

      int32_t rx_time = esp_timer_get_time() - lora->pkt_start_time;    // elapsed time in µs since a preamble was detected

      if (rx_time < (preamble_time_us >> 1)) {    // TXdone IRQ set pkt_start_time like all other IQR sources.
        ESP_LOGD(TAG, "awaits RXdone");
        task_timeout = pdMS_TO_TICKS(C2LORA_MAX_RXGAP_MS);
        continue;
      } // fi TXdone int has firered...
      if ((lora->state == RX_DONE) || (rx_time >= rx_timeout_us)) { // we are done, no futher sync - disable RX
        ESP_LOG_LEVEL((lora->state == RX_DONE? ESP_LOG_DEBUG: ESP_LOG_WARN), TAG, "RX %s", (lora->state == RX_DONE? "ended": "timeout"));
        ESP_LOGV(TAG, "task stack high watermark=%u", uxTaskGetStackHighWaterMark(NULL));
        localaudio_end_stream(lora->dva, true);
        sx126x_lock();
        sx126x_set_dio_irq_params(lora->ctx, SX126X_IRQ_PREAMBLE_DETECTED, C2LORARX_INTMASK, 0x00, 0x00);
        sx126x_unlock();
        if (udp_fwd_enable) C2LORA_stop_HAMdLNK(FWDrtp);
        lora->udp_stream_id = 0;
        task_timeout = portMAX_DELAY;
        lora->state  = RX_NOSYNC;
        continue;
      } // fi

      if (lora->pkt_framecnt == lora->def->dv_frames_per_packet) {
        ESP_LOGD(TAG, "RX DV fini, wait 4 RXdone");
        continue;
      }

      task_timeout = 1;  // run task run - every 10ms look for data

      uint8_t sx126x_rx_ptr;
      sx126x_read_register(lora->ctx, SX126X_REG_RX_ADDRESS_POINTER, &sx126x_rx_ptr, 1);
      if (sx126x_rx_ptr == read_ahead_ofs) {
        continue;
      }
      uint8_t bytes_received = sx126x_rx_ptr - read_ahead_ofs;

      int  rxpkt_free_len = rx_packet + sizeof(rx_packet) - rxpacket_ptr;
      int  rx_byte_offset = rxpacket_ptr - rx_packet;    // offset starts(0) at rx_packet
      int  rx_bit_offset  = rx_byte_offset << 3;         // same as rx_byte_offset*8
      int  min_bytes_need;
      bool rxpkt_is_empty = rxpacket_ptr == rx_packet;

      switch(lora->state) {
      default:  
      case RX_PREAMBLE:
        min_bytes_need = 1; 
        break;
      case RX_HEADER:
        if (with_header) {    // with_header detection
          min_bytes_need = lora->def->bytes_per_header + 1 - rx_byte_offset;
          break;
        } // fi still not enough bytes for the header         
        // fall through
      case RX_DATA:
        min_bytes_need = (lora->dva->bits_per_frame - ((rx_bit_offset) - lora->rd_offset) + 7) >> 3;
        break;
      } // hctiws

//        int symbols_received = (rx_time - preamble_time_us) / symbol_time_us;        
//        int bytes_received_calc = ((symbols_received / (lora->def->mod.cr + 4)) * (lora->def->mod.sf << 2) >> 3) - rx_byte_offset;
//        ESP_LOGD(TAG, "RX@%02xh Sym=%3d | %3d/%3dbytes of %3dmin", rx_byte_offset, symbols_received, bytes_received, bytes_received_calc, min_bytes_need);

      if (bytes_received < min_bytes_need) {
        continue;
      }

      if (bytes_received > rxpkt_free_len) bytes_received = rxpkt_free_len;
      sx126x_lock();
      sx126x_read_buffer(lora->ctx, read_ahead_ofs, rxpacket_ptr, bytes_received);
      sx126x_unlock();
      rxpacket_ptr   += bytes_received;
      read_ahead_ofs += bytes_received;
      rx_byte_offset += bytes_received;

      if (rxpkt_is_empty) { //lora->state == RX_PREAMBLE
        uint8_t pkt_len = lora->def->bytes_per_packet;
        with_header = C2LORA_is_header(rx_packet[0]);
        if (with_header) {
          pkt_len += lora->def->bytes_per_header;
          rx_timeout_us = (C2LORA_WHEADER_FRAMELEN_MS + C2LORA_MAX_RXGAP_MS) * 1000;
        } else {
          rx_timeout_us = (C2LORA_DVOICE_FRAMELEN_MS + C2LORA_MAX_RXGAP_MS) * 1000;
        }
        lora->state = with_header? RX_HEADER: RX_DATA;
        lora->update_pkt_params = lora->pkt_params[SX126X_PKT_LEN_BYTEPOS] != pkt_len;
        lora->pkt_params[SX126X_PKT_LEN_BYTEPOS] = pkt_len; // correct packet length dependend to the first byte value
        lora->pkt_framecnt = 0;
        lora->rd_offset = 8 + (with_header? lora->p.bits_header: 0);
        sx126x_lock();
        sx126x_set_dio_irq_params(lora->ctx, C2LORARX_INTMASK, C2LORARX_INTMASK, 0x00, 0x00);
        sx126x_unlock();
        ESP_LOGD(TAG, "RX %s", (with_header? "has header": "new frame"));
      } // fi determine type of packet

      if (lora->update_pkt_params) {
        sx126x_lock();
        lora->update_pkt_params = sx126x_hal_fast_cmd(lora->ctx, lora->pkt_params, SX126X_SIZE_PKT_PARAMS);
        sx126x_unlock();
        ESP_LOGD(TAG, "length for receiving packet set to %dbytes", lora->pkt_params[SX126X_PKT_LEN_BYTEPOS]);
      } // fi correct packet length

      if ((lora->state == RX_HEADER) && (rx_byte_offset > lora->def->bytes_per_header)) { // header complete? // && (lora->state == RX_HEADER)
        bool header_ok;
        lora->state = RX_DATA;
        header_ok = C2LORA_check_header(rx_packet + 1, lora->def->bytes_per_header);
        kos       = C2LORA_decode_header(rx_packet + 1, callsign, recipient, &was_repeated);
        tC2LORA_mode mode = lora->def->mode;
        if (header_ok) {
          char locator[8];
          uint16_t areacode;
          C2LORA_get_from_header(mode, rx_packet + 1, &areacode, locator, false);
#ifndef CONFIG_USE_TEST_BITPATTERN    
          udp_fwd_enable = C2LORA_start_HAMdLNK(FWDrtp, callsign, recipient, kos, areacode, locator) == ESP_OK;
          if (!udp_fwd_enable) {
            ESP_LOGW(TAG, "no UDP connection");
          }              
#endif
        } else {

        }
#if CONFIG_USE_TEST_BITPATTERN
        ESP_LOGD(TAG, "RX header (%d bits) received", lora->p.bits_header);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, rx_packet + 1, rx_byte_offset - 1, ESP_LOG_DEBUG);
#endif            
        if (!header_ok) {
          ESP_LOGW(TAG, "header data are corrupted!");
        }
        ESP_LOGI(TAG, "from: %s to: %s KoS: %s%s", callsign, recipient, C2LORA_get_kos_name(kos), was_repeated? " (repeated)": "");
      } // fi handle received header

      rx_bit_offset = rx_byte_offset << 3;

      if (rx_bit_offset >= (lora->rd_offset + lora->dva->bits_per_frame) ) {
        for (int frame_lastpos = rx_bit_offset - lora->dva->bits_per_frame;  (lora->pkt_framecnt < lora->def->dv_frames_per_packet) && (lora->rd_offset <= frame_lastpos);  lora->pkt_framecnt++) { 
          udp_packet_ready4tx |= C2LORA_add_frame_2_stream(lora->dva, rx_packet + (lora->rd_offset >> 3), lora->rd_offset & 7, FWDrtp);
          lora->rd_offset += lora->dva->bits_per_frame;
          lora->frame_cnt++;
        } // rof multiple data@once          
#if CONFIG_USE_TEST_BITPATTERN
        C2LORA_log_testpattern_from_buffer(lora->dva);
#else  
        sbuf_update_time(lora->dva->buf, xTaskGetTickCount());
        localaudio_handle_stream(lora->dva->streamid);   // trigger audio task to process this output stream
#endif
      } // fi rx a frame

      if (lora->pkt_framecnt == lora->def->dv_frames_per_packet) {
        task_timeout = cyclic_data_to;
      } // fi done

      if (udp_fwd_enable && udp_packet_ready4tx) {
        UTX_transmit(FWDrtp); 
      }
      udp_packet_ready4tx = false;
      
    } // fi task timeout (cyclic fetching data adead of TXdone)

  } // ehliw RX

  if (rx_bits & (C2LORA_EVENT_RECONFIGURE|C2LORA_EVENT_TERMINATE)) {
    ESP_LOGD(TAG, "receiving ended -> standby (%d)...", lora->state);
    sx126x_lock();
    C2LORA_set_standby(lora, SX126X_STANDBY_CFG_RC);
    sx126x_unlock();
    lora->state = RX_NOSYNC;
    lora->dva   = NULL;
  } // fi reconfigure 

  return (rx_bits & C2LORA_EVENT_TERMINATE)? ESP_ERR_NO_MEM: ESP_OK;  // stop the task!
}


static void C2LORA_control_task(void *thread_data) {
  esp_err_t err = ESP_OK;
  sx126x_hal_status_t s;
  tLoraStream *lora = (tLoraStream *) thread_data;

  tdvstream C2LORArx;
  trtp_data UDPrtp;
  
  if (lora == NULL) goto c2lora_task_shutdown;

  if (lora->events == 0) {
    ESP_LOGE(TAG, "no events - exit thread");
    goto c2lora_task_shutdown;
  }
  ESP_LOGD(TAG, "control task init");

  memset(&C2LORArx, 0, sizeof(tdvstream));
  memset(&UDPrtp,   0, sizeof(trtp_data));
  C2LORArx.srate      = 8000;
  C2LORArx.codec_type = 255;
  C2LORArx.flags      = DVSTREAM_FLAG_KEEP_BUFFER;
  C2LORArx.buf        = (tsimple_buffer *) malloc(sizeof(tsimple_buffer));

  xEventGroupClearBits(lora->events, C2LORA_ALL_EVENTS);

  do {

    tLoraCmdQItem q_item;
    if (xQueueReceive(lora->cmd_queue, &q_item, portMAX_DELAY)) {
      uint32_t ret_value;
      tLoraStream *lora4cmd = lora;
      EventBits_t events = xEventGroupGetBits(lora->events);
      ESP_LOGD(TAG, "run cmd (%d)", lora4cmd->state);
      ret_value = q_item.cmd_fct != NULL? q_item.cmd_fct(lora4cmd, q_item.cmd_arg): ESP_ERR_INVALID_ARG;
      if (lora4cmd->task_to_notify) xTaskNotify(lora4cmd->task_to_notify, ret_value, eSetValueWithOverwrite);

      // reenable receiving instead IDLE
      if ((lora->state == LS_IDLE) && (events & C2LORA_EVENT_RECEIVE_ACTI)) {
        lora->state = RX_NOSYNC;
      } // fi
    
      s = sx126x_clear_irq_status(lora->ctx, SX126X_IRQ_ALL);     // after clear TXdone event is active again
      if (s) {
        sx126x_clear_irq_status(lora->ctx, SX126X_IRQ_ALL);       // try again
        ESP_LOGW(TAG, "clear int status failed");
      }

    } // fi handle cmd

    if (lora->state >= RX_NOSYNC) {         // we are in RX mode 
      // are we receiving now? yes: enter 2nd loop after a bit of preparing
      trtp_data * FWDrtp = NULL;
      C2LORArx.streamid = -1;
      if (isSF_CODEC2(lora->def->codec_type)) {
        C2LORArx.codec_type        = lora->def->codec_type;
        C2LORArx.samples_per_frame = lora->def->dv_frames_per_packet==12? 320: 160;
        C2LORArx.bits_per_frame    = sf_get_subframe_bits(lora->def->codec_type);
        C2LORArx.bytes_per_frame   = (C2LORArx.bits_per_frame + 7) >> 3;
        FWDrtp = C2LORA_setup_HAMdLNK(&UDPrtp, &C2LORArx);
      } else {
        C2LORArx.codec_type        = 255;  // some other stuff - not Codec2
        ESP_LOGE(TAG, "invalid codec type defined.");
        continue;
      }
      lora->dva = &C2LORArx;

      err = c2lora_receive_funct(lora, FWDrtp);
      if (err != ESP_OK) ESP_LOGE(TAG, "receive returns error: %s", esp_err_to_name(err));

      lora->dva = NULL;

    } else if (lora->state >= TX_SEND_HEADER) {    // we in TRANSMIT mode now
      // start transmit loop
      err = c2lora_transmit_funct(lora);
      if (err != ESP_OK) ESP_LOGE(TAG, "transmit returns error: %s", esp_err_to_name(err));

    } // fi TX

    sx126x_lock();
    sx126x_clear_irq_status(lora->ctx, SX126X_IRQ_ALL);
    sx126x_unlock();

  } while (err != ESP_ERR_NO_MEM); // end control-task loop

c2lora_task_shutdown:
  sx126x_lock();
  C2LORA_set_standby(lora, SX126X_STANDBY_CFG_RC);
  sx126x_unlock();

  lora->state = LS_INACTIVE;
  
  // ToDo achtung: Buffer kann noch in verwendung sein!!!
  sbuf_destroy(C2LORArx.buf);

  free(C2LORArx.buf);
  ESP_LOGW(TAG, "c2lora task shutdown.");
  vTaskDelete(NULL);
}



esp_err_t C2LORA_retransmit_last(void)  {
  tRecorderHandle *rec = C2LORA_get_last_record();
  ESP_RETURN_ON_FALSE(rec != NULL, ESP_ERR_NO_MEM, TAG, "no recording.");

  return C2LORA_send_record(rec, C2LORA_get_stream_4_transmit());
}


esp_err_t C2LORA_read_registers_from(unsigned short addr) {
  return C2LORA_read_registers_at(&lora_stream, addr);
}

esp_err_t C2LORA_write_register_to(unsigned short addr, unsigned char value) {
  return sx126x_write_register(lora_stream.ctx,addr, &value, 1);
}




static esp_err_t C2LORA_handle_rxheader(const tLoraStream *lora, const uint8_t *rawheader) {
  bool header_ok;
  bool was_repeated;
  tKindOfSender kos;
  char callsign[8];
  char recipient[8];
  char locator[8];
  uint16_t areacode;
  header_ok = C2LORA_check_header(rawheader, lora->def->bytes_per_header);
  kos       = C2LORA_decode_header(rawheader, callsign, recipient, &was_repeated);
  tC2LORA_mode mode = lora->def->mode;
  if (header_ok) {
    C2LORA_get_from_header(mode, rawheader, &areacode, locator, false);
  } else {
    locator[0] = 0;
    areacode = 0;
  }  
  if (!header_ok) {
    ESP_LOGW(TAG, "header data are corrupted!");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, rawheader, 16, ESP_LOG_DEBUG);
  }
  ESP_LOGI(TAG, "from: '%.7s' to: '%.7s' KoS: %s%s | ac=%04xh, loc='%s'", callsign, recipient, C2LORA_get_kos_name(kos), was_repeated? " (repeated)": "", areacode, locator);
  return header_ok? ESP_OK: ESP_FAIL;
}






static esp_timer_handle_t test_timer;
static uint32_t test_timer_delay_us;
static uint8_t  tx_test_packet[256];
static uint8_t  vgl_buf[256];

static void IRAM_ATTR gpio_tx_test_handler(void *arg) {
  tLoraStream *lora = (tLoraStream *) arg;
  xEventGroupSetBitsFromISR(lora->events, C2LORA_EVENT_SX126X_INT, NULL);
}

/*
static void IRAM_ATTR test_tx_restart_cb(void *arg) {
  tLoraStream *lora = (tLoraStream *) arg;
  int64_t start_time = esp_timer_get_time();
  int32_t done_after_start_us = start_time - lora->pkt_start_time;
  lora->pkt_start_time = start_time;
  
  sx126x_hal_fast_cmd(lora->ctx, sx126x_start_txd, 4);

  ESP_EARLY_LOGV(TAG, "TX start timer isr %d, %dµs", lora->state, done_after_start_us);
}*/


static void pertest_transmit_task(void *thread_data) {
  sx126x_hal_status_t s;
  tLoraStream *loratx = C2LORA_get_stream_4_transmit();
  int no_of_transmits = (int) thread_data;
  uint8_t start_byte  = ~C2LORA_FRAME_STARTBYTE; // 0xCC for header start
  /*
  esp_timer_create_args_t tx_timer_args = {
    .callback = test_tx_restart_cb,
    .arg = loratx,
    .dispatch_method = ESP_TIMER_ISR,
    .skip_unhandled_events = true
  };
  esp_timer_create(&tx_timer_args, &test_timer);
  ESP_LOGD(TAG, "timer set to: %luµs", test_timer_delay_us);
  */

  loratx->pkt_params[SX126X_PKT_PRE_BYTEPOS] = loratx->def->default_preamble;
  loratx->pkt_params[SX126X_PKT_LEN_BYTEPOS] = loratx->def->bytes_per_header + loratx->def->bytes_per_packet;
  sx126x_hal_fast_cmd(loratx->ctx, loratx->pkt_params, SX126X_SIZE_PKT_PARAMS);
  sx126x_hal_wait_busy(loratx->ctx);

  s = sx126x_set_buffer_base_address(loratx->ctx, 0, 0);
  s |= sx126x_set_dio_irq_params(loratx->ctx, SX126X_IRQ_TX_DONE, SX126X_IRQ_TX_DONE, 0x00, 0x00);
  s |= sx126x_set_rx_tx_fallback_mode(loratx->ctx, SX126X_FALLBACK_FS);
  s |= sx126x_write_buffer(loratx->ctx, 0, &start_byte, 1);
  s |= sx126x_write_buffer(loratx->ctx, 1, local_header, loratx->def->bytes_per_header);
  if (s) ESP_LOGE(TAG, "buffer preparation fails");

  s = sx126x_set_rx_tx_fallback_mode(loratx->ctx, SX126X_FALLBACK_FS);
  if (s) ESP_LOGE(TAG, "set rx/tx fallback mode");

  esp_err_t err = gpio_isr_handler_add(loratx->intr_pin, gpio_tx_test_handler, (void*) loratx);
  if (err != ESP_OK) ESP_LOGE(TAG, "init GPIO ISR failed (%s)", esp_err_to_name(err));

  s = sx126x_clear_irq_status(loratx->ctx, SX126X_IRQ_ALL);
  if (s) ESP_LOGE(TAG, "clear TX irq lines");

  err = gpio_set_intr_type(loratx->intr_pin, GPIO_INTR_POSEDGE);
  
  loratx->update_pkt_params = false;
  loratx->state = TX_SEND_FRAME;
  loratx->dva = NULL;
  for (uint8_t pos=0; pos < loratx->def->bytes_per_packet; pos++) {
    tx_test_packet[pos] = (uint8_t)(pos);
  } // rof

  C2LORA_begin_transmit(loratx);
  loratx->pkt_start_time = esp_timer_get_time();
  loratx->frame_cnt = -1;

  s = sx126x_write_buffer(loratx->ctx, loratx->def->bytes_per_header + 1, tx_test_packet, loratx->def->bytes_per_packet - 1);

  for (uint8_t pos=0; pos < loratx->def->bytes_per_packet; pos++) {
    tx_test_packet[pos] = (uint8_t)(16 + pos);
  } // rof
  start_byte = C2LORA_FRAME_STARTBYTE; // 0x33 for normal frames
  s = sx126x_write_buffer(loratx->ctx, 128, &start_byte, 1);
  s |= sx126x_write_buffer(loratx->ctx, 129, tx_test_packet, loratx->def->bytes_per_packet - 1);

  loratx->pkt_params[SX126X_PKT_LEN_BYTEPOS] = loratx->def->bytes_per_packet;
  loratx->update_pkt_params = true; // update at TXdone

  for (int tno=0; tno < no_of_transmits; tno++) {

    ESP_LOGV(TAG, "tx #%d", tno);
    // *** Transmit a packet ***
    loratx->state = tno < (no_of_transmits - 1)? TX_SEND_FRAME: TX_LAST_FRAME;
/*
    sx126x_lock();
    if (loratx->update_pkt_params) {    // update packet parameters esp. packet-lenghth or preamble-count    
      sx126x_hal_wait_busy(loratx->ctx);
      loratx->update_pkt_params = sx126x_hal_fast_cmd(loratx->ctx, loratx->pkt_params, SX126X_SIZE_PKT_PARAMS) != ESP_OK;
      sx126x_hal_wait_busy(loratx->ctx);
      ESP_LOGD(TAG, "update TX to %d bytes packets", loratx->pkt_params[SX126X_PKT_LEN_BYTEPOS]);
    } 
    C2LORA_begin_transmit(loratx);
    loratx->pkt_start_time = esp_timer_get_time();
    sx126x_unlock();

    // after start TX we are writing to buffer
    sx126x_lock();
    if (tno == 0) {
      s = sx126x_write_buffer(loratx->ctx, loratx->def->bytes_per_header + 1, tx_test_packet, loratx->def->bytes_per_packet - 1);
      loratx->pkt_params[SX126X_PKT_LEN_BYTEPOS] = loratx->def->bytes_per_packet;
      loratx->update_pkt_params = true; // update at TXdone
     // vTaskDelay(pdMS_TO_TICKS(60));
    } else {
      start_byte = C2LORA_FRAME_STARTBYTE; // 0x33 for normal frames
      s = sx126x_write_buffer(loratx->ctx, loratx->frame_cnt & 1? 0: 128, &start_byte, 1);
      s |= sx126x_write_buffer(loratx->ctx, loratx->frame_cnt & 1? 1: 129, tx_test_packet, loratx->def->bytes_per_packet - 1);
    }
    sx126x_unlock();
    if (s) ESP_LOGW(TAG, "can't write to SPI buffer (%d)", s);
*/ 

    EventBits_t tx_bits = xEventGroupWaitBits(loratx->events, C2LORA_EVENT_SX126X_INT, pdTRUE, pdFALSE, pdMS_TO_TICKS(590));
    if (tx_bits & C2LORA_EVENT_SX126X_INT) {
      // ESP_LOGD(TAG, "TXdone");
    } else {
      ESP_LOGD(TAG, "TX timeout, INTR pin = %d", gpio_get_level(loratx->intr_pin));
      sx126x_lock();
      sx126x_clear_irq_status(loratx->ctx, SX126X_IRQ_ALL);
      sx126x_unlock();
    }

    uint8_t sx126x_set_baseadr[3] = { SX126X_CMD_SET_BASEADR, loratx->frame_cnt & 1? 128: 0, 0 };
    loratx->frame_cnt++;

    sx126x_lock();
    C2LORA_clear_irq_mask(loratx);
    sx126x_hal_fast_cmd(loratx->ctx, sx126x_set_baseadr, 3);
    if (loratx->update_pkt_params) {    // update packet parameters esp. packet-lenghth or preamble-count
      loratx->update_pkt_params = sx126x_hal_fast_cmd(loratx->ctx, loratx->pkt_params, SX126X_SIZE_PKT_PARAMS) != ESP_OK;
    }

    if (loratx->state == TX_SEND_FRAME) {
      int64_t start_time = esp_timer_get_time();
      int32_t done_after_start_us = start_time - loratx->pkt_start_time;
      loratx->pkt_start_time = start_time;
      sx126x_hal_fast_cmd(loratx->ctx, sx126x_start_txd, 4);
      ESP_LOGD(TAG, "TX restart after %ldµs", done_after_start_us);
    }
    sx126x_unlock();

    // add next frame here:
    for (uint8_t pos=0; pos < loratx->def->bytes_per_packet; pos++) {
      tx_test_packet[pos] = (uint8_t)((loratx->frame_cnt + 2) * 16 + pos);
    } // rof
    sx126x_lock();
    sx126x_read_buffer(loratx->ctx, loratx->frame_cnt & 1? 128: 0, vgl_buf, loratx->def->bytes_per_packet);
    start_byte = C2LORA_FRAME_STARTBYTE; // 0x33 for normal frames
    s = sx126x_write_buffer(loratx->ctx, loratx->frame_cnt & 1? 128: 0, &start_byte, 1);
    s |= sx126x_write_buffer(loratx->ctx, loratx->frame_cnt & 1? 129: 1, tx_test_packet, loratx->def->bytes_per_packet - 1);
    sx126x_unlock();

  } // rof

  loratx->state = LS_IDLE;
  gpio_set_intr_type(loratx->intr_pin, GPIO_INTR_DISABLE); 
  gpio_isr_handler_remove(loratx->intr_pin);
  esp_timer_delete(test_timer);
  vTaskDelete(NULL);
}



static void IRAM_ATTR gpio_rx_test_handler(void *arg) {
  tLoraStream *lora = (tLoraStream *) arg;
  lora->pkt_start_time = esp_timer_get_time(); // invalid if the SX126x INT was a RXdone, doesn't matter
  xEventGroupSetBitsFromISR(lora->events, C2LORA_EVENT_SX126X_INT, NULL);
  ESP_EARLY_LOGV(TAG, "RX test isr");
}


esp_err_t C2LORA_perform_PER_test(unsigned int no_of_transmits, unsigned int tx_delay_us) {
  sx126x_hal_status_t s;
  tLoraStream *lorarx = &lora_stream;
  tLoraStream *loratx = C2LORA_get_stream_4_transmit();
  int transmits_passed = 0;
  uint8_t rx_packet[256];

  test_timer_delay_us = tx_delay_us;

  lorarx->pkt_params[SX126X_PKT_PRE_BYTEPOS] = lorarx->def->default_preamble;
  lorarx->pkt_params[SX126X_PKT_LEN_BYTEPOS] = lorarx->def->bytes_per_header + lorarx->def->bytes_per_packet;

  sx126x_hal_fast_cmd(lorarx->ctx, lorarx->pkt_params, SX126X_SIZE_PKT_PARAMS);
  sx126x_hal_wait_busy(lorarx->ctx);

  s = sx126x_set_buffer_base_address(lorarx->ctx, 0, 0);

  s = sx126x_set_rx_tx_fallback_mode(lorarx->ctx, SX126X_FALLBACK_FS);
  if (s) ESP_LOGE(TAG, "set rx/tx fallback mode");

  s = sx126x_set_rx_with_timeout_in_rtc_step(lorarx->ctx, SX126X_RX_CONTINUOUS);
  if (s) ESP_LOGE(TAG, "set rx continuous");

  s = sx126x_set_dio_irq_params(lorarx->ctx, SX126X_IRQ_PREAMBLE_DETECTED, C2LORARX_INTMASK, 0x00, 0x00);
  if (s) ESP_LOGE(TAG, "set dio irq params");

  esp_err_t err = gpio_isr_handler_add(lorarx->intr_pin, gpio_rx_test_handler, (void*) lorarx);
  ESP_RETURN_ON_ERROR(err, TAG, "init GPIO ISR failed (%s)", esp_err_to_name(err));

  err = gpio_set_intr_type(lorarx->intr_pin, GPIO_INTR_POSEDGE);

  xTaskCreate(pertest_transmit_task, "PER-Test", C2LORARX_TASK_STACK_SIZE, (void *)no_of_transmits, C2LORARX_TASK_PRIORITY, NULL);

  uint8_t read_ahead_ofs = 0;
  uint8_t *rxpacket_ptr  = rx_packet;  
  bool    with_header    = false;

  lorarx->pkt_start_time = esp_timer_get_time();

  do {

    // *** start receiving the packet ***
    lorarx->state = RX_NOSYNC;
    do {
      EventBits_t rx_bits = xEventGroupWaitBits(lorarx->events, C2LORA_EVENT_SX126X_INT, pdTRUE, pdFALSE, 1);
      if (rx_bits & C2LORA_EVENT_SX126X_INT) {
        sx126x_irq_mask_t irq_mask;
        sx126x_lock();
        sx126x_set_dio_irq_params(lorarx->ctx, SX126X_IRQ_RX_DONE, C2LORARX_INTMASK, 0x00, 0x00);
        s = sx126x_get_and_clear_irq_status(lorarx->ctx, &irq_mask);
        sx126x_unlock();
        if (s) ESP_LOGE(TAG, "clear irq status");

        if (irq_mask & SX126X_IRQ_RX_DONE) {
          sx126x_rx_buffer_status_t rxbuffer_status  = { 0, 0 };
          sx126x_pkt_status_lora_t  rxpacket_status;
          int rx_len, err_cnt = 0;
          int frame_txed = loratx->frame_cnt;
          int32_t process_time_us = esp_timer_get_time() - lorarx->pkt_start_time;

          lorarx->state = RX_DONE;
          lorarx->frame_cnt = frame_txed;
          sx126x_lock();
          sx126x_set_dio_irq_params(lorarx->ctx, SX126X_IRQ_PREAMBLE_DETECTED, C2LORARX_INTMASK, 0x00, 0x00);
          sx126x_get_rx_buffer_status(lorarx->ctx, &rxbuffer_status);
 
          rx_len = rx_packet + rxbuffer_status.pld_len_in_bytes - rxpacket_ptr;
          if (rx_len > 0) {
            ESP_LOGD(TAG, "last %d bytes of %d", rx_len, rxbuffer_status.pld_len_in_bytes);
            sx126x_read_buffer(lorarx->ctx, read_ahead_ofs, rxpacket_ptr, rx_len);
          } // fi
          read_ahead_ofs = rxbuffer_status.buffer_start_pointer + rxbuffer_status.pld_len_in_bytes;
          ESP_LOGD(TAG, "RX done, buffer:%ubytes @ %02xh: %02xh after %luµs", rxbuffer_status.pld_len_in_bytes, rxbuffer_status.buffer_start_pointer, rx_packet[0], process_time_us);

          s = sx126x_get_lora_pkt_status(lorarx->ctx, &rxpacket_status);
          sx126x_unlock();
          
          esp_err_t header_err = ESP_OK;
          uint8_t   start_byte, start_ofs, err_pos = 0;
          
          start_ofs = with_header? lorarx->def->bytes_per_header + 1: 1;
          for (uint8_t pos=0; pos < lorarx->def->bytes_per_packet - 1; pos++) {
            if (rx_packet[start_ofs + pos] != ((uint8_t)((frame_txed * 16) + pos))) {
              if (err_cnt == 0) err_pos = pos;
              err_cnt++;
            }
          } // rof          
          if (with_header) {
            header_err = C2LORA_handle_rxheader(lorarx, rx_packet + 1);
            start_byte = ~C2LORA_FRAME_STARTBYTE; // 0xCC for header start            
          } else {
            start_byte = C2LORA_FRAME_STARTBYTE;
          }          

          if (rx_packet[0] != start_byte) err_cnt++;          

          if (s) {
            ESP_LOGE(TAG, "error get packet status");
          } else {
            ESP_LOGD(TAG, "RX rssi %ddBm sig %ddBm, SNR:%ddBm, no of reads:%u, IRQmask:%04xh", 
              rxpacket_status.rssi_pkt_in_dbm,
              rxpacket_status.signal_rssi_pkt_in_dbm,
              rxpacket_status.snr_pkt_in_db, lorarx->pkt_framecnt, irq_mask);
          }
          if (err_cnt > 0) {
            ESP_LOGW(TAG, "#%02d %spacket has %d byte errors (%02xh) @%02xh", frame_txed, (with_header? "header-": ""), err_cnt, rx_packet[0], err_pos);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, &rx_packet[with_header?lorarx->def->bytes_per_header+1: 1], lorarx->def->bytes_per_packet-1, ESP_LOG_DEBUG);
            ESP_LOGW(TAG, "#%02d TX packet read (%02xh):", frame_txed, vgl_buf[0]);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, vgl_buf + 1, lorarx->def->bytes_per_packet - 1 , ESP_LOG_DEBUG);

          } else if (header_err == ESP_OK) {
            if (with_header) {
              ESP_LOGI(TAG, "#%02d data + header ok (%02xh)", frame_txed, rx_packet[0]);
            } else {
              ESP_LOGI(TAG, "#%02d data ok, no header (%02xh)", frame_txed, rx_packet[0]);
            }
            transmits_passed++;
          } else {
            ESP_LOGW(TAG, "#%02d data ok, but header corrupted (%02xh)", frame_txed, rx_packet[0]);
          }

          //sx126x_irq_mask_t irq_mask_refresh = 0;
          //sx126x_lock();
          //sx126x_get_and_clear_irq_status(lorarx->ctx, &irq_mask_refresh);
          //irq_mask |= irq_mask_refresh; // add a preample event, if this occured in the RXdone handling time.
          //sx126x_unlock();

        } // fi tx done

        if (irq_mask & SX126X_IRQ_PREAMBLE_DETECTED) {
          int32_t process_time_us;
          sx126x_rx_buffer_status_t rxbuffer_status = { 0, 0 };
          lorarx->state = RX_PREAMBLE;
          lorarx->rd_offset    = 0;
          lorarx->pkt_framecnt = 0;
          sx126x_lock();
          s = sx126x_get_rx_buffer_status(lorarx->ctx, &rxbuffer_status);
          sx126x_unlock();
          if (s) ESP_LOGW(TAG, "read rx buffer status fails");
          rxpacket_ptr = rx_packet;  // reset packet position
          // *** verursacht Fehler beim 2ten run ***
          //read_ahead_ofs = rxbuffer_status.buffer_start_pointer + rxbuffer_status.pld_len_in_bytes;          
          process_time_us = esp_timer_get_time() - lorarx->pkt_start_time;
          ESP_LOGD(TAG, "RX preamble, %ldµs since INT, %02xh+%d", process_time_us, rxbuffer_status.buffer_start_pointer, rxbuffer_status.pld_len_in_bytes);
        }

      } // fi int

      if ((rx_bits & (C2LORA_EVENT_SX126X_INT)) == 0) {
        uint8_t sx126x_rx_ptr;
        uint8_t bytes_received;
        int32_t process_time_us = esp_timer_get_time() - lorarx->pkt_start_time;
        if (process_time_us > 580000L) {
          ESP_LOGW(TAG, "timeout RX");
          lorarx->state = RX_DONE;
        }
        sx126x_lock();
        sx126x_read_register(lorarx->ctx, SX126X_REG_RX_ADDRESS_POINTER, &sx126x_rx_ptr, 1);
        sx126x_unlock();
        if (sx126x_rx_ptr == read_ahead_ofs) {
          continue;
        }
        bytes_received = sx126x_rx_ptr - read_ahead_ofs;
        sx126x_lock();
        sx126x_read_buffer(lorarx->ctx, read_ahead_ofs, rxpacket_ptr, bytes_received);
        sx126x_unlock();
        if ((rxpacket_ptr == rx_packet) || (lorarx->update_pkt_params)) {
          uint8_t pkt_len = lorarx->def->bytes_per_packet;
          with_header = C2LORA_is_header(rx_packet[0]);
          if (with_header) {
            pkt_len += lorarx->def->bytes_per_header;
            ESP_LOGD(TAG, "packet has header");
          }
          lorarx->update_pkt_params = false;
          if (lorarx->pkt_params[SX126X_PKT_LEN_BYTEPOS] != pkt_len) {
            ESP_LOGD(TAG, "set pkt length to: %dbytes", pkt_len);
            lorarx->pkt_params[SX126X_PKT_LEN_BYTEPOS] = pkt_len; // correct packet length dependend to the first byte value
            sx126x_lock();
            s = sx126x_hal_fast_cmd(lorarx->ctx, lorarx->pkt_params, SX126X_SIZE_PKT_PARAMS);
            sx126x_hal_wait_busy(lorarx->ctx);
            sx126x_unlock();
            if (s) ESP_LOGE(TAG, "packet length update fails");
            lorarx->update_pkt_params = s;
          }
        } // fi first data

        rxpacket_ptr   += bytes_received;
        read_ahead_ofs += bytes_received;
        lorarx->pkt_framecnt++; // debug counter        

      } // fi no event

    } while (lorarx->state != RX_DONE);

    ESP_LOGD(TAG, "####### loop #%lu completed. ########", loratx->frame_cnt + 1);

  } while ((loratx->state != LS_IDLE) && (loratx->frame_cnt <= (no_of_transmits-1))); // rof loop

  gpio_set_intr_type(lorarx->intr_pin, GPIO_INTR_DISABLE); 
  gpio_isr_handler_remove(lorarx->intr_pin);

  s = sx126x_set_rx_tx_fallback_mode(lorarx->ctx, SX126X_FALLBACK_STDBY_RC);  
  if (s) ESP_LOGE(TAG, "set rx fallback mode");
  s = sx126x_set_standby(lorarx->ctx, SX126X_STANDBY_CFG_RC);
  if (s) ESP_LOGE(TAG, "set back RX to standby RC");
  if (transmits_passed==no_of_transmits) {
    ESP_LOGI(TAG, "test passed.");
  } else {
    ESP_LOGW(TAG, "PER rate = %6.2f%% (%d/%d ok)", (no_of_transmits-transmits_passed) * 100.0 /  no_of_transmits, transmits_passed, no_of_transmits);
  }
  return transmits_passed==no_of_transmits? ESP_OK: ESP_FAIL;
}
