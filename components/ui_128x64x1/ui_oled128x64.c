/*
 * ui_xxx.c
 *
 * This source file belongs to project 'C2LoRaGW'.
 * (c) 2024, DO1FJN (Jan Alte)
 *
 *  Created on: 2024-04-10
 *     Project: C2 Lora Gateway
 *      Author: Jan Alte, DO1FJN
 */


#include "ui.h"

#include "OLED.h"
#include "fonts.h"
#include "font_exports.h"

#include "C2LORA.h"
#include "C2LORA_modes.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#define UI_TASK_STACK_SIZE      3072
#define UI_TASK_PRIORITY        4


#define UI_UPDATE_MODE          0x0001
#define UI_UPDATE_FREQ          0x0002
#define UI_UPDATE_STATE         0x0004

#define UI_UPDATE_RX_RSSI       0x0010
#define UI_UPDATE_RX_HEADER     0x0020


static const char *TAG = "UI";

static TaskHandle_t ui_task_hnd;

static const char *state_str[] = { 
  "--", "", "CL", "RX", "TX"
};

static tTextObj FreqDisp = {
  .font  = &font_Hack_13x24, .align = taLEFTBTM,
  .color = 1, .bgcol = 0,
  .box   = { .left = 0, .top = 0, .right = 110, .bottom = 18 }    // numbers only 16px
};

static tTextObj StateInfoDisp = {
.font  = &font_Hack_09x17, .align = taLEFTTOP,
.color = 1, .bgcol = 0,
.box   = { .left = 111, .top = 0, .right = 127, .bottom = 12 }
};

static tTextObj FreqInfoDisp = {
  .font  = &font_Hack_09x17, .align = taCENTER,
  .color = 1, .bgcol = 0,
  .box   = { .left = 111, .top = 13, .right = 127, .bottom = 25 }
};

static tTextObj ModeDisp = {
  .font  = &font_Hack_09x17, .align = taLEFTBTM,
  .color = 1, .bgcol = 0,
  .box   = { .left = 0, .top = 24, .right = 110, .bottom = 39 }
};

static tTextObj RxCallsignDisp = {
  .font  = &font_Hack_16x29, .align = taCENTERTOP,
  .color = 1, .bgcol = 0,
  .box   = { .left = 0, .top = 24, .right = 127, .bottom = 47+3 }
};

static tTextObj RxInfoDisp = {
  .font  = &font_Hack_09x17, .align = taCENTERBTM,
  .color = 1, .bgcol = 0,
  .box   = { .left = 0, .top = 48+3, .right = 127, .bottom = 63 }
};



esp_err_t ui_Init(void) {
  ui_task_hnd = NULL;
  esp_err_t err = OLED_Init();
  return err;
}



static inline void ui_update(uint32_t flags) {
  if (ui_task_hnd) xTaskNotify(ui_task_hnd, flags, eSetBits);
}


void C2LORA_ui_notify_mode_change(tC2LORA_mode new_mode) { 
  OLED_print_only(&ModeDisp, "%X:%s", new_mode, C2LORA_get_mode_name(new_mode));
  ui_update(UI_UPDATE_MODE);
}


void C2LORA_ui_notify_state_change(tC2LORA_state state) {
  OLED_print_only(&StateInfoDisp, "%.2s", state_str[state]);
  ui_update(UI_UPDATE_STATE);
}


void C2LORA_ui_notify_freq_change(int32_t freq_hz, int32_t freq_sft, tC2LORA_state state) {
  double freqf = (double)freq_hz / 1000000;
  OLED_print_only(&FreqDisp, "%8.4f", freqf);
  OLED_print_only(&FreqInfoDisp, "%c", freq_sft == 0? ' ': (freq_sft > 0? '+': '-'));
  ESP_LOGD(TAG, "FREQ = %8.4f | SHIFT = %ldHz | state = %d", freqf, freq_sft, state);
  ui_update(UI_UPDATE_FREQ);
  C2LORA_ui_notify_state_change(state);
}


void C2LORA_ui_notify_rx_header(bool valid, const char *callsign, const char *recipient, tKindOfSender kos, bool repeated, const char *area_code, const char *locator) {
  OLED_print_only(&RxCallsignDisp, "%.7s", callsign);
  ui_update(UI_UPDATE_RX_HEADER);
}


void C2LORA_ui_notify_rx_rssi(int8_t rssi, int8_t snr, int8_t signal) {
  OLED_print_only(&RxInfoDisp, "R%+4d SNR%+ddB", rssi, snr);
  ui_update(UI_UPDATE_RX_RSSI);
}


static void ui_update_task(void *thread_data) {
  uint32_t notify_value;
  vTaskDelay(pdMS_TO_TICKS(50));
  C2LORA_ui_notify_freq_change(C2LORA_get_frequency(), C2LORA_get_freqshift(), C2LORA_get_state());
  C2LORA_ui_notify_mode_change(C2LORA_get_mode());
  while (1) {
    if (xTaskNotifyWait(-1, -1, &notify_value, pdMS_TO_TICKS(15000))) {
      ESP_LOGD(TAG, "update flags: %04lXh", notify_value);

      if (notify_value & UI_UPDATE_MODE) {
        OLED_writeTextbox(&ModeDisp);
      }
      if (notify_value & UI_UPDATE_STATE) {
        OLED_writeTextbox(&StateInfoDisp);
      }
      if (notify_value & UI_UPDATE_FREQ) {
        OLED_writeTextbox(&FreqDisp);
        OLED_writeTextbox(&FreqInfoDisp);
      }
      if (notify_value & UI_UPDATE_RX_HEADER) {
        OLED_writeTextbox(&RxCallsignDisp);
      }
      if (notify_value & UI_UPDATE_RX_RSSI) {
        OLED_writeTextbox(&RxInfoDisp);
      }
      OLED_Update();

    } else {
      OLED_clearTextbox(&RxCallsignDisp);
      OLED_clearTextbox(&RxInfoDisp);
      OLED_writeTextbox(&ModeDisp);  
      OLED_Update();
    }
  }
}


esp_err_t ui_Start(void) {
  esp_err_t err;
  ui_task_hnd = NULL;
  err = xTaskCreate(ui_update_task, "UI_task", UI_TASK_STACK_SIZE, NULL, UI_TASK_PRIORITY, &ui_task_hnd)? ESP_OK: ESP_FAIL;
  ESP_RETURN_ON_ERROR(err, TAG, "no control task created.");
  return ESP_OK;
}


/*
#ifdef TFT_HOST
  tTextObj ProdDisp = {
    .font  = &font_HackBig40sspace,  .align = taCENTERBTM,
    .color = COLOR_WHITE, .bgcol = COLOR_BLUE,
    .box   = { .left = 0, .top = 0, .right = 319, .bottom = 79 }
  };
#endif

#ifdef TFT_HOST
  if (TFT_Init() != ESP_OK) {
    ESP_LOGE(TAG, "TFT init fails");
  }
#endif

#if TFT_HOST==SX126X_HOST  
  vTaskDelay(10);
#endif


#ifdef TFT_HOST  

  //INI_apply_configuration(default_wifi, strlen(default_wifi));
#if TFT_HOST==SX126X_HOST  
  vTaskDelay(pdMS_TO_TICKS(20));
#endif

  if (sspi_device_select(TFT_HOST, TFT_DEVICE_NUM) != ESP_OK) {
    ESP_LOGE(TAG, "error sel. display");
  }

  TFT_ClearAll();
//  TFT_FillRect(40, 40, TFT_X_SIZE / 2, TFT_Y_SIZE / 2, COLOR_RED);
//  vTaskDelay(100); 
  ESP_LOGD(TAG, "writing test-text...!");
  TFT_write(&ProdDisp, "C2LoraGW");
  TFT_SetBrightness(100);
#endif
*/
