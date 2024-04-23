/*
 * ui_xxx.c
 *
 * This source file belongs to project 'C2LoRaGW'.
 * (c) 2024, DO1FJN (Jan Alte)
 *
 *  Created on: 2024-04-15
 *     Project: C2 Lora Gateway
 *      Author: Jan Alte, DO1FJN
 */


#include "ui.h"

#include "hardware.h"

#include "TFT.h"
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


#define UI_UPDATE_MODE          0x0002
#define UI_UPDATE_FREQ          0x0004
#define UI_UPDATE_STATE         0x0008

#define UI_UPDATE_RX_RSSI       0x0010
#define UI_UPDATE_RX_HEADER     0x0020


static const char *TAG = "UI";

static TaskHandle_t ui_task_hnd;

static const char *state_str[] = { 
  "--", "", "CL", "RX", "TX"
};

#define FREQ_DISP_WIDTH     (TFT_X_SIZE-36)
#define FREQ_DISP_kHZ10_W   180

static tTextObj FreqDispMHz = {
  .font  = &font_HackStd20, .align = taRIGHTBTM,
  .color = COLOR_WHITE, .bgcol = COLOR_BLACK,
  .box   = { .left = 0, .top = 0, .right = 59, .bottom = 59 }
};

static tTextObj FreqDispkHz = {
  .font  = &font_HackBig40sspace, .align = taCENTERBTM,
  .color = COLOR_WHITE, .bgcol = COLOR_BLACK,
  .box   = { .left = 60, .top = 0, .right = FREQ_DISP_kHZ10_W + 59, .bottom = 59 }
};

static tTextObj FreqDispHz = {
  .font  = &font_HackStd20, .align = taLEFTBTM,
  .color = COLOR_WHITE, .bgcol = COLOR_BLACK,
  .box   = { .left = FREQ_DISP_kHZ10_W + 60, .top = 0, .right = FREQ_DISP_WIDTH-1, .bottom = 59 }
};

static tTextObj StateInfoDisp = {
.font  = &font_HackSmall12, .align = taCENTERTOP,
.color = COLOR_WHITE, .bgcol = COLOR_BLACK,
.box   = { .left = FREQ_DISP_WIDTH, .top = 0, .right = TFT_X_SIZE-1, .bottom = 29 }
};

static tTextObj FreqInfoDisp = {
  .font  = &font_HackStd20, .align = taCENTER,
  .color = COLOR_WHITE, .bgcol = COLOR_BLACK,
  .box   = { .left = FREQ_DISP_WIDTH, .top = 30, .right = TFT_X_SIZE-1, .bottom = 59 }
};

static tTextObj ModeDisp = {
  .font  = &font_HackSmall12, .align = taLEFTBTM,
  .color = COLOR_WHITE, .bgcol = COLOR_BLACK,
  .box   = { .left = 0, .top = 60, .right = FREQ_DISP_WIDTH, .bottom = 81 }
};

static tTextObj RxCallsignDisp = {
  .font  = &font_HackBig40sspace, .align = taCENTERTOP,
  .color = COLOR_WHITE, .bgcol = COLOR_BLUE,
  .box   = { .left = 0, .top = 82, .right = TFT_X_SIZE, .bottom = 147 }
};

static tTextObj RxInfoDisp = {
  .font  = &font_HackSmall12, .align = taCENTERBTM,
  .color = COLOR_YELLOW, .bgcol = COLOR_DARKGRAY,
  .box   = { .left = 0, .top = 148, .right = TFT_X_SIZE, .bottom = TFT_Y_SIZE-1 }
};



esp_err_t ui_Init(void) {
  ui_task_hnd = NULL;
  esp_err_t err = TFT_Init();
  return err;
}



static inline void ui_update(uint32_t flags) {
  if (ui_task_hnd) xTaskNotify(ui_task_hnd, flags, eSetBits);
}


void C2LORA_ui_notify_mode_change(tC2LORA_mode new_mode) { 
  TFT_print_only(&ModeDisp, "%X: %s", new_mode, C2LORA_get_mode_name(new_mode));
  ui_update(UI_UPDATE_MODE);
}


void C2LORA_ui_notify_state_change(tC2LORA_state state) {
  TFT_print_only(&StateInfoDisp, "%.2s", state_str[state]);
  ui_update(UI_UPDATE_STATE);
}


void C2LORA_ui_notify_freq_change(int32_t freq_hz, int32_t freq_sft, tC2LORA_state state) {
  int32_t freq_MHz = freq_hz / 1000000L;
  int32_t freq_kHz10;
  freq_hz   %= 1000000L;
  freq_kHz10 = freq_hz / 100;
  freq_hz   %= 100;
  TFT_print_only(&FreqDispMHz, "%ld", freq_MHz);
  TFT_print_only(&FreqDispkHz, ".%04ld", freq_kHz10);
  TFT_print_only(&FreqDispHz, "%02ld", freq_hz);
  TFT_print_only(&FreqInfoDisp, "%c", freq_sft == 0? ' ': (freq_sft > 0? '+': '-'));
  ESP_LOGD(TAG, "FREQ = %ld.%04ld | SHIFT = %ldHz | state = %d", freq_MHz, freq_kHz10, freq_sft, state);
  ui_update(UI_UPDATE_FREQ);
  C2LORA_ui_notify_state_change(state);
}


void C2LORA_ui_notify_rx_header(bool valid, const char *callsign, const char *recipient, tKindOfSender kos, bool repeated, const char *area_code, const char *locator) {
  TFT_print_only(&RxCallsignDisp, "%.7s", callsign);
  ui_update(UI_UPDATE_RX_HEADER);
}


void C2LORA_ui_notify_rx_rssi(int8_t rssi, int8_t snr, int8_t signal) {
  TFT_print_only(&RxInfoDisp, "RSSI %+4ddB SNR %+ddB", rssi, snr);
  ui_update(UI_UPDATE_RX_RSSI);
}


static void ui_update_task(void *thread_data) {
  
  vTaskDelay(pdMS_TO_TICKS(50));

  TFT_select();
  TFT_ClearAll();
  //TFT_FillRect(40, 40, TFT_X_SIZE / 2, TFT_Y_SIZE / 2, COLOR_RED);

  TFT_clearTextbox(&RxCallsignDisp);
  TFT_clearTextbox(&RxInfoDisp);
  TFT_SetBrightness(100);
  TFT_wait4completion();

  C2LORA_ui_notify_mode_change(C2LORA_get_mode());
  C2LORA_ui_notify_freq_change(C2LORA_get_frequency(), C2LORA_get_freqshift(), C2LORA_get_state());
 
  while (1) {
    bool     spi_operation_running = false;
    uint32_t notify_value = 0;
    BaseType_t got_notification = xTaskNotifyWait(0, 0x0000ffffUL, &notify_value, pdMS_TO_TICKS(15000));

    if (TFT_select() != ESP_OK) {   
      continue;
    }
    
    if (got_notification == pdPASS) {
      ESP_LOGD(TAG, "update flags: %04lXh", notify_value);

      if (notify_value & UI_UPDATE_MODE) {
        TFT_writeTextbox(&ModeDisp);
      }
      if (notify_value & UI_UPDATE_STATE) {
        TFT_writeTextbox(&StateInfoDisp);
      }
      if (notify_value & UI_UPDATE_FREQ) {
        TFT_writeTextbox(&FreqDispMHz);
        TFT_writeTextbox(&FreqDispkHz);
        TFT_writeTextbox(&FreqDispHz);
        TFT_writeTextbox(&FreqInfoDisp);
      }
      if (notify_value & UI_UPDATE_RX_HEADER) {
        TFT_writeTextbox(&RxCallsignDisp);
      }
      if (notify_value & UI_UPDATE_RX_RSSI) {
        TFT_writeTextbox(&RxInfoDisp);
      }
      spi_operation_running = true;

    } else {
      TFT_clearTextbox(&RxCallsignDisp);
      TFT_clearTextbox(&RxInfoDisp);
      spi_operation_running = true;
      ESP_LOGV(TAG, "high stack water mark is %u\n", uxTaskGetStackHighWaterMark(NULL));        
    }
    
    if (!spi_operation_running) continue;

    TFT_wait4completion();
    vTaskDelay(pdMS_TO_TICKS(40));

  } // ehliw forever
}


esp_err_t ui_Start(void) {
  esp_err_t err;
  ui_task_hnd = NULL;
  err = xTaskCreate(ui_update_task, "UI_task", UI_TASK_STACK_SIZE, NULL, UI_TASK_PRIORITY, &ui_task_hnd)? ESP_OK: ESP_FAIL;
  ESP_RETURN_ON_ERROR(err, TAG, "no control task created.");
  return ESP_OK;
}

