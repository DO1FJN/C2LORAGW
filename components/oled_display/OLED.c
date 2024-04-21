/*
 * OLED.c
 *
 * This source file belongs to project 'C2LoRaGW'.
 * (c) 2024, DO1FJN (Jan Alte)
 *
 *  Created on: 2024-04-10
 *     Project: C2 Lora Gateway
 *      Author: Jan Alte, DO1FJN
 */

#include "OLED.h"
#include "hardware.h"

#include "SSD1306.h"
#include "fonts.h"
#include "font_exports.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "esp_log.h"
#include "esp_check.h"

static const char * TAG = "OLED";


#define MAX_DISP_BLOCKING_MS		100

// ToDo system independent
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifndef NO_TFT_ACCESS_LOCK
#define oled_init_mutex()      disp_access = xSemaphoreCreateMutex()
#define oled_lock()            xSemaphoreTake(disp_access, pdMS_TO_TICKS(MAX_DISP_BLOCKING_MS))
#define oled_unlock()          xSemaphoreGive(disp_access)
#define oled_delete_mutex()    { vSemaphoreDelete(disp_access); disp_access = NULL; }
#else
#define oled_init_mutex()
#define oled_lock()
#define oled_unlock()
#define oled_delete_mutex()
#endif

static SemaphoreHandle_t disp_access = NULL;

esp_err_t OLED_Init(void) {  
  esp_err_t err = SSD1306_Init(OLED_Y_SIZE);
  oled_init_mutex();
  ESP_RETURN_ON_ERROR(err, TAG, "can't init OLED display driver (%s)", esp_err_to_name(err));  
  return SSD1306_OnOff(true);
}


esp_err_t OLED_Enable(bool on_off) {
  esp_err_t err = ESP_ERR_NOT_FINISHED;
  if (oled_lock()) {
    err = SSD1306_OnOff(on_off);
  }
  oled_unlock();
  return err;
}


esp_err_t OLED_InverseDisp(bool on_off) {
  esp_err_t err = ESP_ERR_NOT_FINISHED;
  if (oled_lock()) {
    err = SSD1306_InverseDisp(on_off);
  }
  oled_unlock();
  return err;
}


void OLED_SetContrast(unsigned char value) {
  esp_err_t err = ESP_ERR_NOT_FINISHED;
  if (oled_lock()) {
    err = SSD1306_Contrast(value);
  }
  oled_unlock();
  if (err != ESP_OK) { 
    ESP_LOGW(TAG, "can'T set contrast to %d.", value);
  }
}


void OLED_Update(void) {
  if (!oled_lock()) return;
  SSD1306_UpdateAll();
  oled_unlock();
}


void OLED_FillRect(unsigned char x, unsigned char y, unsigned char w, unsigned char h, bool invers) {
  if (!oled_lock()) return;
  SSD1306_FillRect(x, y, w, h, invers);
  oled_unlock();
}



static int oled_clipBox(tBox *box) {
  if (box->left < 0) box->left = 0; else if (box->left >= OLED_X_SIZE) return 0;
  if (box->top < 0) box->top = 0; else if (box->top >= OLED_Y_SIZE) return 0;
  if (box->right >= OLED_X_SIZE) box->right = OLED_X_SIZE-1;
  if (box->bottom >= OLED_Y_SIZE) box->bottom = OLED_Y_SIZE-1;
  return 1;
}


void OLED_writeTextbox(const tTextObj *t) {
  if ((t == NULL) || !oled_lock()) return;  
  tBox clippedBox = t->box;
  if (oled_clipBox(&clippedBox)) {
#ifdef DEFAULT_FONT     
    const Font *font = t->font == NULL? DEFAULT_FONT: t->font;
    FONT_printStr(font, t->text, clippedBox, t->align, t->pixel, 0);
#else
    if (t->font) FONT_printStr(t->font, t->text, clippedBox, t->align, t->pixel, 0);
#endif            
  } // fi clipped
  oled_unlock();
}


void OLED_clearTextbox(const tTextObj *t) {
  if ((t == NULL) || !oled_lock()) return;   
  tBox clippedBox = t->box;
  if (oled_clipBox(&clippedBox)) {
    SSD1306_FillRect(clippedBox.left, clippedBox.top, clippedBox.right - clippedBox.left + 1, clippedBox.bottom - clippedBox.top + 1, t->pixel == 0);
  }
  oled_unlock();
}



int OLED_print_only(tTextObj *t, const char *str, ...) {
  int len;
  if ((t == NULL) || (str == NULL)) return 0;
  va_list args;
  va_start(args, str);
  len = vsnprintf(t->text, TEXT_MAX_CHARS, str, args);
  va_end(args);
  return len;
}


/**
 * OLED_printf() print like 'printf' but directly into the tTextObj buffer (much more effective)
 * @param t:	textobject
 * @param str:	Format-String
 * @retval:	length of written chars (returns 0 on buffer-overflow or <0 at errs)
 */
int OLED_printf(tTextObj *t, const char *str, ...) {
  int len;
  if ((t == NULL) || (str == NULL)) return 0;
  va_list args;
  va_start(args, str);
  len = vsnprintf(t->text, TEXT_MAX_CHARS, str, args);
  va_end(args);
  if (len > 0) {
    OLED_writeTextbox(t);
  } else {
    OLED_clearTextbox(t);
  }
  return len;
}
