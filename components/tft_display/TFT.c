/*
 * TFT.c
 *
 * This header file belongs to project 'C2LoRaGW'.
 * (c) 2024, DO1FJN (Jan Alte)
 *
 *  Created on: 01.01.2024
 *     Project: C2 Lora Gateway
 *      Author: Jan Alte, DO1FJN
 */

#include "TFT.h"
#include "hardware.h"

#include "Backlight.h"
#include "ST7789.h"
#include "fonts.h"
#include "DynRender.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>


#include "esp_log.h"

static const char * TAG = "TFT";


#define MAX_DISP_BLOCKING_MS		200

// ToDo system independent
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifndef NO_TFT_ACCESS_LOCK
#define tft_init_mutex()      disp_access = xSemaphoreCreateMutex()
#define tft_lock()            xSemaphoreTake(disp_access, pdMS_TO_TICKS(MAX_DISP_BLOCKING_MS))
#define tft_unlock()          xSemaphoreGive(disp_access)
#define tft_delete_mutex()    { vSemaphoreDelete(disp_access); disp_access = NULL; }
#else
#define tft_init_mutex()
#define tft_lock()
#define tft_unlock()
#define tft_delete_mutex()
#endif



int ICON_write(const tIcon *icon, tBox box, tTextAlign align, unsigned char no, unsigned short color, unsigned short bgcol);


static SemaphoreHandle_t disp_access = NULL;

bool TFT_Active;



esp_err_t TFT_Init(void) {
  esp_err_t init_ret;
  tft_init_mutex();	// create Mutex
  tft_lock();
  //if (disp_access == NULL) return ESP_FAIL;
  Backlight_Init();
  TFT_Active = false;
  init_ret = ST7789_Init();
  tft_unlock();
  return init_ret;
}


inline esp_err_t TFT_select(void) {
  esp_err_t err = ST7789_select_spi();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "error SPI select display: %s", esp_err_to_name(err));
  }
  return err;
}


inline int TFT_wait4completion(void) {
  int res;
  if (!tft_lock()) return 0;
  res = ST7789_WaitWrComplete();
  tft_unlock();
  return res;
}


void TFT_SetBrightness(unsigned short value) {
  float br = powf(value * 0.464158883362, 1.8);
  Backlight_setDim(br > MAX_BRIGHTNESS? MAX_BRIGHTNESS: br);
  if (!tft_lock()) return;
  if ((TFT_Active != (value > 0))) {
    TFT_Active = value > 0;
    ST7789_OnOff(TFT_Active);    
  }
  tft_unlock();
}


void TFT_ClearAll(void) {  
  if (!tft_lock()) return;
  ESP_LOGD(TAG, "clear screen");
  ST7789_FillBox(0, 0, TFT_X_SIZE-1, TFT_Y_SIZE-1, COLOR_BLACK);
  tft_unlock();
}

void TFT_SetWindow(unsigned short x, unsigned short y, unsigned short w, unsigned short h) {
  if (!tft_lock()) return;
  ST7789_SetWindow(x, y, x + w - 1, y + h - 1);
  tft_unlock();
}

void TFT_RawWrite(const void *data, unsigned int size) {
  if (!tft_lock()) return;
  ST7789_Write(data, size);
  tft_unlock();
}


void TFT_FillRect(unsigned short x, unsigned short y, unsigned short w, unsigned short h, unsigned short color) {
  if (!tft_lock()) return;
  ST7789_FillRect(x, y, w, h, color);
  tft_unlock();
}

void TFT_ReadRect(unsigned short *raw24data, unsigned short x, unsigned short y, unsigned short w, unsigned short h) {
  if (!tft_lock()) return;
  ST7789_ReadRect(raw24data, x, y, w, h);
  tft_unlock();
}

void TFT_FillArea(unsigned short x1, unsigned short y1, unsigned short x2, unsigned short y2, unsigned short color) {
  if (!tft_lock()) return;
  ST7789_FillBox(x1, y1, x2, y2, color);
  tft_unlock();
}



int TFT_DrawPNG(const tPngPicture *png, unsigned short x, unsigned short y) {
  int res = 0;
  if (!tft_lock()) return 0;
  res = PICTURE_drawPNG(png, x, y);
  tft_unlock();
  return res;
}


static int tft_clipBox(tBox *box) {
  if (box->left < 0) box->left = 0; else if (box->left >= TFT_X_SIZE) return 0;
  if (box->top < 0) box->top = 0; else if (box->top >= TFT_Y_SIZE) return 0;
  if (box->right >= TFT_X_SIZE) box->right = TFT_X_SIZE-1;
  if (box->bottom >= TFT_Y_SIZE) box->bottom = TFT_Y_SIZE-1;
  return 1;
}


void TFT_writeTextbox(const tTextObj *t) {
  if ((t == NULL) || !tft_lock()) return;  
  ESP_LOGD(TAG, "write textbox");    
  tBox clippedBox = t->box;
  if (tft_clipBox(&clippedBox)) {
#ifdef DEFAULT_FONT     
    const Font *font = t->font == NULL? DEFAULT_FONT: t->font;
    FONT_printStr(font, t->text, clippedBox, t->align, t->color, t->bgcol);
#else
    if (t->font) FONT_printStr(t->font, t->text, clippedBox, t->align, t->color, t->bgcol);
#endif            
  } // fi clipped
  tft_unlock();
}


void TFT_clearTextbox(const tTextObj *t) {
  if ((t == NULL) || !tft_lock()) return;   
  ESP_LOGD(TAG, "clear texbox");    
  tBox clippedBox = t->box;
  if (tft_clipBox(&clippedBox)) {
    ST7789_FillBox(clippedBox.left, clippedBox.top, clippedBox.right, clippedBox.bottom, t->bgcol);
  }
  tft_unlock();
}



int TFT_print_only(tTextObj *t, const char *str, ...) {
  int len;
  if ((t == NULL) || (str == NULL)) return 0;
  va_list args;
  va_start(args, str);
  len = vsnprintf(t->text, TEXT_MAX_CHARS, str, args);
  va_end(args);
  return len;
}


/**
 * TFT_printf() print like 'printf' but directly into the tTextObj buffer (much more effective)
 * @param t:	textobject
 * @param str:	Format-String
 * @retval:	length of written chars (returns 0 on buffer-overflow or <0 at errs)
 */
int TFT_printf(tTextObj *t, const char *str, ...) {
  int len;
  if ((t == NULL) || (str == NULL)) return 0;
  va_list args;
  va_start(args, str);
  len = vsnprintf(t->text, TEXT_MAX_CHARS, str, args);
  va_end(args);
  if (len > 0) {
    TFT_writeTextbox(t);
  } else {
    TFT_clearTextbox(t);
  }
  return len;
}


void TFT_write(tTextObj *t, const char *str) {
  if ((t == NULL) || (str == NULL)) return;
  memcpy(t->text, str, TEXT_MAX_CHARS);
  if (t->text[0] != 0) {
    t->text[TEXT_MAX_CHARS-1] = 0;
    TFT_writeTextbox(t);
  } else {
    TFT_clearTextbox(t);
  }
}


void TFT_setColor(tTextObj *t, unsigned short color) {
  if ((t == NULL)) return;
  if (t->color != color) {
    t->color = color;
    TFT_writeTextbox(t);
  }
  //t->bgcol = bgcol;
}


int TFT_writeIcon(const tIcon *icon, tBox box, tTextAlign align, unsigned char no, unsigned short color, unsigned short bgcol) {
  int res = -1;
  if (!tft_lock()) return -1;
  if ((color == bgcol) || (icon == NULL)) {
    ST7789_FillBox(box.left, box.top, box.right, box.bottom, bgcol);
    res = 0;
  } else {
    res = ICON_write(icon, box, align, no, color, bgcol);
  }
  tft_unlock();
  return res;
}

int TFT_writePicture(const tRawPalettePicture *pic, unsigned short x, unsigned short y) {
  int res = -1;
  if (!tft_lock()) return -1;
  res = PICTURE_write(pic, x, y);
  tft_unlock();
  return res;
}

int TFT_writeSprite(const tSprite *sprite, unsigned short x, unsigned short y) {
  int res = -1;
  if ((sprite == NULL) || !tft_lock()) return -1;
  ST7789_Sprite(x, y, sprite->width, sprite->height, sprite->pixel);
  tft_unlock();
  return res;
}


int TFT_writeFastVLine(short x1, short y1, short x2, short y2, unsigned char width, short color, short bg) {
  uint16_t *pixelLines[2];
  uint32_t lineSel;
  short z;
  unsigned int w;
  float start_x, x_inc;

  x_inc = (float)(x2 - x1) / (y2 - y1);
  if (x1 > x2) { z = x1; x1 = x2; x2 = z; }
  if (y1 > y2) { z = y1; y1 = y2; y2 = z; }

  w = (x2 - x1) + width;
  start_x = x_inc < 0.0? x2 - x1: 0;

  pixelLines[0] = pvPortMalloc(w * (2 * sizeof(uint16_t)));
  pixelLines[1] = pixelLines[0] + w; //pvPortMalloc(w * sizeof(uint16_t));
  lineSel = 0;
  ST7789_SetWindow(x1, y1, x1 + w - 1, y2);
  for (int line = y1; line <= y2; line++, lineSel ^= 1, start_x += x_inc) {
    // render line
    int lx1 = roundf(start_x);
    int lx2 = lx1 + width;
    for (int px = 0; px < w; px++) {
      pixelLines[lineSel][px] = (px >= lx1) && (px < lx2)? color: bg;
    }
    // draw line
    ST7789_Write(pixelLines[lineSel], w);
  }
  //vPortFree(pixelLines[1]);
  vPortFree(pixelLines[0]);
  return 0;
}


int TFT_getTextboxCharwidth(const tTextObj *t) {
  int cwdh;
  if (t == NULL) return 0;
  cwdh = t->box.right + 1 - t->box.left;	// width in pixel
  cwdh /= t->font->maxWidth;
  return cwdh;
}


unsigned short TFT_mix565color(unsigned char weight, unsigned short color1, unsigned short color2) {
    unsigned inv_w = 255 - weight;

    inv_w = (weight >= 128)? 256 - weight: 128;
    if (weight > 128) weight = 128;

    int r1 = (color2 >> 11) * weight;
    int g1 = ((color2 >> 5) & 0x3F) * weight;
    int b1 = (color2 & 0x1F) * weight;

    int r2 = (color1 >> 11) * inv_w;
    int g2 = ((color1 >> 5) & 0x3F) * inv_w;
    int b2 = (color1 & 0x1F) * inv_w;

    int rm = (r1+r2) >> 7;
    int gm = (g1+g2) >> 7;
    int bm = (b1+b2) >> 7;

    unsigned short color = (rm << 11) | (gm << 5) | (bm);

    return color;
}

