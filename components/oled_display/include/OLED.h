/*
 * OLED.h
 *
 *  Created on: 2024-04-10
 *     Project: Lora-Gateway
 *      Author: Jan Alte, DO1FJN
 */

#pragma once

#include "fonts.h"
#include "esp_err.h"

#include <stdbool.h>

#define TEXT_MAX_CHARS		16

typedef struct {
  char      		text[TEXT_MAX_CHARS];	// holding the UTF-8 string
  tTextAlign  	align;
  tBox	      	box;
  const Font *	font;
} tTextObj;

typedef struct {
  unsigned char		width, height;
  unsigned char		icon_cnt;
  unsigned char		rsvd;
  const unsigned char *	data;
} tIcon;

typedef struct {
  unsigned char		  width, height;
  unsigned short *	pixel;
} tSprite;


esp_err_t	OLED_Init(void);

esp_err_t OLED_InverseDisp(bool on_off);

void	OLED_SetContrast(unsigned char value);

void	OLED_ClearAll(void);

void  OLED_Update(void);

void	OLED_SetWindow(unsigned short x, unsigned short y, unsigned short w, unsigned short h);
void	OLED_RawWrite(const void *data, unsigned int size);

void	OLED_ReadRect(unsigned short *raw24data, unsigned short x, unsigned short y, unsigned short w, unsigned short h);

void	OLED_FillRect(unsigned char x, unsigned char y, unsigned char w, unsigned char h, bool invers);
void	OLED_FillArea(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2, bool invers);

void	OLED_writeTextbox(const tTextObj *t);
void	OLED_clearTextbox(const tTextObj *t);

int	  OLED_print_only(tTextObj *t, const char *str, ...);
int  	OLED_printf(tTextObj *t, const char *str, ...);
void	OLED_write(tTextObj *t, const char *str);

int	  OLED_writeIcon(const tIcon *icon, tBox box, tTextAlign align, unsigned char no, bool invers);
int	  OLED_writeSprite(const tSprite *sprite, unsigned short x, unsigned short y);

int	  OLED_writeFastVLine(short x1, short y1, short x2, short y2, unsigned char width, bool invers);

int	  OLED_getTextboxCharwidth(const tTextObj *t);
