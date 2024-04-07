/*
 * TFT.h
 *
 *  Created on: 19.01.2022
 *     Project: Lora-Gateway
 *      Author: Jan Alte, DO1FJN
 */

#ifndef _TFT_H_
#define _TFT_H_

#include "fonts.h"
#include "esp_err.h"

#define TEXT_MAX_CHARS		24

#define	COLOR_BLACK		0x0000
#define	COLOR_DARKGRAY		0x31C6
#define	COLOR_LIGHTGRAY		0x8C51
#define	COLOR_WHITE		0xFFFF

#define	COLOR_RED		0xF800
#define	COLOR_GREEN		0x07E0
#define	COLOR_BLUE		0x001F
#define COLOR_YELLOW		(COLOR_RED|COLOR_GREEN)
#define COLOR_BENTRUP		0x04F9	// 27 19
#define COLOR_BENTRUPSH		0x03D9

typedef struct {
  char		text[TEXT_MAX_CHARS];	// holding the UTF-8 string
  tTextAlign	align;
  tBox		box;
  unsigned short color, bgcol;
  const Font *	font;
} tTextObj;

typedef struct {
  unsigned char		width, height;
  unsigned char		icon_cnt;
  unsigned char		rsvd;
  const unsigned char *	data;
} tIcon;

typedef struct {
  unsigned short	width, height;
  const unsigned short *palette;
  const unsigned char *	data;
  unsigned int		size;
  unsigned char		palette_items;
} tRawPalettePicture;

typedef struct {
  const void *		data;
  unsigned int		size;
  unsigned short	bg_color;
} tPngPicture;


typedef struct {
  unsigned char		width, height;
  unsigned short *	pixel;
} tSprite;

typedef enum {
  ICON_GRDC, ICON_GRDF, ICON_RAMPC, ICON_RAMPF, ICON_TIME
} tUnitIcon;




esp_err_t	TFT_Init(void);

void	TFT_SetBrightness(unsigned short value);

void	TFT_ClearAll(void);


void	TFT_SetWindow(unsigned short x, unsigned short y, unsigned short w, unsigned short h);
void	TFT_RawWrite(const void *data, unsigned int size);

void	TFT_ReadRect(unsigned short *raw24data, unsigned short x, unsigned short y, unsigned short w, unsigned short h);

void	TFT_FillRect(unsigned short x, unsigned short y, unsigned short w, unsigned short h, unsigned short color);
void	TFT_FillArea(unsigned short x1, unsigned short y1, unsigned short x2, unsigned short y2, unsigned short color);

int	TFT_DrawPNG(const tPngPicture *ng, unsigned short x, unsigned short y);

void	TFT_writeTextbox(const tTextObj *t);
void	TFT_clearTextbox(const tTextObj *t);

int	TFT_print_only(tTextObj *t, const char *str, ...);
int	TFT_printf(tTextObj *t, const char *str, ...);
void	TFT_write(tTextObj *t, const char *str);

void	TFT_setColor(tTextObj *t, unsigned short color);

int	TFT_writeIcon(const tIcon *icon, tBox box, tTextAlign align, unsigned char no, unsigned short color, unsigned short bgcol);
int	TFT_writePicture(const tRawPalettePicture *pic, unsigned short x, unsigned short y);
int	TFT_writeSprite(const tSprite *sprite, unsigned short x, unsigned short y);

int	TFT_writeFastVLine(short x1, short y1, short x2, short y2, unsigned char width, short color, short bg);

int	TFT_getTextboxCharwidth(const tTextObj *t);


unsigned short TFT_mix565color(unsigned char weight, unsigned short color1, unsigned short color2);


#endif // _TFT_H_
