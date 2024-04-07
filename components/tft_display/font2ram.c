/*
 * font2ram.c
 *
 *  Created on: 10.05.2023
 *     Project: Comp2022
 *      Author: Jan Alte, (c) 2023 bentrup Industriesteuerungen
 */


#include "fonts.h"
#include "TFT.h"	// tSprite

#include <string.h>


/* fnt_calc565pixel()
 * get calculated color of a pixel
 */
static unsigned short fnt_calc565pixel(unsigned char pxbright, unsigned short color565, unsigned short bg565) {
  char bgbright = 15 - pxbright;

  int r1 = (color565 >> 11) * pxbright;
  int g1 = ((color565 >> 5) & 0x3F) * pxbright;
  int b1 = (color565 & 0x1F) * pxbright;

  int r2 = (bg565 >> 11) * bgbright;
  int g2 = ((bg565 >> 5) & 0x3F) * bgbright;
  int b2 = (bg565 & 0x1F) * bgbright;

  int rm = (r1+r2) / 15;
  int gm = (g1+g2) / 15;
  int bm = (b1+b2) / 15;

  unsigned short color = (rm << 11) | (gm << 5) | (bm);

  return color;
}


void FONT_calc_colors(short colbuf[16], unsigned short color565, unsigned short bg565) {
  if ((colbuf[0] == bg565) && (colbuf[15] == color565)) return;
  colbuf[0]  = __builtin_bswap16(bg565);
  colbuf[15] = __builtin_bswap16(color565);
  for (int c=1; c < 15; c++) {
    colbuf[c] = __builtin_bswap16(fnt_calc565pixel(c, color565, bg565));
  } // rof
}


static int fnt_getpixeladdr(short x, short y, short img_width) {
  return y * img_width + x;
}

static unsigned char fnt_getpixeldata(unsigned int pixeladdr, const unsigned char *img_data) {
  unsigned char pxbyte = img_data[pixeladdr >> 1];
  return pixeladdr & 1? pxbyte & 0xF: pxbyte >> 4;
}



static unsigned char fnt2ram_renderspace(tSprite *ram, unsigned char x, unsigned char y, const Font *font, short color) {
  unsigned short *buf;

  int space_width = (ram->width - x) < font->spaceWidth? ram->width - x: font->spaceWidth;
  int lines = (ram->height - y) < font->maxHeight? ram->height - y: font->maxHeight;
  int buf_next;

  buf = &ram->pixel[y * ram->width + x];
  buf_next = ram->width - space_width;
  for (; lines > 0; lines--) {

    for (int pxcnt = space_width; pxcnt > 0; pxcnt--, buf++) {
      buf[0] = color;
    }
    buf += buf_next;
  }
  return lines > 0? space_width: 0;
}


static unsigned char fnt2ram_renderchar(tSprite *ram, unsigned char x, unsigned char y, const Character *c, const Font *font, const short colors[16]) {
  unsigned short *buf;

  int pxdata_ptr  = fnt_getpixeladdr(c->x, c->y, font->imgWidth);
  int pxdata_next = font->imgWidth - c->width;

  int line;
  int line_upper = c->shiftY;
  int line_char	 = c->shiftY + c->height;
  int line_lower = (ram->height - y) < font->maxHeight? ram->height - y: font->maxHeight;
  // cnt: X related
  int cnt_shift	 = c->shiftX;
  int cnt_data	 = c->shiftX + c->width;
  int cnt_width  = cnt_data > c->advance? cnt_data:  c->advance;


  // clipping:
  if (cnt_shift >= (ram->width - x)) {
    return 0;
  }
  if (cnt_data > (ram->width - x)) {
    cnt_data  = ram->width - x;
    cnt_width = cnt_data;
    pxdata_next = font->imgWidth - cnt_data + cnt_shift;
  } else if (cnt_width > (ram->width - x)) {
    cnt_width = ram->width - x;
  }

  int buf_nxline = ram->width - cnt_width;

  buf = &ram->pixel[y * ram->width + x];
  for (line = 0; line < line_upper; line++) {
    for (int row=0; row < cnt_width; row++, buf++) buf[0] = colors[0];
    buf += buf_nxline;
  }

  for (; line < line_char; line++) {
    int pos = 0;
    for (; pos < cnt_shift; pos++, buf++) buf[0] = colors[0];		//fill blank space before char
    for (; pos < cnt_data; pos++, buf++) {
      short pixel = colors[fnt_getpixeldata(pxdata_ptr, font->data)];
      pxdata_ptr++;
      buf[0] = pixel;
    } // rof data
    pxdata_ptr += pxdata_next;						// nxt line from BMP reverse
    for (; pos < cnt_width; pos++, buf++) buf[0] = colors[0];		//fill blank space after
    buf += buf_nxline;
  } // rof

  for (; line < line_lower; line++) {
    for (int row=0; row < cnt_width; row++, buf++) buf[0] = colors[0];
    buf += buf_nxline;
  }

  return cnt_width;
}

