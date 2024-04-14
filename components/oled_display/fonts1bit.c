/*
 * fonts.c
 *
 *  Created on: 10.05.2022
 *     Project: C2LORAGW
 *      Author: Jan Alte, DO1FJN
 *
 * Report:
 * 2022-05-12	First version.
 * 2022-06-21	Reduced ram version (single-buffer)
 * 2022-06-21	1-bit version for bootloader
 * 2024-04-14 rework for OLED display Q'n'D
 */

#include "fonts.h"

// ToDo thread safe API in SSD1306
#include "SSD1306.h"

#include "hardware.h"

#include <string.h>


typedef struct {
  short left, width;
  short upper, height;
} tClipArea;


static int fnt_getpixeladdr(short x, short y, short img_width) {
  return y * img_width + x;
}

static unsigned char fnt_getpixeldata(unsigned int pixeladdr, const unsigned char *img_data) {
  unsigned char pxbyte = img_data[pixeladdr >> 3];
  return (pxbyte >> (7 -(pixeladdr & 7))) & 1;
}


static unsigned char fnt_renderspace(const tClipArea *clip, const Font *font) {
  int buf_width = font->spaceWidth - clip->left;
  if (buf_width > clip->width) buf_width = clip->width;
  int lines   = (clip->height < font->maxHeight? clip->height: font->maxHeight) - clip->upper;
  SSD1306_FillNextRect(buf_width, lines, 0);
  return lines > 0? buf_width: 0;
}


static unsigned char fnt_renderchar(const Character *c, const tClipArea *clip, const Font *font) {

  int pxdata_ptr  = fnt_getpixeladdr(c->x, c->y, font->imgWidth);
  int pxdata_next = font->imgWidth - c->width;

  // line: Y-related
  int line       = clip->upper;
  int line_upper = c->shiftY;
  int line_char	 = c->shiftY + c->height;
  int line_lower = clip->height < font->maxHeight? clip->height: font->maxHeight;
  // cnt: X related
  int cnt_shift	 = c->shiftX;
  int cnt_data	 = c->shiftX + c->width;
  int cnt_width  = cnt_data > c->advance? cnt_data:  c->advance;

  // clipping:
  if (cnt_shift >= (clip->width - clip->left)) {
    return 0;
  }
  if (cnt_data > (clip->width - clip->left)) {
    cnt_data  = clip->width - clip->left;
    cnt_width = cnt_data;
    pxdata_next = font->imgWidth - cnt_data + cnt_shift;
  } else if (cnt_width > (clip->width - clip->left)) {
    cnt_width = clip->width - clip->left;
  }
  //printf("clip left: %d | clip w: %d, cnt shift:%d, cnt data:%d, cnt w:%d\n", clip->left, clip->width, cnt_shift, cnt_data, cnt_width);
  // ToDo upper clip clips in chars -> pxdata_ptr!
  if (line_upper > line) {
    SSD1306_FillNextRect(cnt_width, line_upper - line, 0);
  }
  for (line = line_upper; line < line_char; line++) {
    int pos = clip->left;
    for (; pos < cnt_shift; pos++) SSD1306_SetNextPixel(0);		//fill blank space before char
    for (; pos < cnt_data; pos++) {
      SSD1306_SetNextPixel(fnt_getpixeldata(pxdata_ptr, font->data));      
      pxdata_ptr++;
    } // rof data
    pxdata_ptr += pxdata_next;						// nxt line from BMP reverse
    for (; pos < cnt_width; pos++) SSD1306_SetNextPixel(0);		//fill blank space after
    SSD1306_SetNextLine();
  } // rof
  if (line_lower > line) {
    SSD1306_FillNextRect(cnt_width, line_lower - line, 0);
  }
  return cnt_width;
}



static int fnt_writeClipped(const char *text, uint8_t *x_start, uint8_t y, uint8_t x_end, tClipArea *clip, const Font *font, bool invers) {
  int ccnt;
  unsigned int   xpos = x_start[0], ypos = y;
  unsigned short fdisp_height = (clip->height > font->maxHeight)? font->maxHeight: clip->height;
  for (ccnt=0; (text[0] != 0) && (xpos < x_end); ccnt++) {
    unsigned int  c   = fnt_readUTF8(&text);
    unsigned char idx = fnt_getIndex(font, c);
    clip->width = x_end - xpos + 1;
    SSD1306_SetWindow(xpos, ypos, x_end, ypos + fdisp_height - 1, invers);
    //printf("clipped box (h=%d): %d,%d - %d,%d\t\t", fdisp_height, xpos, ypos, x_end, ypos + fdisp_height - 1);
    int chrW = idx < font->charCount? fnt_renderchar(&font->chars[idx], clip, font): fnt_renderspace(clip, font);
    if (chrW == 0) continue;
    xpos += chrW;
  } // rof;
  x_start[0] = xpos;
  return ccnt;
}



int	fnt_writeStr(const char *text, short x, short y, const Font *font) {
  tClipArea clip = { 0, 0, 0, OLED_Y_SIZE - y };
  uint8_t x_pos = x;
  return fnt_writeClipped(text, &x_pos, y, OLED_X_SIZE, &clip, font, false);
}



int FONT_printStr(const Font *font, const char *text, tBox box, tTextAlign align, unsigned short color, unsigned short bgcol) {
  int ccnt, fntw;
  //fnt_calc_colors(color, bgcol);
  tClipArea clip    = { 0, box.right - box.left + 1, 0, box.bottom - box.top + 1 };
  tBox      borders = { 0, 0, 0, 0 };
  uint8_t xpos = box.left, ypos = box.top;
  uint8_t fdisp_height = (clip.height > font->maxHeight)? font->maxHeight: clip.height;

  switch (align) {
  case taCENTERTOP:
  case taRIGHTTOP:
    fntw = FONT_getWidth(font, text);
    if (fntw < clip.width) {
      if (align == taCENTERTOP) xpos += (clip.width - fntw) >> 1; else xpos = (box.right + 1) - fntw;
    }
    // fall through
  case taLEFTTOP:	// do nothing
    borders.bottom = clip.height - fdisp_height;
    break;

  case taRIGHTBTM:
    fntw = FONT_getWidth(font, text);
    if (fntw < clip.width) {
      xpos = (box.right + 1) - fntw;
    }
    // fall through
  case taLEFTBTM:
    borders.top = clip.height - fdisp_height;
    ypos += borders.top;
    break;

  case taCENTERBTM:
    borders.top = clip.height - fdisp_height;
    // fall through
  case taCENTER:
  case taRIGHT:
    fntw = FONT_getWidth(font, text);
    if (fntw < clip.width) {
      if (align == taRIGHT) xpos = (box.right + 1) - fntw; else xpos += (clip.width - fntw) >> 1;
    }
    // fall through
  case taLEFT:
    if (align != taCENTERBTM) {
      borders.top    = (clip.height - fdisp_height) >> 1;
      borders.bottom = clip.height - fdisp_height - borders.top;
    }
    ypos += borders.top;
    break;
  } // hctiws

  borders.left = xpos - box.left;
  if (borders.top > 0) {
    SSD1306_FillRect(box.left, box.top, box.right - box.left + 1, borders.top, bgcol != 0);
  }
  if (borders.left > 0) {
    SSD1306_FillRect(box.left, box.top + borders.top, borders.left, fdisp_height, bgcol != 0);
  }

  ccnt = fnt_writeClipped(text, &xpos, ypos, box.right, &clip, font, color == 0);

  borders.right = box.right - xpos + 1;
  if (borders.right > 0) {
    SSD1306_FillRect(xpos, box.top + borders.top, borders.right, fdisp_height, bgcol != 0);
  }
  if (borders.bottom > 0) {
    SSD1306_FillRect(box.left, box.bottom - borders.bottom + 1, box.right - box.left + 1, borders.bottom, bgcol != 0);
  }

  return ccnt;
}
