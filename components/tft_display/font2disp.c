/*
 * fonts.c
 *
 *  Created on: 10.05.2022
 *     Project: Comp2022
 *      Author: Jan Alte, (c) 2022 bentrup Industriesteuerungen
 *
 * Report:
 * 2022-05-12	First version.
 */

#include "fonts.h"
#include "hardware.h"

#include "ST7789.h"
#include "TFT.h"

#include <string.h>

#include "esp_attr.h"

#define FONT_MAX_CHAR_WIDTH		  48	// BIGnumbers -> 48x62 -> 11.6KiB RAM Buffer
#define FONT_MAX_CHAR_HEIGHT		62

#define FONT_MAX_PIXEL_BUFSIZE		(FONT_MAX_CHAR_WIDTH * FONT_MAX_CHAR_HEIGHT)

typedef struct {
   short	  colors[16];
   unsigned short pixel[2][FONT_MAX_PIXEL_BUFSIZE];
   int		  pixel_parms;
} tFontRenderBuf;

typedef struct {
  short left, width;
  short upper, height;
} tClipArea;


static tFontRenderBuf	DMA_ATTR fontRB;


static unsigned short *fnt_flipBuffer(void) {
 unsigned short *char_buf = fontRB.pixel[fontRB.pixel_parms & 1];
 fontRB.pixel_parms ^= 1;
 return char_buf;
}


void fnt_calc_colors(unsigned short color565, unsigned short bg565) {
  FONT_calc_colors(fontRB.colors, color565, bg565);
}

static int fnt_getpixeladdr(short x, short y, short img_width) {
  return y * img_width + x;
}

static unsigned char fnt_getpixeldata(unsigned int pixeladdr, const unsigned char *img_data) {
  unsigned char pxbyte = img_data[pixeladdr >> 1];
  return pixeladdr & 1? pxbyte & 0xF: pxbyte >> 4;
}



static unsigned char fnt_renderspace(const tClipArea *clip, const Font *font) {
  unsigned short * buf = fontRB.pixel[fontRB.pixel_parms & 1];
  int buf_width = font->spaceWidth - clip->left;
  if (buf_width > clip->width) buf_width = clip->width;
  int lines   = (clip->height < font->maxHeight? clip->height: font->maxHeight) - clip->upper;
  int countpx = lines * buf_width;
  for (; countpx > 0; countpx--, buf++) buf[0] = fontRB.colors[0];
  return lines > 0? buf_width: 0;
}


static unsigned char fnt_renderchar(const Character *c, const tClipArea *clip, const Font *font) {
  unsigned short *buf = fontRB.pixel[fontRB.pixel_parms & 1];

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

  for (; line < line_upper; line++) {
    for (int row=0; row < cnt_width; row++, buf++) buf[0] = fontRB.colors[0];
  }

  for (; line < line_char; line++) {
    int pos = clip->left;
    for (; pos < cnt_shift; pos++, buf++) buf[0] = fontRB.colors[0];		//fill blank space before char
    for (; pos < cnt_data; pos++, buf++) {
      short pixel = fontRB.colors[fnt_getpixeldata(pxdata_ptr, font->data)];
      pxdata_ptr++;
      buf[0] = pixel;
    } // rof data
    pxdata_ptr += pxdata_next;						// nxt line from BMP reverse
    for (; pos < cnt_width; pos++, buf++) buf[0] = fontRB.colors[0];		//fill blank space after
  } // rof

  for (; line < line_lower; line++) {
    for (int row=0; row < cnt_width; row++, buf++) buf[0] = fontRB.colors[0];
  }

  return cnt_width;
}


#ifdef DEBUG
void fnt_test(const Font *font) {
  //short img_w = (font->imgWidth + 7) & 0xFFF8;

  fnt_calc_colors(0xFFFF, 0x0000);
  tClipArea clip = { 0, 240, 0, 320 };
  for (int cno = 0; cno < font->charCount; cno++) {
    int chrW;
    //memset(test_buf, 0x55, sizeof(test_buf));
    chrW = fnt_renderchar(&font->chars[cno], &clip, font);
    ST7789_Sprite(20, 20, chrW, font->maxHeight, fnt_flipBuffer());
    osDelay(15);
    osDelay(100);
  }
}
#endif


unsigned int fnt_readUTF8(const char **text_ptr) {
  unsigned int utf8 = (*text_ptr)[0];
  (*text_ptr)++;
  if (utf8 & 0x80) switch (utf8 & 0xF0) {
  case 0xF0:
    utf8 = (utf8 << 8) | (*text_ptr)[0];
    (*text_ptr)++;
    // fall through
  case 0xE0:
    utf8 = (utf8 << 8) | (*text_ptr)[0];
    (*text_ptr)++;
    // fall through
  case 0xC0:
    utf8 = (utf8 << 8) | (*text_ptr)[0];
    (*text_ptr)++;
    break;
  } // hctiws
  return utf8;
}



unsigned char fnt_getAltNumIndex(const Font *font, unsigned int chr) {
  for (unsigned char idx=0; idx < font->charCount; idx++) {
    if (font->chars[idx].charId == (chr & 0xFF)) return idx;
  }
  return 255;
}


unsigned char fnt_getIndex(const Font *font, unsigned int chr) {
  if (font->charCount < 94) return fnt_getAltNumIndex(font, chr);
  unsigned char idx = (chr > 32) && (chr < (33 + font->charCount))? chr - 33: 255;
  if (idx == 255) switch (chr) {
  case 0xC2B0:	//'°':
  case 0xB0:
    if (font->charCount > 94) idx = 94;
    break;
  case 0xc3A4:
  case 0xc384:		//'Ä'
    if (font->charCount > 95) idx = 95;
    break;
  case 0xc3b6:		//'ö'
  case 0xc396:		//'Ö'
    if (font->charCount > 96) idx = 96;
    break;
  case 0xc3bc:
  case 0xc39c:		//'Ü'
    if (font->charCount > 97) idx = 97;
    break;
  case 0xc39f:		//'ß'
    if (font->charCount > 98) idx = 98;
    break;
  case 0xe282ac:	//'€'
    if (font->charCount > 99) idx = 99;
    break;
  }
  return idx;
}


int FONT_getWidth(const Font *font, const char *text) {
  int ccnt, cwdth;
  for (ccnt=0, cwdth=0; (text[0] != 0); ccnt++) {
    unsigned int  c   = fnt_readUTF8(&text);
    unsigned char idx = fnt_getIndex(font, c);
    if (idx < font->charCount) {
      unsigned char cw = font->chars[idx].width;
      unsigned char ad = font->chars[idx].advance;
      cwdth += cw > ad? cw: ad;
    } else cwdth += font->spaceWidth;
  } // rof
  return cwdth;
}


int fnt_writeClipped(const char *text, short *x_start, short y, short x_end, tClipArea *clip, const Font *font) {
  int ccnt;
  unsigned int   xpos = x_start[0], ypos = y;
  unsigned short fdisp_height = (clip->height > font->maxHeight)? font->maxHeight: clip->height;
  if (fdisp_height <= 0) return 0;
  for (ccnt=0; (text[0] != 0) && (xpos < x_end); ccnt++) {
    unsigned int  c   = fnt_readUTF8(&text);
    unsigned char idx = fnt_getIndex(font, c);
    clip->width = x_end - xpos + 1;
    int chrW = idx < font->charCount? fnt_renderchar(&font->chars[idx], clip, font): fnt_renderspace(clip, font);
    if (chrW == 0) continue;
    ST7789_Sprite(xpos, ypos, chrW, fdisp_height, fnt_flipBuffer());
    xpos += chrW;
  } // rof;
  x_start[0] = xpos;
  return ccnt;
}


int fnt_writeStr(const char *text, short x, short y, const Font *font) {
  fnt_calc_colors(0xFFFF, 0x0000);
  tClipArea clip = { 0, 0, 0, TFT_Y_SIZE - y };
  return fnt_writeClipped(text, &x, y, TFT_X_SIZE, &clip, font);
}


int FONT_printStr(const Font *font, const char *text, tBox box, tTextAlign align, unsigned short color, unsigned short bgcol) {
  int ccnt, fntw;
  fnt_calc_colors(color, bgcol);
  tClipArea clip    = { 0, box.right - box.left + 1, 0, box.bottom - box.top + 1 };
  tBox      borders = { 0, 0, 0, 0 };
  short xpos = box.left, ypos = box.top;
  short fdisp_height = (clip.height > font->maxHeight)? font->maxHeight: clip.height;

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
    ST7789_FillRect(box.left, box.top, box.right - box.left + 1, borders.top, bgcol);
  }
  if (borders.left > 0) {
    ST7789_FillRect(box.left, box.top + borders.top, borders.left, fdisp_height, bgcol);
  }

  ccnt = fnt_writeClipped(text, &xpos, ypos, box.right, &clip, font);

  borders.right = box.right - xpos + 1;
  if (borders.right > 0) {
    ST7789_FillRect(xpos, box.top + borders.top, borders.right, fdisp_height, bgcol);
  }
  if (borders.bottom > 0) {
    ST7789_FillRect(box.left, box.bottom - borders.bottom + 1, box.right - box.left + 1, borders.bottom, bgcol);
  }

  return ccnt;
}



static int icon_getpixeladdr(const tIcon *ico, tClipArea clip, unsigned char no) {
  int linewidth = ((ico->width + 1) & 0xfffe);
  if (no >= ico->icon_cnt) no = 0;
  return (linewidth * ico->height) * no + (linewidth * clip.upper) + clip.left;
}


static unsigned char icon_render(const tIcon *ico, tClipArea clip, unsigned char no) {
  unsigned short *buf = fontRB.pixel[fontRB.pixel_parms & 1];

  int pxdata_ptr  = icon_getpixeladdr(ico, clip, no);
  int pxdata_next = 0;

  // line: Y-related
  int line       = clip.upper;
  int line_lower = clip.height < ico->height? clip.height: ico->height;
  // cnt: X related
  int cnt_width  = ico->width;

  // clipping:
  if (cnt_width > (clip.width - clip.left)) {
    cnt_width  = clip.width - clip.left;
  }
  pxdata_next = ((ico->width + 1) & 0xfffe) - cnt_width;
  for (; line < line_lower; line++) {
    int pos = clip.left;
    for (; pos < cnt_width; pos++, buf++) {
      short pixel = fontRB.colors[fnt_getpixeldata(pxdata_ptr, ico->data)];
      pxdata_ptr++;
      buf[0] = pixel;
    } // rof data
    pxdata_ptr += pxdata_next;						// nxt line from BMP reverse
  } // rof
  return cnt_width;
}


static unsigned int raw4bit_render(const tRawPalettePicture *pic, unsigned int ofs) {
  unsigned short *buf     = fontRB.pixel[fontRB.pixel_parms & 1];
  unsigned short *buf_end = &buf[FONT_MAX_PIXEL_BUFSIZE - pic->width];

  int pxdata_ptr  = ofs << 1;
  int pxdata_next = pic->width & 1;
  int pxtotal     = pic->size << 1;

  while ((buf < buf_end) && (pxdata_ptr < pxtotal)) {
    for (int pos= 0; (pos < pic->width); pos++, buf++) {	// lines
      short pixel = fontRB.colors[fnt_getpixeldata(pxdata_ptr, pic->data)];
      pxdata_ptr++;
      buf[0] = pixel;
    } // rof data
    pxdata_ptr += pxdata_next;						// nxt line from BMP reverse
  } // rof
  return pxdata_ptr - (ofs << 1);
}


int ICON_write(const tIcon *icon, tBox box, tTextAlign align, unsigned char no, unsigned short color, unsigned short bgcol) {

  fnt_calc_colors(color, bgcol);
  tClipArea clip    = { 0, box.right - box.left + 1, 0, box.bottom - box.top + 1 };
  tBox      borders = { 0, 0, 0, 0 };
  unsigned short xpos = box.left, ypos = box.top;
  unsigned short fdisp_height = (clip.height > icon->height)? icon->height: clip.height;

  switch (align) {
  case taRIGHTTOP:
    if (icon->width < clip.width) {
      xpos = (box.right + 1) - icon->width;
    }
    // fall through
  case taLEFTTOP:	// do nothing
    borders.bottom = clip.height - fdisp_height;
    break;

  case taRIGHT:
    if (icon->width < clip.width) {
      xpos = (box.right + 1) - icon->width;
    }
    // fall through
  case taLEFT:
    ypos += (clip.height - fdisp_height) >> 1;
    break;

  case taRIGHTBTM:
    if (icon->width < clip.width) {
      xpos = (box.right + 1) - icon->width;
    }
    // fall through
  case taLEFTBTM:
    borders.top = clip.height - fdisp_height;
    ypos += borders.top;
    break;
  
  case taCENTER:
  case taCENTERBTM:
    if (icon->width < clip.width) {
      xpos += align == (clip.width - icon->width) >> 1;
    }
    borders.top = align == taCENTER? (clip.height - fdisp_height) >> 1: (clip.height - fdisp_height);
    borders.bottom = clip.height - fdisp_height - borders.top;
    ypos += borders.top;
    break;

  case taCENTERTOP:
    if (icon->width < clip.width) {
      xpos += (clip.width - icon->width) >> 1;
    }
    borders.bottom = clip.height - fdisp_height;
    break;
  } // hctiws

  borders.left = xpos - box.left;
  if (borders.top > 0) {
    ST7789_FillRect(box.left, box.top, box.right - box.left + 1, borders.top, bgcol);
  }
  if (borders.left > 0) {
    ST7789_FillRect(box.left, box.top + borders.top, borders.left, fdisp_height, bgcol);
  }

  int iconW = icon_render(icon, clip, no);
  if (iconW > 0) {
    ST7789_Sprite(xpos, ypos, iconW, fdisp_height, fnt_flipBuffer());
    xpos += iconW;
  }

  borders.right = box.right - xpos + 1;
  if (borders.right > 0) {
    ST7789_FillRect(xpos, box.top + borders.top, borders.right, fdisp_height, bgcol);
  }
  if (borders.bottom > 0) {
    ST7789_FillRect(box.left, box.bottom - borders.bottom + 1, box.right - box.left + 1, borders.bottom, bgcol);
  }

  return 1;
}

