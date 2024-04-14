

#include "fonts.h"



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
