/*
 * fonts.h
 *
 *  Created on: 10.05.2022
 *     Project: Comp2022
 *      Author: Jan Alte, (c) 2022 bentrup Industriesteuerungen
 */

#ifndef FONTS_H_
#define FONTS_H_

typedef struct Character {
  unsigned short	x, y;
  char			charId;
  unsigned char		width, height;
  signed char		shiftX, shiftY;
  unsigned char		advance;
} Character;

typedef struct Font {
  const char *		name;
  unsigned char		fontpxSize, bold, italic;
  unsigned char		charCount;
  unsigned char		maxWidth, maxHeight, spaceWidth;
  unsigned short	imgWidth, imgHeight;
  const Character *	chars;
  const unsigned char *	data;
  unsigned int		size;
} Font;

typedef struct {
   short left, top;
   short right, bottom;
} tBox;

typedef enum {
  taLEFT = 0, taLEFTTOP, taLEFTBTM, taCENTER, taRIGHT, taRIGHTTOP, taRIGHTBTM, taCENTERTOP, taCENTERBTM
} tTextAlign;


unsigned int  fnt_readUTF8(const char **text_ptr);
unsigned char fnt_getIndex(const Font *font, unsigned int chr);

int	fnt_writeStr(const char *text, short x, short y, const Font *font);

int	FONT_getWidth(const Font *font, const char *text);
int	FONT_printStr(const Font *font, const char *text, tBox box, tTextAlign align, unsigned short color, unsigned short bgcol);

void	FONT_calc_colors(short colbuf[16], unsigned short color565, unsigned short bg565);


#endif //FONTS_H_
