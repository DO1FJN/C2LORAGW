/*
 * DynRender.h
 *
 *  Created on: 13.11.2023
 *     Project: Comp2022
 *      Author: Jan Alte, (c) 2023 bentrup Industriesteuerungen
 */

#ifndef _DYNRENDER_H_
#define _DYNRENDER_H_

#include "TFT.h"

typedef struct {
  const unsigned short *palette;
  unsigned char		palette_items;	// max 256 palette items

  unsigned short *	pixel[2];
  unsigned int		pixel_parms;	// Bit0 buffer select
} tFontRenderBuf;


int PICTURE_write(const tRawPalettePicture *pic, unsigned short x, unsigned short y);

int PICTURE_drawPNG(const tPngPicture *png, unsigned short x, unsigned short y);


#endif // _DYNRENDER_H_
