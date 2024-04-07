/*
 * ST7789V.h
 *
 *  Created on: 19.01.2022
 *      Author: Jan Alte, DO1FJN
 */

#ifndef _ST7789V_H_
#define _ST7789V_H_

#include "esp_err.h"

esp_err_t ST7789_Init(void);

void	ST7789_OnOff(int onOff);

void	ST7789_SetWindow(short left, short top, short right, short bottom);
void	ST7789_Fill(unsigned short color_565, unsigned int size);
void	ST7789_Write(const unsigned short *px16data, unsigned int px_count);

void	ST7789_Sprite(short x, short y, unsigned short w, unsigned short h, const unsigned short *px16data);
void	ST7789_FillRect(short x, short y, unsigned short w, unsigned short h, unsigned short color);
void	ST7789_FillBox(short left, short top, short right, short bottom, unsigned short color);

void	ST7789_ReadRect(unsigned short *raw24data, unsigned short x, unsigned short y, unsigned short w, unsigned short h);

#endif // _ST7789V_H_
