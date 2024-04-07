/*
 * Backlight.h
 *
 *  Created on: 13.09.2016
 *      Author: Jan Alte
 */

#ifndef BACKLIGHT_H_
#define BACKLIGHT_H_

#define DEFAULT_BRIGHTNESS	800	// 80%
#define INITIAL_BRIGHTNESS	0
#define MAX_BRIGHTNESS		1000


void	Backlight_Init(void);

void	Backlight_emergency(void);

void	Backlight_OnOff(int onoff);

void	Backlight_setDim(unsigned int brightness);

unsigned int	Backlight_getDimlin(void);


#endif // BACKLIGHT_H_
