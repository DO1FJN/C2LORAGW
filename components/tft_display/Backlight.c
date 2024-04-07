/*
 * Backlight.c
 *
 *  Created on: 13.09.2016
 *      Author: jan
 */

#include "Backlight.h"
#include "hardware.h"

#include "driver/gpio.h"


#define BLTIM		TIM14	// PWM is TIM14, channel 1


void Backlight_Init(void) {
#if TFT_BACKLIGHT_PIN != GPIO_NUM_NC
  gpio_reset_pin(TFT_BACKLIGHT_PIN);
  gpio_set_direction(TFT_BACKLIGHT_PIN, GPIO_MODE_OUTPUT);
#endif  
}


inline void Backlight_emergency(void) {
  //BLTIM->CCR1 = DEFAULT_BRIGHTNESS;
  //BLTIM->CCER = TIM_CCER_CC1E;
}


/*
void setBrightness(uint8_t value)
{
    static uint8_t level = 0;
    static uint8_t steps = 16;
    if (value == 0) {
        digitalWrite(BOARD_BL_PIN, 0);
        delay(3);
        level = 0;
        return;
    }
    if (level == 0) {
        digitalWrite(BOARD_BL_PIN, 1);
        level = steps;
        delayMicroseconds(30);
    }
    int from = steps - level;
    int to = steps - value;
    int num = (steps + to - from) % steps;
    for (int i = 0; i < num; i++) {
        digitalWrite(BOARD_BL_PIN, 0);
        digitalWrite(BOARD_BL_PIN, 1);
    }
    level = value;
}*/


void Backlight_setDim(unsigned int brightness) {

#if TFT_BACKLIGHT_PIN != GPIO_NUM_NC
  if (brightness > 1000) brightness = 1000;
  gpio_set_level(TFT_BACKLIGHT_PIN, (brightness > 10));
  //BLTIM->CCR1 = brightness;
#endif
}

unsigned int Backlight_getDimlin(void) {
  return 0; //BLTIM->CCR1;
}


void Backlight_OnOff(int onoff) {
#if TFT_BACKLIGHT_PIN != GPIO_NUM_NC
  gpio_set_level(TFT_BACKLIGHT_PIN, onoff);
  if (onoff) {
    //BLTIM->CCER = TIM_CCER_CC1E;
  } else {
    //BLTIM->CCER = 0;
  }
#endif
}

