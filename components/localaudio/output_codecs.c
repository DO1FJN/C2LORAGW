/*
 * output_codecs.c
 *
 *  Created on: 01.02.2024
 *     Project: Lora-Gateway
 *      Author: Jan Alte, DO1FJN
 */

#include "output_codecs.h"
#include "hardware.h"

#include "i2s_loop.h"

#include "driver/gpio.h"

#include "esp_err.h"

#include <string.h>



static const ti2sloop_config max_std_config = {
  .sample_rate_hz = 8000,
  .slot.mode      = SLOT_MODE_MONO,
  .slot.bitwidth  = 16,              // total width of a slot in bits (0 = aout, 8, 16, 24 or 32 are valid)
  .slot.databits  = 16,
  .slot.ws_width  = 16,
  .slot.mask      = 0x03,            // Bit 0 = LEFT, Bit 1 = right
  .slot.bit_shift = 1,
  .slot.left_align = 1,
  .gpio = {
    .mclk = GPIO_NUM_NC,             // some codecs may require mclk signal, the MAX98357A hasn't one
    .bclk = AUDIO_I2S_BCLK_Pin,
    .ws   = AUDIO_I2S_WS_Pin,
    .dio  = AUDIO_I2S_DOUT_Pin,
    .invert_flags = {
       .mclk_inv = false,
       .bclk_inv = false,
       .ws_inv   = false,
    }
  }
};


void MAX98357A_get_config(ti2sloop_config *target_buf, unsigned int sample_rate_hz, unsigned char bits_per_sample) {
  ti2sloop_config my_config = max_std_config;
  if (target_buf == NULL) return;
  
  if ((sample_rate_hz < 8000) || (sample_rate_hz > 96000)) {
    sample_rate_hz = sample_rate_hz < 8000? 8000: 96000;
  }
  switch (bits_per_sample) {
  case 24: my_config.slot.databits = 24; break;
  case 32: my_config.slot.databits = 32; break;  
  }
  my_config.slot.bitwidth = my_config.slot.databits;
  my_config.slot.bitwidth = my_config.slot.databits;
  my_config.sample_rate_hz = sample_rate_hz;
  memcpy(target_buf, &my_config, sizeof(ti2sloop_config));
}


void MAX98357A_Init(void) {
#if AUDIO_I2S_ENABLE != GPIO_NUM_NC
  const gpio_config_t enable_pin_conf = {
    .pin_bit_mask = (1 << AUDIO_I2S_ENABLE),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = false
  };
  gpio_set_level(AUDIO_I2S_ENABLE, 0);
  gpio_config(&enable_pin_conf);
#endif  
}


void MAX98357A_enable(bool active) {
#if AUDIO_I2S_ENABLE != GPIO_NUM_NC
  gpio_set_level(AUDIO_I2S_ENABLE, active? 1: 0);
#endif  
}
