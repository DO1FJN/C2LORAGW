/*

This source file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.

*/

#include "input_codecs.h"
#include "hardware.h"
#include "esp_err.h"

#include <string.h>


static const ti2sloop_config inmp_std_config = {
  .sample_rate_hz = 8000,
  .slot.mode      = SLOT_MODE_MONO,
  .slot.bitwidth  = 32,              // total width of a slot in bits (0 = aout, 8, 16, 24 or 32 are valid)
  .slot.databits  = 24,
  .slot.ws_width  = 32,
  .slot.mask      = 0x03,            // Bit 0 = LEFT, Bit 1 = right
  .slot.bit_shift = 1,
  .slot.left_align = 1,
  .gpio = {
    .mclk = GPIO_NUM_NC,             // some codecs may require mclk signal, the MAX98357A hasn't one
    .bclk = CODEC_I2S_BCLK_Pin,
    .ws   = CODEC_I2S_WS_Pin,
    .dio  = CODEC_I2S_DIN_Pin,
    .invert_flags = {
       .mclk_inv = false,
       .bclk_inv = false,
       .ws_inv   = false,
    }
  }
};


void INMP441_get_config(ti2sloop_config *target_buf, unsigned int sample_rate_hz) {
  ti2sloop_config my_config = inmp_std_config;
  if (target_buf == NULL) return;
 
  if ((sample_rate_hz < 8000) || (sample_rate_hz > 48000)) {
    sample_rate_hz = sample_rate_hz < 8000? 8000: 48000;
  }
  my_config.sample_rate_hz = sample_rate_hz;
  memcpy(target_buf, &my_config, sizeof(ti2sloop_config));
}

