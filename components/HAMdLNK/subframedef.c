/*
 * subframedef.c
 *
 *  Created on: 22.01.2024
 *      Author: Jan Alte, DO1FJN
 *
 * This file is part of the DV-RPTR application.
 * For general information about this program see "main.c".
 *
 * DV-RPTR app is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The DV-RPTR app is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "subframedef.h"
#include "utilities.h"

#include <stddef.h>

static const char *sf_speech_names[16] = {
  "IMBE_4400",   "AMBE+_2400",  "AMBE2+",      "undef",
  "Codec2_3200", "Codec2_2400", "Codec2_1600", "Codec2_1400",
  "Codec2_1300", "Codec2_1200", "Codec2_700A", "Codec2_700B",
  "Codec2_700C", "undef",       "undef",       "undef"
};

static const char *sf_audio_names[2] = {
  "PCM_Alaw", "PCM_µlaw"
};

static const char sf_speech_bitsize[16] = {
  88, 48, 48, 0,
  64, 48, 64, 56, 52, 48, 28, 28, 28,
  0, 0, 0
};

const char *sf_get_subframe_name(unsigned char sf_codec_type) {
  unsigned char class = sf_codec_type & 0xF0;
  unsigned char stype = sf_codec_type & 0x0F;
  switch (class) {
  case SUBFRAME_CLASS_SPEECH:
    return sf_speech_names[stype];
  case SUBFRAME_CLASS_AUDIO:
    if (stype < 2) return sf_audio_names[stype];
  } // hctiws
  return NULL;
}


unsigned char sf_get_subfrage_type(const char *sf_name) {
  return 255;
}


unsigned short sf_get_subframe_bits(unsigned char sf_codec_type) {
  unsigned char class = sf_codec_type & 0xF0;
  unsigned char stype = sf_codec_type & 0x0F;
  switch (class) {
  case SUBFRAME_CLASS_SPEECH:
    return sf_speech_bitsize[stype];
  case SUBFRAME_CLASS_AUDIO:
    break;
  } // hctiws
  return 0;
}
