/*
 * output_codecs.h
 *
 *  Created on: 01.02.2024
 *     Project: Lora-Gateway
 *      Author: Jan Alte, DO1FJN

This header file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.

*/

#pragma once

#include "i2s_loop.h"


void       MAX98357A_get_config(ti2sloop_config *target_buf, unsigned int sample_rate_hz, unsigned char bits_per_sample);

void       MAX98357A_Init(void);

void       MAX98357A_enable(bool active);
