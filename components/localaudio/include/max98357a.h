/*
 * max98357a.h
 *
 *  Created on: 01.02.2024
 *     Project: Lora-Gateway
 *      Author: Jan Alte, DO1FJN
 */

#pragma once

#include "i2s_loop.h"


void       MAX98357A_get_config(ti2sloop_config *target_buf, unsigned int sample_rate_hz, unsigned char bits_per_sample);

void       MAX98357A_Init(void);

void       MAX98357A_enable(bool active);


void       INMP441_get_config(ti2sloop_config *target_buf, unsigned int sample_rate_hz);