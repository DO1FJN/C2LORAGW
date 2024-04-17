/*

This header file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.

*/
#pragma once

#include "i2s_loop.h"

void       INMP441_get_config(ti2sloop_config *target_buf, unsigned int sample_rate_hz);
