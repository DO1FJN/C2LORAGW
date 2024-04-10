
#pragma once

#include "sdkconfig.h"

#ifdef CONFIG_TARGET_BREADBOARD
#include "breadboard.h"
#endif

#ifdef CONFIG_TARGET_C2LOARAGW_V1
#include "c2loragw.h"
#endif

#ifdef CONFIG_TARGET_HELTEC_WIFILORA_V3
#include "heltec_wifi_lora_v3.h"
#endif

#ifdef CONFIG_TARGET_LILYGO_TDECK
#include "lilygo_tdeck.h"
#endif
