
#pragma once

#include "esp_err.h"
#include "localaudio.h"


esp_err_t hamdlnk_set_sender_info(const char *callsign, const char *recipient, unsigned short area_code, const char *locator);

esp_err_t ham2lnk_broadcast_audio(unsigned char codec_type, unsigned short dv_len_ms, bool interleave, const char *info_str);

void      ham2lnk_broadcast_stop(void);

//const tdvstream *ham2lnk_get_audio_stream(void);
