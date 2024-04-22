/*

localrecoerder.h

This header file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

main API file for accessing the C2LORA UHF link

*/
#pragma once

#include "compiler.h"
#include "s_buffer.h"

#include "esp_err.h"


#define MAX_RECORDING_LEN_S     180             // 3 minutes for now


typedef struct sRecorderHandle tRecorderHandle;

esp_err_t create_record(tRecorderHandle **hnd, U8 codec_type, U16 frame_cnt, U16 frame_size, U16 steps_per_frame);

esp_err_t record_append_info(tRecorderHandle *hnd, const char *callsign, const char *recipient);

void record_append_dvframe(tRecorderHandle *hnd, const void *dvdata, U16 frame_cnt);

esp_err_t finish_record(tRecorderHandle *hnd);

tRecorderHandle *get_last_record(void);

void rewind_recording(tRecorderHandle *hnd);

U8 record_get_codec_type(tRecorderHandle *hnd);

const char *record_get_callsign(tRecorderHandle *hnd);

tsimple_buffer *record_get_buffer(tRecorderHandle *hnd);

const void *record_get_next_chunk(tRecorderHandle *hnd, U16 frame_cnt, U16 *frames_got);
