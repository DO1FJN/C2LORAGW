/*

localrecorder.c

This source file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

main API file for accessing the C2LORA UHF link

*/
#include "localrecorder.h"


#include "compiler.h"
#include "s_buffer.h"

#include "esp_err.h"
#include "esp_check.h"
#include "esp_heap_caps.h"

#include <string.h>
#include <time.h>

static const char *TAG = "REC";


struct sRecorderHandle {  
  tsimple_buffer        buf;
  U8                    codec_type;
  U32                   time;
  char                  callsign[8];
};

static tRecorderHandle * last_recording = NULL;

#if CONFIG_SPIRAM

#else

tRecorderHandle last_rec = { };

#endif 


esp_err_t create_record(tRecorderHandle **hnd, U8 codec_type, U16 frame_cnt, U16 frame_size, U16 steps_per_frame) {
  time_t now;
  tRecorderHandle *rec;
  ESP_RETURN_ON_FALSE(hnd != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid (NULL) recording handle ptr");
#if CONFIG_SPIRAM

  rec = calloc(1, sizeof(struct sRecorderHandle));
  ESP_RETURN_ON_FALSE(rec != NULL, ESP_ERR_NO_MEM, TAG, "no memory for recoding handle");

  
  void * buffer = heap_caps_malloc((U32)frame_cnt * frame_size, MALLOC_CAP_SPIRAM);

  if (buffer == NULL) {
    ESP_LOGE(TAG, "no memory left for recording (%lu bytes)", (U32)frame_cnt * frame_size);
    return ESP_ERR_NO_MEM;
  }

  sbuf_create_4ext(&rec->buf, frame_cnt, frame_size, steps_per_frame, buffer);
#else

  if (last_rec.buf.start_ptr == NULL) {
    sbuf_create(&last_rec.buf, frame_cnt, frame_size, steps_per_frame);
  } else {
    sbuf_flush(&last_rec.buf);
  }
  rec = &last_rec;

#endif
  time(&now);
  rec->codec_type = codec_type;
  rec->time = now;
  memset(rec->callsign, 0, sizeof(rec->callsign));
  hnd[0] = rec;
  return ESP_OK;
}


esp_err_t record_append_info(tRecorderHandle *hnd, const char *callsign, const char *recipient) {
  ESP_RETURN_ON_FALSE(hnd != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid (NULL) recording handle ptr");
  
  strncpy(hnd->callsign, callsign, sizeof(hnd->callsign));

  return ESP_OK;
}


void record_append_dvframe(tRecorderHandle *hnd, const void *dvdata, U16 frame_cnt) {
  if ((hnd == NULL) || (hnd->buf.size == 0)) return;

  if (frame_cnt > (sbuf_get_size(&hnd->buf) / hnd->buf.frame_size)) {
    frame_cnt = sbuf_get_size(&hnd->buf) / hnd->buf.frame_size;
  }
  sbuf_fillnext(&hnd->buf, dvdata, frame_cnt);
}


esp_err_t finish_record(tRecorderHandle *hnd) {  
#if CONFIG_SPIRAM
  ESP_RETURN_ON_FALSE(hnd != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid (NULL) recording handle ptr");
  if (hnd->buf.start_ptr != NULL) {
    uint32_t new_size = hnd->buf.write_ptr - hnd->buf.start_ptr + 1;
    ESP_LOGD(TAG, "resize recording to %lu bytes.", new_size);
    hnd->buf.start_ptr = heap_caps_realloc((void *)hnd->buf.start_ptr, new_size, MALLOC_CAP_SPIRAM);
    hnd->buf.size = new_size;
    hnd->buf.unsend_ptr = (void *)hnd->buf.start_ptr;
    last_recording = hnd;
  } // fi
#else

  last_recording = &last_rec;

#endif

  return ESP_OK;
}


tRecorderHandle *get_last_record(void) {
  return last_recording;
}



void rewind_recording(tRecorderHandle *hnd) {
  if ((hnd == NULL) || (hnd->buf.start_ptr == NULL)) return;
  hnd->buf.unsend_ptr = (void *) hnd->buf.start_ptr;
}


U8 record_get_codec_type(tRecorderHandle *hnd) {
  if (hnd == NULL) return 255;
  return hnd->codec_type;        
}

const char *record_get_callsign(tRecorderHandle *hnd) {
  if (hnd == NULL) return NULL;      
  return hnd->callsign;
}

tsimple_buffer *record_get_buffer(tRecorderHandle *hnd) {
  if (hnd == NULL) return NULL;      
  return &hnd->buf;
}


const void *record_get_next_chunk(tRecorderHandle *hnd, U16 frame_cnt, U16 *frames_got) {
  if (frames_got != NULL) *frames_got = 0;
  if (hnd == NULL) return NULL;
  
  U32 unsend_size;
  U16 frames_in_buf;

  const void *unsend = sbuf_get_unsend_block(&hnd->buf, &unsend_size);

  frames_in_buf = unsend_size / hnd->buf.frame_size;

  sbuf_unsend_block_processed(&hnd->buf, (frames_in_buf > frame_cnt? frame_cnt: frames_in_buf) * hnd->buf.frame_size);

  if (frames_got != NULL) {
    *frames_got = frames_in_buf > frame_cnt? frame_cnt: frames_in_buf;
  }

  return frames_in_buf > 0? unsend: NULL; // finish - no data
}
