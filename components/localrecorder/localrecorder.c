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

//#include "heap.h"

static const char *TAG = "REC";


struct sRecorderHandle {  
  tsimple_buffer        buf;
  U8                    codec_type;
};


esp_err_t create_record(tRecorderHandle **hnd, U8 codec_type, U16 frame_cnt, U16 frame_size, U16 steps_per_frame) {
  
  ESP_RETURN_ON_FALSE(hnd != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid (NULL) recording handle ptr");
  tRecorderHandle *rec = calloc(1, sizeof(struct sRecorderHandle));
  ESP_RETURN_ON_FALSE(rec != NULL, ESP_ERR_NO_MEM, TAG, "no memory for recoding handle");

  rec->codec_type = codec_type;
  
  void * buffer = heap_caps_malloc((U32)frame_cnt * frame_size, MALLOC_CAP_SPIRAM);

  if (buffer == NULL) {
    ESP_LOGE(TAG, "no memory left for recording (%lu bytes)", (U32)frame_cnt * frame_size);
    return ESP_ERR_NO_MEM;
  }

  sbuf_create_4ext(&rec->buf, frame_cnt, frame_size, steps_per_frame, buffer);

  hnd[0] = rec;
  return ESP_OK;
}


void record_append_dvframe(tRecorderHandle *hnd, const void *dvdata, U16 frame_cnt) {
  if ((hnd == NULL) || (hnd->buf.size == 0)) return;

  if (frame_cnt > (sbuf_get_size(&hnd->buf) / hnd->buf.frame_size)) {
    frame_cnt = sbuf_get_size(&hnd->buf) / hnd->buf.frame_size;
  }
  sbuf_fillnext(&hnd->buf, dvdata, frame_cnt);
}


void rewind_recording(tRecorderHandle *hnd) {
  if (hnd == NULL) return;
  hnd->buf.unsend_ptr = (void *) hnd->buf.start_ptr;
}


U8 record_get_codec_type(tRecorderHandle *hnd) {
  if (hnd == NULL) return 255;
  return hnd->codec_type;        
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
