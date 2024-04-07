/*
 * rtp_buffer.c
 *
 *  Created on: 28.01.2024
 *      Author: Jan Alte, DO1FJN
 */

#include "s_buffer.h"

#include "compiler.h"

#include <string.h>


#define GET_CLOCKTICK_FUNCTION()  xTaskGetTickCount()


#ifndef SBUG_NO_LOCKS

// ToDo system independent
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define sbuf_init_mutex(buf)      xSemaphoreCreateMutex()
#define sbuf_lock(buf)            xSemaphoreTake((SemaphoreHandle_t)buf->mutex, portMAX_DELAY)
#define sbuf_unlock(buf)          xSemaphoreGive((SemaphoreHandle_t)buf->mutex)
#define sbuf_delete_mutex(buf)    vSemaphoreDelete((SemaphoreHandle_t)buf->mutex)
#else
#define sbuf_init_mutex(buf)
#define sbuf_lock(buf)
#define sbuf_unlock(buf)
#define sbuf_delete_mutex(buf)
#endif



void sbuf_setparameter(tsimple_buffer *buf, U16 frame_cnt, U16 frame_size, U16 steps_per_frame) {
  if (buf == NULL) return;
  buf->size  = frame_cnt * frame_size;
  buf->last_clocktick = 0;
  buf->frame_size = frame_size;
  buf->steps_per_frame = steps_per_frame;
  buf->mutex = sbuf_init_mutex(buf);
}


bool sbuf_create(tsimple_buffer *buf, U16 frame_cnt, U16 frame_size, U16 steps_per_frame) {

  if (buf == NULL) return false;

  sbuf_setparameter(buf, frame_cnt, frame_size, steps_per_frame);
  buf->start_ptr  = malloc(buf->size);
  buf->unsend_ptr = (void *) buf->start_ptr;
  buf->write_ptr  = (void *) buf->start_ptr;
  if (buf->start_ptr == NULL) buf->size = 0;
  return buf->start_ptr != NULL;
}

bool sbuf_create_4ext(tsimple_buffer *buf, U16 frame_cnt, U16 frame_size, U16 steps_per_frame, const void *buffer) {

  if (buf == NULL) return false;

  sbuf_setparameter(buf, frame_cnt, frame_size, steps_per_frame);
  buf->start_ptr  = buffer;
  buf->unsend_ptr = (void *) buf->start_ptr;
  buf->write_ptr  = (void *) buf->start_ptr;
  if (buf->start_ptr == NULL) buf->size = 0;
  return buf->start_ptr != NULL;
}


void sbuf_destroy(tsimple_buffer *buf) {
  if (buf == NULL) return;
  sbuf_lock(buf);
  if (buf->start_ptr != NULL) {
    free((void *)buf->start_ptr);
    buf->start_ptr = NULL;
    buf->size = 0;
  }
  sbuf_unlock(buf);
  sbuf_delete_mutex(buf);  // destroy mutex
  buf->mutex = NULL;
}


void sbuf_flush(tsimple_buffer *buf) {
  if (buf == NULL) return;
  sbuf_lock(buf);
  buf->unsend_ptr = (void *) buf->start_ptr;
  buf->write_ptr  = (void *) buf->start_ptr;
  sbuf_unlock(buf);
}



void sbuf_fillnext(tsimple_buffer *buf, const void *data, U16 frames_rx) {
  if ((buf == NULL) || (buf->size < buf->frame_size) || (data == NULL)) return;	// there is no buffer with enough space  
  const void *end_ptr = buf->start_ptr + buf->size;
  void *write_ptr     = buf->write_ptr;
  sbuf_lock(buf);
  while (frames_rx > 0) {
    size_t seg_size;
    U32 n_frames, n_frames_blk;
    if ((end_ptr - write_ptr) < buf->frame_size) write_ptr = (void *)buf->start_ptr;
    seg_size = (write_ptr >= buf->unsend_ptr)? end_ptr - write_ptr: buf->unsend_ptr - write_ptr;
    if (seg_size < buf->frame_size) {   // buffer overflow!    
      goto sbuf_fillnext_x;
    }
    n_frames_blk = seg_size / buf->frame_size;
    n_frames = n_frames_blk < frames_rx? n_frames_blk: frames_rx;
    seg_size = n_frames * buf->frame_size;	// count only frames
    memcpy(write_ptr, data, seg_size);
    frames_rx -= n_frames;
    buf->write_spos += n_frames * buf->steps_per_frame;
    write_ptr += seg_size;
    // ToDo test to prevent overflow (keep on high, but this results in errors)
    if (write_ptr >= end_ptr) write_ptr = (void *)buf->start_ptr;
  } // ehliw
sbuf_fillnext_x:
  buf->write_ptr = write_ptr;
  sbuf_unlock(buf);
}


// ToDo position handling (lost or interleaved packets
// wir wissen noch nicht wie lang ein frame in sps ist.
// frame_len
void sbuf_fillpos(tsimple_buffer *buf, const void *data, U16 frames_rx, U32 stream_pos) {
  U32 packet_end_spos, packet_slength;
  int relative_write_pos;

  if ((buf == NULL) || (buf->size < buf->frame_size) || (data == NULL)) return;	// there is no buffer with enough space

  const void *end_ptr = buf->start_ptr + buf->size;
  void *write_ptr     = buf->write_ptr;
  packet_slength      = frames_rx * buf->steps_per_frame;
  packet_end_spos     = stream_pos + packet_slength;
  relative_write_pos  = (int)(stream_pos - buf->write_spos) / (int)buf->steps_per_frame;	// negative means: overlap / interleave

  if (relative_write_pos < 0) {

    int bytes_before = -relative_write_pos * buf->frame_size;
    if (bytes_before >= buf->size) {
      //printf("a big chunk missed completly\n");
      goto sbuf_fillpos_x;	// out of range
    }
    if (packet_end_spos < buf->write_spos) {	// fill a gap
      write_ptr -= bytes_before;
      if (write_ptr < buf->start_ptr) write_ptr += buf->size;
    } else {					// only interleave      
      data      += bytes_before;
      frames_rx += relative_write_pos;		// copy only new data      
      //printf("append interleaved %d frames\n", frames_rx);
    }
  } else if (relative_write_pos > 0) {		// we missed data!

    int bytes_after = relative_write_pos * buf->frame_size;
    if (bytes_after >= buf->size) {
      //printf("missed data\n");
      goto sbuf_fillpos_x;		// out of range
    }
    write_ptr += bytes_after;
    if (write_ptr >= end_ptr) write_ptr -= buf->size;
  }

  sbuf_lock(buf);

  while (frames_rx > 0) {
    size_t seg_size;
    U32 n_frames, n_frames_blk;
    if ((end_ptr - write_ptr) < buf->frame_size) write_ptr = (void *)buf->start_ptr;
    seg_size = (write_ptr >= buf->unsend_ptr)? end_ptr - write_ptr: buf->unsend_ptr - write_ptr;
    if (seg_size < buf->frame_size) { // buffer overflow!
      //!!! not while locked. printf("buffer is full\n");
      goto sbuf_fillpos_ovr;
    }
    n_frames_blk = seg_size / buf->frame_size;
    n_frames = n_frames_blk < frames_rx? n_frames_blk: frames_rx;
    seg_size = n_frames * buf->frame_size;	// count only frames    
    memcpy(write_ptr, data, seg_size);
    write_ptr += seg_size;
    frames_rx -= n_frames;
  } // ehliw

//if (stream_pos >= (36*160)) return; // TEST
sbuf_fillpos_ovr:
  if (packet_end_spos > buf->write_spos) {
    if ((end_ptr - write_ptr) < buf->frame_size) write_ptr = (void *)buf->start_ptr;
    buf->write_ptr  = write_ptr;
    buf->write_spos = packet_end_spos;
  }
sbuf_fillpos_x:
  sbuf_unlock(buf);
}




void sbuf_update_time(tsimple_buffer *buf, U32 clocktick) {
  if ((buf == NULL)) return;
  buf->last_clocktick = clocktick;
}

void sbuf_set_steppos(tsimple_buffer *buf, U32 stream_pos) {
  if ((buf == NULL)) return;
  buf->write_spos = stream_pos;
}


unsigned int sbuf_get_size(const tsimple_buffer *buf) {
  size_t segment_a, segment_b;
  const void *end_ptr = buf->start_ptr + buf->size;
  if (buf->write_ptr >= buf->unsend_ptr) {
    segment_a = end_ptr - buf->write_ptr;
    segment_b = buf->unsend_ptr - buf->start_ptr;
  } else {
    segment_a = buf->unsend_ptr - buf->write_ptr;
    segment_b = end_ptr - buf->unsend_ptr;
  }
  return segment_a + segment_b;
}

unsigned int sbuf_get_unsend_size(const tsimple_buffer *buf) {
  unsigned int unsend_size = 0;
    if (buf->write_ptr >= buf->unsend_ptr) {
    unsend_size = buf->write_ptr - buf->unsend_ptr;
  } else {
    unsend_size = buf->start_ptr - buf->unsend_ptr + buf->size;	// first before wrap
    unsend_size += buf->write_ptr - buf->start_ptr;
  }
  return unsend_size;
}


unsigned int sbuf_get_age_ms(const tsimple_buffer *buf) {
  unsigned int clocktick = GET_CLOCKTICK_FUNCTION();
  if (buf == NULL) return -1;	// invalid max age
  unsigned int age = (clocktick - buf->last_clocktick) / 1000;
  return age;
}


inline bool sbuf_check_unsend_frame(const tsimple_buffer *buf) {
  unsigned int unsend_size = 0;
    if (buf->write_ptr >= buf->unsend_ptr) {
    unsend_size = buf->write_ptr - buf->unsend_ptr;    
  } else {
    unsend_size = buf->start_ptr - buf->unsend_ptr + buf->size;	// first before wrap
    unsend_size += buf->write_ptr - buf->start_ptr;
  }
  return unsend_size >= buf->frame_size;
}


inline const void *sbuf_get_unsend_block(const tsimple_buffer *buf, U32 *unsend_size) {
  if ((buf == NULL) || (unsend_size == NULL)) return NULL;
  if (buf->write_ptr >= buf->unsend_ptr) {
    *unsend_size = buf->write_ptr - buf->unsend_ptr;
  } else {
    *unsend_size = buf->start_ptr - buf->unsend_ptr + buf->size;	// first before wrap
  }
  return buf->unsend_ptr;
}


inline void sbuf_unsend_block_processed(tsimple_buffer *buf, U16 block_size) {
  if ((buf == NULL) || (buf->start_ptr == NULL)) return;
  sbuf_lock(buf);
  buf->unsend_ptr += block_size;
  if (buf->unsend_ptr >= (buf->start_ptr + buf->size)) {
    buf->unsend_ptr = (void *)buf->start_ptr;
  }
  sbuf_unlock(buf);
}

