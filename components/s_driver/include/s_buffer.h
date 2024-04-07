/*
 * rtp_buffer.h
 *
 *  Created on: 28.01.2024
 *      Author: Jan Alte, DO1FJN
 */

#pragma once

#include "compiler.h"


typedef struct {
  unsigned int		size;			        // allocated size in bytes (at start_ptr)
  const void *		start_ptr;	    	// start of the usable buffer (size bytes big)
  unsigned short	frame_size;
  unsigned short	steps_per_frame;

  void *		unsend_ptr;		// points to the begin of unsend data (updated by reader)
  void *		write_ptr;		// points to the current write position (updated by writer)

  unsigned int		write_spos;		    // stream position at write_ptr
  unsigned int		last_clocktick;		// timestamp of the last update
  void *          mutex;
} tsimple_buffer;


void  sbuf_setparameter(tsimple_buffer *buf, U16 frame_cnt, U16 frame_size, U16 steps_per_frame);

bool	sbuf_create(tsimple_buffer *buf, U16 frame_cnt, U16 frame_size, U16 steps_per_frame);

bool  sbuf_create_4ext(tsimple_buffer *buf, U16 frame_cnt, U16 frame_size, U16 steps_per_frame, const void *buffer);

void	sbuf_destroy(tsimple_buffer *buf);

void  sbuf_flush(tsimple_buffer *buf);

unsigned int sbuf_get_size(const tsimple_buffer *buf);

unsigned int sbuf_get_unsend_size(const tsimple_buffer *buf);

bool sbuf_check_unsend_frame(const tsimple_buffer *buf);

unsigned int sbuf_get_age_ms(const tsimple_buffer *buf);


/* rtpbuf_fillnext()
 * adds a chunk of data frames to the buffer in-order
 * @PARAMS
 * buf	the 	simple buffer
 * data	the 	data
 * frames_rx	number of data-frames in data
 * frame_size	size in bytes of a single frame
 */
void	sbuf_fillnext(tsimple_buffer *buf, const void *data, U16 frames_rx);

void	sbuf_fillpos(tsimple_buffer *buf, const void *data, U16 frames_rx, U32 stream_pos);

void	sbuf_update_time(tsimple_buffer *buf, U32 clocktick);

void	sbuf_set_steppos(tsimple_buffer *buf, U32 stream_pos);


const void *sbuf_get_unsend_block(const tsimple_buffer *buf, U32 *unsend_size);

void	sbuf_unsend_block_processed(tsimple_buffer *buf, U16 block_size);
