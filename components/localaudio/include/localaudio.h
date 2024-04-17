
#pragma once


#include "s_buffer.h"
#include "esp_err.h"


esp_err_t localaudio_start(void);

#define DVSTREAM_FLAG_KEEP_BUFFER     0x0001

typedef void (*tfinished_stream_fct) (void *userdata);

typedef struct {
  int			        streamid;
  unsigned int		srate;
  unsigned int		flags;
  unsigned int		c_quality;
  unsigned char		codec_type;  
  unsigned short	samples_per_frame;
  unsigned short	bits_per_frame;
  unsigned char		bytes_per_frame;
  union {
    tsimple_buffer  *	buf;
  };  
} tdvstream;

typedef void (*thandle_inputstream_funct) (tdvstream *dva, void *userdata);

int     localaudio_create_codec2_outputstream(tdvstream *dva, int mode, unsigned short data_per_packet, tfinished_stream_fct fini_cb);

int     localaudio_create_codec2_inputstream(tdvstream *dva, int mode, unsigned int trigger_after_ms, thandle_inputstream_funct handler, void *userdata);

int     localaudio_end_stream(tdvstream *dva, bool no_more_data);


void    localaudio_handle_stream(int stream_id);

bool    localaudio_create_buffer(tdvstream *dva, int size_ms);

float   localaudio_get_volume_dB(void);
void    localaudio_set_volume_dBm4(signed char volume);

float   localaudio_get_micgain_dB(void);
void    localaudio_set_micgain_dBm4(signed char micgain);
