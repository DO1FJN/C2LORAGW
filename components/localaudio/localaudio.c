/*
 * localaudio.c
 *
 *  Created on: 01.02.2024
 *     Project: Lora-Gateway
 *      Author: Jan Alte, DO1FJN

This source file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.

*/

#include "localaudio.h"

#include "compiler.h"

#include "s_buffer.h"
#include "i2s_loop.h"
#include "output_codecs.h"
#include "input_codecs.h"

#include "HAMdLNK.h"
#include "codec2.h"
#include "codec2_internal.h"


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "esp_log.h"
#include "esp_timer.h"

#include <string.h>
#include <math.h>


#define STREAM_DRAINFINISH_RESULT       (-1)

#define LOCALAUDIO_TASK_STACK_SIZE      0x4B00 //0x4800
#define LOCALAUDIO_TASK_PRIORITY        5

#define LOCALAUDIO_HANDLE_INPUTS        0x100000
#define LOCALAUDIO_UPDATE_EVENTS        0x800000


#ifndef LOCALAUDIO_MAX_STREAMS
#define LOCALAUDIO_MAX_STREAMS          3     // barly handle 3 Codec2 decodes ~6ms each)
#endif

#define LOCALAUDIO_OUTPUT_WATERMARK_MS 60
#define LOCALAUDIO_OUTPUT_TIMEOUT_MS   1500



static TaskHandle_t             la_task_handle;
static EventGroupHandle_t	      la_events;
static EventBits_t              la_wait4events;


#define LA_Event(evnt)		xEventGroupSetBits(la_events, evnt)
#define LA_ClrEvent(evnt)	xEventGroupClearBits(la_events, evnt)

static char *TAG = "LocalAudio";

typedef enum {
  DVS_UNUSED = 0,
  DVS_OUTPUT, DVS_OUTFINISH, 
  DVS_INPUT,  DVS_INPUTENDS,
  DVS_TRANSCODE, DVS_TRANCODEENDS
} tStreamType;

//typedef int (*tget_next_samples_func) (void *dest, unsigned int n_samples_max, void *userdata);

//typedef int (*tprocess_samples_func) (void *src, unsigned int n_samples, void *userdata);

typedef void (*tencode_frame_func) (void *codec_struct, U8 *bytes_out, const S16 *samples_in);

typedef void (*tdecode_frame_func) (void *codec_struct, S16 *samples_out, const U8 *bytes_in);

typedef int (*tdestroy_stream_fct) (void *userdata);


typedef struct {
  tStreamType           kind;
  tdvstream *           dva;
  U8                    trans_channel;
  void *                codec_struct;
  tdestroy_stream_fct   destroy;
  union {
    thandle_inputstream_funct received_cb;  // encoded data (in dva->buf are ready)
    tfinished_stream_fct	    finished_cb;
  };
  tencode_frame_func    prcs_in;
  tdecode_frame_func    prcs_out;
  U16                   iobuffer_size;          // in no_of_samples
  U16                   iobuffer_wrpos;
  S16 *                 iobuffer;
  void *                user_data;
} tdvstream_channel;


static tdvstream_channel DVSchannel[LOCALAUDIO_MAX_STREAMS];

  // ToDo theare are user settings (result of log(10 calc))

S32 volume_raw = CONFIG_AUDIOOUT_DEFAULT_0DB; // 0x10000;
S32 mic_gain   = CONFIG_AUDIOIN_DEFAULT_0DB;  // 0x00800;


static void localaudio_fat_task(void *config_data);


//__weak void localaudio_new_codec2_inputstream(const tdvstream *dva) { }

//__weak void localaudio_process_encoded_frame(const tdvstream *dva, const uint8_t *frame) { }

//__weak void localaudio_codec2_inputstream_ends(const tdvstream *dva) { }


#define AUDIO_FRAME_LEN_MS  40
#define AUDIO_NO_OF_FRAMES  24


bool localaudio_trigger_rx(tsimple_buffer *rx_buf, void *user_data) {
  BaseType_t isAwakeYeah = pdFALSE;
  xEventGroupSetBitsFromISR(la_events, LOCALAUDIO_HANDLE_INPUTS, &isAwakeYeah);
  return isAwakeYeah;
}


esp_err_t localaudio_start(void) {
  esp_err_t res;

  ti2sloop_config i2s_txconf, i2s_rxconf;
  tsimple_buffer *audio_buf;
  tsimple_buffer *input_buf;

  // Todo more stuff,
  ESP_LOGD(TAG, "start local audio...");
  MAX98357A_Init();

  la_events      = xEventGroupCreate();
  la_wait4events = 0;
  memset(DVSchannel, 0, sizeof(DVSchannel));

  MAX98357A_get_config(&i2s_txconf, 8000, 16);

  INMP441_get_config(&i2s_rxconf, 8000);


  audio_buf = calloc(1, sizeof(tsimple_buffer));
  if (audio_buf == NULL) {
     ESP_LOGE(TAG, "no memory for audio loop buffer struct");
     return ESP_ERR_NO_MEM;
  }

  // INMP411 needs 32+32 clocks per frame!
  //i2s_txconf.slot.ws_width = 32;
  //i2s_txconf.slot.bitwidth = 32;

  unsigned int samples_per_frame = AUDIO_FRAME_LEN_MS * i2s_txconf.sample_rate_hz / 1000;
  unsigned int bytes_per_frame   = (i2s_txconf.slot.databits * samples_per_frame) >> 3;

// ToDo set and sync samplerate (8000 or 16kHz) here! Use #defines for buffer len
  res = i2sloop_create_loop(audio_buf, AUDIO_NO_OF_FRAMES, bytes_per_frame, samples_per_frame); // 20ms frames 160 samples
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "no memory for audio loop buffer (%d bytes)", AUDIO_NO_OF_FRAMES * bytes_per_frame);
    return res;
  }
  ESP_LOGD(TAG, "audio buffer %d x %d bytes", AUDIO_NO_OF_FRAMES, bytes_per_frame);

  input_buf = calloc(1, sizeof(tsimple_buffer));
  if (input_buf == NULL) {
     ESP_LOGE(TAG, "no memory for audio input buffer struct");
     return ESP_ERR_NO_MEM;
  }

  // ein Test: i2s_rxconf.slot.databits = 16;
  samples_per_frame = AUDIO_FRAME_LEN_MS * i2s_rxconf.sample_rate_hz / 1000;
  bytes_per_frame   = (i2s_rxconf.slot.databits * samples_per_frame) >> 3;  
  res = i2sloop_create_loop(input_buf, 2, bytes_per_frame, samples_per_frame); // 20ms frames 160 samples
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "no memory for audio loop buffer (%d bytes)", 2 * bytes_per_frame);
    return res;
  }
  ESP_LOGD(TAG, "input buffer 2 x %d bytes", bytes_per_frame);

  res = i2sloop_init(0, &i2s_txconf, audio_buf, &i2s_rxconf, input_buf);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "I2S init failed - can't continue");
    return res;
  }

  res = i2sloop_set_rx_handler(0, localaudio_trigger_rx, NULL);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "I2S set rx handler failed - can't continue");
    return res;
  }

  return xTaskCreate(localaudio_fat_task, "LocalAudio", LOCALAUDIO_TASK_STACK_SIZE, audio_buf, LOCALAUDIO_TASK_PRIORITY, &la_task_handle) == pdPASS? ESP_OK: ESP_FAIL;
}


static void localaudio_add_channel(U8 ch_num) {
  if (ch_num > LOCALAUDIO_MAX_STREAMS) return;
  ESP_LOGD(TAG, "add stream channel %d to event group.", (int)ch_num);
  la_wait4events |= 1 << ch_num;
  LA_Event(LOCALAUDIO_UPDATE_EVENTS);  
}


static void localaudio_remove_channel(U8 ch_num) {
  EventBits_t ch_mask2clr;
  if (ch_num > LOCALAUDIO_MAX_STREAMS) return;
  ch_mask2clr = 1 << ch_num;
  la_wait4events &= ~ch_mask2clr;
  LA_ClrEvent(ch_mask2clr);
  ESP_LOGD(TAG, "remove stream channel %d from event group.", (int)ch_num);
  LA_Event(LOCALAUDIO_UPDATE_EVENTS);  
}


static int localaudio_get_free_stream(void) {
  int stream_id;
  for (stream_id = 0; stream_id < LOCALAUDIO_MAX_STREAMS; stream_id++) {
     if (DVSchannel[stream_id].kind == DVS_UNUSED) {
       ESP_LOGD(TAG, "new stream #%d", stream_id);
       return stream_id;
     }
  }
  ESP_LOGD(TAG, "no new stream channel");
  return -1;
}


static int localaudio_find_stream(const tdvstream_channel *dvs) {
  int stream_id;
  for (stream_id = 0; stream_id < LOCALAUDIO_MAX_STREAMS; stream_id++) {
     if (dvs == &DVSchannel[stream_id]) return stream_id;
  }
  ESP_LOGD(TAG, "stream not found");
  return -1;
}

static int localaudio_find_stream_dva(const tdvstream *dva) {
  int stream_id;
  for (stream_id = 0; stream_id < LOCALAUDIO_MAX_STREAMS; stream_id++) {
     if (dva == (const tdvstream *) &DVSchannel[stream_id].dva) return stream_id;
  }
  ESP_LOGD(TAG, "stream not found");
  return -1;
}



static U32 localaudio_get_srate(const tdvstream *dva) {
  return ((U32)dva->samples_per_frame * HAMdLNK_SP_S) / dva->srate;
}





#define M_PI_M2 ( M_PI + M_PI )

typedef struct {
  double		accumulator;
  float			hz;
  unsigned int	  volume;
  unsigned int		samplerate;
  unsigned int		length;
} ttone_data;


static void fill_sine_2_buffer(ttone_data *tone, S16 *dest, U8* unused_in) {
  int16_t val;
  if ((dest == NULL) || (tone == NULL)) return;
  for (int i = 0; i < tone->length; i++) {
    tone->accumulator += M_PI_M2 * tone->hz / tone->samplerate;
    if (tone->accumulator >= M_PI_M2) tone->accumulator -= M_PI_M2;
    val = sin(tone->accumulator) * (float) tone->volume;
    *dest++ = val;
  } // rof
}


ttone_data *create_sine(unsigned int freq_hz, float volume, int length_buffer) {
  ttone_data *test_ton;
  test_ton = malloc(sizeof(ttone_data));
  assert(test_ton);
  test_ton->accumulator = 0.0;
  test_ton->hz = freq_hz;
  test_ton->volume = 32767.0 * volume;
  test_ton->samplerate = 8000;
  test_ton->length = length_buffer;
  return test_ton;
}

int destroy_sine_struct(void *userdata) {
  tdvstream_channel *dvs = (tdvstream_channel *) userdata;
  if (dvs == NULL) return -1;
  if (dvs->codec_struct != NULL) {
    dvs->codec_struct = NULL;
  }
  dvs->destroy = NULL;
  return 0;
}


void *localaudio_create_sine(tdvstream_channel *dvs, int mode) {
  
  dvs->codec_struct = create_sine(440, 1.0, 160);

  if (dvs->codec_struct == NULL) return NULL;
  
  dvs->dva->srate = 8000;
  dvs->destroy    = destroy_sine_struct;

  dvs->dva->samples_per_frame = 160;
  dvs->dva->bits_per_frame  = 64;
  dvs->dva->bytes_per_frame = (dvs->dva->bits_per_frame + 7) >> 3;

  dvs->prcs_out = (tdecode_frame_func) fill_sine_2_buffer;
  return dvs;
}




int destroy_C2_struct(void *userdata);


void localaudio_codec2_ber_wrapper(void *codec_struct, S16 *samples_out, const U8 *bytes_in) {
  codec2_decode_ber(codec_struct, samples_out, bytes_in, 0.0);
}


void *localaudio_create_codec2(tdvstream_channel *dvs, int mode) {
  dvs->codec_struct = codec2_create(mode);
  if (dvs->codec_struct == NULL) return NULL;
  codec2_set_natural_or_gray(dvs->codec_struct, 0);         // turn off gray-encoding (default) 4 quantizers
  
  dvs->dva->srate = 8000;
  dvs->destroy    = destroy_C2_struct;

  dvs->dva->samples_per_frame = codec2_samples_per_frame(dvs->codec_struct);
  dvs->dva->bits_per_frame  = codec2_bits_per_frame(dvs->codec_struct);
  dvs->dva->bytes_per_frame = (dvs->dva->bits_per_frame + 7) >> 3;

  dvs->prcs_in  = (tencode_frame_func) ((struct CODEC2 *)dvs->codec_struct)->encode;  // direct link
  dvs->prcs_out = (tdecode_frame_func) ((struct CODEC2 *)dvs->codec_struct)->decode;  // direct link
  if (dvs->prcs_out == NULL) dvs->prcs_out = localaudio_codec2_ber_wrapper; // needed for 1300baud mode

// ToDo from settings
  if (mode == CODEC2_MODE_700C) {
    codec2_700c_eq(dvs->codec_struct, true);
    //codec2_700c_post_filter(dvs->codec_struct, true); <- is default
  }
  ESP_LOGD(TAG, "codec2 created");
  return dvs;
}


int destroy_C2_struct(void *userdata) {
  tdvstream_channel *dvs = (tdvstream_channel *) userdata;
  if (dvs == NULL) return -1;

  if (dvs->codec_struct != NULL) {
    codec2_destroy(dvs->codec_struct);
    dvs->codec_struct = NULL;
    ESP_LOGD(TAG, "c2 destroyed");
  }
  dvs->destroy = NULL;
  return 0;
}


int destroy_MBE_struct(void *userdata);

#define MBE_MODE_MIN			0
#define MBE_MODE_IMBE_7200x4400		0
#define MBE_MODE_IMBE_7100x4400		1
#define MBE_MODE_AMBE_3600x2400		2
#define MBE_MODE_AMBE_3600x2450		3
#define MBE_MODE_MAX			3

void *localaudio_create_mbe(tdvstream_channel *dvs, int mode) {
  if ((mode >= MBE_MODE_MIN) && (mode <= MBE_MODE_MAX)) {
    //dva->mbe = mbe_create(mode);
    //if (dva->mbe == NULL) return NULL;
    //mbe_set_quality(dva->mbe, 4);
    dvs->destroy    = destroy_MBE_struct;

    dvs->dva->srate             = 8000;
    dvs->dva->samples_per_frame = 160; //mbe_samples_per_frame(dva->mbe);
    dvs->dva->bits_per_frame    = 48; //mbe_bits_per_frame(dva->mbe);
    dvs->dva->bytes_per_frame   = (dvs->dva->bits_per_frame + 7) >> 3;
    return dvs;
  }
  return NULL;
}


int destroy_MBE_struct(void *userdata) {
  tdvstream_channel *dvs = (tdvstream_channel *) userdata;
  if (dvs == NULL) return -1;

  if (dvs->codec_struct != NULL) {
    //mbe_destroy(dvs->mbe);
    dvs->codec_struct = NULL;
  }
  dvs->destroy = NULL;
  return 0;
}


// ToDo
int destroy_dva_buffer(void *userdata) {
  tdvstream *dva = (tdvstream *) userdata;
  if (dva == NULL) return -1;
  /*
  if (dva->buffer) {
    free(dva->buffer);
    dva->buffer = NULL;
  }
  */
  return 0;
}


void *localaudio_create_pcm8(tdvstream_channel *dvs, int mode) {
  if (mode > 1) return NULL;

  dvs->destroy = NULL; //destroy_PCM8_struct;

  dvs->dva->srate             = 8000;
  dvs->dva->samples_per_frame = 80;
  dvs->dva->bits_per_frame    = 640;
  dvs->dva->bytes_per_frame   = (dvs->dva->bits_per_frame + 7) >> 3;
  return dvs;
}



bool localaudio_create_buffer(tdvstream *dva, int size_ms) {
  U16 frames;
  if ((size_ms <= 0) || (size_ms > 30000)) return NULL;
  frames = size_ms / (1000 * dva->samples_per_frame / dva->srate);
  if (frames < 2) frames = 2;

  if (dva->buf == NULL) {
    dva->buf = (tsimple_buffer *) malloc(sizeof(tsimple_buffer));
  }

  if (sbuf_create(dva->buf, frames, dva->bytes_per_frame, localaudio_get_srate(dva))) {
    //dva->buffer = (void *)dva->buf->start_ptr;
  }
  return dva->buf->start_ptr != NULL;
}




int localaudio_create_codec2_outputstream(tdvstream *dva, int mode, unsigned short data_per_packet, tfinished_stream_fct fini_cb) {
  int stream_id;
  tdvstream_channel *dvs;

  if (dva == NULL) return -2;
  stream_id = localaudio_get_free_stream();
  if (stream_id < 0) {
    ESP_LOGW(TAG, "no free stream available");
    return stream_id;
  }

  dvs = &DVSchannel[stream_id];

  dvs->kind = DVS_OUTFINISH;
  dvs->finished_cb = NULL;
  dvs->dva  = dva;

  if (localaudio_create_codec2(dvs, mode) == NULL) {
    ESP_LOGE(TAG, "create_audiostream: codec2_create() fails");
    dvs->kind = DVS_UNUSED;
    return -2;
  }

//    dva->codec_type = codec_type;
  int ms_per_packet = (data_per_packet / dva->bytes_per_frame) * dva->samples_per_frame * 1000 / dva->srate;
  int buf_size_ms = 3 * ms_per_packet;
  if (!localaudio_create_buffer(dva, buf_size_ms)) {	// create buffer for (encoded data 4 audio-task access)
    ESP_LOGE(TAG, "create_audiostream: create buffer (%d ms) fails", buf_size_ms);
  }

  dvs->iobuffer = malloc(dva->samples_per_frame * sizeof(S16));
  if (dvs->iobuffer) {
    dvs->iobuffer_size = dva->samples_per_frame;
    dvs->iobuffer_wrpos = 0;
  }
  dvs->finished_cb = fini_cb;
  dva->streamid = stream_id;
  dvs->kind     = DVS_OUTPUT;
  ESP_LOGD(TAG, "codec2 stream with %d bytes/%d samples per frame (encoded buffer: %dms)", dva->bytes_per_frame, dva->samples_per_frame, buf_size_ms);
  localaudio_add_channel(stream_id);
  return stream_id;
}



int localaudio_create_codec2_inputstream(tdvstream *dva, int mode, unsigned int trigger_after_ms, thandle_inputstream_funct handler, void *userdata) {
  tdvstream_channel *dvs;
  int stream_id;
  int buf_size_ms;
  unsigned short frame_len_ms;
  unsigned short no_of_frames;

  if (dva == NULL) return -2;
  stream_id = localaudio_get_free_stream();
  if (stream_id < 0) return stream_id;

  dvs = &DVSchannel[stream_id];

  const tsimple_buffer *rx_buf = i2sloop_get_rx_buffer(0);
  if (rx_buf == NULL) {
    ESP_LOGW(TAG, "create_inputstream: no i2s receivebuffer defined");
    return -1;
  }
  dvs->kind = DVS_INPUTENDS;
  dvs->dva  = dva;
  dvs->received_cb = NULL;

  if (localaudio_create_codec2(dvs, mode) == NULL) {
    ESP_LOGE(TAG, "create_inputstream: codec2_create() fails");
    dvs->kind = DVS_UNUSED;
    return -2;
  }

  frame_len_ms = (U32)dva->samples_per_frame * 1000 / dva->srate;
  no_of_frames = 2 * trigger_after_ms / frame_len_ms;
  if (no_of_frames & 1) no_of_frames++; // even numbers

  if (dva->buf == NULL) {
    dva->buf = (tsimple_buffer *) malloc(sizeof(tsimple_buffer));
  }
  if (dva->buf == NULL) {
    ESP_LOGE(TAG, "no memory for dva simple buffer struct");
    return -3;
  }
  if (!sbuf_create(dva->buf, no_of_frames, dva->bytes_per_frame, localaudio_get_srate(dva))) {
    ESP_LOGE(TAG, "no memory for dva simple buffer");
    return -3;    
  }
  
  ESP_LOGD(TAG, "DV buffer %d bytes, %d steps per frame", dva->buf->size, dva->buf->steps_per_frame);

  if (dva->samples_per_frame != rx_buf->steps_per_frame) {
    U16 frame_size = (rx_buf->frame_size / rx_buf->steps_per_frame) * dva->samples_per_frame; // 24bit == 3 * samples_per_frame
    ESP_LOGD(TAG, "reconfigure input buffer (%d->%d) | %dms frame (size %d)", rx_buf->steps_per_frame, dva->samples_per_frame, frame_len_ms, frame_size);
    i2sloop_reconfig_rx(0, 2, frame_size, dva->samples_per_frame);
    LA_Event(LOCALAUDIO_UPDATE_EVENTS);
    //rx_buf = i2sloop_get_rx_buffer(0);
    taskYIELD();
  }

  buf_size_ms = rx_buf->size * (1000/sizeof(S16)) / dva->srate;

  dvs->received_cb = handler;
  dvs->user_data   = userdata;
  dva->streamid    = stream_id;
  dvs->kind        = DVS_INPUT;
  // start sampling
  i2sloop_enable_rx(0);

  // do 3rd party action (C2LORA) with this new stream:
  //localaudio_new_codec2_inputstream(dva);

  ESP_LOGD(TAG, "codec2 stream with %d frames per pkt, %d bytes/%d samples per frame (I2S audio buffer: %dms)", no_of_frames, dva->bytes_per_frame, dva->samples_per_frame, buf_size_ms);
  return stream_id;
}


int localaudio_end_stream(tdvstream *dva, bool no_more_data) {
  int stream_id;
  tdvstream_channel *dvs;

  if (dva == NULL) return -2;
  stream_id = dva->streamid;
  if ((stream_id < 0) || (stream_id >= LOCALAUDIO_MAX_STREAMS)) {
    stream_id = localaudio_find_stream_dva(dva);
  }
  if (stream_id < 0) {
    ESP_LOGD(TAG, "stream not found");
    return -1;
  }

  dvs = &DVSchannel[stream_id];
  switch (dvs->kind) {
  case DVS_INPUT:
  case DVS_OUTPUT:
  case DVS_TRANSCODE:
    dvs->kind++; // switch to ...ENDS state
    ESP_LOGD(TAG, "stopping stream #%d...", stream_id);
    if (no_more_data) {
      LA_Event(1 << stream_id);
    }
    break;  
  default:
    ESP_LOGW(TAG, "stream #%d not active - ignore", stream_id);
    return -3;
  } // hctiws
  return 0;
}


void localaudio_destroy_stream(tdvstream_channel *dvs) {
  int stream_id = dvs->dva->streamid;
  if ((dvs->dva->streamid < 0) || (dvs->dva->streamid >= LOCALAUDIO_MAX_STREAMS)) {
    stream_id = localaudio_find_stream(dvs);
  }
  ESP_LOGD(TAG, "destroy stream #%d (%d)", stream_id, dvs->kind);

  if ((dvs->kind == DVS_OUTFINISH) && dvs->finished_cb) { // only output streams having finisher
    dvs->finished_cb(dvs->dva);
  }

  if (dvs->destroy) dvs->destroy(dvs);

  if (dvs->dva->buf) {
    if (dvs->dva->flags & DVSTREAM_FLAG_KEEP_BUFFER) {  
      sbuf_flush(dvs->dva->buf);
    } else {
      sbuf_destroy(dvs->dva->buf);
      free(dvs->dva->buf);
      dvs->dva->buf = NULL;
    }
  } // fi have buffer
  if (dvs->iobuffer != NULL) {
    free(dvs->iobuffer);
    dvs->iobuffer = NULL;
    dvs->iobuffer_size = 0;
  }
  localaudio_remove_channel(stream_id);
  dvs->dva->streamid = -1;
  dvs->kind = DVS_UNUSED;
}



void localaudio_handle_stream(int stream_id) {
  if ((stream_id >= 0) && (stream_id <= LOCALAUDIO_MAX_STREAMS)) {
    LA_Event(1 << stream_id);
  }
}


void localaudio_stop_stream(int stream_id, bool no_more_data) {
  // ToDo Notlösung
  if ((stream_id < 0) || (stream_id >= LOCALAUDIO_MAX_STREAMS)) return;
  DVSchannel[stream_id].kind = DVSchannel[stream_id].kind == DVS_OUTPUT? DVS_OUTFINISH: DVS_UNUSED;  
  if (no_more_data) {
    LA_Event(1 << stream_id);
  }
}


/* la_load24bit_with_gain()
 * load raw 24bit microphone data into a 16bit buffer. Multiplies it withc gain (0x0010000 = 0dB)
 * need to be optimized and handling result with saturation instead overflow
 */
static inline int la_load24bit_with_gain(S16 *dest, const U8 *src, int samples, S32 gain) {
  S16 *dest_ptr = dest;
  for (int cnt = samples; cnt > 0; cnt--, src += 3, dest_ptr++) {
    Union32 din;
    din.s32   = src[2] & 0x80? -1: 0; // init 32bit value depending on sign
    din.u8[0] = src[0];
    din.u8[1] = src[1];
    din.u8[2] = src[2];
    //memcpy(&din, src, 3);
    //if (din.u8[input_raw_s_size-1] & 0x80) din.u8[3] = 0xFF;
    din.s32 *= gain;
    *dest_ptr = din.s16[1];
  } // rof
  return samples;
}



#define set_start_us(strt)      { strt = esp_timer_get_time(); }
#define get_duration_us(strt)   ((uint32_t)(esp_timer_get_time() - strt))

extern int gap_counter;


static void localaudio_fat_task(void *thread_data) {
  TickType_t task_timeout = portMAX_DELAY;
  TickType_t disable_audio_tick;
  int  input_samples = 0;

  int  sum_samples_read, sum_samples, frames;  
  bool output_enabled;
  bool output_have_stream;

#if CONFIG_LOG_DEFAULT_LEVEL > 3
  uint32_t sum_ticks_encode = 0, min_tick_encode = 0, max_tick_encode = 0, tick_cnt_encode = 0;
  uint32_t sum_ticks_decode = 0, min_tick_decode = 0, max_tick_decode = 0, tick_cnt_decode = 0;
#endif

  tsimple_buffer *audio_out = (tsimple_buffer *)thread_data;
  tsimple_buffer *input_raw = i2sloop_get_rx_buffer(0);

  S16 *input_buf      = NULL;
  U16 input_samples_per_frame = 0;
  U8  input_raw_s_size = 0;

  if (input_raw != NULL) {
    input_samples_per_frame = input_raw->steps_per_frame;
    input_buf = calloc(1, input_samples_per_frame * sizeof(S16));
    input_raw_s_size = input_raw->frame_size / input_samples_per_frame;
  } // fi processed input buffer
  
  // toDo
  const void *audio_buf_watermark = audio_out->start_ptr + (LOCALAUDIO_OUTPUT_WATERMARK_MS * audio_out->frame_size / 20);
  // if write_ptr > this: indicates that enough data stored to fire up I2S

  if (la_events == 0) {
    ESP_LOGE(TAG, "no events - exit localaudio thread");
  }

#ifdef ENABLE_PAPAGAI
  tsimple_buffer papagei;
  sbuf_create(&papagei, 50, 8, 160);
#endif
  output_enabled     = false;
  output_have_stream = false;
  disable_audio_tick = portMAX_DELAY;
  ESP_LOGD(TAG, "task ready (initial stack %u of %u free) on core %d", uxTaskGetStackHighWaterMark(NULL), LOCALAUDIO_TASK_STACK_SIZE, xPortGetCoreID());

  while(1) {
    
    EventBits_t la_chan_bits = xEventGroupWaitBits(la_events, la_wait4events|LOCALAUDIO_HANDLE_INPUTS|LOCALAUDIO_UPDATE_EVENTS, pdTRUE, pdFALSE, task_timeout);
    TickType_t  curr_tick    = xTaskGetTickCount();

    if ((!output_have_stream && output_enabled) && (curr_tick >= disable_audio_tick)) {
      output_enabled = false;
      i2sloop_disable_tx(0);        
      MAX98357A_enable(false); // ToDo
      ESP_LOGD(TAG, "audio out disabled");
    } // fi kill audio-out

    if (la_chan_bits == 0) {  // handle timeout
      task_timeout = portMAX_DELAY;
      ESP_LOGD(TAG, "fat_task timeout");
    } else {
      ESP_LOGV(TAG, "fat_task triggered %04lx, waits %04lx", la_chan_bits & 0xFFFF, la_wait4events);
    }

    if (la_chan_bits & LOCALAUDIO_UPDATE_EVENTS) {
      ESP_LOGD(TAG, "fat_audio_task - la_chan_bits updated to %04lx", la_wait4events);
      bool output_audio = false;
      for (int ch=0; ch < LOCALAUDIO_MAX_STREAMS; ch++) {
        switch(DVSchannel[ch].kind) {
        case DVS_OUTPUT:
          output_audio = true;
          break;
        case DVS_INPUT:
#if CONFIG_LOG_DEFAULT_LEVEL > 3
        sum_ticks_encode = 0; 
        min_tick_encode  = 999999;
        max_tick_encode  = 0;
        tick_cnt_encode  = 0;
#endif
          break;
        default:
          break;
        } // hctiws
      } // rof
      if (output_audio && !output_have_stream) {
        MAX98357A_enable(true);
        sbuf_flush(audio_out);
        task_timeout     = pdMS_TO_TICKS(LOCALAUDIO_OUTPUT_TIMEOUT_MS) - 1;
 #if CONFIG_LOG_DEFAULT_LEVEL > 3
        sum_ticks_decode = 0; 
        min_tick_decode  = 999999;
        max_tick_decode  = 0;
        tick_cnt_decode  = 0;
#endif       
      } // fi turn on audio
      if (output_have_stream && !output_audio) {
        unsigned int unsend = sbuf_get_unsend_size(audio_out);
        i2sloop_shutdown_tx(0);
        task_timeout = pdMS_TO_TICKS((unsend * 20 / audio_out->frame_size) + 40);
  #if CONFIG_LOG_DEFAULT_LEVEL > 3
        printf("decode function %lu times called, average %7.2fµs, min %luµs max %luµs\n", tick_cnt_decode, (float)sum_ticks_decode / tick_cnt_decode, min_tick_decode, max_tick_decode);
        ESP_LOGD(TAG, "%u samples unsed, high stack water mark is %u", unsend >> 1, uxTaskGetStackHighWaterMark(NULL));
        ESP_LOGD(TAG, "%d gaps inserted", gap_counter);
  #endif
      } // fi turn off audio
      output_have_stream = output_audio;
      disable_audio_tick = curr_tick + task_timeout - 1;

      // test if input buffer-size has changed (need for other codec frame length)
      if ((input_raw != NULL) && (input_samples_per_frame != input_raw->steps_per_frame)) {
        input_samples_per_frame = input_raw->steps_per_frame;
        if (input_buf != NULL) free(input_buf);
        input_buf = calloc(1, input_samples_per_frame * sizeof(S16));
        input_raw_s_size = input_raw->frame_size / input_samples_per_frame;
      } // fi processed input buffer

    }

    frames  = 0;
    sum_samples_read = 0;
    // process all data until buffers are empty

    do {
      int samples_read = 0;
      
      sum_samples   = 0;
      input_samples = 0;

      if (la_chan_bits & LOCALAUDIO_HANDLE_INPUTS) {
        // first get new raw samples and size
        ESP_LOGV(TAG, "input raw event");
        U32 frame_in_size = 0;
        const U8 *frame_in_ptr  = sbuf_get_unsend_block(input_raw, &frame_in_size);
        // only if we have enough input...
        if (frame_in_size >= input_raw->frame_size) {
          la_chan_bits &= ~LOCALAUDIO_HANDLE_INPUTS;  // we handle that now
          switch (input_raw_s_size) {
          case 3:
            input_samples = la_load24bit_with_gain(input_buf, frame_in_ptr, input_samples_per_frame, mic_gain);
            break;
          case 2: // just copy
            memcpy(input_buf, frame_in_ptr, input_samples_per_frame * sizeof(S16));
            input_samples = frame_in_size;
            break;
          break;
          } // hctiws
/*
          S16 *input_ptr = input_buf;          
          for (int cnt = input_samples_per_frame; cnt > 0; cnt--, frame_in_ptr += input_raw_s_size, input_ptr++) {
            Union32 din = { .u32 = 0 };
            memcpy(&din, frame_in_ptr, input_raw_s_size);
            if (din.u8[input_raw_s_size-1] & 0x80) din.u8[3] = 0xFF;
            din.s32 *= mic_gain;
            *input_ptr = din.s16[1];
          } // rof
          input_samples = input_samples_per_frame;
*/          
          sbuf_unsend_block_processed(input_raw, frame_in_size);
          // ToDo wrap over

          // trigger input handlers
          for (int ch=0; ch < LOCALAUDIO_MAX_STREAMS; ch++) {
            tdvstream_channel *dvs = &DVSchannel[ch];            
            if (dvs->kind == DVS_INPUT) {
              if ((dvs->dva == NULL) || (dvs->dva->buf == NULL)) {
                ESP_LOGW(TAG, "input stream #%d w/o stream or buffer!", ch);
                continue;
              }
              if ((dvs->prcs_in == NULL) || (dvs->codec_struct == NULL)) {
                ESP_LOGW(TAG, "input stream #%d w/o handler or codec-struct", ch);
                continue;
              }
              if (input_samples >= dvs->dva->samples_per_frame) {
                la_chan_bits |= 1 << ch;  // update channel stuff
                ESP_LOGV(TAG, "trigger input stream #%d", ch);
              } else {
                ESP_LOGV(TAG, "input %d of %d samples", input_samples, dvs->dva->samples_per_frame);
              }
            }
          } // rof

        } // fi have new input data        
      } // fi handle raw input

      for (int ch=0; ch < LOCALAUDIO_MAX_STREAMS; ch++) {
        tdvstream_channel *dvs = &DVSchannel[ch];

        if ( !(la_chan_bits & (1 << ch)) ) continue; // do nothing if not triggered

        tdvstream *dva = dvs->dva;

        switch (dvs->kind) {
        case DVS_INPUTENDS:
        case DVS_INPUT:
          if (input_samples >= dva->samples_per_frame) {
            U8 *encoded_out = dva->buf->write_ptr;
            la_chan_bits &= ~ (1 << ch);  // clear channel bit
#if CONFIG_LOG_DEFAULT_LEVEL > 3
            int64_t strt;
            uint32_t duration;
            set_start_us(strt);
#endif       
            dvs->prcs_in(dvs->codec_struct, encoded_out, input_buf);
#ifdef ENABLE_PAPAGAI
            if (sbuf_get_unsend_size(&papagei) > dva->bytes_per_frame) {
              memcpy(encoded_out, papagei.unsend_ptr, dva->bytes_per_frame);
              sbuf_unsend_block_processed(&papagei, dva->bytes_per_frame);
            }  
#endif
//            localaudio_process_encoded_frame(dva, encoded_out);  // hand over Lora or Whatever

            encoded_out += dva->bytes_per_frame;
            if (encoded_out >= ((U8 *)dva->buf->start_ptr + dva->buf->size)) {
              encoded_out = (U8 *)dva->buf->start_ptr;
            } // fi                
            dva->buf->write_ptr = encoded_out;
#if CONFIG_LOG_DEFAULT_LEVEL > 3      
            duration = get_duration_us(strt);            
            sum_ticks_encode += duration; 
            tick_cnt_encode++;
            if (duration < min_tick_encode) min_tick_encode = duration;
            if (max_tick_encode < duration) max_tick_encode = duration;
            if ((tick_cnt_encode & 63) == 0) {
              ESP_LOGD(TAG, "encode function %lu times called, average %7.2fµs, min %luµs max %luµs", tick_cnt_encode, (float)sum_ticks_encode / tick_cnt_encode, min_tick_encode, max_tick_encode);              
            }
#endif
            // call callback handler after every 50% of buffer are filled (buffer is 2time frame_count given by localaudio_create_codec2_inputstream())
            if ((dvs->received_cb != NULL) && ((encoded_out == (void *)dva->buf->start_ptr) || (encoded_out == ((void *)dva->buf->start_ptr + (dva->buf->size >> 1)))) ) {
              dvs->received_cb(dva, dvs->user_data);
            }
            // Todo better outside, trigger set here
          } // fi enough samples for one frame-decode

          if (dvs->kind == DVS_INPUTENDS) {
            la_chan_bits &= ~ (1<< ch);
            i2sloop_disable_rx(0);
            localaudio_destroy_stream(dvs);
          }
          break;

        case DVS_OUTFINISH:
        case DVS_OUTPUT:
          if ((dvs->iobuffer == NULL)) goto localaudio_noIObuffer_fail;
          dvs->iobuffer_wrpos = 0;
          if (sbuf_check_unsend_frame(dva->buf)) {
            const U8 *encoded_in = dva->buf->unsend_ptr;
            S16 *samples_out     = dvs->iobuffer + dvs->iobuffer_wrpos;
 #if CONFIG_LOG_DEFAULT_LEVEL > 3
            int64_t  strt;
            uint32_t duration;
            set_start_us(strt);
#endif                      
            dvs->prcs_out(dvs->codec_struct, samples_out, encoded_in);
            sbuf_unsend_block_processed(dva->buf, dva->bytes_per_frame);
            samples_read = dvs->dva->samples_per_frame;
            dvs->iobuffer_wrpos += samples_read;
#if CONFIG_LOG_DEFAULT_LEVEL > 3                   
            duration = get_duration_us(strt);            
            sum_ticks_decode += duration; 
            tick_cnt_decode++;
            if (duration < min_tick_decode) min_tick_decode = duration;
            if (max_tick_decode < duration) max_tick_decode = duration;                  
#endif  
#ifdef ENABLE_PAPAGAI
            if (sbuf_get_size(&papagei) > dvs->dva->samples_per_frame) {
              sbuf_fillnext(&papagei, encoded_in, 1);
            } // fill buf
#endif
          } // fi have data

          //if (frames > 5) break;
          //samples_read = fill_sine_2_buffer(dvs->iobuffer, dvs->iobuffer_size, test_sine);

          if (dvs->kind == DVS_OUTFINISH) {
localaudio_noIObuffer_fail:
            la_chan_bits &= ~ (1<< ch);
            localaudio_destroy_stream(dvs);
          }
          break;

        case DVS_TRANCODEENDS:
        case DVS_TRANSCODE:
          if (dvs->kind == DVS_TRANCODEENDS) {
            la_chan_bits &= ~ (1<< ch);
            localaudio_destroy_stream(dvs);
          }
          break;      
        default:
          break;
        } // hctiws

        // ToDo mix buffers and apply volume
        sum_samples_read += sum_samples;

      } // rof all streams

      if (samples_read <= 0) continue;

      disable_audio_tick = curr_tick + task_timeout - 1;

// ToDo optimized version with saturation (no overflow)
      S16* out_ptr = (S16 *)audio_out->write_ptr;
      const S16 *end_ptr = (const S16 *) (audio_out->start_ptr + audio_out->size);
      for (int cnt = 0; cnt < samples_read; cnt++ ) {
        Union32 sample = { .s32 = 0 };        
        for (int ch = 0; ch < LOCALAUDIO_MAX_STREAMS; ch++) {
          tdvstream_channel *dvs = &DVSchannel[ch];
          if ((dvs->kind != DVS_OUTPUT) || (dvs->iobuffer == NULL)) continue;
          sample.s32 += dvs->iobuffer[cnt];
        } // rof all outputs
        // ToDo +++ overflow-free mul ***
        sample.s32 *= volume_raw;        
        out_ptr[0]  = sample.s16[1];
        out_ptr++;
        if ((const S16 *)out_ptr >= end_ptr) out_ptr = (S16 *)audio_out->start_ptr;        
      } // rof adding up
      // ToDo lock audio_out here
      audio_out->write_ptr = (void *)out_ptr;     

      sum_samples += samples_read;
 
      if (sum_samples > 0) frames++;

      if (!output_enabled && (audio_out->write_ptr >= audio_buf_watermark)) {
        i2sloop_enable_tx(0);
        output_enabled = true;
      }

      la_chan_bits |= xEventGroupClearBits(la_events, la_wait4events | LOCALAUDIO_HANDLE_INPUTS);   // update la_chan_bits

    } while (sum_samples > 0);

    //unsigned int unsend = sbuf_get_unsend_size(audio_out);
    //printf("%5d-%3d\n", unsend, gap_counter);
    //ESP_LOGD(TAG, "%d frames, samples: %d", frames, sum_samples_read);
/*
ToDo log sbuf_get_unsend_size(audio_out) for every trigger, find out why buffer is growing
*/

  } // ehliw forever

//localaudio_shutdown_task:
  ESP_LOGW(TAG, "fat audio task shutdown.");
  if (input_buf != NULL) free(input_buf);
  free((void *)audio_out->start_ptr);
  vTaskDelete(NULL);
}


float localaudio_get_volume_dB(void) {
  float volumef = 10 * log10(((double)volume_raw / CONFIG_AUDIOOUT_DEFAULT_0DB));
  return volumef;
}

void localaudio_set_volume_dBm4(signed char volume) {
  double volumef = CONFIG_AUDIOOUT_DEFAULT_0DB * pow10((double)volume / 40);
  volume_raw = round(volumef);
  ESP_LOGD(TAG, "volume set to %5.2fdB (%08lxh)", ((float)volume) / 4.0, volume_raw); 
}


float localaudio_get_micgain_dB(void) {
  float micgainf = 10 * log10(((double)mic_gain / CONFIG_AUDIOIN_DEFAULT_0DB));
  return micgainf;
}

void localaudio_set_micgain_dBm4(signed char micgain) {
  double micgainf = CONFIG_AUDIOIN_DEFAULT_0DB * pow10((double)micgain / 40);
  mic_gain = round(micgainf);
  ESP_LOGD(TAG, "volume set to %5.2fdB (%08lxh)", ((float)micgain) / 4.0, mic_gain);
}
