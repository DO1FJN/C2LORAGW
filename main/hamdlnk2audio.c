
#include "hamdlnk2audio.h"

#include "HAMdLNK.h"
#include "hamdlnk_udp.h"
#include "C2LORA_hamdlnk.h"
#include "localaudio.h"

#include "s_buffer.h"
#include "utilities.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"

#include <string.h>

#include "sdkconfig.h"

typedef struct {
  char   sender[8];
  char   recipient[8];
  char   sender_info[32];
} tHAMdLNK_tx_info;

static char *TAG = "HdL2AUDIO";


static trtp_data      TXrtp;
static tdvstream      DVstream = { .streamid = -1 };

static tHAMdLNK_tx_info DVinfo;


struct netconn *get_udp_out_connection(bool for_dongle_audio); // Provisorium



esp_err_t hamdlnk_set_sender_info(const char *callsign, const char *recipient, unsigned short area_code, const char *locator) {
  if ((callsign == NULL)) return ESP_ERR_INVALID_ARG;

  int length = strnlen(callsign, 8);
  if (length > 4) {
    strncpy(DVinfo.sender, callsign, 8);
  }
  length = strnlen(recipient, 8);
  if((recipient != NULL) && (Check_Call(recipient, length))) {
    strncpy(DVinfo.recipient, recipient, 8);
  }
  if (locator != NULL) {
    snprintf(DVinfo.sender_info, sizeof(DVinfo.sender_info), "locator: %s", locator);
  }
  // Todo more 
  return ESP_OK;
}



int hamdlnk2la_create_audiostream(tdvstream *dva, U8 codec_type, U16 data_per_packet) {
  int stream_id = -1;
  if (dva == NULL) {
    ESP_LOGE(TAG, "create_audiostream: error no memory left");
    return -1;
  }
  memset(dva, 0, sizeof(tdvstream));
  dva->streamid = -1;
  dva->srate    = 8000;
  dva->codec_type = 255;

  if (isSF_CODEC2(codec_type)) {
    int codec2_mode = SF_CODEC2_get_mode(codec_type);
    stream_id = localaudio_create_codec2_outputstream(dva, codec2_mode, data_per_packet, hamdlnk_free_streamuserdata);
  } else switch(codec_type) {

  case SF_CODEC_AMBEplus2400:
    //int mbe_mode = SF_MBE_get_mode(codec_type);
    //stream_id = localaudio_create_mbe_outputstream(dva, mbe_mode, data_per_packet, hamdlnk_free_streamuserdata);
    break;
  case SF_AUDIO_PCM8A:
  case SF_AUDIO_PCM8U:
    //dva->destroy = destroy_dva_buffer;
    //stream_id = dvaudio_createoutputstream(fill_U8_from_buffer, dva->srate, codec_type==SF_AUDIO_PCM8A? SAMPLEFORMAT_ALAW: SAMPLEFORMAT_ULAW,
    //  dva, DVSTREAM_FLAG_FREE_USERDATA|DVSTREAM_FLAG_REALTIME|DVSTREAM_FLAG_ACTIVE);
    break;
  } // hctiws
  
  if (stream_id >= 0) {
    dva->codec_type = codec_type;
  } else {
    ESP_LOGE(TAG, "create_audiostream: localaudio denied");
  }
  return stream_id;
}


static bool hamdlnk2audio_update_dva(const thamdlnk_ref *packet, tdvstream *dva, U8 streamtype, U32 clocktick) {  
  U16 DVdata_length = 0;
  
  static const U8 supported_codec_list[] = {
    SF_AUDIO_PCM8A, SF_AUDIO_PCM8U,
    SF_CODEC_CODEC2_3200, SF_CODEC_CODEC2_2400, SF_CODEC_CODEC2_1600, SF_CODEC_CODEC2_1400,
    SF_CODEC_CODEC2_1300, SF_CODEC_CODEC2_1200, SF_CODEC_CODEC2_700C,
    0
  };
  
  if ((packet == NULL) || (dva == NULL)) return false;

  if (dva->streamid >= 0) { // we already have a stream
    return true;
  } else if ((streamtype != 255) && (!hamdlnk_getsubptr(packet, streamtype, 1, &DVdata_length))) {
    streamtype = 255;
  }
  if (streamtype == 255) {  // search for a DV stream
    streamtype = hamdlnk_findsubptr(packet, supported_codec_list, 1, &DVdata_length);
    if (streamtype != 255) {
#if CONFIG_LOG_DEFAULT_LEVEL > 3
      const char *codec_str = sf_get_subframe_name(streamtype);
      if (codec_str) {
        ESP_LOGD(TAG, "Found Codec %s", codec_str);
      } else {
        ESP_LOGD(TAG, "Found unknown Codec %02Xh", streamtype);
      }
#endif
    }
#if CONFIG_LOG_DEFAULT_LEVEL > 3
  } else {
    const char *codec_str = sf_get_subframe_name(streamtype);
    if (codec_str) {
      ESP_LOGD(TAG, "Use Codec %s", codec_str);
    } else {
      ESP_LOGD(TAG, "Use Codec %02Xh", streamtype);
    } 
#endif
  }
  
  if ((streamtype != 255) && (DVdata_length > 0)) {
    // we found on DV stream within the packet:
    if (dva->buf != NULL) {     // should not be a thing 
      free(dva->buf);
    }
    dva->streamid = hamdlnk2la_create_audiostream(dva, streamtype, DVdata_length);	// create audio stream
    if (dva->streamid >= 0) {
      sbuf_update_time(dva->buf, clocktick);
      sbuf_set_steppos(dva->buf, hamdlnk_gettimestamp(packet));
    }
    return (dva->streamid >= 0);
  }
  return false;
}


void hamdlnk2audio_parse_rxpkt(const thamdlnk_ref *packet, void *userdata, unsigned int clocktick) {
  U32	streamid;		// UDP-RTP stream (marker)
  tdvstream *dva;
  U8  streamtype = 255; // undef
  U16 DVdata_length;
  
  int listener = (int) userdata;

  streamid = hamdlnk_getstreamid(packet);

  dva = hamdlnk_get_streamuserdata(listener, streamid);

  if (dva == NULL) {		// we receive a new stream....
    U16	sender_len;
    char *sender = hamdlnk_getsubptr(packet, SF_SENDERID, 1, &sender_len);
    ESP_LOGD(TAG, "new DV stream %08lXh from '%.*s'", streamid, sender_len, sender);

    dva = malloc(sizeof(tdvstream));
    dva->streamid   = -1;
    dva->codec_type = 255;
    dva->buf        = NULL;
    hamdlnk_set_streamuserdata(listener, streamid, dva);
    if (dva == NULL) return;	// no more memory
  } // fi new stream

  // ToDo parse packets if within transmission and determine the first DV stream.

  if (hamdlnk_get_marker(packet)) {	// check only marked packets
    U16   controlmsglen;
    char *controlmsg = hamdlnk_getsubptr(packet, SF_CONTROL, 1, &controlmsglen);
    
    if ((controlmsg != NULL) && (controlmsglen >= CONTROLMSG_LEN)) {
      U32 controlcmd;      
      GET_CONTROLMSGID(controlcmd, controlmsg);
      switch (controlcmd) {
      case CONTROLMSG_STRT:
        if (controlmsglen==(CONTROLMSG_LEN+2)) {
          streamtype = controlmsg[CONTROLMSG_LEN+1];
        } // fi have Codec(Stream)Type
        break;
      case CONTROLMSG_END:
	      clocktick = 0;			// this updates the last_clocktick to 0, results in a immediate end after buffer is processed
  	    ESP_LOGD(TAG, "Stream %08lXh ends", streamid);
        localaudio_end_stream(dva, false);
        ESP_LOGD(TAG, ">>> high stack water mark is %u", uxTaskGetStackHighWaterMark(NULL));
        break;
      case CONTROLMSG_LOST:
	      clocktick = 0;
        ESP_LOGD(TAG, "Stream %08lXh ends (signal LOST)", streamid);
        localaudio_end_stream(dva, false);
        break;
      } // hctiws control message
    } // fi control messages
  } // fi marked packet

  if ( (dva->streamid < 0) && !hamdlnk2audio_update_dva(packet, dva, streamtype, clocktick) ) {
    return;
  }

  // get (main) DV data from this stream
  char *encoded_data = hamdlnk_getsubptr(packet, dva->codec_type, 1, &DVdata_length);
  U32 pkt_frame_pos  = hamdlnk_gettimestamp(packet);
  //  feed it into a simple stream buffer
  if (encoded_data == NULL) return; // no data in this packet!
  sbuf_fillpos(dva->buf, encoded_data, DVdata_length / dva->bytes_per_frame, pkt_frame_pos);
  sbuf_update_time(dva->buf, clocktick);
  ESP_LOGV(TAG, "Stream %08lXh stime: %6lu DVlen: %d", streamid, pkt_frame_pos, (int)DVdata_length);
  localaudio_handle_stream(dva->streamid);   // trigger audio task to process this output stream
}






// ***************************************************************************
// transmitting stuff
// ***************************************************************************

#define LOCALAUDIO_UNIQUE_STREAM_ID		0x0D1A0C02



/* hla_createsenderinfo(()
 * creates sender and recipient blocks into the HAMdLNK RTP packet based on
 * the localaudio_config
 */
static void hla_createsenderinfo(thamdlnk_data *pkt, const tHAMdLNK_tx_info *info) {
  U16 call_len = 0;

  // measure length of sender-calll
  while ((info->sender[call_len] > ' ') && (call_len < 8)) call_len++;	// max 8 chars for caller

  if ((call_len > 3)) {	// a valid sender?
    ham2lnk_addnew_subframe(pkt, SF_SENDERID, info->sender, call_len);	// copy only the first 8 chars
    call_len = strnlen(info->sender_info, sizeof(info->sender_info));
    if (call_len > 3 ) {
      ham2lnk_addnew_subframe(pkt, SF_SENDERINFO, info->sender_info, call_len);
    }
    // get recipient: we artificially limit this to 32 bytes here, sri
    call_len = strnlen(info->recipient, 8);
    if ((call_len > 3)) {
      ham2lnk_addnew_subframe(pkt, SF_RECIPIENT, info->recipient, call_len);
    }
    hamdlnk_set_marker(pkt);
  } // fi valid sender
}







esp_err_t ham2lnk_broadcast_audio(unsigned char codec_type, unsigned short dv_len_ms, bool interleave, const char *info_str) {  
  int audioin_stream = -1;
  tdvstream *dva = &DVstream;
  struct netconn *outconn = get_udp_out_connection(true);

  ESP_RETURN_ON_FALSE(outconn != NULL, ESP_ERR_NOT_FOUND, TAG, "UDP port not open");

  memset(&TXrtp, 0, sizeof(trtp_data));

  if (isSF_CODEC2(codec_type)) {
    int codec2_mode = SF_CODEC2_get_mode(codec_type);
    int trigger_ms  = interleave? dv_len_ms >> 1: dv_len_ms;

    audioin_stream = localaudio_create_codec2_inputstream(dva, codec2_mode, trigger_ms, UTX_handle_encoded_stream, &TXrtp);

    TXrtp.interleave = interleave;

  } else switch(codec_type) {

  case SF_CODEC_AMBEplus2400:
    //int mbe_mode = SF_MBE_get_mode(codec_type);
    break;
  case SF_AUDIO_PCM8A:
  case SF_AUDIO_PCM8U:
    break;
  } // hctiws
  
  if (audioin_stream >= 0) {
    dva->codec_type = codec_type;
  } else {
    ESP_LOGE(TAG, "create_audiostream: localaudio denied");
    return ESP_FAIL;
  }

  int frame_len_ms = (U32)dva->samples_per_frame * 1000 / dva->srate;
  int no_of_frames = dv_len_ms / frame_len_ms;
  if ((no_of_frames & 1) && (TXrtp.interleave)) no_of_frames++;	// even numbers!
  
  TXrtp.conn = outconn; // connect to default outgoing connection (network boradcast)
  
  TXrtp.chunk_len_sps   = hdl_get_srate(dva);
  TXrtp.subframe_id     = codec_type;
  TXrtp.chunk_len_bytes = dva->bytes_per_frame;
  TXrtp.chunk_count     = dva->buf->size / dva->buf->frame_size;  // a packet every 50% of buffer
  if (!TXrtp.interleave) TXrtp.chunk_count >>= 1;  
  if ((TXrtp.chunk_count & 1) && (TXrtp.interleave)) {
    TXrtp.chunk_count++;	// even numbers!
  }

  if (no_of_frames != TXrtp.chunk_count) {
    ESP_LOGW(TAG, "something is wrong! (%dms framelen; %d - %d chunks)", frame_len_ms, no_of_frames, TXrtp.chunk_count);
  }
  TXrtp.chunk_count = no_of_frames;

  ESP_LOGD(TAG, "TX ptp: %d frames per packet (%dms frame len, %d sps len), interleave=%s", TXrtp.chunk_count, frame_len_ms, TXrtp.chunk_len_sps, TXrtp.interleave?"true":"false");

  thamdlnk_data *tx_packet = &TXrtp.data;
  ham2lnk_create_new_stream(tx_packet, LOCALAUDIO_UNIQUE_STREAM_ID);
  hamdlnk_settimestamp(tx_packet, 0);

  // add sender info (static / must) to this stream
  hla_createsenderinfo(tx_packet, &DVinfo);
  
  ham2lnk_create_subframe(tx_packet, codec_type, dva->buf->size);

  hdl_stream_control_start(tx_packet, codec_type);

  return ESP_OK;
}


void ham2lnk_broadcast_stop(void) {
  tdvstream *dva = &DVstream;
  if (hamdlnk_get_packetsize(&TXrtp.data) > 0) {
    ESP_LOGD(TAG, "out end marker");  
    hdl_stream_control_end(&TXrtp.data);
    // get left DV frames and send out...
    TXrtp.force_transmit = true;
    UTX_handle_encoded_stream(dva, &TXrtp); // before TX, get the rest of available data...
  } // fi valid pkt
  localaudio_end_stream(dva, true);
/*
  if (dva->buf != NULL) {
    free(dva->buf);
    dva->buf = NULL;
  }*/
}


const tdvstream *ham2lnk_get_audio_stream(void) {
  return &DVstream;
}