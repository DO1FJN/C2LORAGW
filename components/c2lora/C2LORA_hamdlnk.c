/*

C2LORA_hamdlnk.c

This source file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

main API file for accessing the C2LORA UHF link

*/
#include "C2LORA_hamdlnk.h"

#include "C2LORA.h"
#include "C2LORA_modes.h"
#include "C2LORA_core.h"
#include "C2LORA_header.h"
#include "c2lora_uhf.h"

#include "hamdlnk_udp.h"

#include "s_buffer.h"
#include "sx126x.h"

#include "HAMdLNK.h"
#include "subframedef.h"
#include "utilities.h"

#include "LAN.h"
#include "lwip/api.h"
#include "lwip/udp.h"
#include "lwip/tcpip.h"
#include "lwip/dns.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"

#include <string.h>

#include "sdkconfig.h"


#define sx126x_lock()
#define sx126x_unlock()

#define MAX_LISTEN_CONNECTIONS          3

#define C2LORA_UNIQUE_STREAM_ID		      0x0D1A0C01

#define HAMDLNK_C2LORA_PKT_LEN          (C2LORA_DVOICE_FRAMELEN_MS/2)

#define HAMDLNK2AUDIO_TASK_STACK_SIZE   3072  //0x2800
#define HAMDLNK2AUDIO_TASK_PRIORITY     5

#define DNS_REQUEST_TIMEOUT_MS          800

#define MAX_UDPRX_GAP_MS                750


static const char *TAG = "C2LORA-UDP";

static const char *special_url_names[] = {
  "broadcast", "multicast", NULL
};

static TaskHandle_t   updrx_task_handle;
static TimerHandle_t  udprx_timer;

static StaticTimer_t  udprx_timer_struct;

static struct netconn *outconn = NULL;          // port for outgoing UDP packets

static uint16_t own_area_code;
static char     own_locator[6];

static tRecorderHandle *last_recording;         // keep the last record accessible


static void hamdlnk2audio_task(void *config_data);

static void hamdlnk_udprx_timeout(TimerHandle_t timer);

void hamdlnk2audio_parse_rxpkt(const thamdlnk_ref *packet, void *userdata, unsigned int clocktick);
void hamdlnk2c2lora_parse_rxpkt(const thamdlnk_ref *packet, void *userdata, unsigned int clocktick); // ToDo better


#if CONFIG_LOG_DEFAULT_LEVEL > 3
#define set_start_us(strt)      { strt = esp_timer_get_time(); }
#define get_duration_us(strt)   ((uint32_t)(esp_timer_get_time() - strt))

static int64_t strt;
#endif


struct netconn *get_udp_out_connection(bool for_dongle_audio) {
  return outconn;
}


esp_err_t C2LORA_start_udp_task(const tHAMdLNK_network_config *config) {
  BaseType_t udp_list_res;
  outconn = NULL;
  own_area_code = config->area_code;
  memcpy(own_locator, config->locator, sizeof(own_locator));
  udp_list_res = xTaskCreate(hamdlnk2audio_task, "HAMdLNK_listen", HAMDLNK2AUDIO_TASK_STACK_SIZE, (void *)config, HAMDLNK2AUDIO_TASK_PRIORITY, &updrx_task_handle);
  udprx_timer  = xTimerCreateStatic("UDP-Timeout", pdMS_TO_TICKS(MAX_UDPRX_GAP_MS), pdFALSE, NULL, hamdlnk_udprx_timeout, &udprx_timer_struct);
  // Todo more stuff...

  return udp_list_res == pdPASS? ESP_OK: ESP_FAIL;
}



static void log_ip_4_host(const ip_addr_t *ipaddr, const char *hostname, bool cached) {
#if CONFIG_LOG_DEFAULT_LEVEL > 3
    char ip_addr_str[128];
    ip_addr_str[0] = 0;
    ipaddr_ntoa_r(ipaddr, ip_addr_str, sizeof(ip_addr_str));
    ESP_LOGD(TAG, "got %s IP: %s for '%s'", cached? "cached": "", ip_addr_str, hostname);
#endif
}


static void my_dns_found_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg) {
  if (ipaddr != NULL) {     
    if (callback_arg != NULL) {
      memcpy(callback_arg, ipaddr, sizeof(ip_addr_t));
    }
    log_ip_4_host(ipaddr, name, false);
  } else {
    ESP_LOGW(TAG, "got no IP address for '%s'", name);
  }
  xTaskNotify(updrx_task_handle, ipaddr != NULL? 1: 0, eSetValueWithOverwrite);
}



static bool set_target_ip_by_name(ip_addr_t *target_ip, const char *dns_hostname) {
  bool ip_is_set = false;

  if ((dns_hostname == NULL) || (target_ip == NULL)) return false;

  int special_names = str_index(dns_hostname, special_url_names);
  if (special_names < 0) {
    uint32_t notify_value;    
    if (dns_gethostbyname(dns_hostname, target_ip, my_dns_found_callback, target_ip) == ERR_OK) {
      log_ip_4_host(target_ip, dns_hostname, true);
      ip_is_set = true;
    } else if (xTaskNotifyWait(-1, -1, &notify_value, pdMS_TO_TICKS(DNS_REQUEST_TIMEOUT_MS))) {
      ip_is_set = (bool) notify_value;
    }
  } else switch (special_names) {
  case 0: // broadcast
    target_ip->type = IPADDR_TYPE_V4;
    target_ip->u_addr.ip4.addr = IPADDR_BROADCAST;
    ip_is_set = true;
    ESP_LOGD(TAG, "sending IPv4 broadcasts");
    break;
  } // hctiws
  return ip_is_set;
}



static void hamdlnk2audio_task(void *config_data) {
  
  err_t     err;
  ip_addr_t TXTO;

  struct udp_pcb *mypcb[MAX_LISTEN_CONNECTIONS];
  const tHAMdLNK_network_config *config = config_data;
  const char *     outgoing_prim = NULL;
  uint16_t         outgoing_prim_port = 0;
  int              rx_connections = 0;
  treceive_handler rx_connection[MAX_LISTEN_CONNECTIONS];

  ESP_LOGD(TAG, "thread started");
  memset(mypcb, 0, sizeof(mypcb));

  if ((config == NULL) || ((config->listen_port_dongle == 0) && (config->listen_port_uhf == 0))) {
    ESP_LOGE(TAG, "config invalid, quit");
    vTaskDelete(NULL);
    updrx_task_handle = NULL;
    return;
  }

  tLoraStream *lora = C2LORA_get_stream_4_transmit();

  if ((config->listen_port_uhf > 0) && (lora->ctx != NULL) && (lora->state > LS_INACTIVE)) {
    rx_connection[0].port_no = config->listen_port_uhf;
    rx_connection[0].handler = hamdlnk2c2lora_parse_rxpkt;
    rx_connection[0].userdata = lora;
    rx_connections = 1;
    outgoing_prim      = config->sendto_hostname_uhf;
    outgoing_prim_port = config->sendto_port_uhf;
    ESP_LOGI(TAG, "forwarding port %d DV to UHF", config->listen_port_uhf);
  } // fi valid stream
#if CONFIG_LOCALAUDIO_ENABLED
  if (config->listen_port_dongle > 0) {
    rx_connection[rx_connections].port_no = config->listen_port_dongle;
    rx_connection[rx_connections].handler = hamdlnk2audio_parse_rxpkt;
    rx_connection[rx_connections].userdata = NULL;
    if (rx_connections == 0) {
      outgoing_prim      = config->sendto_hostname_dongle;
      outgoing_prim_port = config->sendto_port_dongle;
    }
    rx_connections++;
    ESP_LOGI(TAG, "forwarding port %d DV to speaker", config->listen_port_dongle);
  }
#endif
  for (int nc_conns = rx_connections; nc_conns < MAX_LISTEN_CONNECTIONS; nc_conns++) {
    rx_connection[nc_conns].handler = NULL; // not used
  } // rof

  LOCK_TCPIP_CORE();
  for (int c = 0; c < rx_connections; c++) {
    mypcb[c] = udp_new(); 
  } // rof create pcbs
  UNLOCK_TCPIP_CORE();

  hamdlnk_init_randomseed();	// we need a pseudo random seed for calc unique SSRC indentifiers

  ESP_LOGD(TAG, "ready (initial stack %u of %u free) on core %d", uxTaskGetStackHighWaterMark(NULL), HAMDLNK2AUDIO_TASK_STACK_SIZE, xPortGetCoreID());

  set_start_us(strt);
  while(1) {
    
    if (outconn == NULL) {
      outconn = netconn_new(NETCONN_UDP); //_IPV6 geht nicht auf IP4
    }

    ESP_LOGI(TAG, "wait network connect...");
    while (LAN_wait4event(CONNECTED_BIT, -1) == 0);

    err = ESP_OK;
    for (int c = 0; c < rx_connections; c++) {
      if (mypcb[c] == NULL) continue;
      LOCK_TCPIP_CORE();    
      err_t res = udp_bind(mypcb[c], IP_ANY_TYPE, rx_connection[c].port_no);
      UNLOCK_TCPIP_CORE();
      if (res == ESP_OK) {
        hamdlnk_receive(mypcb[c], &rx_connection[c]);
      }
      err |= res;
    } // rof
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "can't bind to port - it's in use (%d)", err);
      while (LAN_wait4event(DISCONNECTED_BIT, -1) == 0);
      continue;
    }
    ESP_LOGI(TAG, "ready 4 listen DV streams");
    
    if ((outgoing_prim_port > 0) && (outgoing_prim != NULL)) {
      bool ip_is_set = set_target_ip_by_name(&TXTO, outgoing_prim);
      if (ip_is_set) {
        err = netconn_connect(outconn, &TXTO, outgoing_prim_port);
        if (err != ERR_OK) {
          ESP_LOGE(TAG, "netconn_connect() returns %d", err);
          netconn_delete(outconn);
          outconn = NULL;
        } else {
          ESP_LOGI(TAG, "stream to '%s' (outgoing UDP)", outgoing_prim);
        }
      } else {
        ESP_LOGW(TAG, "no outgoing connection (no IP)");
      }

    } // fi tx port defined

    ESP_LOGD(TAG, "stack %u of %u free on core %d", uxTaskGetStackHighWaterMark(NULL), HAMDLNK2AUDIO_TASK_STACK_SIZE, xPortGetCoreID());

    unsigned int events;
    do {
      events = LAN_wait4event(DISCONNECTED_BIT | WIFI_SHUTDOWN_BIT, -1);
      // ToDo
    } while ((events & (DISCONNECTED_BIT | WIFI_SHUTDOWN_BIT)) == 0);

    ESP_LOGI(TAG, "disconnected");

    if (outconn != NULL) {
      netconn_disconnect(outconn);
    }
    for (int c = 0; c < rx_connections; c++) {
      if (mypcb[c] == NULL) continue;
      LOCK_TCPIP_CORE();
      udp_disconnect(mypcb[c]);
      UNLOCK_TCPIP_CORE();
    } // rof
    
  } // ehliw forever
  
  if (outconn != NULL) netconn_delete(outconn);
  for (int c = 0; c < MAX_LISTEN_CONNECTIONS; c++) {
    if (mypcb[c] == NULL) continue;
    LOCK_TCPIP_CORE();
    udp_remove(mypcb[c]);
    UNLOCK_TCPIP_CORE();
  }
  ESP_LOGE(TAG, "abnormal exit.");
  vTaskDelete(NULL);
}




/*

HAMdLNK UDP Handler Functions

*/

// ToDo Timeout after "FRAME_GAP" to disable UDP-FWD.


/*
  c2lora_finish_udp_tx() is called from C2LORA control task queued from C2LORA_handle_encoded() processing the "LAST_FRAME"
*/
static uint32_t c2lora_finish_udp_tx(tLoraStream *lora, void *arg) {
  esp_err_t err;
  ESP_RETURN_ON_FALSE(lora != NULL, ESP_ERR_INVALID_ARG, TAG, "no argument for udp-finisher call");
  lora->udp_stream_id = 0;
  if (lora->dva->flags & DVSTREAM_FLAG_KEEP_BUFFER) {  // dva / dva->buf kept?
    sbuf_flush(lora->dva->buf);
  } else {
    sbuf_destroy(lora->dva->buf);
    lora->dva->buf = NULL;
    free(lora->dva);    
  }
  lora->dva = NULL;
  if (lora->rec != NULL) {
    last_recording = lora->rec;
    lora->rec = NULL;
  }
  ESP_LOGD(TAG, "Fwd HAMdLNK done.");
  return (uint32_t) err;
}


/*
  c2lora_start_tx_udp() is called from C2LORA control task queued from C2LORA_start_tx_udp().
*/
static uint32_t c2lora_start_tx_udp(tLoraStream *lora, void * arg) {

  if (lora->dva != NULL) {
    ESP_LOGW(TAG, "a input stream is already in use for C2LORA");
    return ESP_ERR_INVALID_STATE;
  }
  if (lora->state == LS_INACTIVE) {
    ESP_LOGW(TAG, "lora is not active");
    return ESP_ERR_INVALID_STATE;
  }
/*  
  if (arg == NULL) {
    ESP_LOGE(TAG, "no new stream was provided! kill UDP stream.");
    return ESP_ERR_INVALID_ARG;
  }
*/
  lora->dva       = (tdvstream *) arg;
  lora->finish_tx = c2lora_finish_udp_tx;
  lora->p.min_pkt_end_time = (C2LORA_DVOICE_FRAMELEN_MS/2 - 40) * 1000; // don't care about high latency

  sx126x_lock();
  esp_err_t err = C2LORA_prepare_transmit(lora, false);
  sx126x_unlock();
#if CONFIG_USE_TEST_BITPATTERN
  uint8_t trx_packet[256];
  for (int i=0; i < 256; i++) {
    trx_packet[i] = 0xA0 + i;
  }
  sx126x_write_buffer(lora->ctx, lora->pkt_offset + 1 + lora->def->bytes_per_header, trx_packet, sizeof(trx_packet) - lora->def->bytes_per_header - 1);
#endif
  ESP_LOGD(TAG, "Fwd HAMdLNK started.");
  return err;
}



static tdvstream *hamdlnk2c2lora_update_dva(const thamdlnk_ref *packet, tLoraStream *lora, U8 streamtype, U32 clocktick) {  

  static const U8 supported_codec_list[] = {
    SF_CODEC_CODEC2_3200, SF_CODEC_CODEC2_2400, SF_CODEC_CODEC2_1600, SF_CODEC_CODEC2_1400,
    SF_CODEC_CODEC2_1300, /*SF_CODEC_CODEC2_1200,*/ SF_CODEC_CODEC2_700C,
    0
  };
  U16 DVdata_length = 0;
  tdvstream *udp_dva = NULL;

  if ((packet == NULL) || (lora == NULL)) return false;

  if ((streamtype == 255)) {
    // preferred direct transmission
    if (hamdlnk_getsubptr(packet, lora->def->codec_type, 1, &DVdata_length)) streamtype = lora->def->codec_type;   
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
    // we found an DV stream (compatible speech data) within the packet - create a stream struct
    int frame_len_ms;
    int frames_per_packet;
    udp_dva = (tdvstream *) calloc(1, sizeof(tdvstream));
    ESP_RETURN_ON_FALSE(udp_dva != NULL, NULL, TAG, "no memory for stream struct");

    udp_dva->streamid   = -1;    
    udp_dva->codec_type = 255;

    udp_dva->srate = 8000; // all modes uses 8kHz.
    udp_dva->samples_per_frame = (8 * C2LORA_DVOICE_FRAMELEN_MS) / lora->def->dv_frames_per_packet;

    udp_dva->bits_per_frame  = sf_get_subframe_bits(lora->def->codec_type);
    udp_dva->bytes_per_frame = (udp_dva->bits_per_frame + 7) >> 3;

    frame_len_ms = C2LORA_DVOICE_FRAMELEN_MS / lora->def->dv_frames_per_packet;

    ESP_LOGV(TAG, "s/f=%d, bits=%d", udp_dva->samples_per_frame, udp_dva->bits_per_frame);

    if (frame_len_ms != (U32)udp_dva->samples_per_frame * 1000 / udp_dva->srate) {
      ESP_LOGW(TAG, "frame size mismatch. garbled output!");
    } // fi something is wrong!

    udp_dva->codec_type = streamtype;

    frames_per_packet = DVdata_length / udp_dva->bytes_per_frame;

    udp_dva->buf = malloc(sizeof(tsimple_buffer));
    if (!sbuf_create(udp_dva->buf, 3 * frames_per_packet, udp_dva->bytes_per_frame, udp_dva->samples_per_frame)) {	// create buffer for (encoded data 4 audio-task access)
      ESP_LOGE(TAG, "create buffer (%d frames) fails", 3 * frames_per_packet);
    } // fi create buffer
#if CONFIG_USE_TEST_BITPATTERN
    memset((void *)udp_dva->buf->start_ptr, 0xFF, udp_dva->buf->size);
#endif
    sbuf_update_time(udp_dva->buf, clocktick);
    sbuf_set_steppos(udp_dva->buf, hamdlnk_gettimestamp(packet));    
  } // fi found speech data

  return udp_dva;
}




void hamdlnk2c2lora_parse_rxpkt(const thamdlnk_ref *packet, void *userdata, unsigned int clocktick) {
  U32	streamid;		// UDP-RTP stream (marker)
  U8  streamtype = 255; // undef
  U16 DVdata_length;
  bool stop_stream = false;

  tLoraStream *lora  = userdata;
  tdvstream *udp_dva = NULL;

  streamid = hamdlnk_getstreamid(packet);

  //tLoraStream *lora = hamdlnk_get_streamuserdata(listener, streamid);

  if (lora == NULL) return;

  if (lora->udp_stream_id == 0) {		// we receive a new stream....

    if ( (lora->state != LS_IDLE) && (lora->state != RX_NOSYNC) ) {
      ESP_LOGW(TAG, "already in use (%d)", lora->state);
      return;
    }

    U16	sender_len, recipient_len, senderinfo_len;
    const char *sender    = hamdlnk_getsubptr(packet, SF_SENDERID, 1, &sender_len);
    const char *recipient = hamdlnk_getsubptr(packet, SF_RECIPIENT, 1, &recipient_len);
    const char *senderinfo = hamdlnk_getsubptr(packet, SF_SENDERINFO, 1, &senderinfo_len);

    ESP_LOGD(TAG, "new DV stream %08lXh from '%.*s'", streamid, sender_len, sender);
    if ((senderinfo != NULL) && (senderinfo_len > 0)) {
      ESP_LOGD(TAG, "sender-info: %.*s", senderinfo_len, senderinfo);
    }
    //hamdlnk_set_streamuserdata(listener, streamid, &lora_stream);
    if ((recipient == NULL) || (recipient_len < 4)) {
      recipient = CQCALL_DESTINATION;
      recipient_len = strlen(CQCALL_DESTINATION);
    } else { // fi set default recipient
      ESP_LOGD(TAG, "recipient: '%.*s'", recipient_len, recipient);
    }

    // ToDo (KOS_RELAY, if so...)
    C2LORA_upd_header(lora->header, sender, sender_len, recipient, recipient_len, KOS_HOTSPOT, true);
    C2LORA_add_to_header(lora->header, own_area_code, own_locator);
    lora->udp_stream_id = streamid;

    vTimerSetTimerID(udprx_timer, lora);
    xTimerStart(udprx_timer, 0);

  } else {  // fi new stream
    udp_dva = lora->dva;    // not a new stream, dva is already the 'udp_dva', so take it
  }

  if (lora->udp_stream_id != streamid) {
    ESP_LOGW(TAG, "not for me");
    return;
  }

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
          ESP_LOGD(TAG, "Stream %08lXh starts", streamid);
        } // fi have Codec(Stream)Type
        break;
      case CONTROLMSG_END:
	      clocktick = 0;			// this updates the last_clocktick to 0, results in a immediate end after buffer is processed
        stop_stream = true;
  	    ESP_LOGD(TAG, "Stream %08lXh ends", streamid);
        //ESP_LOGD(TAG, ">>> high stack water mark is %u", uxTaskGetStackHighWaterMark(NULL));
        break;
      case CONTROLMSG_LOST:
	      clocktick = 0;
        stop_stream = true;
        ESP_LOGD(TAG, "Stream %08lXh ends (signal LOST)", streamid);
        break;
      }
    } // fi control messages
  } // fi marked packet

  if ( udp_dva == NULL ) {
    if ( stop_stream ) return;
    udp_dva = hamdlnk2c2lora_update_dva(packet, lora, streamtype, clocktick);
    if (udp_dva == NULL) return;
// ToDo create a localaudio transcoding pair, if codec_types don't match
    if (udp_dva->codec_type != lora->def->codec_type) {
      ESP_LOGW(TAG, "UDP stream with differend codec: Need transcode (not implemented jet)");
      if (isSF_CODEC2(udp_dva->codec_type)) {
        //  Todo transcoding in localaudio!
        //int codec2_mode = SF_CODEC2_get_mode(codec_type_udp);
        //localaudio_create_codec2_recodestream(lora->dva, codec2_mode, frame_len_ms, C2LORA_handle_encoded, lora);
      }      
    } // fi
    int frames_per_second = udp_dva->srate / udp_dva->samples_per_frame;
    esp_err_t err = create_record(&lora->rec, udp_dva->codec_type, MAX_RECORDING_LEN_S * frames_per_second, udp_dva->bytes_per_frame, udp_dva->samples_per_frame);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "no record created: %s", esp_err_to_name(err));
    }

    int buffer_size_ms = (udp_dva->buf->size / udp_dva->bytes_per_frame) * udp_dva->samples_per_frame * 1000 / udp_dva->srate;
    ESP_LOGD(TAG, "audio buffer size %dms", buffer_size_ms);
    xTimerChangePeriod(udprx_timer, pdMS_TO_TICKS(buffer_size_ms), 0);
    C2LORA_task_run_cmd(lora, c2lora_start_tx_udp, udp_dva, false);
  } // fi handle first packet (init)
 
  if ( stop_stream ) {
    xEventGroupSetBits(lora->events, C2LORA_EVENT_TRANSMIT_FINI);
  }
  // get (main) DV data from this stream
  char *encoded_data = hamdlnk_getsubptr(packet, udp_dva->codec_type, 1, &DVdata_length);
  U32  pkt_frame_pos = hamdlnk_gettimestamp(packet);
  //  feed it into a simple stream buffer
  if (encoded_data == NULL) return; // no data in this packet!
  sbuf_fillpos(udp_dva->buf, encoded_data, DVdata_length / udp_dva->bytes_per_frame, pkt_frame_pos);
  sbuf_update_time(udp_dva->buf, clocktick);

  ESP_LOGV(TAG, "Stream %08lXh stime: %6lu DVlen: %d", streamid, pkt_frame_pos, (int)DVdata_length);

  if ((DVdata_length > 0) && (udp_dva->codec_type == lora->def->codec_type)) { // using direct copy, re recoding
    C2LORA_handle_encoded(udp_dva, lora);
    xTimerReset(udprx_timer, 0);
  } // fi direct
  if ( stop_stream ) {
    xTimerStop(udprx_timer, 0);
    vTimerSetTimerID(udprx_timer, NULL);
  }
} // end parse funct



static void hamdlnk_udprx_timeout(TimerHandle_t timer) {
  tLoraStream *lora = pvTimerGetTimerID(udprx_timer);
  if (lora != NULL) {
    ESP_LOGW(TAG, "Timeout incoming UDP stream");
    xEventGroupSetBits(lora->events, C2LORA_EVENT_TRANSMIT_FINI);
  } else {
    ESP_LOGW(TAG, "Timeout but no stream active!");
  }
}



/*

HAMdLNK UDP Forward Functions

*/


void UTX_transmit(trtp_data *rtp) {
  if ((rtp == NULL) || ( rtp->conn == NULL)) return;
  hamdlnk_sendout(rtp->conn, &rtp->data);
  ESP_LOGD(TAG, "%3d: s %6ld tx %d frames", rtp->txed_packets, hamdlnk_gettimestamp(&rtp->data.r), rtp->chunk_no);
  rtp->txed_packets++;
  rtp->force_transmit = false;
}


void UTX_handle_encoded_stream(tdvstream *dva, void *userdata) {

  U16  sublen;
  U8   wpos;
  char *wptr, *dvdata;		// write pointer RT-Data
  bool transmit = false;

  trtp_data *rtp = (trtp_data *) userdata;

  if (rtp == NULL) return;

  thamdlnk_data *pkt = &rtp->data;

  if (hamdlnk_get_packetsize(pkt) < 0) {
    ESP_LOGE(TAG, "no valid TX packet");
    return;
  }

  // 1. get WritePointer to RT-Data-Area (first stream only)
  dvdata = hamdlnk_getsubptr(&pkt->r, rtp->subframe_id, 1, &sublen);

  // 2. if not exist, do nothing
  if (dvdata == NULL) return;

  wptr = dvdata;
  wpos = rtp->chunk_no;

  // 4. if subframe exists (not -> failure, invalid packet ...)
  //    move old data, if buffer full and add new data
  if (wpos >= rtp->chunk_count) {
    // normally wpos counts up by one and trigger a rt-data shift:
    if (rtp->interleave) {		// works even with odd numbers as intended (but that's inefficient)
      U8 half_chunks = (rtp->chunk_count + 1) >> 1;
      size_t upperhalf_packet_size = half_chunks * rtp->chunk_len_bytes;
      char * upper_half_ptr = wptr + ((rtp->chunk_count >> 1) * rtp->chunk_len_bytes);
      memmove(wptr, upper_half_ptr, upperhalf_packet_size);
      hamdlnk_addtimestamp(pkt, half_chunks * rtp->chunk_len_sps);
      wpos = half_chunks;
    } else {	// on a gap, reset rt-buffer and begin from start
      hamdlnk_addtimestamp(pkt, wpos * rtp->chunk_len_sps);
      wpos = 0;
    }

    hamdlnk_clear_marker(pkt);

    if (rtp->txed_packets == (FIRST_PACKES_WITH_CONTROL + (rtp->interleave & 1))) {
      // 2 or 3 pakets remove control
      ham2lnk_remove_subframe(pkt, SF_CONTROL, 1);	// remove CONTROL subframe
    }

  } // fi no space in rt-buffer

  // point in target-buffer on calculated position
  wptr += wpos * rtp->chunk_len_bytes;

  // copy data to HAM2LNK packet:
  U32 data_chunk_size = 0;
  U8  new_chunks;
  const char *data_chunk = sbuf_get_unsend_block(dva->buf, &data_chunk_size);
  new_chunks = data_chunk_size / rtp->chunk_len_bytes;
  
  ESP_LOGV(TAG, "got %lu encoded byte (%d chunks)", data_chunk_size, new_chunks);

  // ToDo checks size and pos
  if (new_chunks > (rtp->chunk_count - wpos)) {
    new_chunks = rtp->chunk_count - wpos;
  }
  memcpy(wptr, data_chunk, new_chunks * rtp->chunk_len_bytes);
  sbuf_unsend_block_processed(dva->buf, new_chunks * rtp->chunk_len_bytes);

  // set correct lengths of buffers:
  wpos += new_chunks;	// wpos is now length of buffer (normally = RTP_NO_OF_AMBEBUF)
  ham2lnk_subframe_setlength(pkt, rtp->subframe_id, 1, wpos * rtp->chunk_len_bytes);
  rtp->chunk_no = wpos;

  transmit = (wpos >= rtp->chunk_count) || (rtp->force_transmit);

  // final transmit:
  if (transmit) {
    UTX_transmit(rtp);
//    hamdlnk_sendout(rtp->conn, pkt);
//    ESP_LOGD(TAG, "%3d: s %6ld tx %d frames", rtp->txed_packets, hamdlnk_gettimestamp(&pkt->r), wpos);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, dvdata, 16, ESP_LOG_VERBOSE);
//    rtp->txed_packets++;
//    rtp->force_transmit = false;
  } // fi tx

}



bool UTX_handle_encoded_dvframe(const U8 *dvframe, trtp_data *udp_rtp) {
  U16  sublen;
  U8   wpos;
  char *wptr, *dvdata;		// write pointer RT-Data

  if (udp_rtp == NULL) return false;

  thamdlnk_data *pkt = &udp_rtp->data;

  if (hamdlnk_get_packetsize(pkt) < 0) {
    ESP_LOGE(TAG, "no valid TX packet");
    return false;
  }

  // 1. get WritePointer to RT-Data-Area (first stream only)
  dvdata = hamdlnk_getsubptr(&pkt->r, udp_rtp->subframe_id, 1, &sublen);

  // 2. if not exist, do nothing
  if (dvdata == NULL) return false;

  wptr = dvdata;
  wpos = udp_rtp->chunk_no;

  // 4. if subframe exists (not -> failure, invalid packet ...)
  //    move old data, if buffer full and add new data
  if (wpos >= udp_rtp->chunk_count) {
    // normally wpos counts up by one and trigger a rt-data shift:
    if (udp_rtp->interleave) {		// works even with odd numbers as intended (but that's inefficient)
      U8 half_chunks = (udp_rtp->chunk_count + 1) >> 1;
      size_t upperhalf_packet_size = half_chunks * udp_rtp->chunk_len_bytes;
      char * upper_half_ptr = wptr + ((udp_rtp->chunk_count >> 1) * udp_rtp->chunk_len_bytes);
      memmove(wptr, upper_half_ptr, upperhalf_packet_size);
      hamdlnk_addtimestamp(pkt, half_chunks * udp_rtp->chunk_len_sps);
      wpos = half_chunks;
    } else {	// on a gap, reset rt-buffer and begin from start
      hamdlnk_addtimestamp(pkt, wpos * udp_rtp->chunk_len_sps);
      wpos = 0;
    }
    hamdlnk_clear_marker(pkt);
    if (udp_rtp->txed_packets == (FIRST_PACKES_WITH_CONTROL + (udp_rtp->interleave & 1))) {
      // 2 or 3 pakets remove control
      ham2lnk_remove_subframe(pkt, SF_CONTROL, 1);	// remove CONTROL subframe
    }
  } // fi no space in rt-buffer

  // point in target-buffer on calculated position
  wptr += wpos * udp_rtp->chunk_len_bytes;
  // copy data to HAM2LNK packet:
  memcpy(wptr, dvframe, udp_rtp->chunk_len_bytes);  
  // set correct lengths of buffers:
  wpos++;	// wpos is now length of buffer (normally = RTP_NO_OF_AMBEBUF)
  ham2lnk_subframe_setlength(pkt, udp_rtp->subframe_id, 1, wpos * udp_rtp->chunk_len_bytes);
  udp_rtp->chunk_no = wpos;
  return (wpos >= udp_rtp->chunk_count) || (udp_rtp->force_transmit);
}


U32 hdl_get_srate(const tdvstream *dva) {
  return ((U32)dva->samples_per_frame * HAMdLNK_SP_S) / dva->srate;
}

void hdl_stream_control_start(thamdlnk_data *pkt, U8 codec_type) {
  char *wptr;
  if (pkt == NULL) return;
  wptr = ham2lnk_addnew_subframe(pkt, SF_CONTROL, CONTROLMSG_STRT_S, CONTROLMSG_LEN+2);
  if (wptr != NULL) wptr[CONTROLMSG_LEN+1] = codec_type;
  hamdlnk_set_marker(pkt);
}


void hdl_stream_control_end(thamdlnk_data *pkt) {
  if (pkt == NULL) return;
  ham2lnk_addnew_subframe(pkt, SF_CONTROL, CONTROLMSG_END_S, CONTROLMSG_LEN);
  hamdlnk_set_marker(pkt);
}


esp_err_t hdl_start_HAMdLNK(trtp_data *udp_rtp, U32 udp_stream_id) {
  
  ESP_RETURN_ON_FALSE(udp_rtp != NULL, ESP_ERR_INVALID_ARG, TAG, "no UDPrtp stream struct");

  thamdlnk_data *tx_packet = &udp_rtp->data;
  ham2lnk_create_new_stream(tx_packet, udp_stream_id);
  hamdlnk_settimestamp(tx_packet, 0);
  return ESP_OK;        
}



/* c2lora_createsenderinfo(()
 * creates sender and recipient blocks into the HAMdLNK RTP packet based on
 * the localaudio_config
 */
static void c2lora_createsenderinfo(thamdlnk_data *pkt, const char *callsign, const char *recipient, tKindOfSender kos, const char *areacode, const char *locator) {
  U16 call_len = 0;
  // measure length of sender-calll
  while ((callsign[call_len] > ' ') && (call_len < 8)) call_len++;	// max 8 chars for caller
  if ((call_len > 3)) {         // a valid sender?
    char sender_info[16];
    ham2lnk_addnew_subframe(pkt, SF_SENDERID, callsign, call_len);	// copy only the first 8 chars
    sender_info[0] =0; 
    // Todo: put areacode + locator together in senderinfo
    call_len = strnlen(sender_info, sizeof(sender_info));
    if (call_len > 3 ) {
      ham2lnk_addnew_subframe(pkt, SF_SENDERINFO, sender_info, call_len);
    }
    // get recipient: we artificially limit this to 32 bytes here, sri
    if (recipient != NULL) {
      call_len = strnlen(recipient, 32);
      if ((call_len > 3) && Check_Call(recipient, call_len)) {
        ham2lnk_addnew_subframe(pkt, SF_RECIPIENT, recipient, call_len);
      }
    }
    hamdlnk_set_marker(pkt);
  } // fi valid sender
}





trtp_data * C2LORA_setup_HAMdLNK(trtp_data *udp_rtp, const tdvstream *dva) {
  if (udp_rtp == NULL) return NULL;

  int frame_len_ms = (U32)dva->samples_per_frame * 1000 / dva->srate;
  int no_of_frames = HAMDLNK_C2LORA_PKT_LEN / frame_len_ms;
  if ((no_of_frames & 1) && (udp_rtp->interleave)) no_of_frames++;	// even numbers!

  udp_rtp->subframe_id   = dva->codec_type;
  udp_rtp->chunk_count   = no_of_frames;
  udp_rtp->chunk_len_sps = hdl_get_srate(dva);
  udp_rtp->chunk_len_bytes = dva->bytes_per_frame;
  udp_rtp->conn = NULL;

  ESP_LOGD(TAG, "TX ptp: %d frames per packet (%dms frame len, %d sps len), interleave=%s", udp_rtp->chunk_count, frame_len_ms, udp_rtp->chunk_len_sps, udp_rtp->interleave?"true":"false");

  return udp_rtp;
}


esp_err_t C2LORA_start_HAMdLNK(trtp_data *udp_rtp, const char *callsign, const char *recipient, int kos, uint16_t areacode, const char *locator) {
  thamdlnk_data *tx_packet;
  char area_code_str[4];
  ESP_RETURN_ON_FALSE(udp_rtp != NULL, ESP_ERR_INVALID_ARG, TAG, "no UDPrtp stream struct");

  ESP_RETURN_ON_ERROR(hdl_start_HAMdLNK(udp_rtp, C2LORA_UNIQUE_STREAM_ID), TAG, "create UDPrtp fails");

  udp_rtp->conn = get_udp_out_connection(false);   // connect to default outgoing connection (network boradcast)  
  tx_packet = &udp_rtp->data;

  // add sender info (static / must) to this stream
  if (!C2LORA_unpack_areacode(areacode, area_code_str)) {
    area_code_str[0] = 0;
  }
  c2lora_createsenderinfo(tx_packet, callsign, recipient, kos, area_code_str, locator);
  ham2lnk_create_subframe(tx_packet, udp_rtp->subframe_id, udp_rtp->chunk_count * udp_rtp->chunk_len_bytes);
  hdl_stream_control_start(tx_packet, udp_rtp->subframe_id);
  return udp_rtp->conn? ESP_OK: ESP_FAIL;
}


void C2LORA_stop_HAMdLNK(trtp_data *udp_rtp) {  
  if (udp_rtp == NULL) return;
  if (hamdlnk_get_packetsize(&udp_rtp->data) > 0) {
    //thamdlnk_data *tx_packet = &udp_rtp->data;            
    ESP_LOGD(TAG, "out end marker");  
    hdl_stream_control_end(&udp_rtp->data);
    // get rest and sed out
    // ToDo force transmit this last packet
    UTX_transmit(udp_rtp);
  } // fi valid pkt
}



/*

Playback Functions

*/

tRecorderHandle *C2LORA_get_last_record(void) {
  return last_recording;
}



/*
  c2lora_finish_rec_tx() is called from C2LORA control task queued from C2LORA_handle_encoded() processing the "LAST_FRAME"
*/
static uint32_t c2lora_finish_rec_tx(tLoraStream *lora, void *arg) {
  esp_err_t err;
  ESP_RETURN_ON_FALSE(lora != NULL, ESP_ERR_INVALID_ARG, TAG, "no argument for udp-finisher call");
  lora->udp_stream_id = 0;
  lora->dva = NULL;
  ESP_LOGD(TAG, "REC playback done.");
  return (uint32_t) err;
}


esp_err_t C2LORA_send_record(tRecorderHandle *rec, tLoraStream *lora) {

  if (rec == NULL) return ESP_ERR_NO_MEM;

  tdvstream *udp_dva = NULL;

  // we found an DV stream (compatible speech data) within the packet - create a stream struct
  int frame_len_ms;
  udp_dva = (tdvstream *) calloc(1, sizeof(tdvstream));
  ESP_RETURN_ON_FALSE(udp_dva != NULL, ESP_ERR_NO_MEM, TAG, "no memory for stream struct");

  udp_dva->streamid = -1;
  udp_dva->srate    = 8000; // all modes uses 8kHz.
  udp_dva->samples_per_frame = (8 * C2LORA_DVOICE_FRAMELEN_MS) / lora->def->dv_frames_per_packet;
  udp_dva->bits_per_frame    = sf_get_subframe_bits(lora->def->codec_type);
  udp_dva->bytes_per_frame   = (udp_dva->bits_per_frame + 7) >> 3;

  frame_len_ms = C2LORA_DVOICE_FRAMELEN_MS / lora->def->dv_frames_per_packet;

  if (frame_len_ms != (U32)udp_dva->samples_per_frame * 1000 / udp_dva->srate) {
    ESP_LOGW(TAG, "frame size mismatch. garbled output!");
  } // fi something is wrong!

  udp_dva->codec_type = record_get_codec_type(rec);
  udp_dva->buf        = record_get_buffer(rec);
  udp_dva->flags      = DVSTREAM_FLAG_KEEP_BUFFER;  // don't free this buffer after playback

  if (udp_dva->buf == NULL) {
    ESP_LOGW(TAG, "no record.");
    return ESP_FAIL;
  }

//  sbuf_update_time(udp_dva->buf, 0);
//  sbuf_set_steppos(udp_dva->buf, 0);

  //C2LORA_upd_header(lora->header, sender, sender_len, recipient, recipient_len, KOS_HOTSPOT, true);

  if (C2LORA_task_run_cmd(lora, c2lora_start_tx_udp, udp_dva, false) != ESP_OK) {
    ESP_LOGE(TAG, "can't start retransmit");
  }

  rewind_recording(rec);

  ESP_LOGD(TAG, "retransmit %d bytes speech data", sbuf_get_unsend_size(udp_dva->buf));
  lora->finish_tx = c2lora_finish_rec_tx; // was set to udp_stop_function  @c2lora_start_tx_udp
  lora->rec       = NULL;
  C2LORA_handle_encoded(udp_dva, lora);   // starts, refilling with TXdone triggered function

  do {

    vTaskDelay(pdMS_TO_TICKS(40));
    
  } while (sbuf_get_unsend_size(udp_dva->buf) >= udp_dva->bytes_per_frame);

  xEventGroupSetBits(lora->events, C2LORA_EVENT_TRANSMIT_FINI);
  ESP_LOGD(TAG, "retransmission done.");
  return ESP_OK;  
}

