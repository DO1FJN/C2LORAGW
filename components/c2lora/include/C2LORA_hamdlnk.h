/*

C2LORA_hamdlnk.h

This header file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

main API file for accessing the C2LORA UHF link

*/
#pragma once

#include "HAMdLNK_types.h"
#include "lwip/api.h"
#include "localaudio.h"

#define FIRST_PACKES_WITH_CONTROL	  2

typedef struct {
  const char *          sendto_hostname_uhf;
  unsigned short        sendto_port_uhf;
  unsigned short        listen_port_uhf;
  const char *          sendto_hostname_dongle;
  unsigned short        sendto_port_dongle;
  unsigned short        listen_port_dongle;
  char                  locator[6];
  unsigned short        area_code;
} tHAMdLNK_network_config;


typedef struct {
  struct netconn *conn;
  U16		          chunk_len_sps;	  // number of samples per chunk
  U8              chunk_len_bytes;  // data length of a single DV frame (chunk)
  U8		          chunk_count;		  // no of chunks within an RTP packet, calculated by chunk_length_ms...  
  U8              subframe_id;      // codec / spech data subframe the data will written into it
  U8		          chunk_no;		      // position within then data
  bool		        interleave;
  bool            force_transmit;   // forcing  transmit the UPD HAMdLNK packet even if it't not filled (shorter DV data)
  thamdlnk_data   data;
  unsigned int	  txed_packets;
} trtp_data;



esp_err_t   C2LORA_start_udp_task(const tHAMdLNK_network_config *config);


trtp_data * C2LORA_setup_HAMdLNK(trtp_data *udp_rtp, const tdvstream *dva);

esp_err_t   C2LORA_start_HAMdLNK(trtp_data *udp_rtp, const char *callsign, const char *recipient, int kos, uint16_t areacode, const char *locator);

void        C2LORA_stop_HAMdLNK(trtp_data *udp_rtp);

void        UTX_transmit(trtp_data *udp_rtp);

void        UTX_handle_encoded_stream(tdvstream *dva, void *userdata);

bool        UTX_handle_encoded_dvframe(const U8 *dvframe, trtp_data *udp_rtp);



void hamdlnk2c2lora_parse_rxpkt(const thamdlnk_ref *packet, void *userdata, unsigned int clocktick);

U32  hdl_get_srate(const tdvstream *dva);

void hdl_stream_control_start(thamdlnk_data *pkt, U8 codec_type);

void hdl_stream_control_end(thamdlnk_data *pkt);

