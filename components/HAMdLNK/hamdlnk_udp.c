/*
 * hamdlnk_udp.c
 *
 * Transmit and Receive HAMdLNK via RTP/UDP.
 *
 *  Created on: 26.05.2012
 *      Author: Jan Alte, DO1FJN
 *
 * This file is part of the DV-RPTR application.
 * For general information about this program see "main.c".
 *
 * DV-RPTR app is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The DV-RPTR app is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include "hamdlnk_udp.h"
#include "HAMdLNK.h"

#include "compiler.h"
#include "esp_log.h"

#include "lwip/api.h"
#include "lwip/udp.h"
#include "lwip/tcpip.h"

#include <stdio.h>



static const char * TAG = "HAMdLNK UDP";


typedef struct {
  ip_addr_t   addr;
  U16         port;
} tUDPtarget;


static tUDPtarget targets[MAX_HAMdLNK_TARGETS];


//  const tUDPtarget *targetAdr = &targets[target];


void hamdlnk_init_targes(void) {
  memset(targets, 0, sizeof(targets));  
}


void hamdlnk_free_target(int target) {
  if ((target < 0) || (target >= MAX_HAMdLNK_TARGETS)) {
    ESP_LOGE(TAG, "invalid target (%d).", target);
    return;
  }
  memset(&targets[target], 0, sizeof(tUDPtarget));
}


tUDPtarget *hamdlnk_get_free_target(int *target_num) {
  for (int n=0; n < MAX_HAMdLNK_TARGETS; n++) {
    if (targets[n].port == 0) {
      if (target_num) *target_num = n;
      return &targets[n];
    }
  }
  return NULL;
}


int  hamdlnk_create_broadcast(unsigned short dest_port) {
  int t_num = -1;
  tUDPtarget *bctarget = hamdlnk_get_free_target(&t_num);
  if (bctarget == NULL) return -1;
  bctarget->port = dest_port;
  memcpy(&bctarget->addr, IP_ADDR_BROADCAST, sizeof(bctarget->addr));
  return t_num;
}




void hamdlnk_sendout(struct netconn *conn, thamdlnk_data *txpacket) {  
  int pkt_size;

  if (conn == NULL) return;

  pkt_size = hamdlnk_get_packetsize(txpacket);
  if (pkt_size > 0) { 
    err_t err;
    struct netbuf *buf = netbuf_new();            // create a new netbuf
	  err = netbuf_ref(buf, txpacket->r.headerbegin, pkt_size);  // refer the netbuf to the data to be sent 
    if (err != ERR_OK) {
      ESP_LOGE(TAG, "send(): can't create buffer reference");
      return;
    }
   
    hamdlnk_incsequenceno(txpacket);		// transmit: first sequence-no is "1"...

	  err = netconn_send(conn, buf);      // send the netbuf to the client
	  netbuf_delete(buf);                 // delete the netbuf

    if (err != ERR_OK) {
      ESP_LOGE(TAG, "send() fails (code %d)", err);
    }

  } else {	// fi valid packet
    ESP_LOGE(TAG, "send(): Invalid packet structure detected. Can't transmit.");
  }
}



void receive_hamdlnk_udp(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {

  thamdlnk_ref rxpkt_ref;

  TickType_t curr_tick = xTaskGetTickCount();
  const char *remoteIP = ipaddr_ntoa(addr);

  if ((p->payload == NULL) || (p->len <= 0)) {

    ESP_LOGE(TAG, "received from %s: no data", remoteIP);      

  } else { 
    int rxlength;
    U16 sender_len;
    const char *sender;

    const treceive_handler *hnd = (const treceive_handler *) arg;

    if (rxlength > (HAMdLNK_MAXHEADER_SIZE+HAMdLNK_MAXDATA_SIZE)) rxlength = HAMdLNK_MAXHEADER_SIZE+HAMdLNK_MAXDATA_SIZE;

    rxlength = hamdlnk_init_rxpacket(&rxpkt_ref, p->payload, p->len);	// rxlen is now length of payload
    if (rxlength < 0) {
      ESP_LOGW(TAG, "Invalid HAMdLNK received: %s", (rxlength==-1)?"no RTP'd' header": ((rxlength==-2)? "length shorter than header-length": "packet to large"));
      return;
    }

    // First (important): Get sender information. If don't exist: IGNORE
    sender = hamdlnk_getsubptr(&rxpkt_ref, SF_SENDERID, 1, &sender_len);
    if ((sender == NULL) || (sender_len < 3)) {
      ESP_LOGW(TAG, "packet from %s w/o sender received. Ignored.", remoteIP);
      return;	// no sender: IGNORE
    }
    ESP_LOGV(TAG, "pkt from %s: %.*s", remoteIP, sender_len, sender);      
    if (hnd->handler != NULL) {
      hnd->handler(&rxpkt_ref, hnd->userdata, curr_tick);
    } // fi

#ifdef UDP_TEST_PAPAGAI
    struct udp_pcb *txbuf = pbuf_alloc(PBUF_TRANSPORT, p->len, PBUF_RAM);
    pbuf_take(txbuf, p->payload, p->len);
    udp_sendto(pcb, txbuf, addr, 30004);
    pbuf_free(txbuf);
#endif
  } // fi msg

  pbuf_free(p);
}




void hamdlnk_receive(struct udp_pcb *pcb, const treceive_handler *handler) {
  LOCK_TCPIP_CORE();
  udp_recv(pcb, receive_hamdlnk_udp, (void *)handler);
  UNLOCK_TCPIP_CORE();
}

