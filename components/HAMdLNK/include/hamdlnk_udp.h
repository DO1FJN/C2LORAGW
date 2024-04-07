/*
 * hamdlnk_upd.h
 *
 * description.
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

#pragma once


#include "HAMdLNK_types.h"

#define MAX_HAMdLNK_TARGETS     4

struct netconn;

struct udp_pcb;     // uses for lwip pcb's


typedef void (*receive_funct) (const thamdlnk_ref *packet, void *userdata, unsigned int clocktick);

typedef struct {
  unsigned short        port_no;
  receive_funct         handler;
  void *                userdata;
} treceive_handler;



void    hamdlnk_init_targes(void);

int     hamdlnk_create_broadcast(unsigned short dest_port);



/* ham2lnk_sendto()
 * send created RTP-UDP packet to connected target
 * update value sequence-no.
 * called from hamdlnk_outstream() after building a packet
 * @PARAM
 * conn         netconn connection, having a targed adress via netconn_connect()
 * txpacket     the prepared data packet
 */
void    hamdlnk_sendout(struct netconn *conn, thamdlnk_data *txpacket);




/* hamdlnk_remove_in()
 * removes / close incoming socket
 * @PARAM
 * ifd		receive socket fd; receive_from(ifd, ...)
 * @RETURN
 * close return value: 0 = success, -1 error, read errno
 */


void    hamdlnk_receive(struct udp_pcb *pcb, const treceive_handler *handler);

