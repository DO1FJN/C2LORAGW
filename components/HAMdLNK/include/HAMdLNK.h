/*
 * HAMdLNK.h
 *
 * base function to build or parse a HAMdLNK packet.
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
#include "subframedef.h"

#define	HAMdLNK_SPMS		8     // 8 Samples per Millisecond (8000Sps)
#define	HAMdLNK_SP_S		8000  // 8 Samples per Millisecond (8000Sps)


// *** Initialization ***

void	hamdlnk_init_randomseed(void);



// *** Header Functions ***

bool	hamdlnk_checkpacket(const thamdlnk_data *packet);

void	ham2lnk_create_new_stream(thamdlnk_data *packet, U32 sid);

void	hamdlnk_set_marker(thamdlnk_data *packet);
void	hamdlnk_clear_marker(thamdlnk_data *packet);

int	hamdlnk_get_marker(const thamdlnk_ref *packet);

void	hamdlnk_incsequenceno(thamdlnk_data *packet);

U32	hamdlnk_getstreamid(const thamdlnk_ref *packet);



/* timestamp: RTP value that counts 8000 per second (here independent of real sampling-rates)
 *
 */
U32	hamdlnk_gettimestamp(const thamdlnk_ref *packet);

void	hamdlnk_settimestamp(thamdlnk_data *packet, U32 value);
void	hamdlnk_addtimestamp(thamdlnk_data *packet, U32 value);


int	hamdlnk_getheadersize(const thamdlnk_ref *packet);
int	hamdlnk_getpayloadsize(const thamdlnk_ref *packet);


// **** Receiving a packet ***

int	hamdlnk_init_rxpacket(thamdlnk_ref *packet_ref, const void *rxdata, int length);

int	hamdlnk_get_rx_packetsize(const thamdlnk_ref *packet);


// *** the following function needs a CREATED TRANSMIT packet (payload begins on data[])! ***


/* hamdlnk_get_packetsize() returns the UDP packet size (header + data)
 * returns -1 if *packet was NULL or invalid
 */
int	hamdlnk_get_packetsize(const thamdlnk_data *packet);


/* ham2lnk_create_subframe()
 * creates a new CSRC entry and returns a pointer to the assigned data.
 * Returns NULL, if fails (CSRC table full, no space in packet, ...)
 */
char *  ham2lnk_create_subframe(thamdlnk_data *packet, U8 stype, U16 datalength);


/* ham2lnk_addnew_subframe()
 * creates a subframe using ham2lnk_create_subframe() and copy data into the packet, if success.
 * Returns NULL, if fails otherwise a pointer to the sub-frame data
 */
char *  ham2lnk_addnew_subframe(thamdlnk_data *packet, U8 stype, const char *data, U16 length);

char *  hamdlnk_getsubptr(const thamdlnk_ref *packet, U8 stype, U8 no, U16 *datalength);

U8      hamdlnk_findsubptr(const thamdlnk_ref *packet, const U8 *stype_list, U8 no, U16 *datalength);


bool	ham2lnk_subframe_setlength(thamdlnk_data *packet, U8 stype, U8 no, U16 length);


/* ham2lnk_remove_subframe() removes a subframe(stype) from packet (with data), counts no from top
 *
 */
void	ham2lnk_remove_subframe(thamdlnk_data *packet, U8 stype, U8 no);

void	hamdlnk_init_userdata(void);

void *	hamdlnk_get_streamuserdata(int listener, U32 streamid);

void	hamdlnk_set_streamuserdata(int listener, U32 streamid, void *userdata);

void	hamdlnk_free_streamuserdata(void *userdata);

