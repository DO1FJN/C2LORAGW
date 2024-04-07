/*
 * HAMdLNK.c
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
 * ToDo:
 * - check for a better random-seed for StreamIDs
 */

#include "HAMdLNK.h"

#include "compiler.h"
#include "checksums.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "esp_random.h"

//#define HAMdLNK_CLEANSTART	// For Debugging (memory view) enable memset() on create_new_stream()


U32 rtp_ssrc_seed = 0x55AA0000;


void hamdlnk_init_randomseed(void) {
  rtp_ssrc_seed = esp_random();
}


U32 rtp_create_ssrc(U32 sid) {
  U32 ssrc = rtp_ssrc_seed;
  crc32(&ssrc, MSB0W(sid));
  crc32(&ssrc, MSB1W(sid));
  crc32(&ssrc, MSB2W(sid));
  crc32(&ssrc, MSB3W(sid));
  return ssrc;	// Test
}


void ham2lnk_create_new_stream(thamdlnk_data *packet, U32 sid) {
  U32 ssrc = rtp_create_ssrc(sid);
#ifdef HAMdLNK_CLEANSTART
  memset(packet->header, 0, sizeof(packet->header));
  memset(packet->data, 0, sizeof(packet->data));
#endif
  packet->r.headerbegin     = &packet->header[HAMdLNK_MAXHEADER_SIZE-12];	// a header w/o subframes!
  packet->r.databegin       = packet->data;
  packet->r.freedataofs     = packet->data;
  packet->r.headerbegin[0]  = 0x80;	// RTP V2 packet
  packet->r.headerbegin[1]  = 0x64;	// Type 100
  // sequence-number begins at zero, timestamp too.
  packet->r.headerbegin[8]  = MSB0W(ssrc);
  packet->r.headerbegin[9]  = MSB1W(ssrc);
  packet->r.headerbegin[10] = MSB2W(ssrc);
  packet->r.headerbegin[11] = MSB3W(ssrc);
}


void hamdlnk_incsequenceno(thamdlnk_data *packet) {
  U16 sno;
  MSB(sno) = packet->r.headerbegin[2];
  LSB(sno) = packet->r.headerbegin[3];
  sno++;
  packet->r.headerbegin[2] = MSB(sno);
  packet->r.headerbegin[3] = LSB(sno);
}


U32 hamdlnk_getstreamid(const thamdlnk_ref *packet) {
  U32 value;
  MSB0W(value) = packet->headerbegin[8];
  MSB1W(value) = packet->headerbegin[9];
  MSB2W(value) = packet->headerbegin[10];
  MSB3W(value) = packet->headerbegin[11];
  return value;
}


U32 hamdlnk_gettimestamp(const thamdlnk_ref *packet) {
  U32 value;
  MSB0W(value) = packet->headerbegin[4];
  MSB1W(value) = packet->headerbegin[5];
  MSB2W(value) = packet->headerbegin[6];
  MSB3W(value) = packet->headerbegin[7];
  return value;
}


void hamdlnk_settimestamp(thamdlnk_data *packet, U32 value) {
  packet->r.headerbegin[4] = MSB0W(value);
  packet->r.headerbegin[5] = MSB1W(value);
  packet->r.headerbegin[6] = MSB2W(value);
  packet->r.headerbegin[7] = MSB3W(value);
}


void hamdlnk_addtimestamp(thamdlnk_data *packet, U32 value) {
  U32 timestamp = hamdlnk_gettimestamp(&packet->r);
  timestamp += value;
  hamdlnk_settimestamp(packet, timestamp);
}


bool hamdlnk_check_rxpacket(const thamdlnk_ref *packet, const char *header, const char *data) {
  if (packet == NULL) return false;
  return ((packet->headerbegin >= header) && (packet->headerbegin < data) &&
    (packet->freedataofs >= (packet->headerbegin + 12)) &&
    (packet->freedataofs <= (data + HAMdLNK_MAXDATA_SIZE)));
}


int hamdlnk_getheadersize(const thamdlnk_ref *packet) {
//  if (hamdlnk_check_rxpacket(packet)) {
    return (int)(packet->databegin - packet->headerbegin);
//  } else return -1;
}


int hamdlnk_getpayloadsize(const thamdlnk_ref *packet) {
//  if (hamdlnk_check_rxpacket(packet)) {
    return (int)(packet->freedataofs - packet->databegin);
//  } else return -1;
}



// *** RECEIVING section ***


int hamdlnk_init_rxpacket(thamdlnk_ref *packet_ref, const void *rxdata, int length) {
  char *rx_header = (char *) rxdata;
  //char *rx_data   = (char *) (rxdata + HAMdLNK_MAXHEADER_SIZE);
  if ( ((rx_header[0] & RTPHEADER_RTPVERS_MASK) == 0x80) && ((rx_header[1] & 0x7F) == 0x64) ) {
    int csrc_cnt = rx_header[0] & RTPHEADER_CC_MASK;
    packet_ref->headerbegin = rx_header;
    packet_ref->databegin   = rx_header + 12 + (4 * csrc_cnt);
    packet_ref->freedataofs = packet_ref->databegin;
    length -= 12 + (4 * csrc_cnt);
    if (length < 0) return -2;				// Error received Length shorter than header
    if (length > HAMdLNK_MAXDATA_SIZE) return -3;	// Error length bigger than MAX payload
    packet_ref->freedataofs += length;
    return length;
  } else return -1;	// no RTP'd' packet!
}


int hamdlnk_get_rx_packetsize(const thamdlnk_ref *packet) {
  return (int)(packet->freedataofs - packet->headerbegin);
}


// *** TRANSMITTING section ***

bool hamdlnk_checkpacket(const thamdlnk_data *packet) {
  if (packet == NULL) return false;
  return ((packet->r.headerbegin >= packet->header) && (packet->r.headerbegin < packet->data) &&
    (packet->r.freedataofs >= (packet->r.headerbegin + 12)) &&
    (packet->r.freedataofs <= &packet->data[HAMdLNK_MAXDATA_SIZE]));
}


int hamdlnk_get_packetsize(const thamdlnk_data *packet) {
  if (hamdlnk_checkpacket(packet)) {
    return (int)(packet->r.freedataofs - packet->r.headerbegin);
  } else return -1;
}


U32 ham2lnk_databytes_left(thamdlnk_data *packet) {
  // first check borders -> return zero, if data-pointer is invalid:
  if ((packet->r.freedataofs < packet->data) || (packet->r.freedataofs > (packet->data+sizeof(packet->data))))
    return 0;
  return packet->data + HAMdLNK_MAXDATA_SIZE - packet->r.freedataofs;
}


void hamdlnk_set_marker(thamdlnk_data *packet) {
  if (hamdlnk_checkpacket(packet)) {
    packet->r.headerbegin[1] |= 0x80;
  }
}


void hamdlnk_clear_marker(thamdlnk_data *packet) {
  if (hamdlnk_checkpacket(packet)) {
    packet->r.headerbegin[1] &= 0x7F;
  }
}


int hamdlnk_get_marker(const thamdlnk_ref *packet) {
  return (packet->headerbegin[1] & 0x80)? 1: 0;
}


char *ham2lnk_create_subframe(thamdlnk_data *packet, U8 stype, U16 datalength) {
  U8  csrc_count, cnt;
  U32 csrc_value;
  char *hdrcsrc_begin;

  // 1. check, if enough space available. If not: return NULL (fail).
  if (ham2lnk_databytes_left(packet) < datalength) return NULL;
  csrc_count = packet->r.headerbegin[0] & RTPHEADER_CC_MASK;

  // 2. check, if not all 15 sub-frames filled:
  if (csrc_count >= 0x0F) return NULL;
  hdrcsrc_begin = packet->r.headerbegin - 4;
  if (hdrcsrc_begin < packet->header) return NULL;	// bad pointer!

  // 3. move (overlapped!) existing header 4 bytes up:
  memmove(hdrcsrc_begin, packet->r.headerbegin, (packet->data-packet->r.headerbegin));
  packet->r.headerbegin = hdrcsrc_begin;

  // 4. update existing CSRC entrys (add '4' to every offset)
  hdrcsrc_begin = packet->r.headerbegin + 13;	// CSRC-Table (+1 = optimized)
  for (cnt=0; cnt<csrc_count; cnt++) {		// Add 4 to Offsets
    U16 offset;
    MSB(offset) = hdrcsrc_begin[0];
    LSB(offset) = hdrcsrc_begin[1];		// offset hold value x16 (lowest 4bits are part of len)
    offset += 0x0040;			// add 4 bytes (! lower 4bits of this are part of length field)
    hdrcsrc_begin[0] = MSB(offset);
    hdrcsrc_begin[1] = LSB(offset);
    hdrcsrc_begin += 4;
  } // rof offset update

  // 5. create a new CSRC entry on last position
  csrc_value = ((packet->r.freedataofs - packet->r.headerbegin)<<12) | datalength;
  MSB0W(csrc_value) = stype;
  hdrcsrc_begin[-1] = MSB0W(csrc_value);
  hdrcsrc_begin[0]  = MSB1W(csrc_value);
  hdrcsrc_begin[1]  = MSB2W(csrc_value);
  hdrcsrc_begin[2]  = MSB3W(csrc_value);

  // 6. update CC field and set the free-data-offset pointer to the next free position
  packet->r.headerbegin[0] = 0x80 | (csrc_count+1);	// update CC field
  hdrcsrc_begin = packet->r.freedataofs;
  // add length and prepare for 32bit align
  datalength = (datalength + 3) & 0xFFFFFFFC;
  packet->r.freedataofs += datalength;
  // return data-position for processing
  return hdrcsrc_begin;
}


char *ham2lnk_addnew_subframe(thamdlnk_data *packet, U8 stype, const char *data, U16 length) {
  char *dest = ham2lnk_create_subframe(packet, stype, length);
  if (dest == NULL) return NULL;
  memcpy(dest, data, length);
  return dest;
}


char *get_csrc(const thamdlnk_ref *packet, U8 stype, U8 no) {
  U8 csrc_count = packet->headerbegin[0] & RTPHEADER_CC_MASK;
  char *csrc = packet->headerbegin + 12;
  // get pointer to the no. CSRC in CSRC table
  for (; (csrc_count>0)&&(no>0); csrc_count--, csrc += 4) {
    if (csrc[0] == stype) {
      if (no <=1 ) return csrc;
      no--;
    } // fi found matching CSRC    
  } //all CSRC-items
  return NULL;
}


char *hamdlnk_getsubptr(const thamdlnk_ref *packet, U8 stype, U8 no, U16 *datalength) {
  U16 offset, length;
  char *csrc = get_csrc(packet, stype, no);
  if (csrc == NULL) return NULL;
  MSB(offset) = csrc[1];
  LSB(offset) = csrc[2];
  MSB(length) = csrc[2];
  LSB(length) = csrc[3];
  offset >>= 4;
  length &= 0x7FF;
  *datalength = length;
  return packet->headerbegin + offset;
}


U8 hamdlnk_findsubptr(const thamdlnk_ref *packet, const U8 *stype_list, U8 no, U16 *datalength) {
  U16 length;
  const char *csrc = NULL;
  for (; (stype_list[0] != 0) && (csrc == NULL); stype_list++) {
    csrc = get_csrc(packet, stype_list[0], no);
  } // rof list
  if (csrc == NULL) return 255;
  MSB(length) = csrc[2];
  LSB(length) = csrc[3];
  length &= 0x7FF;
  *datalength = length;
  return csrc[0];
}



bool ham2lnk_subframe_setlength(thamdlnk_data *packet, U8 stype, U8 no, U16 length) {
  char *csrc = get_csrc(&packet->r, stype, no);
  if (csrc != NULL) {
    csrc[2] = (csrc[2] & 0xF8) | (MSB(length) & 0x07);
    csrc[3] = LSB(length);
    return true;
  }
  return false;
}



void ham2lnk_remove_subframe(thamdlnk_data *packet, U8 stype, U8 no) {
  U8  csrc_count = packet->r.headerbegin[0] & RTPHEADER_CC_MASK;
  char *csrc     = packet->r.headerbegin + 12;
  // get pointer to the no. CSRC in CSRC table
  for (; (csrc_count>0)&&(no>0); csrc_count--) {
    if (csrc[0] == stype) {
      if (no<=1) {	// found CSRC, now kill them
	int  cnt;
        U16  suboffset, sublength;
        char *hdrcsrc_begin;
	// 1. get data from CSRC
	MSB(suboffset) = csrc[1];
	LSB(suboffset) = csrc[2];
	MSB(sublength) = csrc[2];
	LSB(sublength) = csrc[3];
	suboffset >>= 4;
	sublength = (sublength+3) & 0x7FC;	// round up to aligned
	// 2. set and decrement CSRC counter
	packet->r.headerbegin[0] = packet->r.headerbegin[0] - 1;
	// 3. move down header
	memmove(packet->r.headerbegin + 4, packet->r.headerbegin, csrc - packet->r.headerbegin);
	// 4. move up data behind CSRC block
	memmove(packet->r.headerbegin + suboffset,
	  packet->r.headerbegin + suboffset + sublength,
	  packet->r.freedataofs - packet->r.headerbegin - suboffset - sublength);
	// 5. set correct header start
	packet->r.headerbegin += 4;
	// 6. update "data end" pointer
	packet->r.freedataofs -= sublength;
	// 7. recalculate offsets
	// all: -4 bytes, all offsets behind suboffset -sublengh
	hdrcsrc_begin = packet->r.headerbegin + 13;	// CSRC-Table (+1 = optimized)
	csrc_count    = packet->r.headerbegin[0] & RTPHEADER_CC_MASK;
	suboffset <<= 4;	// optimize compare and calculation on 4bit shifted offsets
	sublength <<= 4;
	for (cnt=0; cnt<csrc_count; cnt++) {		// Add 4 to Offsets
	  U16 offset;
	  MSB(offset) = hdrcsrc_begin[0];
	  LSB(offset) = hdrcsrc_begin[1];	// offset hold value x16 (lowest 4bits are part of len)
	  if ((offset & 0xFFF0) > suboffset)
	    offset -= sublength;
	  offset -= 0x0040;			// sub 4 bytes
	  hdrcsrc_begin[0] = MSB(offset);
	  hdrcsrc_begin[1] = LSB(offset);
	  hdrcsrc_begin += 4;
	} // rof offset update
	return;
      } // fi wanted CSRC
      no--;
    } // fi found a matching CSRC
    csrc += 4;
  } //all CSRC-items
}
