/*
 * HAMdLNK_types.h
 *
 * Defines of HAMdLNK protocol types
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

#ifndef HAMDLNK_TYPES_H_
#define HAMDLNK_TYPES_H_

#include "compiler.h"


#define HAMdLNK_MAXPACKET_SIZE	1440	// in bytes

#define HAMdLNK_MAXHEADER_SIZE	72	// 72. byte is fixed
#define HAMdLNK_MAXDATA_SIZE	(HAMdLNK_MAXPACKET_SIZE-HAMdLNK_MAXHEADER_SIZE)


typedef struct {
  char		*headerbegin;		// pointer to first byte of the RTP packet (must be in packet)
  char		*databegin;
  char		*freedataofs;
} thamdlnk_ref;


typedef struct {
  thamdlnk_ref r;
  struct PACKED_DATA {
    char	header[HAMdLNK_MAXHEADER_SIZE];
    char	data[HAMdLNK_MAXDATA_SIZE];
  };
} thamdlnk_data;



// RTP Header:
#define RTPHEADER_RTPVERS_MASK	0xF0
#define RTPHEADER_CC_MASK	0x0F



#endif // HAMDLNK_TYPES_H_
