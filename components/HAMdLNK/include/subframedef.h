/*
 * subframedef.h
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

#ifndef SUBFRAMEDEF_H_
#define SUBFRAMEDEF_H_

#define SUBFRAME_CLASS_RAW	0x00
#define SUBFRAME_CLASS_TEXT	0x10
#define SUBFRAME_CLASS_SPEECH	0x20				// low bitrate vocoder and speech-optimized codecs
#define SUBFRAME_CLASS_AUDIO	0x30				// medium bitrate audio codes and uncompressed PCM
#define SUBFRAME_CLASS_IMAGE	0x40
#define SUBFRAME_CLASS_VIDEO	0x50


#define SF_NODE_CTRL		      (SUBFRAME_CLASS_RAW | 0x00)

#define SF_HFEMBEDDED		   (SUBFRAME_CLASS_RAW | 0x01)

// Text-Definitions
#define SF_CONTROL		      (SUBFRAME_CLASS_TEXT | 0x00)	// Control-Dialog
#define SF_SENDERID		      (SUBFRAME_CLASS_TEXT | 0x01)	// CALL sign of sender (from HF)
#define SF_SENDERINFO		   (SUBFRAME_CLASS_TEXT | 0x02)	// additional Info (ASCII only)
#define SF_RECIPIENT          (SUBFRAME_CLASS_TEXT | 0x03)	// recipient field


// Codec Definitions:
#define SF_CODEC_IMBE		(SUBFRAME_CLASS_SPEECH | 0x00)
#define SF_CODEC_AMBEplus2400	(SUBFRAME_CLASS_SPEECH | 0x01)
#define isSF_MBE(sf)		((sf>=SF_CODEC_IMBE) && (sf<=SF_CODEC_AMBEplus2400))
#define SF_MBE_get_mode(sf)	(sf==SF_CODEC_IMBE? 0: (sf==SF_CODEC_AMBEplus2400? 2: -1))

#define SF_CODEC_CODEC2_700C	(SUBFRAME_CLASS_SPEECH | 0x0C)	// 4 + CODEC2_MODE (defined in codec2.h
#define SF_CODEC_CODEC2_1200	(SUBFRAME_CLASS_SPEECH | 0x09)
#define SF_CODEC_CODEC2_1300	(SUBFRAME_CLASS_SPEECH | 0x08)
#define SF_CODEC_CODEC2_1400	(SUBFRAME_CLASS_SPEECH | 0x07)
#define SF_CODEC_CODEC2_1600	(SUBFRAME_CLASS_SPEECH | 0x06)
#define SF_CODEC_CODEC2_2400	(SUBFRAME_CLASS_SPEECH | 0x05)
#define SF_CODEC_CODEC2_3200	(SUBFRAME_CLASS_SPEECH | 0x04)
#define isSF_CODEC2(sf)		((sf>=SF_CODEC_CODEC2_3200) && (sf<=SF_CODEC_CODEC2_700C))
#define SF_CODEC2_get_mode(sf)	((sf&0x0F)-4)

// Audio Definitions:
#define SF_AUDIO_PCM8A		(SUBFRAME_CLASS_AUDIO | 0x00)	// PCM audio 8bit 8000Sps aLaw
#define SF_AUDIO_PCM8U		(SUBFRAME_CLASS_AUDIO | 0x01)	// PCM audio 8bit 8000Sps µLaw


/* SF_CONTROL Sub-Frame Type
 * defined commands
 */

#define GET_CONTROLMSGID(msg, strng)	{  MSB0W(msg) = strng[0]; \
   MSB1W(msg) = strng[1]; MSB2W(msg) = strng[2]; MSB3W(msg) = strng[3]; }

#define CONTROLMSG_LEN		4

// Start: ID + Space + 1 Byte Parameter SubFrame-Type of Main-Codec
// Parameter: Defines in the first packet, (don't having RT data), with kind of stream follows...
// Example: STRT_ByteOf(SF_CODEC_AMBEplus2400) -> A D-Star compattible stream strats now
#define CONTROLMSG_STRT_S	"STRT  "	// a space and a extra parameter byte for Codec
#define CONTROLMSG_STRT		'S'<<24|'T'<<16|'R'<<8|'T'
// End of Stream: Marks last packet
#define CONTROLMSG_END_S	"END "
#define CONTROLMSG_END		'E'<<24|'N'<<16|'D'<<8|' '
// Lost of Stream: Marks last packet too, but stream is broken on HF side
#define CONTROLMSG_LOST_S	"LOST"
#define CONTROLMSG_LOST		'L'<<24|'O'<<16|'S'<<8|'T'


const char *sf_get_subframe_name(unsigned char sf_codec_type);

unsigned char sf_get_subfrage_type(const char *sf_name);

unsigned short sf_get_subframe_bits(unsigned char sf_codec_type);

#define SF_MAX_BITS_PER_FRAME    88    // IMBE

#endif // SUBFRAMEDEF_H_
