/*
 * Headerfile of checksums.c
 *
 * CRC calculating functions
 *
 * Author: Jan Alte, DO1FJN
 *
 *
 * This file is part of the DFU Serial Loader (dfu-loader).
 * For general infomation about dfu-loader see "dfu_serial_main.c".
 *
 * dfu-loader is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * dfu-loader is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __CHECKSUMS_H__
#define __CHECKSUMS_H__

void		crcCCITT(unsigned short *crc, char data);
unsigned short	crcCCITTBlock(const char *data, int length);
void		crcCCITTStream(unsigned short *crc, const char *data, int length);

void		crc32(unsigned long *crc, const char data);
void		crc32Stream(unsigned long *crc, const char *data, int length);

// used for D-Star:
void		append_crc_ccitt_revers(char *buffer, unsigned int len);
unsigned short	crc_ccitt_revers(const char *buffer, unsigned int len);


#endif
