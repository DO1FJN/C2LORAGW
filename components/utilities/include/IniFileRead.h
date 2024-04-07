/*
 * IniFileRead.h
 *
 *  Created on: 05.02.2018
 *      Author: Jan Alte
 *          (c) 2018 bentrup Industriesteuerungen
 */

#ifndef INIFILEREAD_H_
#define INIFILEREAD_H_

typedef void (*tconfigfunction)(int nr, const char *value, void *static_param);


int	config_interpreter(const char *filename, const char *vnames[], tconfigfunction cfg_handle, void *param);
int	config_reader(char *text_buffer, unsigned short text_len, const char *vnames[], tconfigfunction cfg_handle, void *param);

int	str_makelist(char *liststr, char *list[], unsigned char size);

int	str_readlistu8(char *liststr, unsigned char *list, unsigned char size, unsigned char min, unsigned char max);

int	str_readlist16(char *liststr, short *list, unsigned char size, short min, short max);



#endif // INIFILEREAD_H_
