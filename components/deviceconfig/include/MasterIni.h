/*
 * MasterIni.h
 * Config module of ESP32 firmware
 *
 *  Created on: 2020-04-06
 *      Author: Jan Alte
 */

#pragma once
#ifndef __MASTER_INI_H__
#define __MASTER_INI_H__

#include <esp_err.h>

#define WIFI_CFG_SECTION_NAME	"[WifiConfig]"
#define NTP_CFG_SECTION_NAME	"[NTP]"


#define CCONF_ID_WIFIMODE	0
#define CCONF_ID_SSID		1
#define CCONF_ID_PRESHAREDKEY	2
#define CCONF_ID_IDENTITY	3
#define CCONF_ID_USERNAME	4
#define CCONF_ID_PASSWORD	5

#define CCONF_ID_SERVER		6
#define CCONF_ID_PORT		7
#define CCONF_ID_SERVERCERTFILE	8
#define CCONF_ID_BASETOPIC	9
#define CCONF_ID_DATAINTERVAL	10

#define CCONF_ID_URL		11
#define CCONF_ID_DISPLAYNAME	12
#define CCONF_ID_HOSTNAME	13

#define CCONF_ID_APPPATH        14
#define CCONF_ID_ACCESSCODE     15


esp_err_t	INI_apply_configuration(char *config_text, unsigned int txt_len);

int	  	INI_create_configuration(char *textbuf, unsigned int buf_size);

void		INI_create_item(unsigned char CCONF_ID, char **textbuf, int *buf_left, const char *val, ...);
void		INI_create_commented_item(unsigned char CCONF_ID, char **textbuf, int *buf_left, const char *val, ...);

void		INI_create_comment(char **textbuf, int *buf_left, const char *text, ...);

void            INI_add_CRLF(char **textbuf, int *buf_left);

const char *	INI_get_NTPurl(const char *unset_default);


#endif // MASTERINI
