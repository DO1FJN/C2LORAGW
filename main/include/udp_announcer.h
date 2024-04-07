/*
 * udp_announcer.h
 *
 *  Created on: 02.04.2019
 *      Author: Jan Alte, DO1FJN
 */

#pragma once

#include <esp_err.h>


typedef struct {  
  const char *		myname;
  unsigned short  port_listen;
  const char *    json_msg;
} tAnnouncerParam;


esp_err_t upd_startannoucer(const tAnnouncerParam *Param);

