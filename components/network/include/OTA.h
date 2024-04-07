/*
 * OTA.h
 *
 *  Created on: 26.03.2020
 *      Author: Jan Alte
 *          (c) 2020 bentrup Industriesteuerungen
 */

#pragma once
#ifndef __INCLUDE_OTA_H_
#define __INCLUDE_OTA_H_

#include <esp_err.h>

typedef struct {
  const char *	name;
  const char *	version;
  const char *	build_date;
  const char *	build_time;
} tFirmwareInfo;


typedef struct {
  char *	url;
  char *	cert;
  char *	user;
  char *	passwd;
  unsigned int	flags;
} tOTAjob;

typedef void (*tprogresshandler)(void);

#define OTAJOB_FLAG_FORCE	1


#define UPD_INACTIVE		0
#define UPD_PROCESSING		1
#define UPD_DONE		2
#define UPD_ERR_HTTPS_CONN	8
#define UPD_ERR_INVAL_HEADER	9
#define UPD_ERR_SAME_VERSION	10
#define UPD_ERR_DOWNLOAD	11
#define UPD_ERR_FINISHING	12


esp_err_t	OTA_GetFirmwareInfo(tFirmwareInfo *info);
esp_err_t	OTA_GetFirmwareInfoCopy(tFirmwareInfo *info);

unsigned int	OTG_GetVersionAsUInt(void);
int		OTA_GetStatus(void);
int		OTA_GetProcessedSize(void);

esp_err_t	OTA_fwupdate(const char *url, const char *cert, const char *user, const char *passwd, unsigned int flags);

void		OTA_SetProgress_Handler(tprogresshandler func);

#endif
