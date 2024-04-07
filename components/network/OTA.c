/*
 * OTA.c
 *
 *  Created on: 26.03.2020
 *      Author: Jan Alte
 *          (c) 2020 bentrup Industriesteuerungen
 *
 * Report:
 *
 * ToDo list
 * - split Update and Reboot
 * - signalling queue 4 status
 * - better certificate handling: storing server-base-URL + Cert
 */

#include "OTA.h"


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_http_client.h>
#include <esp_https_ota.h>
#include <string.h>


static const char *OTATAG = "OTA";

#define OTA_TASK_STACKSIZE		8192
#define OTA_TASK_PRIORITY		5

#define MIN_PROGRESS_TICKS		(1000 / portTICK_PERIOD_MS)

#define HTTPS_MAX_CERTKEY_FILEZIZE	4096
#define HTTPS_DEFAULT_CERT		"/ifs/certs/isrgrootx1.pem"		// valid after Jan-2021
//define HTTPS_DEFAULT_CERT		"/ifs/certs/lets-encrypt-x3.pem" 



static int		OTA_status  = UPD_INACTIVE;
static int		OTA_rdbytes = 0;
static tprogresshandler	OTA_info_update;


esp_err_t OTA_GetFirmwareInfo(tFirmwareInfo *info) {
  const esp_app_desc_t *app_info;
  if (info == NULL) return ESP_FAIL;
  app_info = esp_app_get_description();
  if (app_info != NULL) {
    info->name    = app_info->project_name;
    info->version = app_info->version;
    info->build_date = app_info->date;
    info->build_time = app_info->time;
    ESP_LOGD(OTATAG, "Firmware: %s %s (%s %s)", info->name, info->version, info->build_date, info->build_time);
  } else {
    ESP_LOGE(OTATAG, "Firmware Info failed");
  }
  return app_info != NULL? ESP_OK: ESP_FAIL;
}


esp_err_t OTA_GetFirmwareInfoCopy(tFirmwareInfo *info) {
  const esp_app_desc_t *app_info;
  if (info == NULL) return ESP_FAIL;
  //memset(info, 0, sizeof(tFirmwareInfo));
  //err = esp_ota_get_partition_description(esp_ota_get_running_partition(), &app_info);
  app_info = esp_app_get_description();
  if (app_info != NULL) {
    info->name    = strndup(app_info->project_name, 32);
    info->version = strndup(app_info->version, 32);
    info->build_date = strndup(app_info->date, 16);
    info->build_time = strndup(app_info->time, 16);
    ESP_LOGD(OTATAG, "Firmware: %s %s (%s %s)", info->name, info->version, info->build_date, info->build_time);
  } else {
    ESP_LOGE(OTATAG, "Firmware Info failed");
  }
  return app_info != NULL? ESP_OK: ESP_FAIL;
}


unsigned int OTG_GetVersionAsUInt(void) {
  unsigned int fwversion = -1;
  const esp_app_desc_t *app_info = esp_app_get_description();
  if (app_info == NULL) {
    ESP_LOGE(OTATAG, "Firmware GetVersion failed");
    return -1;
  }
  ///"0.70.1"
  fwversion = ((app_info->version[0]-'0') << 12) | (strtol(app_info->version + 2, NULL, 16) << 4) | (app_info->version[5] - '0');
  ESP_LOGD(OTATAG, "Firmware: coded version 0x%04x", fwversion);
  return fwversion;
}



int OTA_GetStatus(void) {
  return OTA_status;
}


int OTA_GetProcessedSize(void) {
  return OTA_rdbytes;
}


void OTA_SetProgress_Handler(tprogresshandler func) {
  OTA_info_update = func;
}




char *ota_load_server_certificate(const char *filename, size_t max_cert_size) {
  FILE *file;
  int   cert_len;
  char *https_certificate;
  
  if (filename == NULL) return NULL;
  https_certificate = pvPortMalloc(max_cert_size);
  if (https_certificate == NULL) {
    ESP_LOGE(OTATAG, "malloc cert buffer failed (%d bytes)", max_cert_size);
    return NULL;
  }
  file = fopen(filename, "r");
  if (file == NULL) {
    ESP_LOGE(OTATAG, "fopen(): cert file not found");
    vPortFree(https_certificate);
    return NULL;
  }
  cert_len = fread(https_certificate, 1, max_cert_size, file);
  fclose(file);
  ESP_LOGI(OTATAG, "using cert '%s' (%d bytes)", filename, cert_len);
  if (cert_len > 0) 
    return https_certificate;
    
  vPortFree(https_certificate);  
  return NULL;
}



static esp_err_t validate_image_header(esp_app_desc_t *new_app_info) {
  esp_app_desc_t running_app_info;
  if (new_app_info == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  const esp_partition_t *running = esp_ota_get_running_partition();
  if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
    ESP_LOGI(OTATAG, "Running firmware version: %s; new version: %s", running_app_info.version, new_app_info->version);
  } else {
    ESP_LOGW(OTATAG, "Can't get own app-info! Proceed with update.");
    return ESP_OK;
  }
  if (strncmp(new_app_info->project_name, running_app_info.project_name, sizeof(new_app_info->project_name)) != 0) {
    ESP_LOGW(OTATAG, "Different firmwarename! Update canceled.");
    return ESP_FAIL;
  }
  if (strncmp(new_app_info->version, running_app_info.version, sizeof(new_app_info->version)) == 0) {
    ESP_LOGW(OTATAG, "Current running version is the same as a new. Update canceled.");
    return ESP_FAIL;
  }
  return ESP_OK;
}



static void over_the_air_update(void *threaddata) {
  TickType_t			tickcnt;
  tOTAjob *			otajob;
  char *			cert_filename;	// Buffer for certificate
  esp_err_t			err, ota_err = ESP_FAIL;
  esp_app_desc_t 		app_desc;
  esp_http_client_config_t	http_cconf;
  esp_https_ota_config_t	ota_cfg		= { .http_config = &http_cconf };  
  esp_https_ota_handle_t	https_ota_hnd	= NULL;

  otajob = (tOTAjob *) threaddata;
  
  if ((threaddata == NULL) || (otajob->url == NULL)) {
    ESP_LOGE(OTATAG, "no OTA job data");
    goto ota_task_exit;
  }
  tickcnt = xTaskGetTickCount();
  
  if ((otajob->cert == NULL) || (strnlen(otajob->cert, 8) < 8) ) {
    ESP_LOGD(OTATAG, "no cert defined, using default...");
    otajob->cert = ota_load_server_certificate(HTTPS_DEFAULT_CERT, HTTPS_MAX_CERTKEY_FILEZIZE);
  } else if (strnlen(otajob->cert, 128) < 128) {	// is Filename?
    cert_filename = (char *)otajob->cert;
    ESP_LOGD(OTATAG, "cert is filename '%s'", cert_filename);
    otajob->cert   = ota_load_server_certificate(cert_filename, HTTPS_MAX_CERTKEY_FILEZIZE);
    vPortFree(cert_filename);
  } // fi file?

  
  ESP_LOGI(OTATAG, "fetching firmware '%s'...", otajob->url);
  memset(&http_cconf, 0, sizeof(http_cconf));
  http_cconf.url      = otajob->url;
  http_cconf.cert_pem = otajob->cert;
  http_cconf.username = otajob->user;
  http_cconf.password = otajob->passwd;
  //http_cconf. = otajob->
  
  OTA_status  = UPD_PROCESSING;
  OTA_rdbytes = 0;
  
  err = esp_https_ota_begin(&ota_cfg, &https_ota_hnd);
  if (err != ESP_OK) {
    OTA_status = UPD_ERR_HTTPS_CONN;
    ESP_LOGE(OTATAG, "ESP HTTPS OTA Begin failed");
    goto ota_task_exit;
  } // fi begin
  
  err = esp_https_ota_get_img_desc(https_ota_hnd, &app_desc);
  if (err != ESP_OK) {
    OTA_status = UPD_ERR_INVAL_HEADER;
    ESP_LOGE(OTATAG, "rx APP image description failed");
    goto ota_task_end;
  }
  
  if (otajob->flags & OTAJOB_FLAG_FORCE) {
    ESP_LOGW(OTATAG, "Forced-Update, skip image header validation!");
  } else {
    err = validate_image_header(&app_desc);
    if (err != ESP_OK) {
      OTA_status = UPD_ERR_SAME_VERSION;
      ESP_LOGE(OTATAG, "APP header validation failed");
      goto ota_task_end;
    }
  } // fi not forced
  
  do {
    ota_err = esp_https_ota_perform(https_ota_hnd);
    OTA_rdbytes = esp_https_ota_get_image_len_read(https_ota_hnd);
    if (OTA_info_update && (ota_err == ESP_ERR_HTTPS_OTA_IN_PROGRESS) && (xTaskGetTickCount() > tickcnt)) {
      OTA_info_update();
      tickcnt = xTaskGetTickCount() + MIN_PROGRESS_TICKS;
    }
    ESP_LOGV(OTATAG, "%7d bytes read", OTA_rdbytes);
  } while (ota_err == ESP_ERR_HTTPS_OTA_IN_PROGRESS);

ota_task_end:
  err = esp_https_ota_finish(https_ota_hnd);
  if (ota_err != ESP_OK) {
    OTA_status = UPD_ERR_DOWNLOAD;
    ESP_LOGE(OTATAG, "upgrade failed...");    
  } else if (err != ESP_OK) {
    OTA_status = UPD_ERR_FINISHING;
    ESP_LOGE(OTATAG, "finishing failed");
  } else {
    OTA_status = UPD_DONE;
    ESP_LOGI(OTATAG, "upgrade successful.");
  }
ota_task_exit:
  if (OTA_info_update) OTA_info_update();
  OTA_info_update = NULL;
  if (otajob) {
    if (otajob->passwd) vPortFree(otajob->passwd);
    if (otajob->user) vPortFree(otajob->user);
    if (otajob->cert) vPortFree(otajob->cert);
    if (otajob->url) vPortFree(otajob->url);
    memset(otajob, 0, sizeof(tOTAjob));
  } // fi cleanup
  ESP_LOGI(OTATAG, "canceled");
  vTaskDelete(NULL);
}




esp_err_t OTA_fwupdate(const char *url, const char *cert, const char *user, const char *passwd, unsigned int flags) {
  static tOTAjob otajob;
  int res;
  if ((OTA_status == UPD_PROCESSING))
    return ESP_FAIL;
  OTA_status = UPD_PROCESSING;
  OTA_info_update = NULL;
  memset(&otajob, 0, sizeof(otajob));
  if (url)  otajob.url  = strndup(url, 1024);
  if (cert) otajob.cert = strndup(cert, 4096);
  if (user) otajob.user = strndup(user, 64);
  if (passwd) otajob.passwd = strndup(passwd, 64);
  otajob.flags = flags;
  res = xTaskCreate(over_the_air_update, "OTA", OTA_TASK_STACKSIZE, (void *)&otajob, OTA_TASK_PRIORITY, NULL);
  if (res != pdPASS) {
    ESP_LOGE(OTATAG, "create OTA thread fails (%d)!", res);
    return ESP_FAIL;
  }
  return ESP_OK;
}

