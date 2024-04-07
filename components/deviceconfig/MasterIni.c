/*
 * MasterIni.c
 * Config module of ESP32 firmware
 *
 *  Created on: 2020-04-06
 *      Author: Jan Alte

 Report:

 Bugs:
 ~ "letzter Parameter" ohne Zeilenende (Abgeschnitten und ohne null) wird nicht terminiert!
  (Übertragung vom STM32 nun mit Null-Byte)
 - Ausgabe Config-File mit 2 zusätzlichen Null-Bytes!
 */


#include "MasterIni.h"

#include "IniFileRead.h"
#include "utilities.h"		// nvs_getstring()

#include <nvs_flash.h>
#include <string.h>
#include <stdarg.h>
#include <esp_log.h>

#include <freertos/FreeRTOS.h>
//#include <freertos/event_groups.h>


#define CCONF_ID_MAX		CCONF_ID_ACCESSCODE


static const char *INITAG = "INI";


static const char *ini_cfg_list[CCONF_ID_MAX+2] = {
  "WifiMode", "SSID", "PresharedKey", "Identity", "Username", "Password",
  "Server", "Port", "ServerCertFile", "BaseTopic", "DataInterval",
  "URL",
  "DisplayName", "HostName", "AppPath", "AccessCode",
  NULL
};
#define CCONF_ID_MAX		CCONF_ID_ACCESSCODE


static const char *NTP_namespace = "NTPclient";
static const char *NTPcfg_URL    = "NTPserver";

static const char *NTPdefaultURL;


// BEGIN external forward...

int	WiFiSTA_create_configuration(char *textbuf, unsigned int buf_size);
void	wifista_get_parameter(int ininr, const char *value, void *param);
size_t	wifista_get_param_size(void);


// END external forward.


// BEGIN internal forward...

void	ntp_get_parameter(int ininr, const char *value, void *param);
int	ntp_create_configuration(char *textbuf, unsigned int buf_size);

// END internal forward.


void ini_read_config(int ininr, const char *value, void *context) {
  static tconfigfunction section_handle = NULL;
  static void *section_data = NULL;
  if (ininr == -2) {
    // leave section call:
    if (section_handle != NULL) {
      section_handle(-1, NULL, section_data);
      section_handle = NULL;
#ifdef CONFIG_PARSE_INI_DEBUG
      printf("INI [end section]\r\n\r\n");
#endif
    }
    if (section_data) {
      vPortFree(section_data);		// free wifi_config
      section_data = NULL;
    }

    // select section handler:
#ifdef CONFIG_INTERNAL_WIFI
    if (strcasecmp(value, WIFI_CFG_SECTION_NAME) == 0) {
      section_handle = (tconfigfunction) wifista_get_parameter;
      section_data   = pvPortMalloc(wifista_get_param_size());
    } else
#endif

    if (strcasecmp(value, NTP_CFG_SECTION_NAME) == 0) {
      section_handle = (tconfigfunction) ntp_get_parameter;
    } else if (strcasecmp(value, "eof") == 0) {
      section_handle = NULL;
    }
#ifdef CONFIG_PARSE_INI_DEBUG
      printf("INI %s : %s\r\n", value, section_handle == NULL? "ignore, not in list": "parse...");
#endif

    // ender section call:
    if (section_handle != NULL) section_handle(-2, value, section_data);
    // ***
  } else if ((ininr >= 0) && (section_handle != NULL)) {
#ifdef CONFIG_PARSE_INI_DEBUG
      printf("INI parse #%d: %s\r\n", ininr, value);
#endif
    section_handle(ininr, value, section_data);    
  }
}



esp_err_t INI_apply_configuration(char *config_text, unsigned int txt_len) {
#ifdef CONFIG_PARSE_INI_DEBUG
  printf("INI got text (%d bytes)\n", txt_len);
#endif
  if (config_text != NULL) {
    int exitcode;
    ESP_LOGD(INITAG, "load new config");
    exitcode = config_reader(config_text, txt_len, ini_cfg_list, ini_read_config, NULL);
    if (exitcode != 0) return ESP_FAIL;
  } // esle

  return ESP_OK;
}



void INI_add_value(char **textbuf, int *buf_left, const char *val, va_list args) {
  int chunksize = vsnprintf(*textbuf, *buf_left, val, args);
  *textbuf  += chunksize;
  *buf_left -= chunksize;
}


void INI_add_CRLF(char **textbuf, int *buf_left) {
  if (buf_left[0] > 1) {
    sprintf(*textbuf, "\r\n");
    *textbuf  += 2;
    *buf_left -= 2;
  }
}


void INI_create_item(unsigned char CCONF_ID, char **textbuf, int *buf_left, const char *val, ...) {
  va_list args;
  if (CCONF_ID > CCONF_ID_MAX) return;
  int chunksize = snprintf(*textbuf, *buf_left, "%s = ", ini_cfg_list[CCONF_ID]);
  *textbuf  += chunksize;
  *buf_left -= chunksize;
  va_start(args, val);
  INI_add_value(textbuf, buf_left, val, args);
  va_end(args);
  INI_add_CRLF(textbuf, buf_left);
}


void INI_create_commented_item(unsigned char CCONF_ID, char **textbuf, int *buf_left, const char *val, ...) {
  va_list args;
  if (CCONF_ID > CCONF_ID_MAX) return;
  int chunksize = snprintf(*textbuf, *buf_left, "#%s = ", ini_cfg_list[CCONF_ID]);
  *textbuf  += chunksize;
  *buf_left -= chunksize;
  va_start(args, val);
  INI_add_value(textbuf, buf_left, val, args);
  va_end(args);
  INI_add_CRLF(textbuf, buf_left);
/*  if (CCONF_ID > CCONF_ID_MAX) return;
  int chunksize = snprintf(*textbuf, *buf_left, "#%s = %s\r\n", ini_cfg_list[CCONF_ID], val);
  *textbuf  += chunksize;
  *buf_left -= chunksize;*/
}


void INI_create_comment(char **textbuf, int *buf_left, const char *text, ...) {
  va_list args;
  if (buf_left[0] < 4) return;
  *textbuf[0] = '#';
  (*textbuf)++;
  (*buf_left)--;
  va_start(args, text);
  INI_add_value(textbuf, buf_left, text, args);
  va_end(args);
  INI_add_CRLF(textbuf, buf_left);
}



int INI_create_configuration(char *textbuf, unsigned int buf_size) {
  int sectionsize;
  int buf_left = buf_size;
  ESP_LOGD(INITAG, "create, txt-buffer %d bytes", buf_size);
  if (buf_size < 192) {
    ESP_LOGE(INITAG, "create_config: buffer too small (%d bytes)", buf_size);
    return -1;
  }

#ifdef CONFIG_INTERNAL_WIFI
  // create Wifi-Config
  sectionsize = WiFiSTA_create_configuration(textbuf, buf_left);
  buf_left   -= sectionsize;
  textbuf    += sectionsize;
  if (buf_left <= 0) {
    ESP_LOGE(INITAG, "create_config: buffer too small (%d bytes)", buf_size);
    return -2;
  }
#endif

  sectionsize = ntp_create_configuration(textbuf, buf_left);
  buf_left   -= sectionsize;
  textbuf    += sectionsize;
  if (buf_left <= 0) {
    ESP_LOGE(INITAG, "create_config: buffer too small (%d bytes)", buf_size);
    return -2;
  }
  // ...
  ESP_LOGD(INITAG, "done, txt-size %d bytes", buf_size - buf_left);
  return buf_size - buf_left;
}





void ntp_get_parameter(int ininr, const char *value, void *param) {
  static esp_err_t err;
  //int idx;
  static nvs_handle ntp_hnd;
  switch (ininr) {
  case -2:			// enter section
    err = nvs_open(NTP_namespace, NVS_READWRITE, &ntp_hnd);
    break;
  case -1:			// leave section
    if (err == ESP_OK) nvs_close(ntp_hnd);
    break;
  case CCONF_ID_URL:		// "URL"
    if (err == ESP_OK) nvs_set_str(ntp_hnd, NTPcfg_URL, value);
    break;
  } // hctiws ininr
}




const char *INI_get_NTPurl(const char *unset_default) {
  NTPdefaultURL = unset_default;
  return unset_default;
}



int ntp_create_configuration(char *textbuf, unsigned int buf_size) {
  nvs_handle    hnd;
  esp_err_t	err;
  int		chunksize;
  const char *	ntpurl;
  int buf_left = buf_size;
  if (buf_size < 192) return -1;

  chunksize = snprintf(textbuf, buf_left, NTP_CFG_SECTION_NAME "\r\n");
  textbuf  += chunksize;
  buf_left -= chunksize;

  ntpurl = NTPdefaultURL==NULL? "": NTPdefaultURL;

  err = nvs_open(NTP_namespace, NVS_READONLY, &hnd);
  if (err != ESP_OK) {
    ESP_LOGD(INITAG, "no NTP config");
    INI_create_item(CCONF_ID_URL, &textbuf, &buf_left, ntpurl);
  } else {
    char item[192];
    err = nvs_load_string(hnd, NTPcfg_URL, item, sizeof(item));
    INI_create_item(CCONF_ID_URL, &textbuf, &buf_left, (err == ESP_OK)? item: ntpurl);
    nvs_close(hnd);
  }
  return (buf_left == 0)? -2: buf_size - buf_left;
}
