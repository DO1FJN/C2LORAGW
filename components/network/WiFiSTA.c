/*
 * WiFiSTA.c
 *
 *  Created on: 14.03.2019
 *      Author: Jan Alte
 *          (c) 2019 bentrup Industriesteuerungen
 *
 * Report:
 * 2019-03-15	first version (w/o loading certs)
 * 2020-03-26	config reader prepared 4 universal usage (NTP, MQTT sections...)
 * 2021-11-03	Wifi event handler only registered if WiFi was active.
 * 2023-10-25 Rework / altered shutdown, reload sequences, less action within event handler (crashes sometimes)
 *
 * ToDo list
 * + test altered config_reader
 * - handling certificates for WPA-enterprise connections
 * + test WPA-PSK
 * + test WPS
 */

#include "WiFiSTA.h"
#include "LAN.h"
#include "utilities.h"
#include "MasterIni.h"

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include <esp_wifi.h>
#include <esp_eap_client.h>
#include <esp_wps.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <esp_netif.h>

#include "esp_console.h"
#include "argtable3/argtable3.h"

//define DEBUG_WIFICONF	// => enable for outputting a full set on WiFi settings

#define CONNECT_MIN_DELAY_TICKS   (10 * configTICK_RATE_HZ)   // 10s

#define WAIT_DISCONNECTED_MS      150


#ifndef __weak
#define __weak		__attribute__((weak))
#endif


#define WiFi_SCAN_METHOD		      WIFI_ALL_CHANNEL_SCAN	// WIFI_FAST_SCAN
#define WIFISTA_MAX_RECONN_TRIES  5


typedef void (*wifista_action_handler_t)(int32_t event, const wifi_event_sta_disconnected_t *dis_evnt);


static const char *wifi_mode_list[] = {
  "off", "wpa-psk", "wpa-enterprise", "open",
  NULL
};

#define WiFimode_undef		255
#define WiFimode_off		  0
#define	WiFimode_WPAPSK		1
#define	WiFimode_WPAenterprise	2
#define	WiFimode_OPEN	    3
#define	WiFimode_maxValue	3

static uint8_t	      wifi_mode;
static wifi_config_t  wifi_config;
static const char *   wifi_hostname = NULL;

// FreeRTOS event group to signal when we are connected & ready to make a request
extern EventGroupHandle_t lan_event_group;	// from LAN.c

// Constants that aren't configurable in menuconfig
#define EAP_PEAP 1
#define EAP_TTLS 2

static const char *WIFITAG        = "WiFiSTA";

static const char *WIFI_namespace = "WiFiSTA";
static const char *WIFIcfg_Mde	  = "WiFimode";
static const char *WIFIcfg_SSID	  = "SSID";
static const char *WIFIcfg_PSK	  = "PSK";
static const char *WIFIcfg_ID	    = "WPAeID";
static const char *WIFIcfg_USR	  = "WPAeUser";
static const char *WIFIcfg_PW	    = "WPAePass";


static wifista_action_handler_t wifista_action_handler = NULL;

static nvs_handle	wifi_config_hnd;
static esp_netif_t *	sta_netif = NULL;
static esp_event_handler_instance_t wifi_hnd = NULL;
static uint8_t wifi_reconnect_tries;
static wifi_ap_record_t *	ap_info;
static uint16_t			ap_info_size;
static uint16_t			ap_info_cnt;


#ifdef ENABLE_WPS

static const esp_wps_config_t	default_wps_cfg = {
  .wps_type = WPS_TYPE_DISABLE,
  .factory_info = {
    .manufacturer = "bentrup Industriesteuerungen",
    .model_number = WIFI_WPS_MODEL_NUMBER,
    .model_name   = WIFI_WPS_MODEL_NAME,
    .device_name  = CONFIG_DEVICE_NAME
  }
};

static esp_wps_config_t		wps_config;

#endif



#define	WiFiSTA_exitfunct_on_err(err)	if (err != ESP_OK) { \
  ESP_LOGE(WIFITAG, "%s failed: %s", __func__, esp_err_to_name(err));	\
  return err; }

#define	WiFiSTA_breakswitch_on_err(err)	if (err != ESP_OK) { \
  ESP_LOGE(WIFITAG, "%s failed: %s", __func__, esp_err_to_name(err));	\
  break; }



#define MAX_WIFI_CFG_ITEM_SIZE	128


static void WiFi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);


static const char *wifista_attach2netif(const char *hostname) {
  const char *curr_hostname = hostname;
  esp_netif_config_t cfg = ESP_NETIF_DEFAULT_WIFI_STA();
  // esp_netif_init();
  sta_netif = esp_netif_new(&cfg);
  assert(sta_netif);
  if ( (strnlen(hostname, 4) > 3) ) {
    esp_err_t err = esp_netif_set_hostname(sta_netif, hostname);
    if (err != ESP_OK)
      ESP_LOGE(WIFITAG, "set hostname '%s' fails", hostname);
    else
      ESP_LOGD(WIFITAG, "hostname '%s' set", hostname);
  } // fi
  esp_netif_get_hostname(sta_netif, &curr_hostname);
  esp_netif_attach_wifi_station(sta_netif);    
  esp_wifi_set_default_wifi_sta_handlers();
  ESP_LOGD(WIFITAG, "curr hostname: %s", curr_hostname);
  return curr_hostname;
}


void *WiFiSTA_getNetIf(void) {
  return sta_netif;
}

const char *WiFiSTA_get_hostname(void) { 
  const char *curr_hostname = NULL;
  if (sta_netif == NULL)  return NULL;
  esp_netif_get_hostname(sta_netif, &curr_hostname);
  return curr_hostname;
}


static esp_err_t wifista_create_config(wifi_config_t *config) {  
  if (config == NULL) return ESP_FAIL;
  memset(config, 0, sizeof(wifi_config_t));
  config->sta.scan_method = WiFi_SCAN_METHOD;
  config->sta.listen_interval = 3;  // Default
  config->sta.pmf_cfg.capable = true;
  config->sta.threshold.authmode = WIFI_AUTH_WPA_PSK;
  return ESP_OK;  
}


static esp_err_t wifista_save_config(bool set_mode) {
  int set_flag = 0;
  wifi_config_t wps_config;
  esp_err_t err = esp_wifi_get_config(ESP_IF_WIFI_STA, &wps_config);
  WiFiSTA_exitfunct_on_err(err);
  err = nvs_open(WIFI_namespace, NVS_READWRITE, &wifi_config_hnd);
  WiFiSTA_exitfunct_on_err(err);
  if (strnlen((char *)wps_config.sta.ssid, 3) > 1) {
    nvs_set_str(wifi_config_hnd, WIFIcfg_SSID, (char *)wps_config.sta.ssid);
    set_flag = 1;
  }
  if (strnlen((char *)wps_config.sta.password, 9) > 7) {
    nvs_set_str(wifi_config_hnd, WIFIcfg_PSK, (char *)wps_config.sta.password);
    set_flag |= 2;
  }
  if (set_flag == 3) {
    if (set_mode) {
      wifi_mode = WiFimode_WPAPSK;
      nvs_set_u8(wifi_config_hnd, WIFIcfg_Mde, wifi_mode);
    }
    err = ESP_OK;
    ESP_LOGD(WIFITAG, "got new network '%s' by WPS/Flash", (char *)wps_config.sta.ssid);
  } else {
    err = ESP_FAIL;
    ESP_LOGE(WIFITAG, "config incomplete! (network '%s')", (char *)wps_config.sta.ssid);
  }
  nvs_close(wifi_config_hnd);
  return err;
}


esp_err_t wifista_enable_psk(nvs_handle hnd, wifi_config_t *config) {  
  if (config == NULL) return ESP_FAIL;
  return nvs_load_string(hnd, WIFIcfg_PSK, (char *)config->sta.password, sizeof(config->sta.password));
}


esp_err_t wifista_enable_enterprise(nvs_handle hnd) {
  char item[MAX_WIFI_CFG_ITEM_SIZE];
  esp_err_t err;

  // ToDo Certs
  /*
  ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_set_ca_cert(ca_pem_start, ca_pem_bytes));
  ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_set_cert_key(client_crt_start, client_crt_bytes, client_key_start, client_key_bytes, NULL, 0));
  */
  err = nvs_load_string(hnd, WIFIcfg_ID, item, sizeof(item));
  if (err == ESP_OK) {
    err = esp_eap_client_set_identity((uint8_t *)item, strlen(item));
    if (err != ESP_OK) return err;
  } // fi
  err = nvs_load_string(hnd, WIFIcfg_USR, item, sizeof(item));
  if (err == ESP_OK) {
    err = esp_eap_client_set_username((uint8_t *)item, strlen(item));
    if (err != ESP_OK) return err;
  } // fi
  err = nvs_load_string(hnd, WIFIcfg_PW, item, sizeof(item));
  if (err == ESP_OK) {
    err = esp_eap_client_set_password((uint8_t *)item, strlen(item));
    if (err != ESP_OK) return err;
  } // fi

  return esp_wifi_sta_enterprise_enable();
}


esp_err_t wifista_enable_openwifi(nvs_handle hnd, wifi_config_t *config) {  
  if (config == NULL) return ESP_FAIL;  
  return ESP_OK;
}


/* WiFiSTA_unload()
   stopping disconected wifi and destroys netif link. Unloads wifi driver after.
*/
esp_err_t WiFiSTA_unload(void) {  
  if (wifi_hnd != NULL) {
    ESP_LOGD(WIFITAG, "unregister wifi event handler...");    
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_hnd);
    wifi_hnd = NULL;
  }  
  if (sta_netif != NULL) {
    ESP_LOGD(WIFITAG, "destroying netif...");    
    esp_netif_destroy(sta_netif);
    sta_netif = NULL;
  }
  ESP_LOGD(WIFITAG, "deinit wifi driver...");    
  return esp_wifi_deinit();
}


/* WiFiSTA_init()
   reading nsv and - if wifi_mode != off - initialize wifi and netif interface
*/
esp_err_t WiFiSTA_init(const char *hostname) {
  const wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();

  esp_err_t  err;
  nvs_handle hnd;

  ESP_LOGD(WIFITAG, "init wifi...");
  wifista_action_handler = NULL;
  wifi_mode = WiFimode_off;
  wifi_hnd  = NULL;
  ap_info   = NULL;
  ap_info_size = 0;

  if (hostname != NULL) wifi_hostname = hostname;  // update hostname_ptr

  err = nvs_open(WIFI_namespace, NVS_READONLY, &hnd);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGD(WIFITAG, "no WiFiSTA NVS configuration exists.");
    return ESP_OK;
  } else WiFiSTA_exitfunct_on_err(err);

  err = nvs_get_u8(hnd, WIFIcfg_Mde, &wifi_mode);
  switch (err) {
  case ESP_ERR_NVS_NOT_FOUND:
    wifi_mode = WiFimode_off;
    break;
  case ESP_OK:
    break;
  default:
    ESP_LOGE(WIFITAG, "error reading WiFimode (%s)", esp_err_to_name(err));
    goto WiFiSTA_init_X;
  } // hctiws err

#ifdef ENABLE_WPS
    wps_config = default_wps_cfg;
    snprintf(wps_config.factory_info.device_name, sizeof(wps_config.factory_info.device_name), wifi_hostname);
#endif

  if (wifi_mode == WiFimode_off) return ESP_OK;

  ESP_LOGD(WIFITAG, "enable wifi...");

  err = esp_wifi_init(&wifi_init_cfg);
  if (err == ESP_OK) {    
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);    
    wifista_create_config(&wifi_config);
    wifista_attach2netif(wifi_hostname);
  } else {	// fi err
    wifi_mode = WiFimode_off;
    ESP_LOGE(WIFITAG, "can't enable WiFi (%s)", esp_err_to_name(err));
  }

  err = nvs_load_string(hnd, WIFIcfg_SSID, (char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid));
  if (err != ESP_OK) {
    ESP_LOGI(WIFITAG, "no SSID, recover old wifi_config - %s", esp_err_to_name(err));
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    wifista_save_config(false);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
  }

  switch (wifi_mode) {
  case WiFimode_OPEN:
    ESP_LOGW(WIFITAG, "!!! open Wifi (unprotected) !!!");
    err = wifista_enable_openwifi(hnd, &wifi_config);
    break;  
  case WiFimode_WPAPSK:		// normal station with pre-shared key
    ESP_LOGD(WIFITAG, "default station mode (WPA-PSK)");
    err = wifista_enable_psk(hnd, &wifi_config);
    break;
  case WiFimode_WPAenterprise:	// WPA enterprise network: load additonal config from NVS and activate it
    ESP_LOGD(WIFITAG, "enable WPA enterprise handshake");
    err = wifista_enable_enterprise(hnd);
    if (err != ESP_OK) {
      ESP_LOGE(WIFITAG, "wpa-enterprise-err %s", esp_err_to_name(err));
    }
    break;
  } // hctiws mode
  
  if (err == ESP_OK) err = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
  if (err != ESP_OK) {
    wifi_mode = WiFimode_off;
    WiFiSTA_unload();
  }

  xEventGroupClearBits(lan_event_group, WIFI_SHUTDOWN_BIT|WPSACTIVE_BIT|WIFI_TEMPON_BIT|SCAN_ACTIVE_BIT|SCAN_FINISHED_BIT);

WiFiSTA_init_X:
  nvs_close(hnd);		// close NVS RO handler
  return err;
}



esp_err_t wifista_connect(void);  // forward

/* default disconnect handler function

*/
void default_action_func(int32_t evnt, const wifi_event_sta_disconnected_t *dis_evnt) {
  if ( (evnt != WIFI_EVENT_STA_DISCONNECTED) || (wifi_reconnect_tries >= WIFISTA_MAX_RECONN_TRIES) ) return;
  ESP_LOGI(WIFITAG, "reconnecting WiFi...");
  wifista_connect();  
}

void shutdown_action_func(int32_t evnt, const wifi_event_sta_disconnected_t *dis_evnt) {
  esp_err_t res;
  if (evnt == WIFI_EVENT_STA_CONNECTED) {
    wifista_action_handler = shutdown_action_func;
    esp_wifi_disconnect();
    return;
  }
  // disconnect handling
  if (wifi_mode == WiFimode_WPAenterprise) esp_wifi_sta_enterprise_disable();
  res = esp_wifi_stop();	
  if (res != ESP_OK) {    
    ESP_LOGE(WIFITAG, "shutdown error %s", esp_err_to_name(res));
  } else {
    ESP_LOGI(WIFITAG, "shutdown complete");
  }
}


esp_err_t wifista_connect(void) {
  esp_err_t res;
  if (wifi_mode == WiFimode_off) {
    ESP_LOGD(WIFITAG, "wifi is off - cancel connect!");
    return ESP_OK;
  }
  if (wifi_mode == WiFimode_WPAenterprise) esp_wifi_sta_enterprise_enable();
  res = esp_wifi_connect();
  if (res != ESP_OK) {
    ESP_LOGE(WIFITAG, "connect() error %s", esp_err_to_name(res));
  } else {
    if (lan_event_group) {
      xEventGroupClearBits(lan_event_group, DISCONNECTED_BIT);
      xEventGroupSetBits(lan_event_group, CONNECTING_BIT);
    }
    wifista_action_handler = default_action_func;
    wifi_reconnect_tries++;
  }
  return res;
}

esp_err_t wifista_disconnect(void) {
  esp_err_t err;
  EventBits_t wifievnt = (lan_event_group != NULL)? xEventGroupGetBits(lan_event_group): 0;

  if (wifievnt & DISCONNECTED_BIT) return ESP_FAIL;

  err = esp_wifi_disconnect();
  ESP_LOGD(WIFITAG, "disconnect (%s)", esp_err_to_name(err));
  if (err != ESP_OK) return err;

  err = LAN_wait4event(DISCONNECTED_BIT, WAIT_DISCONNECTED_MS) == 0? ESP_FAIL: ESP_OK;
  if (lan_event_group) xEventGroupSetBits(lan_event_group, DISCONNECTED_BIT);
  if (err != ESP_OK) {
    ESP_LOGD(WIFITAG, "disconnect timeout");
  }
  return err;
}


esp_err_t WiFISTA_temp_turn_on(void) {
  EventBits_t wifievnt = (lan_event_group != NULL)? xEventGroupGetBits(lan_event_group): 0;  
  if ((wifievnt & WIFI_TEMPON_BIT) || (lan_event_group == NULL)) {
    ESP_LOGD(WIFITAG, "TEMP turn-on Wifi is not necessary.");
    return ESP_OK;
  }
  xEventGroupSetBits(lan_event_group, WIFI_TEMPON_BIT);

  if (wifi_mode == WiFimode_off) {    
    const wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_LOGD(WIFITAG, "START WIFI: enable wifi only 4 checking...");    
    for (int i=10; (i > 0) && (xEventGroupGetBits(lan_event_group) & WIFI_SHUTDOWN_BIT); i--) {
      vTaskDelay(5);
    }
    xEventGroupClearBits(lan_event_group, DISCONNECTED_BIT|WPSACTIVE_BIT||SCAN_FINISHED_BIT|WIFI_SHUTDOWN_BIT);    
    if (wifi_hnd == NULL) {
      ESP_ERROR_CHECK( esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, WiFi_event_handler, NULL, &wifi_hnd) );
    }
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_cfg));
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    wifista_create_config(&wifi_config);
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    //wifista_attach2netif();
    ESP_LOGD(WIFITAG, "WIFI START done.");
    return esp_wifi_start();
  } // fi is off
  return ESP_OK;
}


esp_err_t WiFISTA_temp_turn_off(void) {
  esp_err_t err = ESP_FAIL;
  EventBits_t wifievnt = (lan_event_group != NULL)? xEventGroupGetBits(lan_event_group): 0;
  if ((lan_event_group == NULL) || ((wifievnt & WIFI_TEMPON_BIT)==0) ) {
    ESP_LOGD(WIFITAG, "TEMP turn-off Wifi is not necessary.");
    return ESP_OK;
  }
  xEventGroupClearBits(lan_event_group, WIFI_TEMPON_BIT);
  if (wifi_mode == WiFimode_off) {
    ESP_LOGD(WIFITAG, "TEMP turn-off...");
    if (wifievnt & (WPSACTIVE_BIT | SCAN_ACTIVE_BIT | WIFI_SHUTDOWN_BIT)) {
      ESP_LOGD(WIFITAG, "turn-off WiFi prohibited (EventBits=%03lxh).", wifievnt);
      return ESP_OK;
    }
    ESP_LOGD(WIFITAG, "disable wifi...");
    xEventGroupSetBits(lan_event_group, WIFI_SHUTDOWN_BIT);

    if ((wifievnt & DISCONNECTED_BIT) == 0) {
      wifista_action_handler = shutdown_action_func;
      err = wifista_disconnect();      
    }
    if (err != ESP_OK) {
      err = esp_wifi_stop();
      ESP_LOGD(WIFITAG, "wifi stop %s", esp_err_to_name(err));
    }
    vTaskDelay(5);
    esp_wifi_deinit();
    return err;
  } else {
    ESP_LOGD(WIFITAG, "reconnecting wifi? (EventBits=%03lxh)", wifievnt);
    if ((wifievnt & DISCONNECTED_BIT) && ((wifievnt & (WPSACTIVE_BIT | SCAN_ACTIVE_BIT | WIFI_SHUTDOWN_BIT))==0) ) {
      ESP_LOGD(WIFITAG, "yes, reconnecting...");
      wifista_connect();  
    }
  }
  return ESP_OK;
}


esp_err_t WiFiSTA_turnoff(void) {
  esp_err_t err = nvs_open(WIFI_namespace, NVS_READWRITE, &wifi_config_hnd);
  WiFiSTA_exitfunct_on_err(err);
  err = nvs_set_u8(wifi_config_hnd, "WiFimode", WiFimode_off);
  nvs_close(wifi_config_hnd);
  return err;
}



esp_err_t WiFiSTA_reset(void) {
  //EventBits_t wifievnt;
  if (wifi_mode == WiFimode_off) return ESP_OK;
  if (wifi_mode == WiFimode_WPAenterprise) {
    esp_wifi_sta_enterprise_disable();
    vTaskDelay(2);	
  } // fi WPA-Enterprise  
  //wifievnt = (lan_event_group != NULL)? xEventGroupGetBits(lan_event_group): 0;
  return wifista_disconnect();  
}


uint16_t WiFiSTA_store_scanresults(void) {
  uint16_t no_ap_scanned;

  esp_err_t err = esp_wifi_scan_get_ap_num(&no_ap_scanned);
  if (err != ESP_OK) {
    ESP_LOGE(WIFITAG, "no scan results available. (%d)", err);
    return 0;
  }
  ESP_LOGD(WIFITAG, "NO OF APs found \t\t%d", (int)no_ap_scanned);

  if (no_ap_scanned > SCANAP_MAX_RESULTS) no_ap_scanned = SCANAP_MAX_RESULTS;

  if ((ap_info == NULL) || (ap_info_size < no_ap_scanned)) {
    if (ap_info != NULL) vPortFree(ap_info);    
    ap_info      = pvPortMalloc(no_ap_scanned * sizeof(wifi_ap_record_t));
    ap_info_size = no_ap_scanned;
  }
  if ((ap_info == NULL) && (ap_info_size > 0)) {
    ap_info_size = 0;
    ESP_LOGE(WIFITAG, "no memory free for %d scan results.", no_ap_scanned);
  }
  if (ap_info_size == 0) return 0;
  err = esp_wifi_scan_get_ap_records(&no_ap_scanned, ap_info);
  if (err != ESP_OK) {
    ESP_LOGE(WIFITAG, "get scan results fails. (%d)", err);
    vPortFree(ap_info);
    return 0;
  }
  ap_info_cnt = no_ap_scanned;
  ESP_LOGD(WIFITAG, "NO OF APs loaded \t\t%d", no_ap_scanned);
  return no_ap_scanned;
}



#ifdef ENABLE_WPS
static void wifista_WPS_action(int wps_event);
#endif

/* *** WiFi_event_handler() ***
   main EVENT HANDLER of wifi station mode
*/
static void WiFi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
  EventBits_t curr_wifi_bits;
  const wifi_event_sta_disconnected_t *dis_evnt;

  if (event_base != WIFI_EVENT) {
    ESP_LOGE(WIFITAG, "unknown event base %.20s", event_base);
    return;
  }
  curr_wifi_bits = xEventGroupGetBits(lan_event_group);

  switch (event_id) {
  case WIFI_EVENT_STA_START:
    ESP_LOGD(WIFITAG, "WIFI_EVENT_STA_START");
    if (lan_event_group) {
      xEventGroupSetBits(lan_event_group, DISCONNECTED_BIT); // , &context_switch_needed);
      if ((curr_wifi_bits & WIFI_TEMPON_BIT) == 0) {
        ESP_LOGD(WIFITAG, "first connect...");
        wifi_reconnect_tries = 0;
        wifista_connect();
      }
    }    
    break;
  case WIFI_EVENT_STA_STOP:
    ESP_LOGD(WIFITAG, "WIFI_EVENT_STA_STOP");
    if (curr_wifi_bits & WIFI_SHUTDOWN_BIT) {
      ESP_LOGD(WIFITAG, "WiFi shutdown complete.");
    }
    if (lan_event_group) {
      xEventGroupClearBits(lan_event_group, WIFI_SHUTDOWN_BIT|WPSACTIVE_BIT|SCAN_ACTIVE_BIT|SCAN_FINISHED_BIT|WIFI_TEMPON_BIT|CONNECTING_BIT|CONNECTED_BIT|DISCONNECTED_BIT);
    }
    break;

  case WIFI_EVENT_STA_DISCONNECTED:
    ESP_LOGD(WIFITAG, "WIFI_EVENT_STA_DISCONNECTED");    
    if (lan_event_group) {
      xEventGroupClearBits(lan_event_group, CONNECTED_BIT|CONNECTING_BIT);
      xEventGroupSetBits(lan_event_group, DISCONNECTED_BIT);
    } // fi chane state
    dis_evnt = (const wifi_event_sta_disconnected_t *) event_data;
#if CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_DEBUG
    printf("DISCON reason:%d, SSID:%.32s(%d)\n", dis_evnt->reason, dis_evnt->ssid, dis_evnt->ssid_len); // bssid
#endif
    if (wifista_action_handler != NULL) {
      wifista_action_handler_t hnd =  wifista_action_handler;
      wifista_action_handler = NULL;    // called only once, handler resets before action
      hnd(event_id, dis_evnt);
    } // fi
    LAN_status_handler(LANethLinkDn);
//    ESP_LOGD(WIFITAG, "free heap size = %d", esp_get_free_internal_heap_size());
    break;

  case WIFI_EVENT_STA_CONNECTED:
    ESP_LOGD(WIFITAG, "WIFI_EVENT_STA_CONNECTED");
    if (lan_event_group) {
      xEventGroupClearBits(lan_event_group, DISCONNECTED_BIT|CONNECTING_BIT);
    }
    LAN_status_handler(LANethLinkUp);

    if (wifista_action_handler != NULL) {
      wifista_action_handler_t hnd =  wifista_action_handler;
      wifista_action_handler = NULL;    // called only once, handler resets before action
      hnd(event_id, NULL);
    } // fi
    
    break;

  case WIFI_EVENT_SCAN_DONE:  // AP scan finished...
    xEventGroupClearBits(lan_event_group, SCAN_ACTIVE_BIT);
    xEventGroupSetBits(lan_event_group, SCAN_FINISHED_BIT); //, &context_switch_needed);
    ESP_LOGD(WIFITAG, "WIFI_EVENT_SCAN_DONE");
    WiFISTA_scan_callback(WiFiSTA_store_scanresults());
    WiFISTA_temp_turn_off();
    break;

#ifdef ENABLE_WPS
  case WIFI_EVENT_STA_WPS_ER_SUCCESS:
    // point: the function esp_wifi_wps_start() only get ssid & password
    // so call the function wifista_connect(() here
    ESP_LOGD(WIFITAG, "WIFI_EVENT_STA_WPS_ER_SUCCESS");
    ESP_ERROR_CHECK(esp_wifi_wps_disable());
    if (lan_event_group) xEventGroupClearBits(lan_event_group, WPSACTIVE_BIT);
    if (wifista_save_config(true) == ESP_OK) {
      if (lan_event_group) {
        xEventGroupClearBits(lan_event_group, WIFI_TEMPON_BIT);        
      }
      ESP_LOGD(WIFITAG, "new config saved, connect new wifi network...");
      wifista_connect();
    // } else {
    //  WiFISTA_temp_turn_off();
    }
    WiFiSTA_WPS_callback(0, NULL);
   break;
  case WIFI_EVENT_STA_WPS_ER_FAILED:
    ESP_LOGD(WIFITAG, "WIFI_EVENT_STA_WPS_ER_FAILED");
    wifista_WPS_action(1);
    break;
  case WIFI_EVENT_STA_WPS_ER_TIMEOUT:
    ESP_LOGD(WIFITAG, "WIFI_EVENT_STA_WPS_ER_TIMEOUT");
    wifista_WPS_action(2);
    break;
  case WIFI_EVENT_STA_WPS_ER_PIN:
    ESP_LOGD(WIFITAG, "WIFI_EVENT_STA_WPS_ER_PIN");
    wifi_event_sta_wps_er_pin_t* event = (wifi_event_sta_wps_er_pin_t*) event_data;
    WiFiSTA_WPS_callback(3, (const char *)event->pin_code);
    break;
#endif // WPS
  default:
    ESP_LOGD(WIFITAG, "undhandled WIFI_EVENT_xxx %ld", event_id);
    break;
  }
}

// *** END HANDLER ***


esp_err_t WiFiSTA_start(void) {
  esp_err_t start_result;
  if (wifi_mode == WiFimode_off) {
    ESP_LOGD(WIFITAG, "WiFi stays turned off");
    return ESP_OK;
  }
  if (sta_netif == NULL) {
    ESP_LOGE(WIFITAG, "no NETIF for WiFi Station Mode!");
    return ESP_FAIL;
  }
  if (wifi_hnd == NULL) {
    ESP_ERROR_CHECK( esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, WiFi_event_handler, NULL, &wifi_hnd) );
  }
#if CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_DEBUG

  start_result = esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_config);
  if (start_result == ESP_OK) {
    printf("SSID: |%.32s|\n", wifi_config.sta.ssid);
    printf("PSK : |%.64s|\n", wifi_config.sta.password);
    printf("scanm=%c,ch=%d,sortm=%d,tresh(rssi=%d,authm=%d),pmf(capable=%d,required=%d)\n", wifi_config.sta.scan_method?'A':'F', wifi_config.sta.channel, 
      wifi_config.sta.sort_method, wifi_config.sta.threshold.rssi, wifi_config.sta.threshold.authmode, wifi_config.sta.pmf_cfg.capable, wifi_config.sta.pmf_cfg.required);
    ESP_LOG_BUFFER_HEX_LEVEL(WIFITAG, &wifi_config, sizeof(wifi_config), ESP_LOG_DEBUG);

  } else {
    ESP_LOGE(WIFITAG, "can't get WiFi config, %s", esp_err_to_name(start_result));
  }
#endif

  ESP_LOGD(WIFITAG, "start WiFi...");  
  start_result = esp_wifi_start();  
  ESP_LOGD(WIFITAG, "start Wifi returns %s", esp_err_to_name(start_result));
  return start_result;
}




esp_err_t WiFiSTA_shutdown(void)  {
  esp_err_t result = ESP_OK;
  if (wifi_mode != WiFimode_off) {
    ESP_LOGD(WIFITAG, "stop WiFi...");
    xEventGroupSetBits(lan_event_group, WIFI_SHUTDOWN_BIT);
    wifista_action_handler = shutdown_action_func;
    result = WiFiSTA_reset();
    if (result != ESP_OK) {
      ESP_LOGE(WIFITAG, "reset failed, %d", result);
    }
    LAN_wait4event(DISCONNECTED_BIT, 50);
    result = esp_wifi_stop();
    if (result != ESP_OK) {
      ESP_LOGE(WIFITAG, "deinit() failed, %d", result);
    } else {
      ESP_LOGD(WIFITAG, "stopped");
    }
    vTaskDelay(5);    
    WiFiSTA_unload();        
  } else {
    ESP_LOGD(WIFITAG, "Shutdown: WiFi wasn't active.");
  }
  return result;
}


esp_err_t WiFiSTA_reloadDriver(void) {
  esp_err_t   err;
  EventBits_t wifievnt;
  if (wifi_mode == WiFimode_off) return ESP_OK;
  wifievnt = (lan_event_group != NULL)? xEventGroupGetBits(lan_event_group): 0;
  ESP_LOGD(WIFITAG, "force reload (after timeout)...");
  if (wifievnt & (WPSACTIVE_BIT | SCAN_ACTIVE_BIT | WIFI_SHUTDOWN_BIT)) {
    ESP_LOGD(WIFITAG, "reload WiFi driver prohibited (EventBits=%03lxh).", wifievnt);
    return ESP_OK;
  }
  ESP_LOGD(WIFITAG, "RELOAD stopping WiFi...");
  xEventGroupSetBits(lan_event_group, WIFI_SHUTDOWN_BIT);
  wifista_action_handler = shutdown_action_func;
  err = WiFiSTA_reset();
  if (err != ESP_OK) {
    ESP_LOGE(WIFITAG, "RELOAD disconnect failed, %s", esp_err_to_name(err));
  }
  vTaskDelay(2);			// don't wait for disconnected bit (assume we are already disconnected)
  err = esp_wifi_stop();
  if (err != ESP_OK) {
    ESP_LOGE(WIFITAG, "RELOAD stop() failed, %s", esp_err_to_name(err));
  }
  vTaskDelay(5);
  WiFiSTA_unload();
  vTaskDelay(20);
  ESP_LOGD(WIFITAG, "RELOAD init + starting WiFi...");
  err = WiFiSTA_init(NULL);
  vTaskDelay(5);  
  return WiFiSTA_start();
}




#ifdef ENABLE_WPS


__weak void WiFiSTA_WPS_callback(int event, const char pin[8]) {
  ESP_LOGD(WIFITAG, "WPA default callback %d", event);
}


esp_err_t WiFiSTA_reenableWPS(void) {
  esp_err_t err;
  if (lan_event_group == NULL) return ESP_FAIL;  
  if ((wps_config.wps_type == WPS_TYPE_DISABLE) || (wps_config.wps_type >= WPS_TYPE_MAX)) {
    ESP_LOGI(WIFITAG, "WPS no restart (disabled).");
    return ESP_OK;
  }
  WiFiSTA_cancel_scan();
  ESP_LOGD(WIFITAG, "WPS start...");
  xEventGroupSetBits(lan_event_group, WPSACTIVE_BIT);
  ESP_ERROR_CHECK(esp_wifi_wps_enable(&wps_config));
  err = esp_wifi_wps_start(0);
  if (err != ESP_OK) {
    xEventGroupClearBits(lan_event_group, WPSACTIVE_BIT);
    if (wifi_mode != WiFimode_off) {
      wifista_action_handler = default_action_func;
    }
    ESP_LOGE(WIFITAG, "WPS error start (%s)", esp_err_to_name(err));
  }
  return err;
}


static void wifista_WPS_action(int wps_event) {
  ESP_LOGD(WIFITAG, "WPS cancel/stop (event=%d)...", wps_event);  
  if (lan_event_group) xEventGroupClearBits(lan_event_group, WPSACTIVE_BIT);
  wifista_action_handler = NULL;
  WiFiSTA_WPS_callback(wps_event, NULL);
}


esp_err_t WiFiSTA_disableWPS(void) {
  esp_err_t res = ESP_OK;;
  if (lan_event_group && (xEventGroupGetBits(lan_event_group) & WPSACTIVE_BIT)) {
    res = esp_wifi_wps_disable();
    if (res != ESP_OK) {
      ESP_LOGE(WIFITAG, "WPS disable error %s", esp_err_to_name(res));  
    }
    wifista_WPS_action(27);
    WiFISTA_temp_turn_off();
    if (wifi_mode != WiFimode_off) {
      esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config); // restore config
      wifista_connect();
    }
  } else {
    ESP_LOGW(WIFITAG, "WPS disable(): is not active!");
  }
  return res;
}



void wps_action_func(int32_t evnt, const wifi_event_sta_disconnected_t *dis_evnt) {
  if ((dis_evnt != NULL) && (dis_evnt->reason == WIFI_REASON_AUTH_FAIL)) {
    ESP_LOGE(WIFITAG, "WPS auth_fail event while WPS active");
    WiFiSTA_disableWPS();
    return;
  }
  WiFiSTA_reenableWPS();
}



esp_err_t WiFiSTA_enableWPS(tWPStype t) {
  wps_config.wps_type = t;
  if (lan_event_group == NULL) return -2;

  if (wps_config.wps_type != WPS_TYPE_DISABLE) {
    EventBits_t wifi_bits = xEventGroupGetBits(lan_event_group);
    if (wifi_bits & WPSACTIVE_BIT) return ESP_OK;    
    if (wifi_bits & WIFI_SHUTDOWN_BIT) {
      ESP_LOGE(WIFITAG, "WPS wifi in shutdown");
      return ESP_FAIL;
    }
    ESP_LOGD(WIFITAG, "WPS enable...");
    if (wifi_bits & SCAN_ACTIVE_BIT) {
      ESP_LOGD(WIFITAG, "WPS stop AP scanning...");
      WiFiSTA_cancel_scan();
      vTaskDelay(2);
    }    
    // 1. turn wifi on, if off
    WiFISTA_temp_turn_on();
    // 2. set action
    xEventGroupSetBits(lan_event_group, WPSACTIVE_BIT);
    // 3. wait a bit for turning on
    if (wifi_mode == WiFimode_off) {
      LAN_wait4event(DISCONNECTED_BIT, 15);
    }
    // 4. start WPS (only if state machine is not in connection / connected)
    wifista_action_handler = wps_action_func;    
    if (wifista_disconnect() != ESP_OK) {
      return WiFiSTA_reenableWPS();
    }
    return ESP_OK;
  } // fi not disable

  return WiFiSTA_disableWPS();
}


#endif


__weak void WiFISTA_scan_callback(int no_of_aps) {
  ESP_LOGD(WIFITAG, "SCAN default callback, %d AP's found", no_of_aps);
}


void scanmode_action_func(int32_t evnt, const wifi_event_sta_disconnected_t *dis_evnt) {
  wifi_scan_config_t scan_conf;
  
  memset(&scan_conf, 0, sizeof(scan_conf));
  scan_conf.scan_type = WIFI_SCAN_TYPE_PASSIVE;
  scan_conf.scan_time.passive = 400;
  //  scan_conf.scan_type = xEventGroupGetBits(lan_event_group)&CONNECTED_BIT? WIFI_SCAN_TYPE_PASSIVE: WIFI_SCAN_TYPE_ACTIVE;

  esp_err_t result = esp_wifi_scan_start(&scan_conf, false);
  ESP_LOGD(WIFITAG, "AP scanning %s", (result == ESP_OK)?"in progress": "fails" );

  if (result == ESP_OK) return;

  // fail to activate, try at next event...
  wifista_action_handler = scanmode_action_func;
  ESP_LOGE(WIFITAG, "AP scan fails (%s)", esp_err_to_name(result));
}


esp_err_t WiFiSTA_cancel_scan(void) {
  esp_err_t result = ESP_OK;
  if (xEventGroupGetBits(lan_event_group) & SCAN_ACTIVE_BIT) {
    ESP_LOGD(WIFITAG, "cancel AP scanning...");
    result = esp_wifi_scan_stop();
    xEventGroupClearBits(lan_event_group, SCAN_ACTIVE_BIT);
    wifista_action_handler = NULL;
    WiFISTA_scan_callback(-1);
    if (result != ESP_OK) {
      ESP_LOGE(WIFITAG, "Error stopping previous scan");
    }    
  }
  return result;
}


esp_err_t WiFiSTA_scan(void) {  
  //esp_err_t result;
  if (lan_event_group == NULL) return ESP_FAIL;
  
#ifdef ENABLE_WPS
  if (xEventGroupGetBits(lan_event_group) & WPSACTIVE_BIT) {
    ESP_LOGD(WIFITAG, "AP scanning: Disable WPS now...");
    esp_wifi_wps_disable();   
    wifista_WPS_action(27);
    vTaskDelay(2);
  }
#endif  
  // 1. turn wifi on, if off
  WiFISTA_temp_turn_on();
  // 2. wait a bit for turning on
  if (wifi_mode == WiFimode_off) {
    LAN_wait4event(DISCONNECTED_BIT, 15);
  }
  // 3. cancel action
  xEventGroupClearBits(lan_event_group, SCAN_FINISHED_BIT);
  WiFiSTA_cancel_scan();
  // 4. set action
  xEventGroupSetBits(lan_event_group, SCAN_ACTIVE_BIT);
  // 5. if connected or in-between: cancel
  wifista_action_handler = scanmode_action_func;
  if (wifista_disconnect() != ESP_OK) {
    ESP_LOGD(WIFITAG, "SCAN disconnect fails");
    scanmode_action_func(0, NULL);
  }
  return ESP_OK;
}

// *** END SCAN FUNCTIONS ***


signed char WiFiSTA_getSignalStrength(void) {
  signed char rssi = -128;
  wifi_ap_record_t ap_info;
  if ((wifi_mode == WiFimode_off) || (wifi_hnd == NULL) || (lan_event_group == NULL) || !(xEventGroupGetBits(lan_event_group) & CONNECTED_BIT) )
    return rssi;
  esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
  if (err == ESP_OK) {
    rssi = ap_info.rssi;
    if (rssi == -128) rssi++;
  }
  return rssi;
}



unsigned char WiFiSTA_calcSignalQuality(signed char rssi) {
  float q;
  if (rssi == -128) return 0;
  q = (rssi - LINKQUALITY_0_DBM) / (LINKQUALITY_100_DBM - LINKQUALITY_0_DBM);
  if (q >= 1.0) return 100; else if (q <= 0.01) return 1;
  return q * 100;
}



static void ScanList_Insert(twifi_item *list, uint8_t *list_cnt, const uint8_t *ssid, wifi_auth_mode_t authmode) {
  unsigned char pos = list_cnt[0];
  for (int i = 0; i < pos; i++) {
    if (strncmp((const char *)ssid, list[i].ssid, 33) == 0) return;	// skip double
  } // rof
  strncpy(list[pos].ssid, (const char *)ssid, 33);
  list[pos].auth_mode = (char) authmode;
  (list_cnt[0])++;
}


void wifista_clear_scanresults(void) {
  if (ap_info != NULL) {
    vPortFree(ap_info);
    ap_info = NULL;
  }
  ap_info_cnt  = 0;
  ap_info_size = 0;
}


twifi_item *WiFiSTA_get_scanresults(int *result_size) {
  uint8_t  ScanResults;
  twifi_item *result = NULL;
  if ((ap_info == NULL) || (ap_info_cnt == 0)) return NULL;

  result = pvPortMalloc(ap_info_cnt * sizeof(twifi_item));
  if (result == NULL) {
    ESP_LOGE(WIFITAG, "no memory free for %d wifi results.", ap_info_cnt);
    wifista_clear_scanresults();
    return NULL;
  }

  result_size[0] = ap_info_cnt * sizeof(twifi_item);
  ScanResults = 0;
  ESP_LOGD(WIFITAG, "requesting scan results: %d items (%d bytes)", ap_info_cnt, result_size[0]);
  for (uint16_t i = 0; (i <  ap_info_cnt) && (ScanResults < ap_info_cnt); i++) {
      ScanList_Insert(result, &ScanResults, ap_info[i].ssid, ap_info[i].authmode);
      ESP_LOGD(WIFITAG, "SSID\t%s", ap_info[i].ssid);
  } // rof
  wifista_clear_scanresults();
  result_size[0] = ScanResults * sizeof(twifi_item);
  ESP_LOGD(WIFITAG, "scan results total: %d items (%d bytes)", ScanResults, result_size[0]);
  return result;
}


void WiFiSTA_free_scanresults(twifi_item *scan_result_list) {
  if (scan_result_list != NULL) vPortFree(scan_result_list);
  //wifista_clear_scanresults();
}


int WiFiSTA_get_ip(esp_netif_ip_info_t *ip_info) {
  if ((ip_info == NULL) || (sta_netif == NULL)) return -2;
  return esp_netif_get_ip_info(sta_netif, ip_info) == ESP_OK? 0: -1;
}


static int wifista_param_set = 0;

void wifista_get_parameter(int ininr, const char *value, void *param) {
  esp_err_t err;
  int idx;
  //wifi_config_t *cfg = (wifi_config_t *)param;
  
  switch (ininr) {
  case -2:			// enter section
    err = nvs_open(WIFI_namespace, NVS_READWRITE, &wifi_config_hnd);
    WiFiSTA_breakswitch_on_err(err);
    if (wifi_mode != WiFimode_off) {
      WiFiSTA_shutdown();
    } else if (sta_netif != NULL) {
      esp_netif_destroy(sta_netif);
      sta_netif = NULL;
    }
    wifi_mode = WiFimode_undef; 	// we can now determine the correct mode later
    wifista_param_set = 0;
    break;
  case -1:			// leave section
    if (wifi_mode == WiFimode_undef) {	// determine mode
      if ((wifista_param_set & 0x61) == 0x61) {     // WPA-Enterprise
	      wifi_mode = WiFimode_WPAenterprise;
      } else if ((wifista_param_set & 0x03) == 0x03) {	// WPA-PSK
      	wifi_mode = WiFimode_WPAPSK;
      } else {
        wifi_mode = WiFimode_off;		// sorry - no valid config!
      }
      nvs_set_u8(wifi_config_hnd, WIFIcfg_Mde, wifi_mode);	// STORE setting
      ESP_LOGI(WIFITAG, "Auto-Define mode = %d (params = %02x)", wifi_mode, wifista_param_set);
    } // fi get it auto

    nvs_close(wifi_config_hnd);
    if (wifi_mode != WiFimode_off) {
      // ToDo Kein Connect Event hier, nach Änderung von WiFi OFF -> ON
      err = WiFiSTA_init(NULL);
      if (err == ESP_OK) WiFiSTA_start();	// re
    }
    break;
  case CCONF_ID_WIFIMODE:			// "wifimode"
    idx = str_index(value, wifi_mode_list);
    if (idx < 0) idx = str_readnum(value, 0, 0, WiFimode_maxValue);
    wifi_mode = idx;
    nvs_set_u8(wifi_config_hnd, WIFIcfg_Mde, wifi_mode);
    break;
  case CCONF_ID_SSID:			// "ssid"
    if (strnlen(value, 3) > 1) {
      nvs_set_str(wifi_config_hnd, WIFIcfg_SSID, value);    
      wifista_param_set |= 0x01;
    } else {
      ESP_LOGE(WIFITAG, "SSID is to short. (%s)", value);
    }
    break;
  case CCONF_ID_PRESHAREDKEY: // "presharedkey"
    if (strnlen(value, 9) > 7) {
      nvs_set_str(wifi_config_hnd, WIFIcfg_PSK, value);
      wifista_param_set |= 0x02;
    } else {
      ESP_LOGE(WIFITAG, "PSK is to short. (%s)", value);
    }
    break;
  case CCONF_ID_IDENTITY:			// "identity"
    nvs_set_str(wifi_config_hnd, WIFIcfg_ID, value);
    wifista_param_set |= 0x10;
    break;
  case CCONF_ID_USERNAME:			// "username"
    nvs_set_str(wifi_config_hnd, WIFIcfg_USR, value);
    wifista_param_set |= 0x20;
    break;
  case CCONF_ID_PASSWORD:			// "password"
    nvs_set_str(wifi_config_hnd, WIFIcfg_PW, value);
    wifista_param_set |= 0x40;
    break;
  case CCONF_ID_HOSTNAME:			// "hostname"
    break;
  } // hctiws ininr
}


size_t wifista_get_param_size(void) {
  return sizeof(wifi_config_t);
}


void wifista_create_nowifi_comment(char **textbuf, int *buf_left) {
  INI_create_comment(textbuf, buf_left, "use parameter 'presharedkey = ' to set a WPA-PSK connection. - OR -");
  INI_create_comment(textbuf, buf_left, "use parameter 'username = ' together with 'password = 'to set a WPA-Enterprise connection.");
  INI_add_CRLF(textbuf, buf_left);
}


int WiFiSTA_create_configuration(char *textbuf, unsigned int buf_size) {
  nvs_handle hnd;
  esp_err_t	 err;
  uint8_t wifimode_nvm = WiFimode_undef;
  char item[MAX_WIFI_CFG_ITEM_SIZE];
  int chunksize;
  int buf_left = buf_size;

  if (buf_size < 192) return -1;

  chunksize = snprintf(textbuf, buf_left, WIFI_CFG_SECTION_NAME "\r\n");

  textbuf  += chunksize;
  buf_left -= chunksize;
  
  INI_create_comment(&textbuf, &buf_left, "optional 'wifimode' parameter: Set to off, wpa-psk or wpa-enterprise.");

  err = nvs_open(WIFI_namespace, NVS_READONLY, &hnd);
  if (err != ESP_OK) {
    ESP_LOGD(WIFITAG, "no WiFiSTA NVS configuration exists");
    INI_create_commented_item(CCONF_ID_SSID, &textbuf, &buf_left, "<no config>");
    wifista_create_nowifi_comment(&textbuf, &buf_left);
    return (buf_left == 0)? -2: buf_size - buf_left;
  }


  err = nvs_get_u8(hnd, WIFIcfg_Mde, &wifimode_nvm);
  if (err != ESP_OK) {
    ESP_LOGD(WIFITAG, "no wifi_mode parameter exists in NVM");
    INI_create_commented_item(CCONF_ID_WIFIMODE, &textbuf, &buf_left, "<not set>");
    wifista_create_nowifi_comment(&textbuf, &buf_left);
    return (buf_left == 0)? -2: buf_size - buf_left;
  }

  INI_create_item(CCONF_ID_WIFIMODE, &textbuf, &buf_left, (wifimode_nvm <= WiFimode_maxValue)? wifi_mode_list[wifimode_nvm]: "off");

  err = nvs_load_string(hnd, WIFIcfg_SSID, item, sizeof(item));
  if (err == ESP_OK) {
    INI_create_item(CCONF_ID_SSID, &textbuf, &buf_left, item);
  } else {
    INI_create_commented_item(CCONF_ID_SSID, &textbuf, &buf_left, "<is not set>");
  }

#ifdef CONFIG_PLAIN_PASSWORDS
  INI_create_comment(&textbuf, &buf_left, "Scan=%c,BSSIDset=%c,ch=%d,listen_interval=%d,PMF=(capable:%c;required:%c)", wifi_config.sta.scan_method?'A':'F',wifi_config.sta.bssid_set?'y':'n', 
    wifi_config.sta.channel, wifi_config.sta.listen_interval, wifi_config.sta.pmf_cfg.capable?'y':'n', wifi_config.sta.pmf_cfg.required?'y':'n');
#endif


  switch (wifimode_nvm) {
  case WiFimode_off:
    wifista_create_nowifi_comment(&textbuf, &buf_left);
    break;
  case WiFimode_WPAPSK:
    err = nvs_load_string(hnd, WIFIcfg_PSK, item, sizeof(item));
    if (err == ESP_OK) {
#ifdef CONFIG_PLAIN_PASSWORDS
    INI_create_item(CCONF_ID_PRESHAREDKEY, &textbuf, &buf_left, item);
#else
    INI_create_commented_item(CCONF_ID_PRESHAREDKEY, &textbuf, &buf_left, \
      (strnlen((const char *)wifi_config.sta.password, 8) > 5)? "<is set>": "<is not set>");
#endif
    } else {
      INI_create_commented_item(CCONF_ID_PRESHAREDKEY, &textbuf, &buf_left, "<no config>");
    }
    break;
  case WiFimode_WPAenterprise:
    err = nvs_load_string(hnd, WIFIcfg_ID, item, sizeof(item));
    if (err == ESP_OK) INI_create_item(CCONF_ID_IDENTITY, &textbuf, &buf_left, item);
    err = nvs_load_string(hnd, WIFIcfg_USR, item, sizeof(item));
    if (err == ESP_OK) {
      INI_create_item(CCONF_ID_USERNAME, &textbuf, &buf_left, item);
    } else {
      INI_create_commented_item(CCONF_ID_USERNAME, &textbuf, &buf_left, "<is not set>");
    }
    err = nvs_load_string(hnd, WIFIcfg_PW, item, sizeof(item));
    if (err == ESP_OK) {
#ifdef CONFIG_PLAIN_PASSWORDS
      INI_create_item(CCONF_ID_PASSWORD, &textbuf, &buf_left, item);
#else
      INI_create_commented_item(CCONF_ID_PASSWORD, &textbuf, &buf_left, strnlen(item, 3) > 2? "<is set>": "<is too short / not set>");
#endif
    } else {
      INI_create_commented_item(CCONF_ID_USERNAME, &textbuf, &buf_left, "<is not set>");
    }        
    break;
  } // hctiws WiFimode

  nvs_close(hnd);
  INI_add_CRLF(&textbuf, &buf_left);
  return (buf_left == 0)? -2: buf_size - buf_left;
}





static struct {
    struct arg_str *sel;
    struct arg_str *val;
    struct arg_end *end;
} wifista_args;


static int wifista_console(int argc, char **argv) {
  const char *wifista_cmd_list[] = {
    "mode", "ssid", "presharedkey", "identity", "username", "password", "info", "set", NULL
  };
  esp_err_t err;
  char *config_text;
  const char *value = NULL;
  int nerrors = arg_parse(argc, argv, (void **)&wifista_args);
  if (nerrors != 0) {
    arg_print_errors(stderr, wifista_args.end, argv[0]);
    return 1;
  }

  assert(wifista_args.sel->count == 1);
  int selection = str_index(wifista_args.sel->sval[0], wifista_cmd_list);
  if (selection < 0) {
    printf("unknow command '%s'\n", wifista_args.sel->sval[0]);
    return 1;
  }
  if ((wifista_args.val != NULL) && (wifista_args.val->count == 1)) {
    value = wifista_args.val->sval[0];
  }
  ESP_LOGD(WIFITAG, "wifista console: %d, %d: %s", argc, selection, value != NULL? value: "---");
  
  switch (selection) {
  case 6:  // info
    config_text = malloc(512);
    if (config_text == NULL) {
      ESP_LOGE(WIFITAG, "no memory for config text");
      return 1;
    }
    WiFiSTA_create_configuration(config_text, 512);
    printf("%s", config_text);
    free(config_text);
    break;
  case 7:  // set
    if (wifi_mode != WiFimode_off) {
      WiFiSTA_shutdown();
    } else if (sta_netif != NULL) {
      esp_netif_destroy(sta_netif);
      sta_netif = NULL;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    if (wifi_mode != WiFimode_off) {
      // force changes
      if ((wifista_param_set & 0x61) == 0x61) {     // WPA-Enterprise
	      wifi_mode = WiFimode_WPAenterprise;
      } else if ((wifista_param_set & 0x03) == 0x03) {	// WPA-PSK
      	wifi_mode = WiFimode_WPAPSK;
      }
      err = WiFiSTA_init(NULL);
      if (err == ESP_OK) WiFiSTA_start();	// re
    }
    wifista_param_set = 0;
    break;

  default: // 0..5 -> use
    err = nvs_open(WIFI_namespace, NVS_READWRITE, &wifi_config_hnd);
    if (err != ESP_OK) {
      printf("can't open WIFISTA NVS settigns.\n");
      return 1;
    }
    wifista_get_parameter(selection, value, NULL);
    nvs_close(wifi_config_hnd);
    break;
  }
  return 0;
}



void register_wifista(void) {
  wifista_args.sel = arg_str1(NULL, NULL, "<info|mode|ssid|presharedkey|identity|username|password|set>", "change Wifi settings");
  wifista_args.val = arg_str0(NULL, NULL, "<value>", "optional parameter value");
  wifista_args.end = arg_end(2);
  const esp_console_cmd_t cmd = {
    .command = "wifi",
    .help = "Set Wifi parameters for station-mode",
    .hint = NULL,
    .func = &wifista_console,
    .argtable = &wifista_args};
  ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
