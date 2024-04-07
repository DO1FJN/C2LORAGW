/*
 * LAN.c
 *
 *  Created on: 15.07.2021
 *      Author: Jan Alte
 *          (c) 2021 bentrup Industriesteuerungen
 *
 * Report:
 * 2021-07-15	First version
 *
 */

#include "LAN.h"
#include "compiler.h"

#include "Ethernet.h"
#include "WiFiSTA.h"
#include "utilities.h"

#include <esp_wifi.h>
#include <esp_mac.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include <esp_netif.h>

#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <string.h>
#include "sdkconfig.h"


static const char *LANTAG	  = "LAN";

static const char *LAN_namespace = "LAN";
static const char *LANcfg_hostn  = "Hostname";

static char LAN_hostname[64];


// FreeRTOS event group to signal when we are connected & ready to make a request
EventGroupHandle_t lan_event_group = NULL;


__weak void LAN_status_handler(tLANstatus s) { }


static void IP_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
  ip_event_got_ip_t* event;

  if (event_base == IP_EVENT) switch (event_id) {
    case IP_EVENT_STA_GOT_IP:
      ESP_LOGD(LANTAG, "IP_EVENT_STA_GOT_IP");
      event = (ip_event_got_ip_t*)event_data;
      if (lan_event_group) {
        EventBits_t new_lan_events = CONNECTED_BIT;

        //if (event->event_info.got_ip.ip_changed) new_lan_events |= ADDR_CHANGE_BIT; +++ ToDo

        xEventGroupClearBits(lan_event_group, DISCONNECTED_BIT | ADDR_CHANGE_BIT);
        xEventGroupSetBits(lan_event_group, new_lan_events);
      } // fi
      LAN_status_handler(LANwifiLinkOk);
      ESP_LOGI(LANTAG, "got ip 4 wifi: " IPSTR, IP2STR(&event->ip_info.ip));
      break;
    case IP_EVENT_STA_LOST_IP:
      ESP_LOGD(LANTAG, "IP_EVENT_STA_LOST_IP");
#ifdef CONFIG_INTERNAL_WIFI
      esp_wifi_disconnect();
#endif
      LAN_status_handler(LANlostIP);
      break;
    // IP_EVENT_AP_STAIPASSIGNED,         //  soft-AP assign an IP to a connected station
    // IP_EVENT_GOT_IP6,                  //  station or ap or ethernet interface v6IP addr is preferred

    case IP_EVENT_ETH_GOT_IP:
      ESP_LOGD(LANTAG, "IP_EVENT_ETH_GOT_IP");
      if (lan_event_group) xEventGroupSetBits(lan_event_group, CONNECTED_BIT);
      event = (ip_event_got_ip_t*)event_data;
      LAN_status_handler(LANethLinkOk);
      ESP_LOGI(LANTAG, "got ip 4 ethernet: " IPSTR, IP2STR(&event->ip_info.ip));
      break;

    //   IP_EVENT_PPP_GOT_IP,             // PPP interface got IP
    //   IP_EVENT_PPP_LOST_IP,            // PPP interface lost IP
    default:
      ESP_LOGD(LANTAG, "undhandled IP_EVENT_xxx %ld", event_id);
      break;
  } // hctiws IP event
}



esp_err_t LAN_init(void) {
  esp_err_t err = ESP_OK;
  nvs_handle lanconf;
  strncpy(LAN_hostname, CONFIG_DEVICE_NAME, sizeof(LAN_hostname));  
  lan_event_group = xEventGroupCreate();
  if (lan_event_group == NULL) return ESP_FAIL;
  // Initialize TCP/IP network interface (should be called only once in application)
  err = esp_netif_init();
  if (err != ESP_OK) return err;

  if (nvs_open(LAN_namespace, NVS_READONLY, &lanconf) == ESP_OK) {
    nvs_load_string(lanconf, LANcfg_hostn, LAN_hostname, sizeof(LAN_hostname));
    nvs_close(lanconf);
  }
  ESP_LOGD(LANTAG, "LAN_init host '%s'", LAN_hostname);

#ifdef CONFIG_INTERNAL_ETHERNET
  err = ETH_init();
#endif
#ifdef CONFIG_INTERNAL_WIFI
  // *** Enable WiFi Module ***
  err |= WiFiSTA_init(LAN_hostname);
#endif
  return err;
}



esp_err_t LAN_shutdown(void) {
  esp_err_t err = ESP_OK;
#ifdef CONFIG_INTERNAL_ETHERNET
  err = ETH_shutdown();
#endif
#ifdef CONFIG_INTERNAL_WIFI
  err |= WiFiSTA_shutdown();
#endif
  err |= esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID, IP_event_handler);
  // err |= esp_netif_deinit(); no deinit!
  return err;
}


esp_err_t LAN_start(void) {
  esp_err_t err = ESP_OK;
  ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, IP_event_handler, NULL) );

#ifdef CONFIG_INTERNAL_WIFI
  err = WiFiSTA_start();
  if (err != ESP_OK) return err;
  register_wifista();
#endif
#ifdef CONFIG_INTERNAL_ETHERNET
  char hostname[33];    
  LAN_load_hostname(hostname, sizeof(hostname));
  err = ETH_start(hostname);
#endif
#if (defined CONFIG_INTERNAL_WIFI) || (defined CONFIG_INTERNAL_ETHERNET)
  if (err == ESP_OK) LAN_status_handler(LANconnecting);
#endif
  return err;
}


bool LAN_has_ethernet(void) {
#ifdef CONFIG_INTERNAL_ETHERNET
  return ETH_hasPHY();
#else
  return false;
#endif
}


EventBits_t LAN_events_get(void) {
  return (lan_event_group != NULL)? xEventGroupGetBits(lan_event_group): 0;
}

void LAN_events_set(unsigned int events) {
  if (lan_event_group == NULL) return;
  xEventGroupSetBits(lan_event_group, (EventBits_t) events);
}

void LAN_events_clear(unsigned int events) {
  if (lan_event_group == NULL) return;
  xEventGroupClearBits(lan_event_group, (EventBits_t) events);
}


unsigned int LAN_wait4event(unsigned int event_mask, int timeout_ms) {
  EventBits_t r = 0;
  TickType_t  dly = (timeout_ms == -1)? portMAX_DELAY: timeout_ms / portTICK_PERIOD_MS;
  if (lan_event_group == NULL) return 0;
  if (timeout_ms == 0)
    r = xEventGroupGetBits(lan_event_group);
  else
    r = xEventGroupWaitBits(lan_event_group, event_mask, pdFALSE, pdFALSE, dly);
  return r & event_mask;
}


int LAN_wait4ConnectionWD(void) {
  esp_err_t   err = ESP_OK;
  TickType_t  dly = (WIFI_WAIT4CONNECTION_TO * 1000) /  portTICK_PERIOD_MS;
  EventBits_t r;

  if (lan_event_group == NULL) return 0;
  do {
    r = xEventGroupWaitBits(lan_event_group, CONNECTED_BIT, pdFALSE, pdFALSE, dly);
#ifdef CONFIG_INTERNAL_WIFI
    if ((r & CONNECTED_BIT) == 0) {	// no connection
      err = WiFiSTA_reloadDriver();
    }
#endif
  } while ((r & CONNECTED_BIT) == 0);
  return err == ESP_OK? 1: 0;
}



void LAN_get_mac(unsigned char mac[6]) {
  esp_err_t err = esp_read_mac(mac, 0);
  if (err != ESP_OK) {
    ESP_LOGE(LANTAG, "can't get MAC address");
    memset(mac, 0xFF, 6);
  } else {
    ESP_LOGD(LANTAG, "My MAC: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
}


int LAN_get_ip(esp_netif_ip_info_t *ip_info) {  
  int ip_ok = 0;
#ifdef CONFIG_INTERNAL_ETHERNET
  ip_ok = ETH_get_ip(ip_info);
  if (ip_ok == 0) return 0;
#endif
#ifdef CONFIG_INTERNAL_WIFI
  ip_ok = WiFiSTA_get_ip(ip_info);
#endif
  return ip_ok;
}




const char *LAN_get_hostname(void) {
  const char *hostname_ptr = NULL;
#ifdef CONFIG_INTERNAL_WIFI
  hostname_ptr = WiFiSTA_get_hostname();
#endif
  return (hostname_ptr != NULL)? hostname_ptr: LAN_hostname;
}


void LAN_set_hostname(const char *hostname) {
  esp_err_t  err;
  nvs_handle lanconf;
  if ((hostname == NULL) || (strnlen(hostname, 4) < 2)) return;
  err = nvs_open(LAN_namespace, NVS_READWRITE, &lanconf);
  if (err != ESP_OK) {
    ESP_LOGE(LANTAG, "LAN_set_hostname() - nvs_open failed");
    return;
  }
  err = nvs_set_str(lanconf, LANcfg_hostn, hostname);
  if (err != ESP_OK) {
    ESP_LOGE(LANTAG, "hostname set failed: %s", esp_err_to_name(err));
  } else {
    strncpy(LAN_hostname, hostname, sizeof(LAN_hostname));
    ESP_LOGD(LANTAG, "hostname set.");
  }
  nvs_close(lanconf);
}
