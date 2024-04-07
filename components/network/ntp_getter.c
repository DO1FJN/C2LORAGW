/*
 * ntp_getter.c
 *
 *  Created on: 11.04.2019
 *      Author: Jan Alte, DO1FJN
 *
 * Report:
 * 2024-02-02: new ESP_IDF v5 version
 */

#include "LAN.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <time.h>
#include <sys/time.h>

#include "esp_sntp.h"
#include "esp_netif_sntp.h"
#include "esp_log.h"

#include "sdkconfig.h"


#define NTP_RETRIES			    15

#define NTP_WAIT_FIRST_MS		500
#define NTP_WAIT_MS			    2000

#define NTP_RXTIME_TASK_STACKSIZE	3072

static const char *NTP_TAG = "NTPclient";

static tntp_gottime_handler NTP_HND = NULL;


static void time_sync_notification_cb(struct timeval *tv) {
  sntp_sync_status_t status = sntp_get_sync_status();
  ESP_LOGI(NTP_TAG, "time synchronization event #%d", status);
  switch(status) {
  case SNTP_SYNC_STATUS_RESET:		// 0
    break;
  case SNTP_SYNC_STATUS_COMPLETED:	// 1
    if (NTP_HND) NTP_HND( tv->tv_sec);
    break;
  case SNTP_SYNC_STATUS_IN_PROGRESS:	// 2
    break;
  } // hctiws
}


static void ntp_rxtime_task(void *ini) {
  time_t now = 0;
  struct tm timeinfo = { 0 };
  char   strftime_buf[64];
  int    retry;
  if (ini == NULL) {
    ESP_LOGE(NTP_TAG, "no timeserver / argument empty");
    vTaskDelete(NULL);
  }

  esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG((char *)ini);
//  esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(2, ESP_SNTP_SERVER_LIST(CONFIG_SNTP_TIME_SERVER, "pool.ntp.org" ) );

  config.start = false;                       // start SNTP service explicitly (after connecting)
#ifdef CONFIG_LWIP_DHCP_GET_NTP_SRV
  config.server_from_dhcp = true;             // accept NTP offers from DHCP server, if any (need to enable *before* connecting)
  config.index_of_first_server = 1;           // updates from server num 1, leaving server 0 (from DHCP) intact
#else
  config.server_from_dhcp = false;             // accept NTP offers from DHCP server, if any (need to enable *before* connecting)
  config.index_of_first_server = 0;           // updates from server num 1, leaving server 0 (from DHCP) intact
#endif
  config.renew_servers_after_new_IP = true;   // let esp-netif update configured SNTP server(s) after receiving DHCP lease
  config.sync_cb = time_sync_notification_cb; // only if we need the notification function
  esp_netif_sntp_init(&config);

NTP_RESTART:
  ESP_LOGI(NTP_TAG, "wait network connect...");
  while (LAN_wait4event(CONNECTED_BIT, -1) == 0);
  ESP_LOGI(NTP_TAG, "get time from NTP server '%s'...", (char *)ini);
  
  esp_netif_sntp_start();

  esp_netif_sntp_sync_wait(NTP_WAIT_FIRST_MS / portTICK_PERIOD_MS);

  for (retry = NTP_RETRIES; LAN_wait4event(CONNECTED_BIT, 0) && (retry > 0); retry--) {
    ESP_LOGD(NTP_TAG, "waiting for system time to be set... (%d/10)", NTP_RETRIES+1-retry);
    if (esp_netif_sntp_sync_wait(NTP_WAIT_MS / portTICK_PERIOD_MS) == ESP_OK) break;
  } // rof

  time(&now);
  localtime_r(&now, &timeinfo);

  if (timeinfo.tm_year < (2019 - 1900)) {
    ESP_LOGI(NTP_TAG, "no time set - give up");
    LAN_wait4event(DISCONNECTED_BIT, -1);
    sntp_restart();
    goto NTP_RESTART;
  }
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  ESP_LOGI(NTP_TAG, "UTC time is %s. exit NTP client task", strftime_buf);
  vTaskDelete(NULL);
}


void ntp_startclienttask(const char *ntp_pool_server_url, tntp_gottime_handler hnd) {
  NTP_HND = hnd;
  xTaskCreate(ntp_rxtime_task, "ntp_task", NTP_RXTIME_TASK_STACKSIZE, (void *)ntp_pool_server_url, 2, NULL);
}


void ntp_handle_foreigntime(unsigned int utctime) {
  time_t ctrl_time = utctime;
  time_t my_time = 0;
  my_time = time(NULL);
  ESP_LOGD(NTP_TAG, "got own time of %lld, foreign source reports %lld", my_time, ctrl_time);
  if (my_time < 31536000) {
    struct timeval tval;
    struct tm timeinfo = { 0 };
    tval.tv_sec  = utctime;
    tval.tv_usec = 0;
    settimeofday(&tval, NULL);
    ESP_LOGI(NTP_TAG, "time set by foreign source");
    my_time = time(NULL);
    localtime_r(&my_time, &timeinfo);
    ESP_LOGI(NTP_TAG, "got time = %2d:%02d, year %d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_year);
  } // my_time is before 1971
}
