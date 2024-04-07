/*
 * WiFiSTA.h
 *
 *  Created on: 14.03.2019
 *      Author: Jan Alte
 *          (c) 2019 bentrup Industriesteuerungen
 */

#ifndef NETWORK_WIFISTA_H_
#define NETWORK_WIFISTA_H_

#include <esp_err.h>
#include <esp_netif.h>


#define ENABLE_WPS
#define SCANAP_MAX_RESULTS	30

#define WIFI_WPS_MODEL_NUMBER	"V010"
#ifdef CONFIG_DEVICE_NAME
#define WIFI_WPS_MODEL_NAME CONFIG_DEVICE_NAME
#else
#define WIFI_WPS_MODEL_NAME	"LoraGW_WPS"
#endif

/* values for WiFiSTA_calcSignalQuality()
 * first: min dBm (RSSI) for 100%
 * 2nd  : lowes dBm (RSSI) - below that the function returns 1%
 */
#define LINKQUALITY_100_DBM	(-50.0)
#define LINKQUALITY_0_DBM	(-85.0)



typedef enum {	// same as in esp_wps.h enum 'wps_type_t'
  WPSTYPE_DISABLE = 0, WPSTYPE_PBC, WPSTYPE_PIN,  WPSTYPE_MAX
} tWPStype;

//typedef void (*tWPScallback_funct) (int event, const char pin[8]);

typedef struct  __attribute__((__packed__)) {
  char			ssid[33];
  unsigned char		auth_mode;
  char			rsvd1[2];
} twifi_item;



const char *WiFiSTA_get_hostname(void); // gets hostname (ptr) from Wifi netif OR NULL if WiFi isn't initialized

esp_err_t	WiFiSTA_init(const char *hostname);

esp_err_t	WiFiSTA_turnoff(void);			// set NVS WiFi mode to OFF permanently

esp_err_t	WiFiSTA_start(void);
esp_err_t	WiFiSTA_shutdown(void);			// turns off WiFi at all
esp_err_t	WiFiSTA_reset(void);			// disconnects from an AP

esp_err_t	WiFiSTA_reloadDriver(void);


esp_err_t	WiFiSTA_scan(void);
esp_err_t 	WiFiSTA_cancel_scan(void);
void		WiFISTA_scan_callback(int no_of_aps);

int		WiFiSTA_create_configuration(char *textbuf, unsigned int buf_size);


//unsigned int	WiFiSTA_wait4event(unsigned int event_mask, int timeout_ms);


#ifdef ENABLE_WPS

esp_err_t	WiFiSTA_enableWPS(tWPStype t);

void		WiFiSTA_WPS_callback(int event, const char pin[8]);	// WEAK

#else

#define WiFiSTA_enableWPS(t)	ESP_OK

#endif

/* WiFiSTA_getSignalStrength()
 * returns RSSI value in STAtion mode (if connected to an AP).
 * return zero otherwise.
 */
signed char	WiFiSTA_getSignalStrength(void);


/* WiFiSTA_calcSignalQuality()
 * estimates a 1 to 100% singal quality value from AP RSSI
 * return 0% if no lint is established
 */
unsigned char	WiFiSTA_calcSignalQuality(signed char rssi);


twifi_item *	WiFiSTA_get_scanresults(int *result_size);
void		WiFiSTA_free_scanresults(twifi_item *scan_result_list);

int		  WiFiSTA_get_ip(esp_netif_ip_info_t *ip_info);

void    register_wifista(void);


#endif // MAIN_INCLUDE_WIFISTA_H_
