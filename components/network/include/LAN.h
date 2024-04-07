/*
 * LAN.h
 *
 *  Created on: 15.07.2021
 *      Author: Jan Alte
 *          (c) 2021 bentrup Industriesteuerungen
 */

#ifndef NETWORK_LAN_
#define NETWORK_LAN_

#include <esp_err.h>
#include <stdbool.h>


/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
#define CONNECTED_BIT		  BIT0
#define DISCONNECTED_BIT	BIT1
#define ADDR_CHANGE_BIT		BIT2
#define CONNECTING_BIT    BIT3
#define WPSACTIVE_BIT		  BIT4
#define SCAN_ACTIVE_BIT		BIT6
#define SCAN_FINISHED_BIT	BIT7
#define WIFI_SHUTDOWN_BIT	BIT8

#define WIFI_TEMPON_BIT		BIT9

#define ETH_SHUTDOWN_BIT	BIT16

#define WIFI_WAIT4CONNECTION_TO	60		// 60s - if fails in this time, a restart is triggered


typedef enum {
  LANdisabled, LANconnecting, LANethLinkDn, LANethLinkUp, LANethLinkOk, LANwifiLinkDn, LANwifiLinkUp, LANwifiLinkOk, LANlostIP
} tLANstatus;

esp_err_t	LAN_init(void);

esp_err_t	LAN_start(void);

esp_err_t LAN_load_hostname(char *hostname, int hostname_size);

esp_err_t	LAN_shutdown(void);

int		LAN_wait4ConnectionWD(void);	// handles a WiFi Watchdog if not connected.

void  LAN_events_set(unsigned int events);
void  LAN_events_clear(unsigned int events);

unsigned int	LAN_wait4event(unsigned int event_mask, int timeout_ms);


void		LAN_get_mac(unsigned char mac[6]);

const char *LAN_get_hostname(void);

void		LAN_set_hostname(const char *hostname);

void		LAN_status_handler(tLANstatus s);

bool    LAN_has_ethernet(void);

// NTP 
typedef void (*tntp_gottime_handler) (int);

void	ntp_startclienttask(const char *ntp_pool_server_url, tntp_gottime_handler hnd);

void	ntp_handle_foreigntime(unsigned int utctime);

#endif
