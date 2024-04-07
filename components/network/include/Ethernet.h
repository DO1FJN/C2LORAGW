/*
 * Ethernet.h
 *
 *  Created on: 15.07.2021
 *      Author: Jan Alte
 *          (c) 2021 bentrup Industriesteuerungen
 */

#ifndef NETWORK_ETHERNET_
#define NETWORK_ETHERNET_

#include <esp_err.h>
#include <esp_netif.h>


esp_err_t	ETH_init(void);
esp_err_t	ETH_start(const char *hostname);
esp_err_t	ETH_shutdown(void);

void		ETH_set_default_hostname(const char *hostname);

int		ETH_get_ip(esp_netif_ip_info_t *ip_info);

bool            ETH_hasPHY(void);

#endif

