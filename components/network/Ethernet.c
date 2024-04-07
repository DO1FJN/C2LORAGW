/*
 * Ethernet.c
 *
 *  Created on: 15.07.2021
 *      Author: Jan Alte
 *          (c) 2021 bentrup Industriesteuerungen
 *
 * Report:
 * 2021-07-15	First version
 *
 */

#include "Ethernet.h"
#include "LAN.h"
#include "WiFiSTA.h"

#include "MasterIni.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include <esp_netif.h>
#include <esp_eth.h>
#include <esp_event.h>
#include <esp_log.h>
#include "driver/gpio.h"

#include <string.h>

#include "sdkconfig.h"

#ifdef CONFIG_INTERNAL_ETHERNET

static const char *TAG = "ETH";


static esp_eth_handle_t	eth_handle;
static esp_netif_t *	eth_netif;

// FreeRTOS event group to signal when we are connected & ready to make a request
extern EventGroupHandle_t lan_event_group;	// from LAN.c

#ifdef CONFIG_ETH_AUTODETECT_PHY
bool HAS_ETHERNET_PHY;
#else
#define HAS_ETHERNET_PHY   (1)
#endif


/** Event handler for Ethernet events */
static void ETH_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  uint8_t mac_addr[6] = {0};
  /* we can get the ethernet driver handle from event data */
  esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;
  switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
      ESP_LOGI(TAG, "Ethernet Link Up");
#ifdef CONFIG_INTERNAL_WIFI
      WiFiSTA_shutdown();
#endif
      esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
      if (lan_event_group) {
	      xEventGroupClearBits(lan_event_group, DISCONNECTED_BIT);
      } // fi
      ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
      LAN_status_handler(LANethLinkUp);
      break;
    case ETHERNET_EVENT_DISCONNECTED:
      ESP_LOGI(TAG, "Ethernet Link Down");
      if (lan_event_group) {
	      xEventGroupClearBits(lan_event_group, CONNECTED_BIT);
	      xEventGroupSetBits(lan_event_group, DISCONNECTED_BIT);
      } // fi
#ifdef CONFIG_INTERNAL_WIFI
      WiFiSTA_start();
#endif
      LAN_status_handler(LANethLinkDn);
      break;
    case ETHERNET_EVENT_START:
      ESP_LOGI(TAG, "Ethernet Started");
      break;
    case ETHERNET_EVENT_STOP:
      ESP_LOGI(TAG, "Ethernet Stopped");
      if (lan_event_group) {
        xEventGroupClearBits(lan_event_group, CONNECTED_BIT);
        xEventGroupSetBits(lan_event_group, DISCONNECTED_BIT);
      }
      break;
    default:
      break;
  } // hctiws
}



esp_err_t ETH_init(void) {
  eth_netif  = NULL;
#ifdef CONFIG_ETH_AUTODETECT_PHY
  gpio_config_t MDIOpin = {
    .pin_bit_mask = 1 << CONFIG_SW_ETH_MDIO_GPIO,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
    .intr_type = GPIO_INTR_DISABLE    
  };
  esp_err_t err = gpio_config(&MDIOpin);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "autodetect fails on MDIO pin #%d", CONFIG_SW_ETH_MDIO_GPIO);
    return err;
  }
  
  HAS_ETHERNET_PHY = gpio_get_level(CONFIG_SW_ETH_MDIO_GPIO);

  MDIOpin.pin_bit_mask = 1 << CONFIG_SW_ETH_PHY_RST_GPIO;
  MDIOpin.pull_down_en = GPIO_PULLDOWN_DISABLE;
  MDIOpin.mode = GPIO_MODE_OUTPUT;
  err = gpio_config(&MDIOpin);

  gpio_set_level(CONFIG_SW_ETH_PHY_RST_GPIO, 0);

  gpio_reset_pin(CONFIG_SW_ETH_MDIO_GPIO);

  ESP_LOGI(TAG, "PHY is %s", HAS_ETHERNET_PHY?"present": "absend");
  if (!HAS_ETHERNET_PHY) return ESP_OK;
#else
  gpio_set_level(CONFIG_SW_ETH_PHY_RST_GPIO, 0);
  vTaskDelay(30);
#endif

  eth_mac_config_t mac_config  = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phy_config  = ETH_PHY_DEFAULT_CONFIG();
  
  phy_config.phy_addr          = CONFIG_SW_ETH_PHY_ADDR;
  phy_config.reset_gpio_num    = CONFIG_SW_ETH_PHY_RST_GPIO;
  phy_config.reset_timeout_ms  = 250;
  
  mac_config.smi_mdc_gpio_num  = CONFIG_SW_ETH_MDC_GPIO;
  mac_config.smi_mdio_gpio_num = CONFIG_SW_ETH_MDIO_GPIO;
  mac_config.sw_reset_timeout_ms  = 250;
  
  esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&mac_config);
  esp_eth_phy_t *phy = esp_eth_phy_new_lan8720(&phy_config);

  esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);

  return esp_eth_driver_install(&config, &eth_handle);
}



esp_err_t ETH_start(const char *hostname) {
  // Create default event loop that running in background
  esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
  
  if (!HAS_ETHERNET_PHY) return ESP_OK;

  eth_netif = esp_netif_new(&cfg);

  if ((hostname != NULL) && (strnlen(hostname, 4) > 3) ) {
    esp_err_t err = esp_netif_set_hostname(eth_netif, hostname);
    if (err != ESP_OK)
      ESP_LOGE(TAG, "set hostname '%s' fails", hostname);
    else
      ESP_LOGD(TAG, "hostname '%s' set", hostname);
  } // fi

  // Set default handlers to process TCP/IP stuffs
  ESP_ERROR_CHECK(esp_eth_set_default_handlers(eth_netif));

  // Register user defined event handers
  ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, ETH_event_handler, NULL));

  // attach Ethernet driver to TCP/IP stack
  ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

  ESP_LOGD(TAG, "start connection...");
  xEventGroupClearBits(lan_event_group, ETH_SHUTDOWN_BIT);

  // start Ethernet driver state machine
  return esp_eth_start(eth_handle);
}


esp_err_t ETH_shutdown(void) {
  esp_err_t dres;
  if (!HAS_ETHERNET_PHY) return ESP_OK;  
  dres = esp_eth_stop(eth_handle);
  if (dres != ESP_OK) {
    ESP_LOGE(TAG, "stopping failed, %d", dres);
  }
  xEventGroupSetBits(lan_event_group, ETH_SHUTDOWN_BIT);
  ESP_LOGD(TAG, "stop connection...");
  LAN_wait4event(DISCONNECTED_BIT, 50);
  esp_netif_destroy(eth_netif);
  eth_netif = NULL;

  // turn off PHY:
  gpio_set_pull_mode(CONFIG_SW_ETH_PHY_RST_GPIO, GPIO_PULLDOWN_ONLY);
  gpio_set_direction(CONFIG_SW_ETH_PHY_RST_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(CONFIG_SW_ETH_PHY_RST_GPIO, 0);

  //  ESP_ERROR_CHECK( esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID, IP_event_handler) );
  return esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, &ETH_event_handler);
}


int ETH_get_ip(esp_netif_ip_info_t *ip_info) {
  if (!HAS_ETHERNET_PHY || (ip_info == NULL) || (eth_netif == NULL)) return -2;
  return esp_netif_get_ip_info(eth_netif, ip_info) == ESP_OK? 0: -1;
}


bool ETH_hasPHY(void) {
  return HAS_ETHERNET_PHY;
}

#endif
