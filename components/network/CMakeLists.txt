
set(network_srcs "LAN.c" "OTA.c" "ntp_getter.c")
set(network_reqs "lwip")

if(CONFIG_BT_SPP_ENABLED)
  list(APPEND network_srcs "BlueSPP.c")
  list(APPEND network_reqs "bt")
endif()

if(CONFIG_INTERNAL_WIFI)
  list(APPEND network_srcs "WiFiSTA.c")
endif()


if(CONFIG_INTERNAL_ETHERNET)
  list(APPEND network_srcs "Ethernet.c")
endif()


idf_component_register(
  SRCS ${network_srcs}
  INCLUDE_DIRS "include"
#  PRIV_INCLUDE_DIRS "../../main/include"
  REQUIRES ${network_reqs}
  PRIV_REQUIRES "utilities" "deviceconfig" "arch" "freertos" "nvs_flash" "esp_wifi" "wpa_supplicant" "esp_http_client" "esp_https_ota" "app_update" "console"
  )
