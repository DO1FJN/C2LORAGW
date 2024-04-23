
set(priv_requires "arch" "board" "c2lora" "sx126x_driver" "fonts")

if (CONFIG_TARGET_HELTEC_WIFILORA_V3 OR CONFIG_TARGET_C2LOARAGW_V1)
  set(srcs "ui_oled128x64.c")
  list(APPEND priv_requires "oled_display")
elseif(CONFIG_TARGET_BREADBOARD)
  set(srcs "ui_tft320x170.c")
  list(APPEND priv_requires "tft_display")
elseif(CONFIG_TARGET_LILYGO_TDECK)
  set(srcs "ui_tft320x240.c")
  list(APPEND priv_requires "tft_display")
else() 
  set(srcs "")
endif()

idf_component_register(
  SRCS ${srcs}
  INCLUDE_DIRS "include"
  REQUIRES "tft_display"
  PRIV_REQUIRES ${priv_requires}
)
