if (CONFIG_TARGET_HELTEC_WIFILORA_V3 OR CONFIG_TARGET_C2LOARAGW_V1)
  set(srcs "SSD1306.c" "OLED.c" "fonts1bit.c")
else()
  set(srcs "")
endif()

idf_component_register(
  SRCS ${srcs}
  INCLUDE_DIRS "include"
  REQUIRES "fonts"
  PRIV_REQUIRES "driver" "arch" "board" "s_driver"
)
