
idf_component_register(
  SRCS "SSD1306.c" "OLED.c" "fonts1bit.c"
  INCLUDE_DIRS "include"
  REQUIRES "tft_display"
  PRIV_REQUIRES "driver" "arch" "board" "s_driver" "fonts"
)
