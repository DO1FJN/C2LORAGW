
if (CONFIG_TARGET_LILYGO_TDECK OR CONFIG_TARGET_BREADBOARD)
set(tft_srcs "ST7789.c" "TFT.c" "Backlight.c" "font2disp.c" "font2ram.c")
set(tft_reqs "fonts")
else()
set(tft_srcs )
set(tft_reqs )
endif()


idf_component_register(
  SRCS ${tft_srcs}
  INCLUDE_DIRS "include"
  REQUIRES ${tft_reqs}
  PRIV_REQUIRES "driver" "arch" "board" "s_driver" "fonts"
)
