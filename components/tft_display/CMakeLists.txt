
if (CONFIG_TARGET_BREADBOARD OR CONFIG_TARGET_LILYGO_TDECK)
  set(srcs "ST7789.c" "TFT.c" "Backlight.c" "font2disp.c" "font2ram.c")
else()
  set(srcs "")  
endif()


idf_component_register(
  INCLUDE_DIRS "include"
  SRCS ${srcs}
  REQUIRES "fonts"
  PRIV_REQUIRES "arch" "board" "driver" "s_driver"
)
