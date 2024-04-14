
idf_component_register(
  SRCS "ui_oled128x64.c"
  INCLUDE_DIRS "include"
  REQUIRES ""
  PRIV_REQUIRES "oled_display" "fonts" "c2lora" "sx126x_driver"
)
