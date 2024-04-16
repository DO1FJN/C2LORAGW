if (CONFIG_TARGET_HELTEC_WIFILORA_V3 OR CONFIG_TARGET_C2LOARAGW_V1)
  set(srcs "ui_oled128x64.c")
else() 
  set(srcs "")
endif()

idf_component_register(
  SRCS ${srcs}
  INCLUDE_DIRS "include"
  PRIV_REQUIRES "c2lora" "sx126x_driver" "oled_display" "fonts"
)
