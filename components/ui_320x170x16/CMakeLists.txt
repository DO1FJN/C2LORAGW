if (CONFIG_TARGET_BREADBOARD)
  set(srcs "ui_tft320x170.c")
else()
  set(srcs "")
endif()

idf_component_register(
  INCLUDE_DIRS "include"
  SRCS ${srcs}
  PRIV_REQUIRES "arch" "board" "c2lora" "s_driver" "sx126x_driver" "tft_display" "fonts"
)
