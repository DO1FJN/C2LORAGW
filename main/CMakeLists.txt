
set(requrired_components "arch" "board" "utilities" "s_driver" "c2lora" "localaudio" "HAMdLNK" )

# adding UI component (target dependent)
if(CONFIG_TARGET_BREADBOARD)
  list(APPEND requrired_components "ui_320x170x16")
endif()

if(CONFIG_TARGET_HELTEC_WIFILORA_V3 OR CONFIG_TARGET_C2LOARAGW_V1)
  list(APPEND requrired_components "ui_128x64x1")
endif()

if(CONFIG_TARGET_LILYGO_TDECK)
  list(APPEND requrired_components "ui_320x240x16")
endif()


idf_component_register(
   SRCS "main.c" "hamdlnk2audio.c" "udp_announcer.c"
   INCLUDE_DIRS "include"
   REQUIRES ""
   PRIV_REQUIRES ${required_components}
)

# creates filesystem used internal
#spiffs_create_partition_image(storage ../factoryfs FLASH_IN_PROJECT)
