
set(font_srcs "fonts.c")

# adding 4bit or bit fonts component (target dependent)
if(CONFIG_TARGET_BREADBOARD OR CONFIG_TARGET_LILYGO_TDECK)
  list(APPEND font_srcs "HackSmall12.c" "HackStd20.c" "HackBig40.c" "BIGnumbers.c")
endif()

if(CONFIG_TARGET_HELTEC_WIFILORA_V3 OR CONFIG_TARGET_C2LOARAGW_V1)
  list(APPEND font_srcs "1bit_Hack_09x17.c" "1bit_Hack_13x24.c" "1bit_Hack_16x29.c")
endif()

idf_component_register(
  INCLUDE_DIRS "include"
  SRCS ${font_srcs}
)
