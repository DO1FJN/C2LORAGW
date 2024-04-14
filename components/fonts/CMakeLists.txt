if (CONFIG_TARGET_LILYGO_TDECK) 
# OR CONFIG_TARGET_BREADBOARD)
set(font_srcs "fonts.c" "HackSmall12.c" "HackStd20.c" "HackBig40.c" "BIGnumbers.c")
else()
set(font_srcs "fonts.c" "1bit_Hack_09x17.c" "1bit_Hack_13x24.c" "1bit_Hack_16x29.c")
endif()

idf_component_register(
  SRCS ${font_srcs}
  INCLUDE_DIRS "include"
)
