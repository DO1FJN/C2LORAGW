idf_component_register(
  SRCS "utilities.c" "IniFileRead.c" "checksums.c"
  INCLUDE_DIRS "include"  
  PRIV_REQUIRES "arch" "fatfs" "nvs_flash"
)
