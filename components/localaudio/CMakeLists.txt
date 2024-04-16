
idf_component_register(
  SRCS "localaudio.c" "max98357a.c"
  INCLUDE_DIRS "include"
  REQUIRES ""
  PRIV_REQUIRES "driver" "board" "arch" "s_driver" "codec2" "HAMdLNK" "esp_timer"
)
