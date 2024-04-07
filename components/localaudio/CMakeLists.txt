
set(localaudio_srcs "localaudio.c" "max98357a.c" )
set(localaudio_reqs "")


idf_component_register(
  SRCS ${localaudio_srcs}
  INCLUDE_DIRS "include"
  REQUIRES ${localaudio_reqs}
  PRIV_REQUIRES "driver" "board" "arch" "s_driver" "codec2" "HAMdLNK" "esp_timer"
  )
