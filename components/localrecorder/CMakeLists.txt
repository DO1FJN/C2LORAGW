
set(localrecorder_srcs "localrecorder.c" )
set(localrecorder_reqs "")


idf_component_register(
  SRCS ${localrecorder_srcs}
  INCLUDE_DIRS "include"
  REQUIRES ${localrecorder_reqs}
  PRIV_REQUIRES "driver" "board" "arch" "s_driver" "esp_timer"
  )
