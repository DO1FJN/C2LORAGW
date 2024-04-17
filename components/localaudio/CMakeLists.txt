
idf_component_register(
  SRCS "localaudio.c" "input_codecs.c" "output_codecs.c"
  INCLUDE_DIRS "include"
  REQUIRES "s_driver"
  PRIV_REQUIRES "driver" "board" "arch" "codec2" "HAMdLNK" "esp_timer"
)
