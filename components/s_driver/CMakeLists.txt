


idf_component_register(
  SRCS "s_buffer.c" "i2s_loop.c" "s_spi.c"
  INCLUDE_DIRS "include"
  PRIV_REQUIRES "arch" "board" "sx126x_driver" "soc" "esp_timer" "driver" "esp_hw_support" "heap"
  )
