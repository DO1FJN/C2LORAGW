


idf_component_register(
  SRCS "s_buffer.c" "i2s_loop.c" "s_spi.c"
  INCLUDE_DIRS "include"
  PRIV_REQUIRES "soc" "esp_timer" "driver" "board" "arch" "esp_hw_support" "heap" "sx126x_driver"
  )
