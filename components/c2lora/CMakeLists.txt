
set(C2LORA_srcs "C2LORA_core.c" "C2LORA_modes.c" "C2LORA_header.c" "C2LORA_hamdlnk.c" "c2lora_uhf.c")
set(C2LORA_reqs "HAMdLNK" "localaudio" "network" "localrecorder")


idf_component_register(
  SRCS ${C2LORA_srcs}
  INCLUDE_DIRS "include"
  REQUIRES ${C2LORA_reqs}
  PRIV_REQUIRES "arch" "utilities" "board" "esp_timer" "driver" "s_driver" "sx126x_driver" "console" "nvs_flash"
)
