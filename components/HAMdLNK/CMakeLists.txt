
set(HAMdLNK_srcs "HAMdLNK.c" "hamdlnk_udp.c" "subframedef.c" "userdata.c")
set(HAMdLNK_reqs "network")


idf_component_register(
  SRCS ${HAMdLNK_srcs}
  INCLUDE_DIRS "include"
  REQUIRES ${HAMdLNK_reqs}
  PRIV_REQUIRES "arch" "utilities" "deviceconfig" "s_driver" "lwip" "c2lora"
)
