
idf_component_register(
  SRCS "sx126x.c" "sx126x_driver_version.c" "sx126x_lr_fhss.c" "lr_fhss_mac.c"
  INCLUDE_DIRS "include"
  PRIV_REQUIRES "s_driver"
  )
