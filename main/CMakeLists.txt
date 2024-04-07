idf_component_register(
   SRCS "main.c" "hamdlnk2audio.c" "udp_announcer.c"
   INCLUDE_DIRS "include"
#  EMBED_TXTFILES certs/bentrup_cacert.pem certs/bentrupWebbox_cert.pem certs/bentrupWebbox_private.key
)

# creates filesystem used internal
#spiffs_create_partition_image(storage ../factoryfs FLASH_IN_PROJECT)
