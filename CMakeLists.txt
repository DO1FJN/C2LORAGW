# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(C2LoraGateway)


set(PROJECT_VER "0.01.2")
set(COMPONENTS "arch" "main" "board" "utilities" "network" "localaudio" "c2lora" "HAMdLNK")

if (CONFIG_TARGET_LILYGO_TDECK)
  list(APPEND components "tft_display")
endif()

if (CONFIG_TARGET_C2LOARAGW_V1 OR CONFIG_TARGET_HELTEC_WIFILORA_V3)
  list(APPEND components "oled_display" "ui_128x64x1")
endif()

if (CONFIG_TARGET_BREADBOARD)
  list(APPEND components "oled_display" "ui_128x64x1")
endif()
