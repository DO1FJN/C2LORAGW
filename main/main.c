/*

C2 LoRa Gateway

This file is the main source file of project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.

*/

#include "sdkconfig.h"

#include "hardware.h"

#include "ui.h"
#include "OLED.h" // for tests only


#include "MasterIni.h"
#include "utilities.h"

#include "C2LORA.h"
#include "C2LORA_modes.h"
#include "C2LORA_header.h"
#include "c2lora_uhf.h"
#include "C2LORA_hamdlnk.h"

#include "LAN.h"
#include "HAMdLNK.h"
#include "hamdlnk2audio.h"
#include "localaudio.h"

#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_check.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_spiffs.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <fcntl.h>


#ifdef TFT_HOST
#include "font_exports.h"
#include "TFT.h"
#endif


typedef enum {
  SM_OFFLINE, SM_DONGLE, SM_STANDBY, SM_ONAIR, SM_NOWIFI
} tStartupMode;


/*
 * We warn if a secondary serial console is enabled. A secondary serial console is always output-only and
 * hence not very useful for interactive console applications. If you encounter this warning, consider disabling
 * the secondary serial console in menuconfig unless you know what you are doing.
 */
#if SOC_USB_SERIAL_JTAG_SUPPORTED
#if !CONFIG_ESP_CONSOLE_SECONDARY_NONE
#warning "A secondary serial console is not useful when using the console component. Please disable it in menuconfig."
#endif
#endif

#define PROMPT_STR    CONFIG_DEVICE_NAME

#define NTP_DEFAULT_URL		"pool.ntp.org"


static const char *TAG = "LoRaGW";

#define ASCIIartlines 6
static const char *ASCIIartTitle[ASCIIartlines] = {
" _____  _____  _          ______      _____ _    _ ",
"/  __ \\/ __  \\| |         | ___ \\    |  __ \\ |  | |",
"| /  \\/`' / /'| |     ___ | |_/ /__ _| |  \\/ |  | |",
"| |      / /  | |    / _ \\|    // _` | | __| |/\\| |",
"| \\__/\\./ /___| |___| (_) | |\\ \\ (_| | |_\\ \\  /\\  /",
" \\____/\\_____/\\_____/\\___/\\_| \\_\\__,_|\\____/\\/  \\/ "
};



static const char *APP_namespace    = "C2LORAGW";

static const char *APP_startupmode  = "start_opmode";
static const char *APP_sendto_host  = "sendto_host";
static const char *APP_sendto_host2 = "sendto_host2";

static const char *APP_callsign     = "callsign";
static const char *APP_recipient    = "destination";
static const char *APP_kindofsender = "k_o_s";
static const char *APP_areacode     = "areacode";
static const char *APP_locator      = "locator";

static const char *APP_volume       = "volume";
static const char *APP_micgain      = "micgain";


static const tHAMdLNK_network_config ham2lnk_default_config = {
  .sendto_hostname_uhf = "broadcast",
  .sendto_port_uhf    = CONFIG_HAMDLNK_PORT_MAIN,  // received C2LORA frames are forwarded to this port
  .listen_port_uhf    = CONFIG_HAMDLNK_PORT_MAIN,
  .sendto_hostname_dongle = "broadcast",
  .sendto_port_dongle = CONFIG_HAMDLNK_PORT_2ND,  // local audio from microphoe are send to this port (can be the same as UHF)
  .listen_port_dongle = CONFIG_HAMDLNK_PORT_2ND
};


static char cmd_prompt[24];


static void app_set_default_debuglevel(void) {
#if CONFIG_LOG_DEFAULT_LEVEL > 1
  esp_log_level_set("*", ESP_LOG_INFO);    
  esp_log_level_set("wifi", ESP_LOG_WARN);
  esp_log_level_set("wifi-init", ESP_LOG_WARN);
  //esp_log_level_set("wpa", ESP_LOG_VERBOSE);
  //esp_log_level_set("WiFiSTA", ESP_LOG_VERBOSE);

  esp_log_level_set("LoRaGW", ESP_LOG_VERBOSE);
  //esp_log_level_set("UI", ESP_LOG_VERBOSE);
  //esp_log_level_set("LAN", ESP_LOG_VERBOSE);
  //esp_log_level_set("UDPan", ESP_LOG_VERBOSE);
  //esp_log_level_set("NTPclient", ESP_LOG_VERBOSE);

  //esp_log_level_set("OLED", ESP_LOG_VERBOSE);
  //esp_log_level_set("SSD1306", ESP_LOG_VERBOSE);

  esp_log_level_set("C2LORA", ESP_LOG_VERBOSE);
  //esp_log_level_set("C2LORAcore", ESP_LOG_VERBOSE);
  //esp_log_level_set("C2LRA_HDR", ESP_LOG_VERBOSE);
  //esp_log_level_set("C2LORA-UDP", ESP_LOG_VERBOSE);
  //esp_log_level_set("HdL2AUDIO", ESP_LOG_DEBUG);
  //esp_log_level_set("LocalAudio", ESP_LOG_VERBOSE);

  //esp_log_level_set("ST7789", ESP_LOG_VERBOSE);
  //esp_log_level_set("S_SPI", ESP_LOG_VERBOSE);  
  //esp_log_level_set("i2s_loop", ESP_LOG_VERBOSE);
  //esp_log_level_set("gdma", ESP_LOG_VERBOSE);
#endif
}


static esp_err_t fs_open(void) {
  static const esp_vfs_spiffs_conf_t fsconf = {
    .base_path = "/ifs",
    .partition_label = NULL,
    .max_files = 5,
    .format_if_mount_failed = false
  };
  ESP_LOGD(TAG, "Initializing SPIFFS");

  esp_err_t ret = esp_vfs_spiffs_register(&fsconf);
  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      ESP_LOGE(TAG, "Failed to mount filesystem");
    } else if (ret == ESP_ERR_NOT_FOUND) {
      ESP_LOGE(TAG, "Failed to find SPIFFS partition");
    } else {
      ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
    }
    return ret;
  }
#if CONFIG_LOG_DEFAULT_LEVEL > 2
  size_t total = 0, used = 0;
  ret = esp_spiffs_info(NULL, &total, &used);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
  } else {
    ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
  }
#endif
  return ret;
}

static void fs_close(void) {
  esp_vfs_spiffs_unregister(NULL);
}


#define BLINK_PATTERN_NOSPI     0x0
#define BLINK_PATTERN_NOI2C     0x0
#define BLINK_PATTERN_NOWIFI    0x0
#define BLINK_PATTERN_NOHAMDLNK 0x0
#define BLINK_PATTERN_NOAUDIO   0x0
#define BLINK_PATTERN_NOC2LORA  0x0


static void stop_boot_with_fail(uint32_t blink_pattern) {
  ESP_LOGE(TAG, "stopped with fail blink pattern %08lxh", blink_pattern);
#if CONFIG_BOOT_FAIL_LED != GPIO_NUM_NC
  uint32_t shift_pattern;
  gpio_set_level(CONFIG_BOOT_FAIL_LED, 1);
  gpio_set_direction(CONFIG_BOOT_FAIL_LED, GPIO_MODE_OUTPUT_OD);
  while(1) {
    shift_pattern = blink_pattern;
    for (int i=32; i>0; i--, shift_pattern <<= 1) {
      gpio_set_level(CONFIG_BOOT_FAIL_LED, shift_pattern & 0x80000000L? 0: 1);
      vTaskDelay(pdMS_TO_TICKS(250));
    }    
    vTaskDelay(pdMS_TO_TICKS(2000));
  } // ehliw
#else
  while(1) {
    vTaskDelay(10);
  } // ehliw
#endif
}


/*
  fwversion = OTG_GetVersionAsUInt();
  sprintf(UDP_BC_answer, UDP_BC_fmtstr, Param->myname,
    fwversion>>12, (fwversion>>4)&0xFF, (fwversion&0x0F)? ('a'+(fwversion&0x0F)-1): ' ',
    Param->tcp_port, Param->backlog, Param->ssl_port);
*/


static int app_print_chipinfo(int argc, char **argv) {
  // Print chip information
  const char *model;
  esp_chip_info_t chip_info;
  uint32_t flash_size;
  esp_chip_info(&chip_info);
  switch (chip_info.model) {
  case CHIP_ESP32:
    model = "ESP32";
    break;
  case CHIP_ESP32S2:
    model = "ESP32-S2";
    break;
  case CHIP_ESP32S3:
    model = "ESP32-S3";
    break;
  case CHIP_ESP32C3:
    model = "ESP32-C3";
    break;
  case CHIP_ESP32H2:
    model = "ESP32-H2";
    break;
  case CHIP_ESP32C2:
    model = "ESP32-C2";
    break;
  default:
    model = "Unknown " CONFIG_IDF_TARGET;
    break;
  } // hctiws
  printf("\nThis is %s chip with %d CPU core(s), %s%s%s%s, ", model, chip_info.cores,
         (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
         (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
         (chip_info.features & CHIP_FEATURE_IEEE802154)? ", 802.15.4 (Zigbee/Thread)": "");

  unsigned major_rev = chip_info.revision / 100;
  unsigned minor_rev = chip_info.revision % 100;
  printf("silicon revision v%d.%d, ", major_rev, minor_rev);
  if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
    printf("Get flash size failed");
    return 1;
  }
  printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024), (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
  printf("IDF Version: %s\r\n", esp_get_idf_version());
  printf("Internal free heap size: %" PRIu32 " bytes\n\n", esp_get_free_internal_heap_size());
  return 0;
}


static int app_print_memory_usage(int argc, char **argv) {
  printf("int free heap size: %" PRIu32 " bytes\n", esp_get_free_internal_heap_size());
  printf("min free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
  printf("    free heap size: %" PRIu32 " bytes\n", esp_get_free_heap_size());  

  printf("MALLOC_CAP_EXEC     %d bytes\n", heap_caps_get_free_size( MALLOC_CAP_EXEC ));
  printf("MALLOC_CAP_INTERNAL %d bytes\n", heap_caps_get_free_size( MALLOC_CAP_INTERNAL ));
  printf("MALLOC_CAP_32BIT    %d bytes\n", heap_caps_get_free_size( MALLOC_CAP_32BIT ));
  printf("MALLOC_CAP_DEFAULT  %d bytes\n", heap_caps_get_free_size( MALLOC_CAP_DEFAULT ));
  return 0;
}


static int restart(int argc, char **argv) {
  ESP_LOGI(TAG, "Restarting...");
  esp_restart();
//  return 0; meckern????
}


static void register_chipinfo(void) {
  const esp_console_cmd_t cmd = {
    .command = "chipinfo",
    .help = "Get model/revision and features of this SOC and IDF version",
    .hint = NULL,
    .func = &app_print_chipinfo,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_free(void) {
  const esp_console_cmd_t cmd = {
    .command = "free",
    .help = "Get the current size of free heap memory",
    .hint = NULL,
    .func = &app_print_memory_usage,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_restart(void) {
  const esp_console_cmd_t cmd = {
    .command = "restart",
    .help = "Software reset of the SOC",
    .hint = NULL,
    .func = &restart,
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}



// log_level command changes log level via esp_log_level_set

static struct {
    struct arg_str *tag;
    struct arg_str *level;
    struct arg_end *end;
} log_level_args;

static const char *s_log_level_names[] = {
    "none", "error", "warn", "info", "debug", "verbose"
};


static int log_level(int argc, char **argv) {
  int nerrors = arg_parse(argc, argv, (void **)&log_level_args);
  if (nerrors != 0) {
    arg_print_errors(stderr, log_level_args.end, argv[0]);
    return 1;
  }
  assert(log_level_args.tag->count == 1);
  assert(log_level_args.level->count == 1);
  const char *tag = log_level_args.tag->sval[0];
  const char *level_str = log_level_args.level->sval[0];
  esp_log_level_t level;
  size_t level_len = strlen(level_str);
  for (level = ESP_LOG_NONE; level <= ESP_LOG_VERBOSE; level++) {
    if (memcmp(level_str, s_log_level_names[level], level_len) == 0) {
      break;
    }
  } //rof
  if (level > ESP_LOG_VERBOSE) {
    printf("Invalid log level '%s', choose from none|error|warn|info|debug|verbose\n", level_str);
    return 1;
  }
  if (level > CONFIG_LOG_MAXIMUM_LEVEL) {
    printf("Can't set log level to %s, max level limited in menuconfig to %s. "
      "Please increase CONFIG_LOG_MAXIMUM_LEVEL in menuconfig.\n",
      s_log_level_names[level], s_log_level_names[CONFIG_LOG_MAXIMUM_LEVEL]);
    return 1;
  }
  esp_log_level_set(tag, level);
  return 0;
}

static void register_log_level(void) {
  log_level_args.tag   = arg_str1(NULL, NULL, "<tag|*>", "Log tag to set the level for, or * to set for all tags");
  log_level_args.level = arg_str1(NULL, NULL, "<none|error|warn|debug|verbose>", "Log level to set. Abbreviated words are accepted.");
  log_level_args.end   = arg_end(2);

  const esp_console_cmd_t cmd = {
    .command = "log_level",
    .help = "Set log level for all tags or a specific tag.",
    .hint = NULL,
    .func = &log_level,
    .argtable = &log_level_args};
  ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}


void register_main_console_cmd(void) {
  register_free();
  register_chipinfo();
  register_restart();
  register_log_level();
}



void ntp_time_notifier(int now) {
  printf("time has changed to %d\n", now);
}




#include "C2LORA_modes.h"

static void print_mode_delay_table(void) {
  for (tC2LORA_mode m = C2M_MIN; m <= C2M_MAX; m++) {
     // hint: all byte lengths are 1 byte short, because we need the time before buffer on length-pos is read by SX126x
     const tC2LORAdef *def = C2LORA_get_parameter4mode(m);     
     uint32_t symbol_time  = c2lora_calc_symboltime(def);
     uint32_t frame_bytes  = (sf_get_subframe_bits(def->codec_type) * def->dv_frames_per_packet + 7) >> 3; // bytes till all DV is xmited
     uint32_t header_time  = c2lora_calc_firstprefetch(def, symbol_time, def->bytes_per_header, true);
     uint32_t frame_time   = c2lora_calc_firstprefetch(def, symbol_time, frame_bytes, false);
     uint32_t hframe_time  = c2lora_calc_firstprefetch(def, symbol_time, def->bytes_per_header + frame_bytes, true);
     uint32_t fframe_time  = c2lora_calc_firstprefetch(def, symbol_time, def->bytes_per_packet, false);
     printf("C2M%d %s:\n", m, C2LORA_get_mode_name(m));
     printf("symbol time: %luµs\t\ttransmission time: %luµs (%d bytes)\n", symbol_time, fframe_time, def->bytes_per_packet);
     printf("time to first DV is needed: %luµs (header pkt)\n", header_time);
     printf("time to last DV is needed : %luµs (header pkt)\t %ldµs to start\n", hframe_time, (int32_t)480000 - (int32_t)hframe_time);
     printf("time to last DV is needed : %luµs (frame pkt with %lu speech bytes)\n", frame_time, frame_bytes);
     // caluclate:
     uint32_t max_time_last = frame_time - symbol_time - (symbol_time >> 1); // minus 1.5x symbol time so that is a safty margin fort correction next frame
     uint32_t min_time_last = max_time_last - ( 4 * symbol_time);
     printf("calc sfe parameter: %lums..%lums\n", (min_time_last) / 1000,  (max_time_last) / 1000);
     printf("mode parameter set: %dms..%dms\n\n", 480-def->min_finish_ms + (def->dv_frames_per_packet==24? 20:40), 480-def->max_finish_ms + (def->dv_frames_per_packet==24? 20:40));
     // caculate time needed in the first fram
  } // rof all.
}


static void print_c2lora_info(bool with_table) {
  int freq_hz  = C2LORA_get_frequency();
  int freq_sft = C2LORA_get_freqshift();
  tC2LORA_mode mode = C2LORA_get_mode();
  double freqf = (double)freq_hz / 1000000;
  double fshftf = (double)freq_sft / 1000000;
  C2LORA_print_status();
  printf("tuned to %8.4fMHz (%+6.4f TX), mode: C2M%d %s\n\n", freqf, fshftf, mode, C2LORA_get_mode_name(mode));
  if (with_table) {
     print_mode_delay_table();
  }
}


static int32_t parse_frequency_mhz(const char *freq_str) {
  int32_t freq_hz = 0;
  char *fraction_str;
  long freq_mhz = strtol(freq_str, &fraction_str, 10);
  if ((fraction_str != freq_str) && (fraction_str[0] != 0)) {
    if (fraction_str[0] != '.') {
      printf("invalid frequency format (xxx.y{z} MHz)\n");
      return 0x80000000;
    }
    fraction_str++;    
    for (int p=0; p < 6; p++) {
      freq_hz *= 10;
      if ((fraction_str[0] >= '0') && (fraction_str[0] <= '9')) {
        freq_hz += fraction_str[0] - '0';
        fraction_str++;
      } // fi
    }
  } // fi 
  if (freq_mhz >= 0) {
    freq_hz += freq_mhz * 1000000L;
  } else {
    freq_hz = (freq_mhz * 1000000L) - freq_hz;
  }
  return freq_hz;
}


static void set_c2lora_frequency(int argc, const char **argv) {
  int32_t freq_hz = parse_frequency_mhz(argv[0]);
  int32_t freq_sft = 0;
  if (freq_hz == 0x80000000) {
    printf("invalid frequency format (xxx.y{z} MHz)\n");
    return;
  }
  if (argc > 1) {
    freq_sft = parse_frequency_mhz(argv[1]);
    if (freq_sft == 0x80000000) {
      printf("invalid frequency shift format (-/+x.y{z} MHz)\n");
      return;
    }
  }
  if (((freq_hz + freq_sft) >= C2LORA_MIN_FREQ_HZ) && ((freq_hz + freq_sft) <= C2LORA_MAX_FREQ_HZ)) {
    printf("set frequency to %luHz %+ldHz\n\n", freq_hz, freq_sft);
    C2LORA_set_frequency(freq_hz, freq_sft);
  } else { // fi
    printf("frequency out of bounds.\n");
  }
}


static void set_c2lora_mode(const char *arg) {
  tC2LORA_mode mode = C2LORA_get_mode_by_name(arg);
  if ((mode >= C2M_MIN) && (mode <= C2M_MAX)) {
    printf("set mode to #%d [%s]\n", mode, C2LORA_get_mode_name(mode));
    C2LORA_set_mode(mode);
  }
}


static void set_c2lora_txpower(const char *arg) {
  int power_dBm = str_readnum(arg, -128, -14, 40);
  if (power_dBm == -128) {
    printf("tx power out of bounds (-14 to 40dBm)\n");
    return;
  }
  C2LORA_set_txpower(power_dBm);
}


static struct {
    struct arg_str *sel;
    struct arg_dbl *val;
    struct arg_end *end;
} c2lora_args;


static int c2lora_console(int argc, char **argv) {
  const char *c2lora_cmd_list[] = { "info", "freq", "mode", "txpower", "receive", "cali", "offset", "save", "retransmit", "reg", "pertest", NULL };
  const char *selection = argv[1];

  if (argc < 2) return 1;
  int cmd_index = str_index(selection, c2lora_cmd_list);
  if (cmd_index < 0) return 1;

  switch (cmd_index) {
  case 0: // info
    print_c2lora_info(argc > 2? true: false);
    break;
  case 1:
    if (argc < 3) break;
    set_c2lora_frequency(argc - 2, (const char **) &argv[2]);
    break;
  case 2:
    if (argc != 3) break;
    set_c2lora_mode(argv[2]);
    break;
  case 3:   // txpower
    if (argc != 3) break;
    set_c2lora_txpower(argv[2]);
    break;
  case 4:   // receive:
    if (argc < 3) break;
    C2LORA_start_continuous_rx(str_getOnOff(argv[2]));
    break;
  case 5:   // unmodulated carrier / continuous wave for frequency calibration
    if (argc < 3) break;
    C2LORA_set_calibration(str_getOnOff(argv[2]), argc > 3? str_getOnOff(argv[3]): false);
    break;
  case 6:
    if (argc < 3) break;
    C2LORA_set_freq_offset(str_readnum(argv[2], 0, -32000, 32000), argc > 3? str_getOnOff(argv[3]): false);
    break;
  case 7: // save
    C2LORA_save_config();
    break;  
  case 8:
    printf("transmitting last HAMdLNK reception...\n");
    if (C2LORA_retransmit_last() == ESP_OK) {
      printf("transmit done.\n");
    } else {
      printf("transmit failed.\n");
    }
    break;  
 
  case 9: // reg(ister dump)
    if (argc >= 3) {
      int addr = strtol(argv[2], NULL, 0);	// auto base
      int val  = 0;
      if ((addr < 0) || (addr > 0xFFFF)) {
        printf("invalid address (%08xh)\n", addr);
        break;
      }
      if (argc == 4) {
        val = strtol(argv[3], NULL, 0);
        if ((val < -128) || (val > 255)) {
          printf("invalid value (%04xh)\n", val);
          break;
        }
        C2LORA_write_register_to(addr, val);
      } else {
        C2LORA_read_registers_from(addr);
      }
    }
    break;  

  case 10:  // pertest
    printf("performing PER test...\n");
    printf("test result: %s\n" , esp_err_to_name(C2LORA_perform_PER_test(argc > 2? str_readnum(argv[2], 10, 1, 5000): 10, argc > 3? str_readnum(argv[3], 20000, 100, 50000): 20000) ));
    break; 
  } // hctiws

  return 0;
}


static void register_c2lora(void) {
  c2lora_args.sel = arg_str1(NULL, NULL, "<info|freq|mode|txpower|receive|cali|offset|save|retransmit>", "change frequency or mode, save settings");
  c2lora_args.val = arg_dbln(NULL, NULL, "<value>", 0, 4, "frequency [MHz] | mode 0..9 | calibration on/off");
  c2lora_args.end = arg_end(2);
  const esp_console_cmd_t cmd = {
    .command = "c2lora",
    .help = "Set parameter (frequency, mode) for UHF operation",
    .hint = NULL,
    .func = &c2lora_console,
    .argtable = &c2lora_args};
  ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}



static esp_err_t set_c2lora_header(void) {
  esp_err_t  err, loc_err = ESP_FAIL, area_err = ESP_FAIL;
  nvs_handle hnd;
  uint64_t callsign    = 0;
  uint64_t destination = 0;
  uint64_t locator     = 0;   // maidenhead c2grid sqare location (6 characters)
  uint16_t area_code   = 0;
  uint8_t  kos = KOS_HOTSPOT;
#if CONFIG_KOS_PORTABLE
  kos = KOS_PORTABLE;
#elif CONFIG_KOS_MOBILE
  kos = KOS_MOBILE;
#elif CONFIG_KOS_STATION
  kos = KOS_STATIONARY;
#elif CONFIG_KOS_RELAY
  kos = KOS_RELAY;
#endif
  memcpy(&callsign, CONFIG_DEFAULT_CALLSIGN, sizeof(CONFIG_DEFAULT_CALLSIGN));
  memcpy(&destination, CQCALL_DESTINATION, sizeof(CQCALL_DESTINATION));
  err = nvs_open(APP_namespace, NVS_READONLY, &hnd);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "can't open APP NVS");
  } else {
    err = nvs_get_u64(hnd, APP_callsign, &callsign);  
    err |= nvs_get_u64(hnd, APP_recipient, &destination);
    err |= nvs_get_u8(hnd, APP_kindofsender, &kos);
    area_err = nvs_get_u16(hnd, APP_areacode, &area_code);
    loc_err  = nvs_get_u64(hnd, APP_locator, &locator);
    nvs_close(hnd);
    if (err) {
      ESP_LOGW(TAG, "not all parameter loaded (%s)", esp_err_to_name(err));
    }    
  }
  C2LORA_set_local_header((const char *) &callsign, (const char *) &destination, (tKindOfSender) kos);
  if ((area_err == ESP_OK) ) {
    C2LORA_set_local_header_additionals(area_code, loc_err == ESP_OK? (const char *) &locator: NULL);
  }
  hamdlnk_set_sender_info((const char *) &callsign, (const char *) &destination, area_code, loc_err == ESP_OK? (const char *) &locator: NULL);
  return err;
}


static tStartupMode get_app_config_from_nvs(tHAMdLNK_network_config *net_cfg) {
  esp_err_t  err;
  nvs_handle hnd;
  uint8_t mode = SM_STANDBY;
  err = nvs_open(APP_namespace, NVS_READONLY, &hnd);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "can't open APP NVS");
  } else {
    const char *sendto_host;
    uint64_t locator;
    uint16_t area_code;
    int8_t ivalue;

    err = nvs_get_u8(hnd, APP_startupmode, &mode);
    if (err != ESP_OK) {
      mode = SM_STANDBY;
      ESP_LOGW(TAG, "startup mode in NVS (%s)", esp_err_to_name(err));
    }
    sendto_host = nvs_get_string(hnd, APP_sendto_host, 256);
    if (sendto_host != NULL) {
      net_cfg->sendto_hostname_uhf = sendto_host;
    }
    sendto_host = nvs_get_string(hnd, APP_sendto_host2, 256);
    if (sendto_host != NULL) {
      net_cfg->sendto_hostname_dongle = sendto_host;
    } else {
      net_cfg->sendto_hostname_dongle = net_cfg->sendto_hostname_uhf; // same host
    }

    err = nvs_get_u16(hnd, APP_areacode, &area_code);
    net_cfg->area_code = (err == ESP_OK)? area_code: 0;
    err = nvs_get_u64(hnd, APP_locator, &locator);
    if (err == ESP_OK) memcpy(net_cfg->locator, &locator, sizeof(net_cfg->locator));
    
    err = nvs_get_i8(hnd, APP_volume, &ivalue);
    if (err == ESP_OK) localaudio_set_volume_dBm4(ivalue);
    err = nvs_get_i8(hnd, APP_micgain, &ivalue);
    if (err == ESP_OK) localaudio_set_micgain_dBm4(ivalue);

    nvs_close(hnd);
  } // fi have NVS

#if CONFIG_LOCALAUDIO_ENABLED
  if (mode == SM_DONGLE) {
    net_cfg->listen_port_dongle     = net_cfg->listen_port_uhf;
    net_cfg->sendto_hostname_dongle = net_cfg->sendto_hostname_uhf;
    net_cfg->sendto_port_dongle     = net_cfg->sendto_port_uhf;
    net_cfg->sendto_port_uhf = 0; // disable UHF port
  }
#else
  net_cfg->listen_port_dongle = 0;  // disable 2nd port for local audion
#endif
  ESP_LOGD(TAG, "startup = %d", mode);
  return (tStartupMode) mode;
}


static tStartupMode get_startup_from_nvs(void) {
  esp_err_t  err;
  nvs_handle hnd;
  uint8_t    mode = SM_OFFLINE;
  err = nvs_open(APP_namespace, NVS_READONLY, &hnd);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "can't open APP NVS");
  } else {   
    err = nvs_get_u8(hnd, APP_startupmode, &mode);
    if (err != ESP_OK) {
      mode = SM_STANDBY;
      ESP_LOGW(TAG, "startup mode in NVS (%s)", esp_err_to_name(err));
    }
    
    nvs_close(hnd);
  }
  return (tStartupMode) mode;
}


static void store_startup_mode(tStartupMode mode) {
  esp_err_t  err;
  nvs_handle hnd;
  err = nvs_open(APP_namespace, NVS_READWRITE, &hnd);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "can't open APP NVS");
  } else {
    err = nvs_set_u8(hnd, APP_startupmode, mode);
    nvs_close(hnd);
    ESP_LOGI(TAG, "startup mode (%d) stored.", mode);
  } 
  if (err) {
    ESP_LOGW(TAG, "startup mode not stored (%s)", esp_err_to_name(err));
  }
}


static void store_local_callsign(const char *value) {
  esp_err_t  err;
  nvs_handle hnd;
  uint64_t callsign = 0;
  memcpy(&callsign, value, strnlen(value, 8));
  err = nvs_open(APP_namespace, NVS_READWRITE, &hnd);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "can't open APP NVS");
  } else {
    err = nvs_set_u64(hnd, APP_callsign, callsign);  
    nvs_close(hnd);
  } 
  if (err) {
    ESP_LOGW(TAG, "callsign not stored (%s)", esp_err_to_name(err));
  }
}

static void store_local_recipient(const char *value) {
  esp_err_t  err;
  nvs_handle hnd;
  uint64_t destination = 0;
  memcpy(&destination, value, strnlen(value, 8));
  err = nvs_open(APP_namespace, NVS_READWRITE, &hnd);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "can't open APP NVS");
  } else {
    err = nvs_set_u64(hnd, APP_recipient, destination);
    nvs_close(hnd);
  } 
  if (err) {
    ESP_LOGW(TAG, "destination not stored (%s)", esp_err_to_name(err));
  }
}


static bool store_local_kos(tKindOfSender kos) {
  esp_err_t  err;
  nvs_handle hnd;
  err = nvs_open(APP_namespace, NVS_READWRITE, &hnd);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "can't open APP NVS");
  } else {
    err = nvs_set_u8(hnd, APP_kindofsender, (uint8_t)kos);
    nvs_close(hnd);
  }
  if (err) {
    ESP_LOGW(TAG, "kind-of-sender not stored (%s)", esp_err_to_name(err));
  }
  return err == ESP_OK;
}


static bool store_local_areacode(const char *value) {
  esp_err_t  err;
  nvs_handle hnd;
  uint16_t   areacode = C2LORA_pack_areacode(value);
  if ((areacode == 0) || (areacode == 0xFFFF)) {
    ESP_LOGW(TAG, "invalid area code (not 1-3 letters).");
    return false;
  }
  err = nvs_open(APP_namespace, NVS_READWRITE, &hnd);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "can't open APP NVS");
  } else {
    err = nvs_set_u16(hnd, APP_areacode, areacode);
    nvs_close(hnd);
  } 
  if (err) {
    ESP_LOGW(TAG, "areacode not stored (%s)", esp_err_to_name(err));
  }
  return err == ESP_OK;
}


static void store_local_locator(const char *value) {
  esp_err_t  err;
  nvs_handle hnd;
  uint64_t locator = 0;
  memcpy(&locator, value, strnlen(value, 8));
  err = nvs_open(APP_namespace, NVS_READWRITE, &hnd);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "can't open APP NVS");
  } else {
    err = nvs_set_u64(hnd, APP_locator, locator);
    nvs_close(hnd);
  } 
  if (err) {
    ESP_LOGW(TAG, "locator not stored (%s)", esp_err_to_name(err));
  }
}


static void store_volume(signed char volume) {
  esp_err_t  err;
  nvs_handle hnd;
  err = nvs_open(APP_namespace, NVS_READWRITE, &hnd);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "can't open APP NVS");
  } else {
    err = nvs_set_i8(hnd, APP_volume, volume);
    nvs_close(hnd);
  } 
  if (err) {
    ESP_LOGW(TAG, "volume not stored (%s)", esp_err_to_name(err));
  }
}


static void store_micgain(signed char micgain) {
  esp_err_t  err;
  nvs_handle hnd;
  err = nvs_open(APP_namespace, NVS_READWRITE, &hnd);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "can't open APP NVS");
  } else {
    err = nvs_set_i8(hnd, APP_micgain, micgain);
    nvs_close(hnd);
  } 
  if (err) {
    ESP_LOGW(TAG, "micgain not stored (%s)", esp_err_to_name(err));
  }
}


static bool store_udptarget(const char *target) {
  esp_err_t  err;
  nvs_handle hnd;
  if ((target == NULL) || (strnlen(target, 3) < 2)) return false;

  err = nvs_open(APP_namespace, NVS_READWRITE, &hnd);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "can't open APP NVS");
  } else {
    err = nvs_set_str(hnd, APP_sendto_host, target);
    nvs_close(hnd);
  } 
  if (err) {
    ESP_LOGW(TAG, "UDP target not stored (%s)", esp_err_to_name(err));
  }  
  return err == ESP_OK;
}


#if CONFIG_LOCAL_PTT_ENABLED

static struct {
    struct arg_str *sel;
    struct arg_str *val;
    struct arg_end *end;
} senderinfo_args;


static void print_header_additionals(void) {
  uint16_t area_code;
  char     locator_str[8];
  char     area_code_str[4];
  if (C2LORA_get_local_header_additionals(&area_code, locator_str) == ESP_OK) {
    C2LORA_unpack_areacode(area_code, area_code_str);
    printf("                areacode '%.3s' %04x locator '%s'\n", area_code_str, area_code, locator_str);
  }
}


static const char *local_cmd_list[] = { 
  "info", "startup", "callsign", "recipient", "KoS", "areacode", "locator", "hostname",
  "volume", "micgain", 
  "udptarget", 
  "oledtest",
  NULL 
};
static const char *startup_modes[]  = { "OFFLINE", "DONGLE", "STANDBY", "ONAIR", "NOWIFI", NULL };

static int localset_console(int argc, char **argv) {
  tKindOfSender kos;
  bool success    = false;
  bool cmd_failed = true;

  int ivalue = -1;
  int cmd_index = str_index(argv[1], local_cmd_list);
  const char *value = argc >2? argv[2]: "";

  if (cmd_index < 0) return 1;
  switch (cmd_index) {
  case 0: // info
    cmd_failed = false;
    break;
  case 1: //startup mode
    ivalue = str_index(value, startup_modes);
    if (ivalue < 0) {
      printf("can't find a mode '%s'. Ignored.\n", value);
    } else {
      printf("new startup after restart: %s\n", startup_modes[ivalue]);
      store_startup_mode((tStartupMode) ivalue);
      cmd_failed = false;
    }
    break;
  case 2: // set callsign
    success = Check_Call(value, strnlen(value, 8));
    if (success) {
      store_local_callsign(value);
    }
    break;
  case 3: // destination
    success = strnlen(value, 8) > 3;
    if (success) {
      store_local_recipient(value);
    }
    break;
  case 4: // KoS
    kos = C2LORA_get_kos_byname(value);
    success = kos != KOS_UNDEF;
    if (success) {
      success = store_local_kos(kos);
    }
    break;
  case 5: // areacode
    success = store_local_areacode(value);
    break; 
  case 6: // locator
    success = Check_Locator(value, strnlen(value, 8)); // locator in the form XYNNZA (maidenhead grid square)
    if (success) {
      store_local_locator(value);
    }
    break;
  case 7: // hostname
    if (strnlen(value, 5) > 3) {
      LAN_set_hostname(value);
      cmd_failed = false;
    } else {
      printf("hostname should be longer than 3 characters.\n");
      cmd_failed = true;
    }
    break;  
  case 8: // volume
    if (str_getnumber(&ivalue, value) == 0) {
      if ((ivalue < -120) || (ivalue > 120)) {
        printf("volume out of bounds (-120..120)\n");
        break;
      }
      store_volume(ivalue);
      localaudio_set_volume_dBm4(ivalue);
      cmd_failed = false;
    } else {
      printf("volume must be a number (-120..120)\n");
    }
    break;
  case 9: // micgain
    if (str_getnumber(&ivalue, value) == 0) {
      if ((ivalue < -120) || (ivalue > 120)) {
        printf("micgain out of bounds (-120..120)\n");
        break;
      }
      store_micgain(ivalue);
      localaudio_set_micgain_dBm4(ivalue);
      cmd_failed = false;
    } else {
      printf("micgain must be a number (-120..120)\n");
    }
    break;
  case 10: // udptarget
    if (store_udptarget(value)) {
      printf("UDP target '%s' stored.\n", value);
      cmd_failed = false;
    }
    break;  
  case 11:  // oled test
    OLED_InverseDisp(str_getOnOff(value));
    break;  
  } // hctiws

  if (success) {
    printf("%s updated.\n", local_cmd_list[cmd_index]);
    set_c2lora_header();
    if (cmd_index >= 4) print_header_additionals();
  } else if (cmd_failed) {
    printf("%s invalid. No update\n", local_cmd_list[cmd_index]);
  }
  return 0;
}


#include "lwip/api.h"
#include "lwip/dns.h"

static void my_dns_found_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg) {
  char ip_addr_str[128];
  ip_addr_str[0] = 0;
  if (ipaddr != NULL) {
    ipaddr_ntoa_r(ipaddr, ip_addr_str, sizeof(ip_addr_str));
    printf("got IP: %s for '%s'\n", ip_addr_str, name != NULL? name: "---");
  } else {
    printf("got no IP (null).\n");
  }
}



static void localget_hostname(const char *hostname) {
  ip_addr_t ip_addr_cached;
  const char *my_hostname = NULL;
  if (hostname == NULL) {
    my_hostname = LAN_get_hostname();
    printf("my hostname: %s\n", my_hostname);
    hostname = my_hostname;
  }
  if (dns_gethostbyname(hostname, &ip_addr_cached, my_dns_found_callback, NULL) == ERR_OK) {
    my_dns_found_callback(hostname, &ip_addr_cached, NULL);
  }
}


static int localget_console(int argc, char **argv) {
  char callsign_buf[8];
  char dest_buf[8];
  tKindOfSender kos;
  tStartupMode mode;
  int nerrors = arg_parse(argc, argv, (void **)&senderinfo_args);
  if (nerrors != 0) {
    arg_print_errors(stderr, senderinfo_args.end, argv[0]);
    return 1;
  }
  assert(senderinfo_args.sel->count == 1);
  const char *selection = senderinfo_args.sel->sval[0];
  const char *value = NULL;
  if ((senderinfo_args.val != NULL) && (senderinfo_args.val->count == 1)) {
    value = senderinfo_args.val->sval[0];
  }
  int cmd_index = str_index(selection, local_cmd_list);
  switch (cmd_index) {
  case 0: // info
    if (C2LORA_get_local_header(callsign_buf, dest_buf, &kos) == ESP_OK) {
      printf("local PTT info: callsign '%7.7s' recipient '%7.7s' kind of station: %s\n", callsign_buf, dest_buf, C2LORA_get_kos_name(kos));
      print_header_additionals();
    } else {
      printf("no vaild header stored for local PTT.\n");
    }

    print_mode_delay_table();

    break;
  case 1: //startup mode
    mode = get_startup_from_nvs();
    if (mode <= SM_NOWIFI) {
      printf("startup mode: %s\n", startup_modes[mode]);
    } else {
      printf("startup moide invalid (%d)\n", mode);
    }
    break;
  case 2: // callsign
    break;
  case 3: // destination
    break;
  case 4: // KoS
    break;
  case 5: // areacode
    break; 
  case 6: // locator
    break;
  case 7: // hostname
    localget_hostname(value);
    break;  
  case 8: // volume
    printf("volume = %6.2fdB\n", localaudio_get_volume_dBm());    
    break;
  case 9: // micgain
    printf("micgain = %6.2fdB\n", localaudio_get_micgain_dBm());    
    break;
  } // hctiws

  return 0;
}


static void register_localsetfunct(void) {
  senderinfo_args.sel = arg_str1(NULL, NULL, "<info|startup|callsign|recipient|KoS|areacode|locator|hostname|volume|micgain|udptarget>", "change information transmitted if PTT is used.");
  senderinfo_args.val = arg_str0(NULL, NULL, "<value>", "callsign, recipient, ...");
  senderinfo_args.end = arg_end(2);

  const esp_console_cmd_t cmd_set = {
    .command = "set",
    .help = "Set callsign, recipient and more for local transmission",
    .hint = NULL,
    .func = &localset_console,
    .argtable = &senderinfo_args};
  ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_set));
  const esp_console_cmd_t cmd_get = {
    .command = "get",
    .help = "Get information",
    .hint = NULL,
    .func = &localget_console,
    .argtable = &senderinfo_args};
  ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_get));
}

#endif // local PTT



static bool ptt_button_state(bool go_active) {
  int io_pin_level = gpio_get_level(CONFIG_LOCAL_PTT_IOPIN);
  if (io_pin_level==(go_active? 0: 1)) { // Repeat a 2nd time, 10ms later
    vTaskDelay(1);
    io_pin_level = gpio_get_level(CONFIG_LOCAL_PTT_IOPIN);
  } // fi
// ESP_LOGD(TAG, "PTT: %s", io_pin_level==0? "ON": "OFF");
  return io_pin_level==0? true: false;
}


static esp_err_t board_gpio_init(void) {
#if (defined BOARD_POWER_ON_Pin) && (BOARD_POWER_ON_Pin != GPIO_NUM_NC)
  gpio_set_level(BOARD_POWER_ON_Pin, 0);
  gpio_set_direction(BOARD_POWER_ON_Pin, GPIO_MODE_OUTPUT);
#if (defined BOARD_5V_POWERMODE_Pin) && (BOARD_5V_POWERMODE_Pin != GPIO_NUM_NC)
  gpio_set_level(BOARD_5V_POWERMODE_Pin, 1);  // no power-save
  gpio_set_direction(BOARD_5V_POWERMODE_Pin, GPIO_MODE_OUTPUT);
#endif
#if (defined SX126X_RST_Pin) && (SX126X_RST_Pin != GPIO_NUM_NC)
  gpio_set_level(SX126X_RST_Pin, 0);         // keep Lora modul in reset while powering up
  gpio_set_direction(SX126X_RST_Pin, GPIO_MODE_OUTPUT);
#endif
  gpio_set_level(BOARD_POWER_ON_Pin, 1);      // turn on power now
#endif

#if CONFIG_LOCAL_PTT_IOPIN != GPIO_NUM_NC
  gpio_set_direction(CONFIG_LOCAL_PTT_IOPIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(CONFIG_LOCAL_PTT_IOPIN, GPIO_PULLUP_ONLY);
#endif

  ESP_RETURN_ON_ERROR(gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL2), TAG, "can't install GPIO ISR service");
  return ESP_OK;
}


#if (defined I2C_SDA_Pin) && (I2C_SDA_Pin != GPIO_NUM_NC)

#include "driver/i2c.h"

#ifndef I2C_CLK_SPEED_HZ
#define I2C_CLK_SPEED_HZ  400000L
#endif

static esp_err_t board_i2c_init(void) {
  const i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_SDA_Pin,
    .scl_io_num = I2C_SCL_Pin,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_CLK_SPEED_HZ
  };
  i2c_param_config(I2C_NUM_0, &conf);
  return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0); 
}

#endif



void app_main(void) {
  esp_err_t ret;
  tStartupMode startup_mode = SM_STANDBY;

  tHAMdLNK_network_config ham2lnk_config = ham2lnk_default_config;

  esp_console_repl_t *      repl        = NULL;
  esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
  strncpy(cmd_prompt, PROMPT_STR ">", sizeof(cmd_prompt));    // Prompt to be printed before each line.
  repl_config.prompt = cmd_prompt;
  repl_config.max_cmdline_length = 128;

  app_set_default_debuglevel();

  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_event_loop_create_default());	// create default system event loop

  fs_open();

  board_gpio_init();   // initialize important GPIO together at start.

#if (defined I2C_SDA_Pin) && (I2C_SDA_Pin != GPIO_NUM_NC)
  ret = board_i2c_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "*** I2C init fail ***");
    stop_boot_with_fail(BLINK_PATTERN_NOI2C);
  }
#endif

// Todo load all in om funcion
  startup_mode = get_app_config_from_nvs(&ham2lnk_config);

  if (startup_mode <= SM_NOWIFI) {
    printf("startup mode: %s\n", startup_modes[startup_mode]);
  }

#if (defined BOARD_POWER_ON_Pin) && (BOARD_POWER_ON_Pin != GPIO_NUM_NC)
  vTaskDelay(pdMS_TO_TICKS(150));  // wait powering up 5V...
#endif
  // *** Enable S_SPI Module ***
  if (C2LORA_init_spi() != ESP_OK) {
    ESP_LOGE(TAG, "SPI 4 SX126x init fails");
    stop_boot_with_fail(BLINK_PATTERN_NOSPI);
  }

#if CONFIG_LOCALAUDIO_ENABLED
  if (localaudio_start() != ESP_OK) {
    ESP_LOGE(TAG, "*** no audio - no task! ***");
    stop_boot_with_fail(BLINK_PATTERN_NOAUDIO);
  }
#endif

  // *** Enable WiFi Module ***
  if ((startup_mode < SM_NOWIFI) && (startup_mode > SM_OFFLINE)) {
    if (LAN_init() != ESP_OK) {
      ESP_LOGE(TAG, "WiFi init fails");
      stop_boot_with_fail(BLINK_PATTERN_NOWIFI);
    }
    // update command prompt with hostname:
    snprintf(cmd_prompt, sizeof(cmd_prompt), "%s>", LAN_get_hostname());
    ESP_LOGD(TAG, "new promt: '%s'", cmd_prompt);
  } // fi wifi


#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
  esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
  esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
  esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));

#else
#error Unsupported console type
#endif

  // Register commands
  esp_console_register_help_command();
  register_main_console_cmd();

  register_localsetfunct();

  if (startup_mode > SM_DONGLE) {
    ret = C2LORA_init_tranceiver(SX126X_DEVICE_NUM, CONFIG_DEFAULT_FREQUENCY_HZ, CONFIG_DEFAULT_C2LORA_MODE, SX126X_FREQUENCY_OFFSET);
    if (ret == ESP_OK) {
      register_c2lora();   
#ifdef SX126X_2_DEVICE_NUM      
      C2LORA_init_tranceiver_2(SX126X_2_DEVICE_NUM, SX126X_2_FREQUENCY_OFFSET);
#endif
      if (startup_mode >= SM_ONAIR) {
        C2LORA_start_continuous_rx(true);
      }
    } else {
      ESP_LOGE(TAG, "*** no UHF link! ***");
      stop_boot_with_fail(BLINK_PATTERN_NOC2LORA);
    }
  } else { // fi not DONGLE
    C2LORA_init_donglemode(CONFIG_DEFAULT_C2LORA_MODE);
  }

  set_c2lora_header();

  // *** Start WiFi Station Mode OR Ethernet ***
  if ((startup_mode < SM_NOWIFI) && (startup_mode > SM_OFFLINE)) {
    ret = LAN_start();
    if (ret == ESP_OK) {
      ntp_startclienttask(INI_get_NTPurl(NTP_DEFAULT_URL), ntp_time_notifier);

      if (C2LORA_start_udp_task(&ham2lnk_config) != ESP_OK) {
        ESP_LOGE(TAG, "*** can't listen to UDP-RTP HAMdLNK - no task. ***");
        stop_boot_with_fail(BLINK_PATTERN_NOHAMDLNK);
      }

    } else {    
      ESP_ERROR_CHECK(ret);
    }    
  }

  ret = ui_Init();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "NO UI! Init of user interface (UI) failed.");
  }

  vTaskDelay(2); // wait for ESP_LOG messages (trom running tasks)
  for (int l=0; l < ASCIIartlines; l++) {
    printf("%s\n", ASCIIartTitle[l]);
  } // rof

  ESP_ERROR_CHECK(esp_console_start_repl(repl));

  if (ret == ESP_OK) { // last result from ui_Init()
    ret = ui_Start();
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Start of user interface (UI) failed.");
    }
  } // fi ok

  while (startup_mode == SM_DONGLE) {
    if (ptt_button_state(true)) {
      ESP_LOGI(TAG, "*** start streaming ***");
      ret = ham2lnk_broadcast_audio(C2LORA_get_codec_type(), 240, true, "");
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "*** error streaming *** (%s)", esp_err_to_name(ret));
        break;    
      }
      do {
        vTaskDelay(pdMS_TO_TICKS(60));
      } while (ptt_button_state(false));
      ESP_LOGI(TAG, "*** end streaming ***");
      ham2lnk_broadcast_stop();
    } // fi tx HAMdLNK
    vTaskDelay(10);
  } // ehliw dongle operation

#if CONFIG_LOCAL_PTT_ENABLED
  while (1) {
    if (ptt_button_state(true)) {
      ESP_LOGI(TAG, "*** start transmitting ***");
      ret = C2LORA_start_tx_microphone();
      if (ret != ESP_OK) {  
        ESP_LOGE(TAG, "*** error transmitting *** (%s)", esp_err_to_name(ret));
        break;    
      }
//      ESP_LOGI(TAG, "high stack water mark is %u\n", uxTaskGetStackHighWaterMark(NULL));
      do {
        vTaskDelay(pdMS_TO_TICKS(120));
      } while (ptt_button_state(false));

      ESP_LOGI(TAG, "*** end transmitting ***");
      C2LORA_end_tx_microphone();
    } // fi tx local
    vTaskDelay(10);
  } // ehliw foirever
#endif
  
  fs_close(); 

} // END main
