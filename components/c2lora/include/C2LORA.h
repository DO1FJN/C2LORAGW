
/*

C2LORA.h

This header file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

main API file for accessing the C2LORA UHF link

*/

#pragma once

#include "esp_err.h"
#include <stdbool.h>

#include "sdkconfig.h"

#ifdef CONFIG_LOW_FREQUENCY_RANGE_MHZ
#define C2LORA_MIN_FREQ_HZ            (CONFIG_LOW_FREQUENCY_RANGE_MHZ * 1000000L)
#endif
#ifdef CONFIG_HIGH_FREQUENCY_RANGE_MHZ
#define C2LORA_MAX_FREQ_HZ            (CONFIG_HIGH_FREQUENCY_RANGE_MHZ * 1000000L)
#endif

#ifndef C2LORA_MIN_FREQ_HZ
#define C2LORA_MIN_FREQ_HZ            430000000L
#endif

#ifndef C2LORA_MAX_FREQ_HZ
#define C2LORA_MAX_FREQ_HZ            440000000L
#endif

#define C2LORA_MIN_OFFSET_HZ          (-32000)
#define C2LORA_MAX_OFFSET_HZ          (+32000)

#define CQCALL_DESTINATION            "CQCQCQ"


typedef enum {
  C2M0_10LR,
  C2M1_15LR, C2M2_15STD, C2M3_15MQ,
  C2M4_15HQ,
  C2M5_20LR, C2M6_20, 
  C2M7_31STD, C2M8_31LL, C2M9_31HQ, C2MA_7NB, C2MB_WIDE
} tC2LORA_mode;

#define C2M_MIN C2M0_10LR
#define C2M_MAX C2MB_WIDE  //C2M9_31HQ

typedef enum {
  KOS_UNDEF, KOS_PORTABLE, KOS_MOBILE, KOS_STATIONARY, KOS_HOTSPOT, KOS_RELAY
} tKindOfSender;

typedef enum {
  C2S_OFF, C2S_STANDBY, C2S_CAL, C2S_RX, C2S_TX
} tC2LORA_state;


typedef struct sC2LORAdef tC2LORAdef;
typedef struct sLoraStream tLoraStream;


esp_err_t C2LORA_init_spi(void);

esp_err_t C2LORA_init_donglemode(tC2LORA_mode default_mode); // reads parameter only.

esp_err_t C2LORA_init_tranceiver(int spi_device_no, unsigned int default_frequency_hz, tC2LORA_mode default_mode, int freq_offset_hz);

// only if there is a 2nd TRX defined:
esp_err_t C2LORA_init_tranceiver_2(int spi_device_no, int freq_offset_hz);

esp_err_t C2LORA_set_channel(unsigned int frequency_hz, tC2LORA_mode mode);

esp_err_t C2LORA_set_local_header(const char *callsign, const char *destination, tKindOfSender kos);
esp_err_t C2LORA_set_local_header_additionals(unsigned short area_code, const char *locator);

unsigned int C2LORA_get_frequency(void);

signed int C2LORA_get_freqshift(void);

esp_err_t C2LORA_set_frequency(unsigned int frequency_hz, signed int tx_shift_hz);

tC2LORA_state C2LORA_get_state(void);

tC2LORA_mode C2LORA_get_mode(void);
unsigned char C2LORA_get_codec_type(void);

esp_err_t C2LORA_set_mode(tC2LORA_mode mode);

signed char C2LORA_get_txpower(void);
esp_err_t C2LORA_set_txpower(signed char power_dBm);

esp_err_t C2LORA_set_calibration(bool active, bool rx_module);

esp_err_t C2LORA_set_freq_offset(int freq_offset_hz, bool rx_module);

esp_err_t C2LORA_get_local_header(char *callsign, char *destination, tKindOfSender *kos);
esp_err_t C2LORA_get_local_header_additionals(unsigned short *area_code, char *locator);

esp_err_t C2LORA_start_continuous_rx(bool enable);

esp_err_t C2LORA_start_tx_microphone(void);

esp_err_t C2LORA_end_tx_microphone(void);

esp_err_t C2LORA_start_rx_speaker(void);

esp_err_t C2LORA_end_rx_speaker(void);

esp_err_t C2LORA_save_config(void);

void      C2LORA_print_status(void);

esp_err_t C2LORA_retransmit_last(void);
