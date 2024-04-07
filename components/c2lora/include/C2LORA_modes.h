
/*
This header file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

C2LORA defines 10 modes for usage on 70cm HAM radio frequencies. All half-duplex, single frequency
transmissions use standard IQ polarity. On full-duplex, shifted frequencies the downstream uses
inverted IQ polarity.

*/

#pragma once

#include "C2LORA.h"
#include "sx126x.h"

#define C2LORA_NO_OF_MODES    12

#define C2LORA_SYNC_WORD      0x1464                        // alter the default syncwords (3444h public / 1424h private) to separete a bit from other lora transmissions

#define C2LORA_MAX_CYDATA_LENGTH  30                        // max 29 bytes now


struct sC2LORAdef {
  tC2LORA_mode                mode;
  uint8_t                     codec_type;                   // Codec2 (or other vocoder) type, here coded as in 'subframedef.h'
  uint8_t                     dv_frames_per_packet;         // how many encoded voice frames are in (usually 12, 40ms each or 24 on higher C2 rates)
  uint8_t                     bytes_per_packet;
  uint8_t                     bytes_per_header;
  uint8_t                     min_finish_ms, max_finish_ms; // timing adjustment so that the last encoded data is written before this part is 'on air'
  uint8_t                     default_preamble;             // no of symbols for the preamble (no of preambles can jitter +/- 1 for timing)
  uint8_t                     firstDV_preamble;             // fixed longer preamble for the first packet (contains header), buys time to get the first speech data
  sx126x_mod_params_lora_t    mod;                          // LoRa modulation parameters
};

const char *       C2LORA_get_mode_name(tC2LORA_mode mode);
tC2LORA_mode       C2LORA_get_mode_by_name(const char *name);

const tC2LORAdef * C2LORA_get_parameter4mode(tC2LORA_mode mode);

tC2LORA_mode       C2LORA_get_mode_by_parameter(const tC2LORAdef *def);

/*
| mode      | name | bandwidth | SF | CR  | Codec2 | RX sensitivity | eff. bitrate |
| --------- | ---- | :-------: | -- | --  | -----: | -------------: | -----------: |
|C2M0_10LR  | 10kHz_700C | 10kHz | SF5 | 4/7 | 700C | -129.8dBm |  930bps |
|C2M1_15LR  | 15kHz_700C | 15kHz | SF6 | 4/6 | 700C | -130.6dBm |  977bps |
|C2M2_15STD | 15kHz_1300 | 15kHz | SF5 | 4/6 | 1300 | -128.1dBm | 1628bps |
|C2M3_15MQ  | 15kHz_1400 | 15kHz | SF5 | 4/6 | 1400 | -128.1dBm | 1628bps |
|C2M4_15HQ  | 15kHz_1600 | 15kHz | SF5 | 4/5 | 1600 | -128.1dBm | 1953bps |
|C2M5_20LR  | 20kHz_1300 | 20kHz | SF6 | 4/5 | 1300 | -129.3dBm | 1524bps |
|C2M6_20    | 20kHz_2400 | 20kHz | SF5 | 4/5 | 2400 | -126.8dBm | 2604bps |
|C2M7_31STD | 31kHz_1600 | 31kHz | SF6 | 4/6 | 1600 | -127.6dBm | 1953bps |
|C2M8_31LL  | 31kHz_2400 | 31kHz | SF5 | 4/7 | 2400 | -125.1dBm | 3255bps |
|C2M9_31HQ  | 31kHz_3200 | 31kHz | SF5 | 4/5 | 3200 | -125.1dBm | 3906bps |
*/


uint32_t c2lora_calc_symboltime(const tC2LORAdef *def);

uint32_t c2lora_calc_firstprefetch(const tC2LORAdef *def, uint32_t symboltime_us, uint8_t first_bytes, bool dv_preamble);

uint32_t c2lora_calc_rxlength(const tC2LORAdef *def, uint32_t symboltime_us, uint8_t pkt_byte_len, int8_t add_symbols);

uint32_t c2lora_calc_txdelay_4_encoded(const tC2LORAdef *def, int32_t time_elapsed_us); // return delay in ms
