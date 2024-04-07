/*

This source file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

*/
#include "C2LORA_modes.h"
#include "subframedef.h"

#include "esp_log.h"
static const char *TAG = "C2LORA"; // Temporär

#include <stddef.h>
#include <string.h>


typedef struct {
  const char *      name;
  const tC2LORAdef *def;
} tLoraModeDef;


static tC2LORAdef C2LORA_mode0_def = {
  .mode = C2M0_10LR,
  .codec_type       = SF_CODEC_CODEC2_700C,
  .dv_frames_per_packet = 12,  // we have 40ms frames (with 28bit per frame)
  .bytes_per_packet = 47,
  .bytes_per_header = 13,      // absolute minimum header length (2 callsigns and some more bits)
  .default_preamble = 15,
  .firstDV_preamble = 16,
  .min_finish_ms    = 108, //104,     // minimum time needed so that the last encoded frame is written before send
  .max_finish_ms    = 95, //91,      // max time so that there will be corrected down (before TXdone)
  .mod = {
    .sf = SX126X_LORA_SF5,
    .bw = SX126X_LORA_BW_010,
    .cr = SX126X_LORA_CR_4_7,
    .ldro = 0x00
  }
};

static tC2LORAdef C2LORA_mode1_def = {
  .mode = C2M1_15LR,
  .codec_type       = SF_CODEC_CODEC2_700C,
  .dv_frames_per_packet = 12,  // we have 40ms frames (with 28 bit per frame)
  .bytes_per_packet = 48,
  .bytes_per_header = 13,      // absolute minimum header length (2 callsigns and some more bits)
  .default_preamble = 12,
  .firstDV_preamble = 14,
  .min_finish_ms    = 104,      // minimum time needed so that the last encoded frame is written before send
  .max_finish_ms    = 88,      // max time so that there will be corrected down (before TXdone)
  .mod = {
    .sf = SX126X_LORA_SF6,
    .bw = SX126X_LORA_BW_015,
    .cr = SX126X_LORA_CR_4_6,
    .ldro = 0x00
  }
};

static tC2LORAdef C2LORA_mode2_def = {
  .mode = C2M2_15STD,  
  .codec_type       = SF_CODEC_CODEC2_1300,
  .dv_frames_per_packet = 12,  // we have 40ms frames (with 52 bit per frame)
  .bytes_per_packet = 87,
  .bytes_per_header = 15,
  .default_preamble = 15,
  .firstDV_preamble = 20,
  .min_finish_ms    = 106,      // minimum time needed so that the last encoded frame is written before send
  .max_finish_ms    = 88,  //90,      // max time so that there will be corrected down (before TXdone)
  .mod = {
    .sf = SX126X_LORA_SF5,
    .bw = SX126X_LORA_BW_015,
    .cr = SX126X_LORA_CR_4_6,
    .ldro = 0x00
  }
};

static tC2LORAdef C2LORA_mode3_def = {
  .mode = C2M3_15MQ,
  .codec_type       = SF_CODEC_CODEC2_1400,
  .dv_frames_per_packet = 12,  // we have 40ms frames (with 56 bit per frame)
  .bytes_per_packet = 87,
  .bytes_per_header = 15,      // absolute minimum header length (2 callsigns and some more bits)
  .default_preamble = 15,
  .firstDV_preamble = 20,      // all values above 20 corrupts first data!
  .min_finish_ms    = 66,      // minimum time needed so that the last encoded frame is written before send
  .max_finish_ms    = 58,      // <- ToDo kürzer möglich!  max time so that there will be corrected down (before TXdone)
  .mod = {
    .sf = SX126X_LORA_SF5,
    .bw = SX126X_LORA_BW_015,
    .cr = SX126X_LORA_CR_4_6,
    .ldro = 0x00
  }
};

static tC2LORAdef C2LORA_mode4_def = {
  .mode = C2M4_15HQ,
  .codec_type       = SF_CODEC_CODEC2_1600,
  .dv_frames_per_packet = 12,  // we have 40ms frames (with 64 bit per frame)
  .bytes_per_packet = 105,
  .bytes_per_header = 15,
  .default_preamble = 14,
  .firstDV_preamble = 20,      // all values above 20 corrupts first data!
  .min_finish_ms    = 85,      // minimum time needed so that the last encoded frame is written before send
  .max_finish_ms    = 77,      // max time so that there will be corrected down (before TXdone)
  .mod = {
    .sf = SX126X_LORA_SF5,
    .bw = SX126X_LORA_BW_015,
    .cr = SX126X_LORA_CR_4_5,
    .ldro = 0x00
  }
};

static tC2LORAdef C2LORA_mode5_def = {
  .mode = C2M5_20LR,
  .codec_type       = SF_CODEC_CODEC2_1300,
  .dv_frames_per_packet = 12,  // we have 40ms frames (with 52 bit per frame)
  .bytes_per_packet = 81,
  .bytes_per_header = 15,
  .default_preamble = 11,
  .firstDV_preamble = 13,
  .min_finish_ms    = 76, //66,      // minimum time needed so that the last encoded frame is written before send
  .max_finish_ms    = 64, //57,      // max time so that there will be corrected down (before TXdone)
  .mod = {
    .sf = SX126X_LORA_SF6,
    .bw = SX126X_LORA_BW_020,
    .cr = SX126X_LORA_CR_4_5,
    .ldro = 0x00
  }
};

static tC2LORAdef C2LORA_mode6_def = {
  .mode = C2M6_20,
  .codec_type       = SF_CODEC_CODEC2_2400,
  .dv_frames_per_packet = 24,  // we have 20ms frames (with 48 bit per frame)
  .bytes_per_packet = 145,
  .bytes_per_header = 15,
  .default_preamble = 12,
  .firstDV_preamble = 12,
  .min_finish_ms    = 32,      // minimum time needed so that the last encoded frame is written before send
  .max_finish_ms    = 26,      // max time so that there will be corrected down (before TXdone)
  .mod = {
    .sf = SX126X_LORA_SF5,
    .bw = SX126X_LORA_BW_020,
    .cr = SX126X_LORA_CR_4_5,
    .ldro = 0x00
  }
};

static tC2LORAdef C2LORA_mode7_def = {
  .mode = C2M7_31STD,
  .codec_type       = SF_CODEC_CODEC2_1600,
  .dv_frames_per_packet = 12,  // we have 40ms frames (with 64 bit per frame)
  .bytes_per_packet = 105,
  .bytes_per_header = 21,
  .default_preamble = 16,
  .firstDV_preamble = 13,       // add ~12ms more preamble to the first packet while sending from localaudio
  .min_finish_ms    = 89, //156,      // minimum time needed so that the last encoded frame is written before send
  .max_finish_ms    = 81, //148,      // max time so that there will be corrected down (before TXdone)
  .mod = {
    .sf = SX126X_LORA_SF6,
    .bw = SX126X_LORA_BW_031,
    .cr = SX126X_LORA_CR_4_6,
    .ldro = 0x00
  }
};

static tC2LORAdef C2LORA_mode8_def = {
  .mode = C2M8_31LL,
  .codec_type       = SF_CODEC_CODEC2_2400,
  .dv_frames_per_packet = 24,  // we have 20ms frames (with 48 bit per frame)
  .bytes_per_packet = 160,
  .bytes_per_header = 15,
  .default_preamble = 12,
  .firstDV_preamble = 20,
  .min_finish_ms    = 71,      // minimum time needed so that the last encoded frame is written before send
  .max_finish_ms    = 67,      // max time so that there will be corrected down (before TXdone)
  .mod = {
    .sf = SX126X_LORA_SF5,
    .bw = SX126X_LORA_BW_031,
    .cr = SX126X_LORA_CR_4_7,
    .ldro = 0x00
  }
};

static tC2LORAdef C2LORA_mode9_def = {
  .mode = C2M9_31HQ,
  .codec_type       = SF_CODEC_CODEC2_3200,
  .dv_frames_per_packet = 24,  // we have 20ms frames (with 64 bit per frame)
  .bytes_per_packet = 222,
  .bytes_per_header = 20,
  .default_preamble = 13,
  .firstDV_preamble = 14,
  .min_finish_ms    = 88,      // minimum time needed so that the last encoded frame is written before send
  .max_finish_ms    = 84,      // max time so that there will be corrected down (before TXdone)
  .mod = {
    .sf = SX126X_LORA_SF5,
    .bw = SX126X_LORA_BW_031,
    .cr = SX126X_LORA_CR_4_5,
    .ldro = 0x00
  }
};

static tC2LORAdef C2LORA_modeA_def = {
  .mode = C2MA_7NB,
  .codec_type       = SF_CODEC_CODEC2_700C,
  .dv_frames_per_packet = 12,  // we have 40ms frames (with 28bit per frame)
  .bytes_per_packet = 47,
  .bytes_per_header = 13,      // absolute minimum header length (2 callsigns and some more bits)
  .default_preamble = 12,
  .firstDV_preamble = 12,
  .min_finish_ms    = 108,     // minimum time needed so that the last encoded frame is written before send
  .max_finish_ms    = 95,      // max time so that there will be corrected down (before TXdone)
  .mod = {
    .sf = SX126X_LORA_SF5,
    .bw = SX126X_LORA_BW_007,
    .cr = SX126X_LORA_CR_4_5,
    .ldro = 0x00
  }
};

static tC2LORAdef C2LORA_modeB_def = {
  .mode = C2MB_WIDE,
  .codec_type       = SF_CODEC_CODEC2_2400,
  .dv_frames_per_packet = 24,  // we have 20ms frames (with 64 bit per frame)
  .bytes_per_packet = 171,
  .bytes_per_header = 16,
  .default_preamble = 11,
  .firstDV_preamble = 12,
  .min_finish_ms    = 103,     // minimum time needed so that the last encoded frame is written before send
  .max_finish_ms    = 94,      // max time so that there will be corrected down (before TXdone)
  .mod = {
    .sf = SX126X_LORA_SF8,
    .bw = SX126X_LORA_BW_125,
    .cr = SX126X_LORA_CR_4_5,
    .ldro = 0x00
  }
};


static tLoraModeDef C2LORA_modes[C2LORA_NO_OF_MODES] = {
  { "10kHz_700C", &C2LORA_mode0_def },
  { "15kHz_700C", &C2LORA_mode1_def },
  { "15kHz_1300", &C2LORA_mode2_def },
  { "15kHz_1400", &C2LORA_mode3_def },
  { "15kHz_1600", &C2LORA_mode4_def },
  { "20kHz_1300", &C2LORA_mode5_def },
  { "20kHz_2400", &C2LORA_mode6_def },
  { "31kHz_1600", &C2LORA_mode7_def },
  { "31kHz_2400", &C2LORA_mode8_def },
  { "31kHz_3200", &C2LORA_mode9_def },
  { "7kHz_700C",  &C2LORA_modeA_def },
  { "125kHz_2400", &C2LORA_modeB_def }
};


const char * C2LORA_get_mode_name(tC2LORA_mode mode) {
  if (mode >= C2LORA_NO_OF_MODES) return NULL;
  return C2LORA_modes[mode].name;
}


tC2LORA_mode C2LORA_get_mode_by_name(const char *name) {
  tC2LORA_mode m;
  for (m = C2M_MIN; m <= C2M_MAX; m++) {
    if (strcasecmp(name, C2LORA_modes[m].name) == 0) {
      return m;
    }
  }
  m = strtol(name, NULL, 0);
  if ((m >= C2M_MIN) && (m <= C2M_MAX)) return m;
  return -1;  // invalid
}


const tC2LORAdef * C2LORA_get_parameter4mode(tC2LORA_mode mode) {
  if (mode >= C2LORA_NO_OF_MODES) return NULL;
  return C2LORA_modes[mode].def;
}


tC2LORA_mode C2LORA_get_mode_by_parameter(const tC2LORAdef *def) {
  return (def == NULL)? C2M_MAX: def->mode;
/*  for (tC2LORA_mode m = 0; m < C2LORA_NO_OF_MODES; m++) {
    if (def == C2LORA_modes[m].def) return m;
  }
*/
  return C2M_MAX;
}



uint32_t c2lora_calc_symboltime(const tC2LORAdef *def) {
  return (1 << def->mod.sf) * 1000000L / sx126x_get_lora_bw_in_hz(def->mod.bw);
}

uint32_t c2lora_calc_firstprefetch(const tC2LORAdef *def, uint32_t symboltime_us, uint8_t first_bytes, bool dv_preamble) {
  uint32_t preamble_symbols = (dv_preamble? def->firstDV_preamble: def->default_preamble) + 14;
  int32_t  spread_bits      = def->mod.sf << 2;
  int32_t  data_length_bits = (int32_t) first_bytes << 3;
  int32_t  data_length_sym  = ((data_length_bits < spread_bits)? 0: ((data_length_bits - 1) / spread_bits)) * (def->mod.cr + 4);
  //ESP_LOGD(TAG, "%u bytes -> %ld sym", first_bytes, data_length_sym);
  return ((preamble_symbols + data_length_sym) * symboltime_us) + (symboltime_us >> 2);
}

uint32_t c2lora_calc_rxlength(const tC2LORAdef *def, uint32_t symboltime_us, uint8_t pkt_byte_len, int8_t add_symbols) {
  int32_t spread_bits      = def->mod.sf << 2;
  int32_t data_length_bits = (int32_t) pkt_byte_len << 3;
  int32_t data_length_sym  = ((data_length_bits < spread_bits)? 0: ((data_length_bits - 1) / spread_bits)) * (def->mod.cr + 4);
  //ESP_LOGD(TAG, "%u bytes -> %ld sym", pkt_byte_len, data_length_sym);
  if (add_symbols < 0) add_symbols = 0;
  return (data_length_sym + add_symbols) * symboltime_us;
}


#define C2CODEC_ENCODING_WORST_40MS   (24+40)
#define C2CODEC_ENCODING_WORST_20MS   (12+20)

uint32_t c2lora_calc_txdelay_4_encoded(const tC2LORAdef *def, int32_t time_elapsed_us) {
  uint32_t symbol_time = c2lora_calc_symboltime(def);
/* 
  int32_t frame_fini_ms = def->min_finish_ms - (def->min_finish_ms - def->max_finish_ms) / 2;

  uint32_t header_xtra_us = c2lora_calc_rxlength(def, symbol_time, def->bytes_per_header, def->firstDV_preamble - def->default_preamble);

  frame_fini_ms -= (header_xtra_us + 500) / 1000 - (def->dv_frames_per_packet==12? C2CODEC_ENCODING_WORST_40MS: C2CODEC_ENCODING_WORST_20MS);
*/
  uint32_t frame_bytes = (sf_get_subframe_bits(def->codec_type) * def->dv_frames_per_packet + 7) >> 3; // bytes for speech data
  // calculate for the first frame after the header!
  uint32_t hframe_time = c2lora_calc_firstprefetch(def, symbol_time, def->bytes_per_header + def->bytes_per_packet, true);
  uint32_t dframe_time = c2lora_calc_firstprefetch(def, symbol_time, frame_bytes, false);

  int32_t  full_frame_encoded_time = (960 + (def->dv_frames_per_packet==12? C2CODEC_ENCODING_WORST_40MS: C2CODEC_ENCODING_WORST_20MS)) * 1000;
  int32_t  prestart_delay_us = full_frame_encoded_time - hframe_time - dframe_time - time_elapsed_us;

  ESP_LOGD(TAG, "times: %luµs|%ldµs => %ldµs (%lu bytes)", hframe_time + dframe_time, full_frame_encoded_time, prestart_delay_us, frame_bytes);
 
  return prestart_delay_us > 0? (prestart_delay_us + 250) / 1000: 0;
}

