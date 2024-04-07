/*

This source file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

C2LORA defines 10 modes for usage on 70cm HAM radio frequencies. All half-duplex, single frequency
transmissions use standard IQ polarity. On full-duplex, shifted frequencies the downstream uses
inverted IQ polarity.

*/

#include "C2LORA_header.h"

#include "C2LORA_core.h"

#include "utilities.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"

#include "rom/crc.h"

#include <ctype.h>

#define HEADER_POS_AREACODE   13    // 2byte (used in modes witch 15 or more header bytes)
#define HEADER_POS_LOCATOR_S  15    // 5byte

static const char *TAG = "C2LRA_HDR";

static const char *KoS_names[8] = {
  "not set", "portable", "mobile", "station", "hotspot", "relay",
  "reserved6", "reserved7"
};


const char *C2LORA_get_kos_name(tKindOfSender kos) {
  if (kos > 7) return "invalid KoS";
  return KoS_names[kos];
}


tKindOfSender C2LORA_get_kos_byname(const char *value) {
  for (tKindOfSender k=0; k <= KOS_RELAY; k++) {
    if (strcasecmp(value, KoS_names[k]) == 0) return k;
  }
  return KOS_UNDEF;
}



static uint8_t get_6bit_char(char c) {
  char cx = c == 0? 0: (toupper(c) - 0x20);
  if (cx > 0x3F) cx = 0x3F;
  return cx;
}

static inline uint8_t get_char_from_6bit(char c6) {
  return (c6 & 0x3F) + 0x20;
}


static void pack_6bit_to(uint8_t *dest, const char *src, int char_cnt) {
  int bitpos;
  int bits_total = char_cnt * 6;
  //uint8_t byte;
  for (bitpos = 0; bitpos < bits_total; bitpos += 6, src++) {
    if ((bitpos & 7) == 0) {
      dest[0] = src[0] << 2;
    } else {
      dest[0] |= src[0] >> ((bitpos & 7) - 2);
      dest++;
      dest[0] = src[0] << (10 - (bitpos & 7));
    }
  }  // rof
}


static void unpack_6bit_to(char *dest, const uint8_t *src, int char_cnt) {
  int bitpos;
  int bits_total = char_cnt * 6;
  //uint8_t byte;
  for (bitpos = 0; bitpos < bits_total; bitpos += 6, dest++) {
    int bit_byte_pos = bitpos & 7;
    if (bit_byte_pos == 0) {
      dest[0] = src[0] >> 2;
    } else {
      dest[0] = ((src[0] << (bit_byte_pos - 2)) | (src[1] >> (10 - bit_byte_pos))) & 0x3F;
      src++;
    }
  }  // rof
}



esp_err_t C2LORA_upd_header(unsigned char *header, const char *callsign, int callsign_len, const char *recipient, int recipient_len, tKindOfSender kos, bool repeated) {
  Union16 crc;
  char buffer_6bit[14];
  ESP_RETURN_ON_FALSE(header != NULL, ESP_ERR_INVALID_ARG, TAG, "header is NULL");
  ESP_RETURN_ON_FALSE(callsign != NULL, ESP_ERR_INVALID_ARG, TAG, "callsign is NULL");
  ESP_RETURN_ON_FALSE(recipient != NULL, ESP_ERR_INVALID_ARG, TAG, "recipient is NULL");

  if (!Check_Call(callsign, callsign_len)) {
    ESP_LOGW(TAG, "callsign '%.*s' is not valid.", callsign_len, callsign);
  } // fi

  memset(buffer_6bit, 0, sizeof(buffer_6bit));
  if (callsign_len > 7) callsign_len = 7;
  for (int i=0; i < callsign_len; i++) {
    buffer_6bit[i] = get_6bit_char(callsign[i]);
  }
  if (recipient_len > 7) recipient_len = 7;
  for (int i=0; i < recipient_len; i++) {
    buffer_6bit[i + 7] = get_6bit_char(recipient[i]);
  }
  pack_6bit_to(header, buffer_6bit, sizeof(buffer_6bit)); // pack to 84bit
  header[10] = (header[10] & 0xF0) | (repeated? 0x8: 0x0) | (kos & 0x7);          // add kos, 1bit left...

  crc.u16 = crc16_le(0, header, 11);    
  header[11] = crc.u8[1];
  header[12] = crc.u8[0];

#if CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_DEBUG
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, header, 16, ESP_LOG_DEBUG);
  kos = C2LORA_decode_header(header, buffer_6bit, buffer_6bit + 7, NULL);
  printf("callsign: %7.7s, recipient: %7.7s, KoS: %s\n", buffer_6bit, buffer_6bit+7, C2LORA_get_kos_name(kos));
#endif
  return ESP_OK;
}


tKindOfSender C2LORA_decode_header(const unsigned char *header, char *callsign, char *recipient, bool *repeated) {
  char buffer_6bit[14];
  unpack_6bit_to(buffer_6bit, header, sizeof(buffer_6bit)); // unpack 84bit
  for (int i=0; i<7; i++) {
    callsign[i] = get_char_from_6bit(buffer_6bit[i]);
  }
  for (int i=0; i<7; i++) {
    recipient[i] = get_char_from_6bit(buffer_6bit[i + 7]);
  }
  if (repeated != NULL) repeated[0] = header[10] & 0x8? true: false;
  return header[10] & 0x7;
}


bool C2LORA_check_header(const unsigned char *header, int header_len) {
  if (header_len > 12) {
    Union16 crc;    
    crc.u16 = crc16_le(0, header, 11);    
    return (header[11] == crc.u8[1]) && (header[12] == crc.u8[0]);
  } // fi add crc        
  return true;
}


/*
C2LORA_update_header()
swaps lopcator with areacode if C2M7 was or is used.
*/
void C2LORA_update_header(uint8_t *header, tC2LORA_mode old_mode, tC2LORA_mode new_mode) {
  if (new_mode != old_mode) {
    uint8_t hdr_bak[8];
    memcpy(&hdr_bak, header + HEADER_POS_AREACODE, sizeof(hdr_bak));
    if (new_mode == C2M7_31STD) {
      memcpy(header + HEADER_POS_AREACODE, hdr_bak + 2, 5);
      memcpy(header + HEADER_POS_AREACODE + 5, hdr_bak, 2);
    } else if (old_mode == C2M7_31STD) {
      memcpy(header + HEADER_POS_LOCATOR_S, hdr_bak, 5);
      memcpy(header + HEADER_POS_AREACODE, hdr_bak + 5, 2);
    }
  } // if
  for (int i = 0; i < (C2LORA_MAX_HEADER_SIZE - HEADER_POS_LOCATOR_S - 5); i++) { // fill unused
    header[i + HEADER_POS_LOCATOR_S + 5] = (new_mode << 4) | i;
  }
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, header + HEADER_POS_AREACODE, C2LORA_MAX_HEADER_SIZE - HEADER_POS_AREACODE, ESP_LOG_DEBUG);
}


unsigned short C2LORA_pack_areacode(const char *letters) {
  char area_code[3];
  unsigned short result = 0;
  int len = strnlen(letters, 5);
  if ((len < 1) || (len > 3)) return 0;
  area_code[0] = toupper(letters[0]);
  area_code[1] = len > 1? toupper(letters[1]): '@';
  area_code[2] = get_6bit_char(letters[2]);
  for (int l=0; l < 2; l++) {
    if (area_code[l] <= 0x20) {
      area_code[l] = 0;
      continue;
    }
    if ((area_code[l] >= '0') && (area_code[l] <= '4')) area_code[l] += 0x2B; //map numbers 0 to 4
    area_code[l] -= 0x40; // subract '@'
    if (area_code[l] > 0x1F) return 0xFFFF; // error invalid letter / char
  } // fi 
  result = (area_code[0] << 11) | (area_code[1] << 6) | area_code[2];
  ESP_LOGD(TAG, "'%.3s' => %02xh %02xh %02xh => %04xh", letters, area_code[0], area_code[1], area_code[2], result);
  return result;
}


bool C2LORA_unpack_areacode(unsigned short area_code, char *letters) {
  char area_chars[3];
  area_chars[0] = area_code >> 11;
  area_chars[1] = (area_code >> 6) & 0x1F;
  area_chars[2] = area_code & 0x3F;
  letters[0] = area_chars[0] == 0x00? ' ': area_chars[0] + (area_chars[0] < 0x1B? 0x40: 0x15);
  letters[1] = area_chars[1] == 0x00? ' ': area_chars[1] + (area_chars[1] < 0x1B? 0x40: 0x15);
  letters[2] = get_char_from_6bit(area_chars[2]);
  return false;
}


esp_err_t C2LORA_add_to_header(unsigned char *header, unsigned short area_code, const char *locator) {
  ESP_RETURN_ON_FALSE(header != NULL, ESP_ERR_INVALID_ARG, TAG, "header is NULL");
  if ((area_code != 0) && (area_code != 0xFFFF)) {    
    header[HEADER_POS_AREACODE]   = (uint8_t) (area_code >> 8);
    header[HEADER_POS_AREACODE+1] = (uint8_t) (area_code & 0xFF);
  } // fi
  if (locator != NULL) {
    char buffer_6bit[6];
    memset(buffer_6bit, 0, 6);
    for (int i=0; (i < 6) && (locator[i] != 0); i++) {
      buffer_6bit[i] = get_6bit_char(locator[i]);
    } // rof
    pack_6bit_to(header + HEADER_POS_LOCATOR_S, buffer_6bit, sizeof(buffer_6bit)); // pack to 36bit
  } // rof
  return ESP_OK;
}


esp_err_t C2LORA_get_from_header(tC2LORA_mode mode, const unsigned char *header, unsigned short *area_code, char *locator, bool force_all) {
  if (area_code != NULL) {
    uint16_t areacode_i = header[HEADER_POS_AREACODE];
    areacode_i = (areacode_i << 8) | header[HEADER_POS_AREACODE + 1];
    area_code[0] = force_all || (mode >= C2M4_15HQ) ? areacode_i: 0;
  }
  if (locator != NULL) {
    if (force_all || (mode == C2M7_31STD) || (mode == C2M9_31HQ)) {
      char buffer_6bit[6];
      unpack_6bit_to(buffer_6bit, header + HEADER_POS_LOCATOR_S, sizeof(buffer_6bit)); 
      for (int i=0; i<6; i++) {
        locator[i] = get_char_from_6bit(buffer_6bit[i]);
      } // rof
      locator[6] = 0;
    } else {
      locator[0] = 0; // no locator
    }
  }
  return ESP_OK;
}
