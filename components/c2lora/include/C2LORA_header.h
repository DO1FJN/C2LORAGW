
/*
This header file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

C2LORA defines 10 modes for usage on 70cm HAM radio frequencies. All half-duplex, single frequency
transmissions use standard IQ polarity. On full-duplex, shifted frequencies the downstream uses
inverted IQ polarity.

header structure:
bits     meaning
00..41   callsign (sender)
42..83   destination (recipient or area or 'CQCQCQ')
84       repeated flag ('1' = transmision was repeated by a relay or hotspot)
85..87   kind of station
88..103  crc16 (big endian) from bits 00 to 87 (bytes 0..10)
- END -  of base header (same for all modes)
104..119 area_code
120..155 locator (only in mode 9)
- OR - C2LORA mode 7:
104..139 locator

*/

#pragma once

#include "C2LORA.h"

#include "esp_err.h"

#include "compiler.h"


const char *C2LORA_get_kos_name(tKindOfSender kos);

tKindOfSender C2LORA_get_kos_byname(const char *value);

esp_err_t C2LORA_upd_header(unsigned char *header, const char *callsign, int callsign_len, const char *recipient, int recipient_len, tKindOfSender kos, bool repeated);

esp_err_t C2LORA_upd_recipient(unsigned char *header, const char *recipient);

tKindOfSender C2LORA_decode_header(const unsigned char *header, char *callsign, char *recipient, bool *repeated);

bool C2LORA_check_header(const unsigned char *header, int header_len);

/*
C2LORA_update_header()
swaps lopcator with areacode if C2M7 was or is used.
*/
void C2LORA_update_header(uint8_t *header, tC2LORA_mode old_mode, tC2LORA_mode new_mode);

/*
C2LORA_get_from_header() and C2LORA_add_to_header()
===
There are som additional bytes left in some C2LORA modes. These bytes can be filled with:
- area code (16bit, 3 letters and digits 0 to 4, last letter are 6bit coded - all digits)
- locator (36bit, maidenhead grid square, 6 chars)

Area code can be used in Germany for car number plate codes. There are more posibilities to define a sheme for this.
The area code will be transmitted ONLY in C2LORA modes 4, 5, 6, 8, 9.
A locator will be tranmitted ONLY in C2LORA modes 7 and 9.

*/

unsigned short C2LORA_pack_areacode(const char *letters);

bool C2LORA_unpack_areacode(unsigned short area_code, char *letters);

esp_err_t C2LORA_get_from_header(tC2LORA_mode mode, const unsigned char *header, unsigned short *area_code, char *locator, bool force_all);

esp_err_t C2LORA_add_to_header(unsigned char *header, unsigned short area_code, const char *locator);
