
/*
This header file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)
*/
#pragma once

#include "C2LORA.h"
#include "localrecorder.h"


void SX126x_print_statuscode_deverrs(const void *context);


tLoraStream *C2LORA_get_primary_stream(void);

tLoraStream *C2LORA_get_stream_4_transmit(void);

tLoraStream *C2LORA_get_2nd_stream(void);


tRecorderHandle *C2LORA_get_last_record(void);

esp_err_t C2LORA_send_record(tRecorderHandle *rec, tLoraStream *lora);

esp_err_t C2LORA_read_registers_from(unsigned short addr);

esp_err_t C2LORA_write_register_to(unsigned short addr, unsigned char value);

esp_err_t C2LORA_perform_PER_test(unsigned int no_of_transmits, unsigned int tx_delay_us);
