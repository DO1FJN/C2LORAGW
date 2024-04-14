/*
 * SSD1306.h
 *
 * This header file belongs to project 'C2LoRaGW'.
 * (c) 2024, DO1FJN (Jan Alte)
 *
 *  Created on: 2024-04-10
 *     Project: C2 Lora Gateway
 *      Author: Jan Alte, DO1FJN
 */


#pragma once

#include "esp_err.h"

#include <stdbool.h>

esp_err_t SSD1306_Init(unsigned char no_of_lines);

esp_err_t SSD1306_OnOff(bool on_off);

esp_err_t SSD1306_Contrast(unsigned char contrast);

esp_err_t SSD1306_UpdateAll(void);


esp_err_t SSD1306_InverseDisp(bool on_off);


esp_err_t SSD1306_SetWindow(char left, char top, char right, char bottom, bool invers);

esp_err_t SSD1306_write_data(const unsigned char *buffer, unsigned short len);

esp_err_t SSD1306_write_command(unsigned char cmd_byte, const unsigned char *cmd_arg, unsigned short len);

esp_err_t SSD1306_FillRect(unsigned char x, unsigned char y, unsigned char w, unsigned char h, bool invers);

void      SSD1306_FillNextRect(unsigned char width, unsigned char no_of_lines, unsigned char onOff);

void      SSD1306_SetNextPixel(unsigned char onOff);

void      SSD1306_SetNextLine(void);
