/*
This header file contain to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

hardware GPIO defines for Heltec WiFi LoRa 32 V3 / V3.1
*/

#pragma once

#define PROG_BUTTON_Pin         0       // IO0

#define LED_Pin                 35
#define VEXT_CTRL_Pin           36      // turns on 3.3V supply on JP2 pins 3+4 and OLED
#define ADC_CTRL_Pin            37      // turns on voltage divider to measure VBAT at ADC_IN pin
#define ADC_INPUT_Pin           1

#define BOARD_POWER_ON_Pin      VEXT_CTRL_Pin
#define RECEIVING_INDICATOR_LED LED_Pin

// we use an I²C bus, the only device connected is an OLED monochrome display
#define I2C_SDA_Pin             17
#define I2C_SCL_Pin             18

// only an separate reset pin is needed beside I²C
#define OLED_RST_Pin            21

#define OLED_IF_TYPE            0       // (0) I2C, (1) SPI
#define	OLED_X_SIZE		128
#define OLED_Y_SIZE		64
#define OLED_CTRL_POSITION      BOTTOM
#define OLED_Y_OFFSET           0

// tbd (JP2 / JP3 attached hardware)
#define AUDIO_I2S_WS_Pin        26
#define AUDIO_I2S_DOUT_Pin      20
#define AUDIO_I2S_BCLK_Pin      19      // MAX98357A connected to JP2 pins 15,17,18, VDD_5V and GND

#define CODEC_I2S_WS_Pin        47
#define CODEC_I2S_DIN_Pin       33
#define CODEC_I2S_BCLK_Pin      34      // INMP441 connected to JP2 pins 11-13, Vext and GND


// SX1262 LoRa GPIO connectivity and other defines:

#define SX126X_RX_BOOST_GAIN    1      // with boost
#define SX126X_FREQUENCY_OFFSET 0
#define SX126X_ENABLE_DCDC      1      // '1' if the DCDC can be turned on (Inductor on DCC_SW pin)

#define SX126X_HOST             SPI2_NUM
#define SX126X_SPISPEED_MHZ     10

#define SX126X_SCK_Pin		9
#define SX126X_MOSI_Pin		10
#define SX126X_MISO_Pin         11
#define SX126X_RST_Pin	        12

#define SX126X_DEVICE_NUM       0       // logical device number (on the same SPI bus)
#define SX126X_NSS_Pin	        8
#define SX126X_BUSY_Pin	        13
#define SX126X_DIO1_Pin         14

#define SX126X_TCXO_VOLTAGE     SX126X_TCXO_CTRL_1_8V
#define SX126X_TCXO_STARTTIME   5       // [ms]
