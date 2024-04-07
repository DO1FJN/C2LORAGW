/*
This header file contain to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

hardware GPIO defines for a ESP32-S3 Noname HMI Board with TFT Display mounted ta a bread board
*/

#pragma once

#define PROG_BUTTON_Pin         0       // IO0

#define TFT_BACKLIGHT_PIN       14


#define TFT_HOST                SPI2_NUM
#define TFT_DEVICE_NUM          0
#define TFT_SPISPEED_MHZ        26

#define TFT_SCK_Pin		12
#define TFT_MOSI_Pin		13
#define TFT_MISO_Pin            GPIO_NUM_NC
#define TFT_DC_Pin	        11
#define TFT_CS_Pin	        10
#define TFT_RST_Pin	        1


#define	TFT_X_SIZE		320
#define TFT_Y_SIZE		170
#define TFT_BYTE_PER_COLOR	2
#define TFT_CTRL_POSITION       RIGHT
#define TFT_Y_OFFSET            35

#define AUDIO_I2S_WS_Pin        5
#define AUDIO_I2S_DOUT_Pin      6
#define AUDIO_I2S_BCLK_Pin      7

#define CODEC_I2S_DIN_Pin       48
#define CODEC_I2S_BCLK_Pin      47
#define CODEC_I2S_WS_Pin        21


#define AUDIO_I2S_ENABLE        17


#define SX126X_RX_BOOT_GAIN     0       // no boost

#define SX126X_HOST             SPI3_NUM
#define SX126X_SPISPEED_MHZ     10

#define SX126X_SCK_Pin		38
#define SX126X_MOSI_Pin		39
#define SX126X_MISO_Pin         40
#define SX126X_RST_Pin	        2

#define SX126X_DEVICE_NUM       0
#define SX126X_FREQUENCY_OFFSET   17450
#define SX126X_NSS_Pin	        41
#define SX126X_BUSY_Pin	        42
#define SX126X_DIO1_Pin         15
//#define SX126X_DIO3_Pin       16

#define SX126X_2_DEVICE_NUM     1
#define SX126X_2_FREQUENCY_OFFSET   18300
#define SX126X_2_NSS_Pin	    20
#define SX126X_2_BUSY_Pin           18
#define SX126X_2_DIO1_Pin           19
//#define SX126X_DIO3_Pin 
