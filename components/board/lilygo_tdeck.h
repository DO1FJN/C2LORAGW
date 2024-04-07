#pragma once

/*
 * hardware GPIO defines for a ESP32-S3 Noname HMI Board with TFT Display mounted ta a bread board
 */

#define BOARD_POWER_ON_Pin      10

#define TFT_BACKLIGHT_PIN       42


#define TFT_HOST                SPI3_NUM
#define TFT_DEVICE_NUM          0
#define TFT_SPISPEED_MHZ        16

#define TFT_SCK_Pin		40
#define TFT_MOSI_Pin		41
#define TFT_MISO_Pin            38
#define TFT_DC_Pin	        11
#define TFT_CS_Pin	        12
#define TFT_RST_Pin	        GPIO_NUM_NC


#define	TFT_X_SIZE		320
#define TFT_Y_SIZE		240
#define TFT_BYTE_PER_COLOR	2
#define TFT_CTRL_POSITION       RIGHT
#define TFT_Y_OFFSET            0

#define SX126X_HOST             SPI3_NUM
#define SX126X_SPISPEED_MHZ     8

#define SX126X_SCK_Pin		TFT_SCK_Pin
#define SX126X_MOSI_Pin		TFT_MOSI_Pin
#define SX126X_MISO_Pin         TFT_MISO_Pin

#define SX126X_DEVICE_NUM       1
#define SX126X_NSS_Pin	        9
#define SX126X_DIO1_Pin         45
#define SX126X_BUSY_Pin	        13
#define SX126X_RST_Pin	        17
#define SX126X_DIO3_Pin         GPIO_NUM_NC

#define SX126X_FREQUENCY_OFFSET 0

#define PROG_BUTTON_Pin         0       // IO0

#define AUDIO_I2S_WS_Pin        5
#define AUDIO_I2S_DOUT_Pin      6
#define AUDIO_I2S_BCLK_Pin      7

#define CODEC_I2S_DIN_Pin       14
#define CODEC_I2S_BCLK_Pin      47
#define CODEC_I2S_WS_Pin        21
#define CODEC_I2S_MCLK_Pin      48

#define AUDIO_I2S_ENABLE        GPIO_NUM_NC

