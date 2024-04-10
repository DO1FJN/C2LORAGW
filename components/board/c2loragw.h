#pragma once

/*
 * hardware GPIO defines for a ESP32-S3 Noname HMI Board with TFT Display mounted ta a bread board
 */

#define BOARD_POWER_ON_Pin      8

#define BOARD_5V_POWERMODE_Pin  18

#define SX126X_HOST             SPI2_NUM
#define SX126X_SPISPEED_MHZ     10

#define SX126X_SCK_Pin		12
#define SX126X_MOSI_Pin		11
#define SX126X_MISO_Pin         13

#define SX126X_DEVICE_NUM       0
#define SX126X_NSS_Pin	        10
#define SX126X_DIO1_Pin         3
#define SX126X_BUSY_Pin	        46
#define SX126X_RST_Pin	        9
#define SX126X_DIO2_Pin         48                      // DIO2 line as input - if available, gets feedback if TX keeps transmitting or don't
#define SX126X_DIO3_Pin         GPIO_NUM_NC             // general purpos INT line - if available, not used jet
#define SX126X_RXSWITCH_Pin     45                      // separate RX antenna switch (used in moduls with LAN + PA)

#define SX126X_RX_BOOST_GAIN    0      // without boost
#define SX126X_FREQUENCY_OFFSET 0                       // default frequencyoffet matches roughly at room temperature
#define SX126X_ENABLE_DCDC      1                       // bool if the DCDC can be turned on (Inductor on DCC_SW pin)

#define SX126X_TCXO_VOLTAGE     SX126X_TCXO_CTRL_1_8V
#define SX126X_TCXO_STARTTIME   5       // [ms]

#define SX126X_LNA_RSSI_SHIFT   18                      // having a additional LNA before the SX126x results in wrong RSSI readings
#define SX126X_PA_GAIN_DB       9                       // having a PA attached the TX power-settings must be converted down by this offset
#define SX126X_RX_BOOT_GAIN     0                       // no boost

#define AUDIO_I2S_WS_Pin        47
#define AUDIO_I2S_DOUT_Pin      41
#define AUDIO_I2S_BCLK_Pin      39

#define CODEC_I2S_DIN_Pin       42
#define CODEC_I2S_BCLK_Pin      40
#define CODEC_I2S_WS_Pin        38
#define CODEC_I2S_MCLK_Pin      21

#define AUDIO_I2S_ENABLE        GPIO_NUM_NC

// unused
#define	TFT_X_SIZE		320
#define TFT_Y_SIZE		240


