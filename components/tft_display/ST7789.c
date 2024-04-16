/*
 * ST7789V.c
 *
 *      Author: Jan Alte, (c) bentrup Industriesteuerungen
 */

#include "ST7789.h"

#include "hardware.h"
#include "s_spi.h"

#include "esp_log.h"

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>


#define ST_CMD_DELAY		0x80	// special signifier for command lists

#define ST7789_NOP		0x00	// no operation dummy command
#define ST7789_SWRESET		0x01	// software reset
#define ST7789_RDDID		0x04	// read ID | + dummy pulse > 3 byte data
#define ST7789_RDDST		0x09	// read display status | + dummy pulse > 4 byte data
#define ST7789_RDDPM		0x0A	// read display power | + dummy read > 1 byte data
#define ST7789_RDDMADCTL	0x0B	// read MADCTL | + dummy read > 1 byte data
#define ST7789_RDDCOLMOD	0x0C	// read display pixel mode | + dummy read > 1 byte data
#define ST7789_RDDIM		0x0D	// read display image mode | + dummy read > 1 byte data
#define ST7789_RDDSM		0x0E	// read display signal mode | + dummy read > 1 byte data
#define ST7789_RDDSDR		0x0F	// read self-diagnostic result | + dummy read > 1 byte data
#define ST7789_SLPIN		0x10	// switch into sleep mode
#define ST7789_SLPOUT		0x11	// switch from sleep mode to active
#define ST7789_PTLON		0x12	// partial mode on
#define ST7789_NORON		0x13	// partial mode off (normal mode on)
#define ST7789_INVOFF		0x20	// color inversion off
#define ST7789_INVON		0x21	// color inversion on
#define ST7789_GAMSET		0x26	// gamma set | + 1 byte parameter
#define ST7789_DISPOFF		0x28	// display off
#define ST7789_DISPON		0x29	// display on
#define ST7789_CASET		0x2A	// column address set | + 4 byte parameter
#define ST7789_RASET		0x2B	// row address set | + 4 byte parameter
#define ST7789_RAMWR		0x2C	// write to memory (pixel) | + n byte pixel-data
#define ST7789_RAMRD		0x2E	// read memory | +dummy pulse > n byte data

#define ST7789_PTLAR		0x30	// partial start / end address set | + 4 byte parameter
#define ST7789_VSCRDEF		0x33	// vertical scrolling definition | + 6 byte parameter
#define ST7789_TEOFF		0x34	// tearing effect line off
#define ST7789_TEON		0x35	// tearing effect line on
#define ST7789_MADCTL		0x36	// memory access control | +1 byte parameter
#define ST7789_VSCRADD		0x34	// vertical scrolling start address | +2 byte parameter
#define ST7789_IDOFF		0x38	// idle mode off
#define ST7789_IDON		0x39	// idle mode on
#define ST7789_COLMOD		0x3A	// interface pixel format (color mode) | + 1 byte parameter
#define ST7789_RAMWRC		0x3C	// memory write continue | + n byte data
#define ST7789_RAMRDC		0x3E	// memory read continue | +dummy pulse > n byte data

#define ST7789_TESCAN		0x44	// set tear scanline | + 2 byte parameter
#define ST7789_RDTESCAN		0x45	// get scanline  | +dummy pulse > 2 byte data

#define	ST7789_WAIT_MS		5
#define	ST7789_WAITSLPOUT_MS	120

#define TFT_COLORMODE		0x55	// 16bit RGB 5-6-5 pixel



#define ST7789_MADCTL_MY 0x80
#define ST7789_MADCTL_MX 0x40
#define ST7789_MADCTL_MV 0x20
#define ST7789_MADCTL_ML 0x10
#define ST7789_MADCTL_BGR 0x08
#define ST7789_MADCTL_RGB 0x00

#define ST7789_RDID1 0xDA
#define ST7789_RDID2 0xDB
#define ST7789_RDID3 0xDC
#define ST7789_RDID4 0xDD

#ifndef ST7789_MAD_VALUE
#if TFT_X_SIZE > TFT_Y_SIZE // implicit landscape
#if TFT_CTRL_POSITION==RIGHT
#define ST7789_MAD_VALUE  (ST7789_MADCTL_MV|ST7789_MADCTL_MX) // rotate -90°#else
#else
#define ST7789_MAD_VALUE  (ST7789_MADCTL_MV|ST7789_MADCTL_MY) // rotate +90°
#endif
#else
#define ST7789_MAD_VALUE  0x00
#endif
#endif

#ifndef TFT_X_OFFSET
#define TFT_X_OFFSET  0
#endif

#ifndef TFT_Y_OFFSET
#define TFT_Y_OFFSET  0
#endif


#define MAX_XFER_SIZE     32768 //max for a S3

#define MAX_CMDDAT_TICKS  5

#define MAX_XFER_TIME_MS	100	//2


//static const char * TAG = "ST7789";

#define st7789_ini_cmds		6
static const uint8_t st7789_ini[] =  {	// Init commands for 7789 screens
    ST7789_SWRESET,   ST_CMD_DELAY,	//  1: Software reset, no args, w/delay
    ST7789_WAITSLPOUT_MS,		// ~120 ms delay
    ST7789_SLPOUT,    ST_CMD_DELAY,	//  2: Out of sleep mode, no args, w/delay
    ST7789_WAITSLPOUT_MS,		//  120 ms delay
    ST7789_COLMOD, 1, TFT_COLORMODE,	     //  3: Set color mode, 1 arg, 16-bit color
    ST7789_MADCTL, 1, ST7789_MAD_VALUE,		//  4: Mem access ctrl (directions), 1 arg: Row/col addr, bottom-top refresh
    ST7789_INVON, 0,
    ST7789_DISPOFF, 0,			//  5: Main screen turn off, no args, delay
    ST7789_NOP
};

#define TFT_LOOPBUF_SIZE    32
static unsigned short DMA_ATTR tft_loop_buffer[TFT_LOOPBUF_SIZE];



inline static void tft_enable(void) {
  //spi_device_acquire_bus(tft_spi, portMAX_DELAY);
}

inline static void tft_disable(void) {
  //spi_device_release_bus(tft_spi);
}


#define tft_send_cmd(cmd, arg, n_arg)   sspi_send_cmd_w_arg(TFT_HOST, cmd, n_arg, arg)

//#define tft_cmd()       gpio_set_level(TFT_DC_Pin, 0)
//#define tft_data()      gpio_set_level(TFT_DC_Pin, 1)



esp_err_t ST7789_Init(void) {
  const tsspi_bus_config tft_spibus_config = {
    .speed_hz = TFT_SPISPEED_MHZ * 1000 * 1000,     //Clock out at 26 MHz
    .flags = 0,
    .gpio = {
      .sclk   = TFT_SCK_Pin,
      .mosi   = TFT_MOSI_Pin,
      .miso   = TFT_MISO_Pin,
      .quadwp = GPIO_NUM_NC,
      .quadhd = GPIO_NUM_NC,
      .cd_sel = TFT_DC_Pin
    }
  };
  const tsspi_device_cfg tft_device_config = {
    .cs_pin = TFT_CS_Pin,
    .dc_pin = TFT_DC_Pin,
    .conf.half_duplex = 0,
    .mode.cpha = 0,
    .mode.cpol = 0,
  };

  esp_err_t ret = ESP_OK;

  uint8_t numCommands = st7789_ini_cmds;	// Number of commands to follow
  const uint8_t *addr = st7789_ini;

  // Reset the display
#if (defined TFT_RST_Pin) && (TFT_RST_Pin != GPIO_NUM_NC)  
  gpio_set_direction(TFT_RST_Pin, GPIO_MODE_OUTPUT);
  gpio_set_level(TFT_RST_Pin, 0);
  vTaskDelay(20 / portTICK_PERIOD_MS);
  gpio_set_level(TFT_RST_Pin, 1);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  addr += 3;
  numCommands--;
#endif

#if SX126X_HOST != TFT_HOST
  //Initialize the SPI bus
  ret = sspi_init(TFT_HOST, &tft_spibus_config);
  ESP_ERROR_CHECK(ret);
#endif
  //Attach the LCD to the SPI bus
  ret = sspi_device_init(TFT_HOST, TFT_DEVICE_NUM, &tft_device_config);
  ESP_ERROR_CHECK(ret);

  ret = sspi_device_select(TFT_HOST, TFT_DEVICE_NUM);
  ESP_ERROR_CHECK(ret);

  //Initialize the LCD
  tft_enable();
  for (; numCommands > 0; numCommands--) {	// For each command...
    uint8_t  cmd, num_args, wait_ms;
    uint32_t arg;
    cmd      = addr[0];			// Read command
    num_args = addr[1] & ~ST_CMD_DELAY;	// Number of args to follow
    wait_ms  = addr[1] & ST_CMD_DELAY;
    addr += 2;
    memcpy(&arg, addr, num_args>4? 4: num_args);
    addr += num_args;
    tft_send_cmd(cmd, arg, num_args);    
    if (wait_ms > 0) {
      wait_ms = addr[0];
      addr++;
      vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }
  } // rof
  tft_disable();
  return ret;
}


esp_err_t ST7789_select_spi(void) {
  return sspi_device_select(TFT_HOST, TFT_DEVICE_NUM);
}


inline int ST7789_WaitWrComplete(void) {
  return sspi_wait_finish(TFT_HOST, ST7789_WAITSLPOUT_MS);
}


static void st7789_setWindow(uint32_t xa, uint32_t ya) {
  tft_enable();
  tft_send_cmd(ST7789_CASET, xa, 4);
  tft_send_cmd(ST7789_RASET, ya, 4);
  tft_send_cmd(ST7789_RAMWR, 0, 0);
}


static esp_err_t st7789_fill(unsigned short color_565, unsigned int px_count) {
  color_565 = __builtin_bswap16(color_565);
  for (int i=0; i < TFT_LOOPBUF_SIZE; i++) {
    tft_loop_buffer[i] = color_565;
  } // rof fill buffer
  return sspi_loop_data(TFT_HOST, tft_loop_buffer, TFT_LOOPBUF_SIZE * sizeof(unsigned short), px_count << 1, 0);  
}


void ST7789_OnOff(int onOff) {
  if (ST7789_WaitWrComplete()) {
    tft_enable();
    tft_send_cmd(onOff? ST7789_DISPON: ST7789_DISPOFF, 0, 0);
    tft_disable();
  }
}


void ST7789_InversionOnOff(int onOff) {
  if (ST7789_WaitWrComplete()) {
    tft_enable();
    tft_send_cmd(onOff? ST7789_INVON: ST7789_INVOFF, 0, 0);
    tft_disable();
  }
}

void ST7789_SetGammaCurve(int curveNo) {
  if (ST7789_WaitWrComplete()) {
    uint8_t curve_sel = 1;
    if (curveNo < 4) curve_sel <<= curveNo;
    tft_enable();
    tft_send_cmd(ST7789_GAMSET, curve_sel, 1);
    tft_disable();
  }
}



void ST7789_SetWindow(short left, short top, short right, short bottom) {
  uint32_t xa = __builtin_bswap32(((uint32_t)(left+TFT_X_OFFSET) << 16) | (right+TFT_X_OFFSET));
  uint32_t ya = __builtin_bswap32(((uint32_t)(top+TFT_Y_OFFSET) << 16) | (bottom+TFT_Y_OFFSET));
  if (ST7789_WaitWrComplete()) {
    st7789_setWindow(xa, ya);
    tft_disable();
  }
}


void ST7789_Write(const unsigned short *px16data, unsigned int px_count) {
  if (ST7789_WaitWrComplete()) {
    tft_enable();
    sspi_transfer_data(TFT_HOST, px16data, NULL, px_count << 1, 0);
    tft_disable();
  }
}


void ST7789_Fill(unsigned short color_565, unsigned int size) {
  if (ST7789_WaitWrComplete()) {
    tft_enable();
    st7789_fill(color_565, size);
    tft_disable();
  } // fi
}


void ST7789_Sprite(short x, short y, unsigned short w, unsigned short h, const unsigned short *px16data) {
  x += TFT_X_OFFSET;
  y += TFT_Y_OFFSET;
  uint32_t xa = __builtin_bswap32(((uint32_t)x << 16) | (x + w - 1));
  uint32_t ya = __builtin_bswap32(((uint32_t)y << 16) | (y + h - 1));
  uint32_t px = ((uint32_t) w * h) << 1;
  if (ST7789_WaitWrComplete()) {
    st7789_setWindow(xa, ya);
    sspi_transfer_data(TFT_HOST, px16data, NULL, px, 0);
    tft_disable();
  }
}


void ST7789_FillRect(short x, short y, unsigned short w, unsigned short h, unsigned short color) {
  x += TFT_X_OFFSET;
  y += TFT_Y_OFFSET;
  uint32_t xa = __builtin_bswap32(((uint32_t)x << 16) | (x + w - 1));
  uint32_t ya = __builtin_bswap32(((uint32_t)y << 16) | (y + h - 1));
  uint32_t px =  (uint32_t) w * h;
  if (ST7789_WaitWrComplete()) {
    st7789_setWindow(xa, ya);
    st7789_fill(color, px);    
    tft_disable();
  }
}


void ST7789_FillBox(short left, short top, short right, short bottom, unsigned short color) {
  uint32_t xa = __builtin_bswap32(((uint32_t)(left+TFT_X_OFFSET) << 16) | (right+TFT_X_OFFSET));
  uint32_t ya = __builtin_bswap32(((uint32_t)(top+TFT_Y_OFFSET) << 16) | (bottom+TFT_Y_OFFSET));
  uint32_t px =  (uint32_t)(right-left+1) * (bottom-top+1);
  if (ST7789_WaitWrComplete()) {
    st7789_setWindow(xa, ya);  
    st7789_fill(color, px); 
    tft_disable();
  }
}




#ifdef TFT_MISO_Pin
void ST7789_ReadRect(unsigned short *raw24data, unsigned short x, unsigned short y, unsigned short w, unsigned short h) {
  uint32_t xa = __builtin_bswap32(((uint32_t)x << 16) | (x + w - 1));
  uint32_t ya = __builtin_bswap32(((uint32_t)y << 16) | (y + h - 1));
  uint32_t px =  (uint32_t)w * h;
  if (ST7789_WaitWrComplete()) {
    /*
    uint32_t raw_count = 0xFF;
    TFTSPI->CR1 = SPI_CR1_MSTR | SPI_BAUDRATEPRESCALER_8 | SPI_FIRSTBIT_MSB | SPI_POLARITY_HIGH | SPI_PHASE_2EDGE;
    tft_spi8bit();
    tft_enable();
    tft_send_cmd(ST7789_CASET, &xa, 4);
    tft_send_cmd(ST7789_RASET, &ya, 4);
    tft_send_cmd(ST7789_RAMRD, &raw_count, 2);	// dummy output (needed for read-en)
    tft_waitBSY();
    while (TFTSPI->SR & SPI_SR_RXNE) raw24data[0] = TFTSPI->DR; // empty FIFO
    tft_disable();
    tft_spi16bit();
    raw_count = ((px * 3)+2) >> 1;	// 24bit only
    tft_enable();
    tft_data();
    for (; raw_count > 0; raw_count--, raw24data++) {
      TFTSPI->DR = 0xFFFF;
      tft_waitBSY();
      raw24data[0] = __builtin_bswap16(TFTSPI->DR);
    } // rof
    tft_disable();
    TFTSPI->CR1 = SPI_CR1_MSTR | SPI_BAUDRATEPRESCALER_2 | SPI_FIRSTBIT_MSB | SPI_POLARITY_HIGH | SPI_PHASE_2EDGE;
    ST7789_Release();
    */
  }
}
#endif