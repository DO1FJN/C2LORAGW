/*
 * SSD1306.c
 *
 * This source file belongs to project 'C2LoRaGW'.
 * (c) 2024, DO1FJN (Jan Alte)
 *
 *  Created on: 2024-04-10
 *     Project: C2 Lora Gateway
 *      Author: Jan Alte, DO1FJN
 * 
 * Report:
 * 2024-04-14 first "quick'n'dirty works a bit" verison.
 */

#include "SSD1306.h"

#include "hardware.h"
#include "compiler.h"

#include "s_spi.h"
#include "driver/i2c.h"

#include "esp_err.h"
#include "esp_log.h"

#include <string.h>

#define SSD1306_I2C_WAIT_TICKS  pdMS_TO_TICKS(100)

#ifndef SSD1206_I2C_PORT
#define SSD1306_I2C_PORT        I2C_NUM_0
#endif

#ifndef SSD1306_SLAVE_ADDR
#define SSD1306_SLAVE_ADDR      (0x3C)
#endif

#define SSD1306_I2C_CTRL_CMD    0x00    // Co=0, D/C=0
#define SSD1306_I2C_CTRL_DATA   0x40    // Co=0, D/C=1
#define SSD1306_I2C_CTRL_CMD1   0x80    // Co=1, D/C=0


#define SSD1306_CMD_CONTRAST    0x81    // with 1 byte parameter (0 to 255, reset 0x7E)
#define SSD1306_CMD_ENTIRE      0xA4    // Bit 0 (1) = entire display light up (0) = normal operation
#define SSD1306_CMD_INVERSE     0xA6    // Bit 0 (1) = inverse display
#define SSD1306_CMD_ONOFF       0xAE    // Bit 0 (1) = enables display (normal mode) (0) = sleep mode

#define SSD1306_CMD_PAGESTART   0xB0    // Bits 2,1,0 = GDDRAM page start address

#define SSD1306_CMD_SEGREMAP    0xA0
#define SSD1306_CMD_VSCROLL     0xA3

#define SSD1306_CMD_MUXRATIO    0xA8    // next byte (parameter)
#define SSD1306_CMD_COMSCANDIR  0xC0    // Bit 3 (1) remapped, (0) normal mode
#define SSD1306_CMD_OFFSET      0xD3    // next byte = vertical shift by COM
#define SSD1306_CMD_COMPINCONF  0xDA    // next byte = Bit 5,4 alternative COM pin config
#define SSD1306_CMD_CLOCKRATIO  0xD5    // next byte = 2 nibbles: lower = D ratio, higher = osc freq
#define SSD1306_CMD_PRECHRGE    0xD9    // next byte = 2 nibbles: Phase I | Phase II
#define SSD1306_CMD_VCOMHLVL    0xDB    // Vcomh deselect level
#define SSD1306_CMD_NOP         0xE3

#define SSD1306_CMD_LOCOLSTART  0x00
#define SSD1306_CMD_HICOLSTART  0x10
#define SSD1306_CMD_ADDRMODE    0x20
#define SSD1306_CMD_COLADDR     0x21
#define SSD1306_CMD_PAGEADDR    0x22    // 2 byte cmd (start | end) for hor/vert addressing modes

#define SSD1306_CMD_STARTLINE   0x40    // Bits 5-0 1= line 0 to 63


#define SSD1306_STD_MUX_RATIO   0x3F    // multiplex Ratio for 128x64 (64-1), reset value 
#define SSD1306_STD_CLOCKRATIO  0x80    // reset value
#define SSD1306_STD_DISP_OFFSET 0x00    // reset value
#define SSD1306_PAR_COMPINCONF  0x10    // reset value

#define SSD1306_STD_CONTRAST    0xCF    // higher than reset (7Fh)
#define SSD1306_STD_PRECHARGE   0xF1    // reset is 2-2
#define SSD1306_STD_VCOMDSELLVL 0x40    // 0x30 adafruit init 0x40

// not in ocumentation ???
#define SSD1306_CMD_CHARGEPUMP  0x8D
#define SSD1306_CHARGEPUMP_INT  0x14


static const char *TAG = "SSD1306";


static const uint8_t generic_init_seq[] = {
  SSD1306_I2C_CTRL_CMD1, SSD1306_CMD_ONOFF,
#ifdef SSD1306_STD_CLOCKRATIO 
  SSD1306_I2C_CTRL_CMD, SSD1306_CMD_CLOCKRATIO, SSD1306_STD_CLOCKRATIO,
#endif
#ifdef SSD1306_STD_MUX_RATIO
  SSD1306_I2C_CTRL_CMD, SSD1306_CMD_MUXRATIO, SSD1306_STD_MUX_RATIO,
#endif
#ifdef SSD1306_STD_DISP_OFFSET
  SSD1306_I2C_CTRL_CMD, SSD1306_CMD_OFFSET, SSD1306_STD_DISP_OFFSET,
#endif
  SSD1306_I2C_CTRL_CMD1, SSD1306_CMD_STARTLINE,
  
  SSD1306_I2C_CTRL_CMD, SSD1306_CMD_CHARGEPUMP, SSD1306_CHARGEPUMP_INT,
  SSD1306_I2C_CTRL_CMD1, SSD1306_CMD_SEGREMAP   | 1,
  SSD1306_I2C_CTRL_CMD1, SSD1306_CMD_COMSCANDIR | 8,
  SSD1306_I2C_CTRL_CMD, SSD1306_CMD_COMPINCONF, SSD1306_PAR_COMPINCONF | 2,
  SSD1306_I2C_CTRL_CMD, SSD1306_CMD_CONTRAST, SSD1306_STD_CONTRAST,
  SSD1306_I2C_CTRL_CMD, SSD1306_CMD_PRECHRGE, SSD1306_STD_PRECHARGE,
  SSD1306_I2C_CTRL_CMD, SSD1306_CMD_VCOMHLVL, SSD1306_STD_VCOMDSELLVL,

  SSD1306_I2C_CTRL_CMD1, SSD1306_CMD_ENTIRE | 0,
  SSD1306_I2C_CTRL_CMD1, SSD1306_CMD_INVERSE | 0,
};

#define I2C_TRANS_BUF_SIZE      I2C_LINK_RECOMMENDED_SIZE(8)

#define SSD1306_WINDOW_INVERS_FLAG  0x01

typedef struct {
  uint8_t top, left;
  uint8_t right, bottom;
  uint8_t wx_pos, wy_pos;
  uint8_t flags;
} tWindow;


static uint8_t lines_visible;
#if OLED_IF_TYPE==0
static uint8_t i2c_link_buffer[I2C_TRANS_BUF_SIZE];
#endif
static uint8_t ssd1306_disp_buf[1024];

// ToDo this is no thread safe. Need to be a handler.
static tWindow ssd1306_op_win;


esp_err_t SSD1306_Init(unsigned char no_of_lines) {
  esp_err_t err;
#if OLED_IF_TYPE==0
  err = i2c_master_write_to_device(SSD1306_I2C_PORT, SSD1306_SLAVE_ADDR, generic_init_seq, sizeof(generic_init_seq), SSD1306_I2C_WAIT_TICKS);
  memset(i2c_link_buffer, 0, sizeof(i2c_link_buffer));
#endif
#if OLED_IF_TYPE==1
// tbd
#endif
  
  memset(ssd1306_disp_buf, 0, sizeof(ssd1306_disp_buf));  
  lines_visible = no_of_lines;
  uint8_t addr_mode_par = 0;
  SSD1306_write_command(SSD1306_CMD_ADDRMODE, &addr_mode_par, 1);
  SSD1306_write_data(ssd1306_disp_buf, sizeof(ssd1306_disp_buf));
  return err;
}


#if OLED_IF_TYPE==0

esp_err_t SSD1306_write_data(const unsigned char *buffer, unsigned short len) {        
  esp_err_t err = ESP_OK;
  i2c_cmd_handle_t ssd_hnd;
  ssd_hnd = i2c_cmd_link_create_static(i2c_link_buffer, sizeof(i2c_link_buffer));
  if (ssd_hnd == NULL) return ESP_ERR_NOT_FOUND;

  if (len == 0) return ESP_OK;

  err = i2c_master_start(ssd_hnd);
  if (err != ESP_OK) goto end;
  err = i2c_master_write_byte(ssd_hnd, SSD1306_SLAVE_ADDR << 1 | I2C_MASTER_WRITE, true);
  if (err != ESP_OK) goto end;
  err = i2c_master_write_byte(ssd_hnd, SSD1306_I2C_CTRL_DATA, true);
  if (err != ESP_OK) goto end;
  err = i2c_master_write(ssd_hnd, buffer, len, true);
  if (err != ESP_OK) goto end;
  i2c_master_stop(ssd_hnd);
  err = i2c_master_cmd_begin(SSD1306_I2C_PORT, ssd_hnd, SSD1306_I2C_WAIT_TICKS);
end:
  i2c_cmd_link_delete_static(ssd_hnd);
  return err;
}


esp_err_t SSD1306_write_command(unsigned char cmd_byte, const unsigned char *cmd_arg, unsigned short len) {
  esp_err_t err = ESP_OK;
  i2c_cmd_handle_t ssd_hnd;
  ssd_hnd = i2c_cmd_link_create_static(i2c_link_buffer, sizeof(i2c_link_buffer));
  if (ssd_hnd == NULL) return ESP_ERR_NOT_FOUND;

  err = i2c_master_start(ssd_hnd);
  if (err != ESP_OK) goto end;
  err = i2c_master_write_byte(ssd_hnd, SSD1306_SLAVE_ADDR << 1 | I2C_MASTER_WRITE, true);
  if (err != ESP_OK) goto end;
  err = i2c_master_write_byte(ssd_hnd, SSD1306_I2C_CTRL_CMD, true);
  if (err != ESP_OK) goto end;
  err = i2c_master_write_byte(ssd_hnd, cmd_byte, true);
  if (err != ESP_OK) goto end;
  if (len > 0) {
    err = i2c_master_write(ssd_hnd, cmd_arg, len, true);
    if (err != ESP_OK) goto end;
  }
  i2c_master_stop(ssd_hnd);
  err = i2c_master_cmd_begin(SSD1306_I2C_PORT, ssd_hnd, SSD1306_I2C_WAIT_TICKS);

end:
  i2c_cmd_link_delete_static(ssd_hnd);
  return err;
}


esp_err_t SSD1306_set_window(uint8_t col_s, uint8_t page_s, uint8_t col_e, uint8_t page_e) {
  esp_err_t err = ESP_OK;
  i2c_cmd_handle_t ssd_hnd;
  ssd_hnd = i2c_cmd_link_create_static(i2c_link_buffer, sizeof(i2c_link_buffer));
  if (ssd_hnd == NULL) return ESP_ERR_NOT_FOUND;
  uint8_t set_colpage[] = {
    SSD1306_SLAVE_ADDR << 1 | I2C_MASTER_WRITE,
    SSD1306_I2C_CTRL_CMD, SSD1306_CMD_PAGEADDR, page_s, page_e,
    SSD1306_I2C_CTRL_CMD, SSD1306_CMD_COLADDR, col_s, col_e
  };
  err = i2c_master_start(ssd_hnd);
  if (err != ESP_OK) goto end;
  err = i2c_master_write(ssd_hnd, set_colpage, sizeof(set_colpage), true);
  if (err != ESP_OK) goto end;
  err = i2c_master_stop(ssd_hnd);
  if (err != ESP_OK) goto end;

  err = i2c_master_cmd_begin(SSD1306_I2C_PORT, ssd_hnd, SSD1306_I2C_WAIT_TICKS);
end:
  i2c_cmd_link_delete_static(ssd_hnd);
  return err;
}


#endif



esp_err_t SSD1306_OnOff(bool on_off) {
  return SSD1306_write_command(SSD1306_CMD_ONOFF | (on_off? 1: 0), NULL, 0);
}


esp_err_t SSD1306_Contrast(unsigned char contrast) {
  return SSD1306_write_command(SSD1306_CMD_CONTRAST, &contrast, 1);
}


esp_err_t SSD1306_UpdateAll(void) {
  esp_err_t err;
  ESP_LOGV(TAG, "update disp");
  //ESP_LOG_BUFFER_HEX(TAG, ssd1306_disp_buf, sizeof(ssd1306_disp_buf));

  err = SSD1306_set_window(0, 0, 127, 7);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "can't set window");
    return err;
  }
  return SSD1306_write_data(ssd1306_disp_buf, sizeof(ssd1306_disp_buf));
}


esp_err_t SSD1306_InverseDisp(bool on_off) {  
  return SSD1306_write_command(SSD1306_CMD_INVERSE | (on_off? 1: 0), NULL, 0);
}


esp_err_t SSD1306_FillRect(unsigned char x, unsigned char y, unsigned char w, unsigned char h, bool px_on) {
  uint8_t *col = ssd1306_disp_buf + ((y & 0x38) << 4) + x;
  uint8_t col_next_line = 128 - w;

  for ( ; (h > 0) && (w > 0); ) {
    uint8_t sftp  = y & 7;
    uint8_t block = 8 - sftp;
    uint8_t mask  = 0xFF << sftp;
    if (h < block) {
      mask &= 0xFF >> (8 - h);
      block = h;
    }
    //ESP_LOGD(TAG, "fill nxt %u,%u [%uw %ul], %02xm %ub", ssd1306_op_win.wx_pos, ssd1306_op_win.wy_pos, width, no_of_lines, mask, block);
    for (int c = w; c > 0; c--, col++) {
      if (px_on) col[0] |= mask; else col[0] &= ~mask;
    } // cols   
    col += col_next_line;
    y   += block;
    h   -= block;
  } // rof
  //ESP_LOGD(TAG, "wypos = %u", ssd1306_op_win.wy_pos);
  return ESP_OK;
}



esp_err_t SSD1306_SetWindow(char left, char top, char right, char bottom, bool px_on) {
  esp_err_t err = ESP_OK;
  ssd1306_op_win.left   = left;
  ssd1306_op_win.top    = top;
  ssd1306_op_win.right  = right;
  ssd1306_op_win.bottom = bottom;
  ssd1306_op_win.wx_pos = left;
  ssd1306_op_win.wy_pos = top;
  ssd1306_op_win.flags  = px_on? SSD1306_WINDOW_INVERS_FLAG: 0;
  return err;  
}

static inline uint8_t *ssd1306_getcolbyte(void) {
  uint8_t *col = ssd1306_disp_buf + ((ssd1306_op_win.wy_pos & 0x38) << 4) + ssd1306_op_win.wx_pos;
  return col;
}

static inline uint8_t ssd1306_getcolmask(void) {
  uint8_t mask = 1 << (ssd1306_op_win.wy_pos & 7);
  return mask;
}

static inline uint8_t ssd1306_getcolarea(uint8_t no_of_lines) {
  uint8_t sftp = ssd1306_op_win.wy_pos & 7;
  uint8_t mask = 0xFF << sftp;
  if (no_of_lines < (8-sftp)) {
    mask &= 0xFF >> (8-no_of_lines);
  }
  return mask;
}


void SSD1306_FillNextRect(unsigned char width, unsigned char no_of_lines, unsigned char onOff) {
  if (ssd1306_op_win.flags & SSD1306_WINDOW_INVERS_FLAG) onOff = !onOff;
  for ( ; no_of_lines > 0; ) {
    uint8_t block = 8 - (ssd1306_op_win.wy_pos & 7);
    uint8_t *col = ssd1306_getcolbyte();
    uint8_t mask = ssd1306_getcolarea(no_of_lines);    
    if (block > no_of_lines) block = no_of_lines;

    //ESP_LOGD(TAG, "fill nxt %u,%u [%uw %ul], %02xm %ub", ssd1306_op_win.wx_pos, ssd1306_op_win.wy_pos, width, no_of_lines, mask, block);
    for (int c = width; c > 0; c--, col++) {
      if (onOff) col[0] |= mask; else col[0] &= ~mask;
    } // cols    
    ssd1306_op_win.wy_pos += block;
    no_of_lines           -= block;
  } // rof
  //ESP_LOGD(TAG, "wypos = %u", ssd1306_op_win.wy_pos);
}


void SSD1306_SetNextLine(void) {
  if (ssd1306_op_win.wx_pos == ssd1306_op_win.left) return;
  ssd1306_op_win.wy_pos++;
  ssd1306_op_win.wx_pos = ssd1306_op_win.left;
}


void SSD1306_SetNextPixel(unsigned char onOff) {
  uint8_t *col = ssd1306_getcolbyte();
  uint8_t mask = ssd1306_getcolmask();
  // nothing to see here...
  if (ssd1306_op_win.flags & SSD1306_WINDOW_INVERS_FLAG) onOff = !onOff;
  if (onOff) col[0] |= mask; else col[0] &= ~mask;
  ssd1306_op_win.wx_pos++;
  if (ssd1306_op_win.wx_pos > ssd1306_op_win.right) {
    SSD1306_SetNextLine();
  }
}
