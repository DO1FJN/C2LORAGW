
#pragma once

#include "s_buffer.h"
#include "esp_err.h"

#include <soc/gpio_num.h>


typedef enum {
  SLOT_MODE_MONO   = 1,          // !< I2S channel slot format mono, transmit same data in all slots for tx mode, only receive the data in the first slots for rx mode.
  SLOT_MODE_STEREO = 2,          // !< I2S channel slot format stereo, transmit different data in different slots for tx mode, receive the data in all slots for rx mode.
} ti2sloop_slotmode;

typedef struct {
  ti2sloop_slotmode       mode;
  unsigned char           bitwidth;        // total width of a slot in bits (0 = aout, 8, 16, 24 or 32 are valid)
  unsigned char           databits;        // used bits (<= slot_bitwith) per slot als data (8, 16, 24 or 32 are valid)
  unsigned char           ws_width;        // WS signal width in bits
  unsigned char           mask;            // Bit 0 = LEFT, Bit 1 = right
  union {
    unsigned char flags;
    struct {
      unsigned char ws_pol: 1;
      unsigned char bit_shift: 1;               // Philips mode
      unsigned char left_align: 1;              // not used in ESP32 + ESP32-S2 targets!
      unsigned char big_endian: 1;              // not used in ESP32 + ESP32-S2 targets!
      unsigned char bit_order_lsb: 1;           // not used in ESP32 + ESP32-S2 targets!
      unsigned char msb_right: 1;               // ESP32 + ESP32-S2 ONLY: !< Set to place right channel data at the MSB in the FIFO
    };
  };
} ti2sloop_slotcfg;

typedef struct {
  gpio_num_t mclk;               // !< MCK pin, output by default, input if the clock source is selected to `I2S_CLK_SRC_EXTERNAL`
  // ! only one and only MCKL pin exists even for a ESP32-S3! Using in a Dual-Channel config: set one to GPIO_NUM_NC
  gpio_num_t bclk;               // !< BCK pin, input in slave role, output in master role
  gpio_num_t ws;                 // !< WS pin, input in slave role, output in master role
  gpio_num_t dio;                // !< DATA pin, input or output
  struct {
    uint32_t   mclk_inv: 1;      // !< Set 1 to invert the MCLK input/output
    uint32_t   bclk_inv: 1;      // !< Set 1 to invert the BCLK input/output 
    uint32_t   ws_inv: 1;        // !< Set 1 to invert the WS input/output 
  } invert_flags;                // !< GPIO pin invert flags
} ti2sloop_gpio;


typedef struct {
  unsigned int            sample_rate_hz;
  ti2sloop_slotcfg        slot;   // TX slot config (MASTER 4 calculate clock frequencies)
  ti2sloop_gpio           gpio;
} ti2sloop_config;


typedef bool (*thandle_received_frame_funct) (tsimple_buffer *rx_buf, void *user_data);


esp_err_t i2sloop_create_loop(tsimple_buffer *loop_buffer, U16 frame_cnt, U16 frame_size, U16 steps_per_frame);

esp_err_t i2sloop_init(int num, const ti2sloop_config *tx_conf, tsimple_buffer *tx_buf, const ti2sloop_config *rx_conf, tsimple_buffer *rx_buf);

tsimple_buffer *i2sloop_get_tx_buffer(int num);
tsimple_buffer *i2sloop_get_rx_buffer(int num);

esp_err_t i2sloop_set_rx_handler(int num, thandle_received_frame_funct handler, void *user_data);

esp_err_t i2sloop_reconfig_rx(int num, U16 frame_cnt, U16 frame_size, U16 steps_per_frame);

esp_err_t i2sloop_enable(int num);
esp_err_t i2sloop_disable(int num);
esp_err_t i2sloop_shutdown_both(int num);

esp_err_t i2sloop_enable_tx(int num);
esp_err_t i2sloop_disable_tx(int num);
esp_err_t i2sloop_shutdown_tx(int num);

esp_err_t i2sloop_enable_rx(int num);
esp_err_t i2sloop_disable_rx(int num);
esp_err_t i2sloop_shutdown_rx(int num);

