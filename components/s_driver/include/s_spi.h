
#pragma once

#include "compiler.h"

#include "esp_err.h"
#include "soc/gpio_num.h"

#define SSPI_MAX_DEVICES        3        // maxdevices per SPI master

#define SPI1_NUM    0
#define SPI2_NUM    1
#define SPI3_NUM    2


typedef struct {
  gpio_num_t sclk;
  gpio_num_t mosi;
  gpio_num_t miso;
  union {
    gpio_num_t quadwp;
    gpio_num_t data2;
  };
  union {
    gpio_num_t quadhd;
    gpio_num_t data3;  
  };
  gpio_num_t cd_sel;
} tsspi_gpio;


typedef struct {
  unsigned int          speed_hz;
  unsigned int          flags;
  tsspi_gpio            gpio;
} tsspi_bus_config;


typedef struct {
  gpio_num_t    cs_pin;
  gpio_num_t    dc_pin;                 // command / data gpio pin used on many TFT displays
  uint8_t       cmd_bits;               ///< Default amount of bits in command phase (0-16), used when ``SPI_TRANS_VARIABLE_CMD`` is not used, otherwise ignored.
  uint8_t       addr_bits;              ///< Default amount of bits in address phase (0-64), used when ``SPI_TRANS_VARIABLE_ADDR`` is not used, otherwise ignored.
  uint8_t       dummy_bits;             ///< Amount of dummy bits to insert between address and data phase
  uint8_t       cs_setup_time;
  uint8_t       cs_hold_time;
  struct {
    uint8_t cpol: 1;
    uint8_t cpha: 1;
  } mode;
  struct {
    uint8_t half_duplex: 1;
    uint8_t sio_mode: 1;    // in half duplex only: a single line (MOSI) is used fpr Data-Out and Data-Im
    uint8_t cs_setup: 1;
    uint8_t cs_hold: 1;
  } conf;
  
} tsspi_device_cfg;


esp_err_t       sspi_init(int num, const tsspi_bus_config *bus_config);

esp_err_t       sspi_device_init(int num, int device, const tsspi_device_cfg *dev_config);


esp_err_t       sspi_device_select(int num, int device);

/* sspi_wait_finish()
 * waits till the running transfer is done (startet with wait_ms=0)
 * must be called within the same thead. changing to another devide is not possible due a runnig transfer.
 */
bool            sspi_wait_finish(int num, unsigned int timeout_ms);


esp_err_t       sspi_send_cmd_w_arg(int num, U8 cmd, U8 arg_size, U32 arg);

esp_err_t       sspi_transfer_data(int num, const void *data_to_send, void *receive_buffer, unsigned int length, int wait_ms);

esp_err_t       sspi_loop_data(int num, const void *loop_buffer_to_send, unsigned int buffer_size, unsigned int length, int wait_ms);




typedef struct {
  int           spi_num;
  int           spi_device;
  // pins used by HAL driver
  gpio_num_t    nss_pin;
  gpio_num_t    busy_pin;
  gpio_num_t    reset_pin;
  // Pins below are not used by HAL and not configured in INIT
  gpio_num_t    dio_pin[3];
  gpio_num_t    ext_rxsw_pin;
} tsx126x_config;


typedef struct sx126x_ctx_struct tsx126x_ctx;


/* sx126x_init_hal()
 * configure and init a SPI device for the sx126x_driver, returns a created (malloc) context needed for handling
_* sx126x related IO
 */
esp_err_t       sx126x_init_hal(tsx126x_ctx **context, const tsx126x_config *sx126x_config);

bool            sx126x_is_busy(const tsx126x_ctx *ctx);
bool            sx126x_hal_wait_busy(const tsx126x_ctx* context);

unsigned char * sx126x_get_status_buffer(const tsx126x_ctx *context);

// ISR proof non-block fire-and-forget SPI write function:
esp_err_t       sx126x_hal_fast_cmd(const tsx126x_ctx *ctx, const uint8_t* command, uint16_t command_length);

esp_err_t       sx126x_hal_fast_bufferwrite(const tsx126x_ctx *ctx, uint8_t offset, const uint8_t* data, uint8_t length);

esp_err_t       sx126x_hal_wait_spi(const tsx126x_ctx *ctx);
