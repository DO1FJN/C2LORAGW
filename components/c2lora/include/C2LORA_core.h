/*

C2LORA_code.h

This header file belongs to project 'C2LoRaGW'.
(c) 2024, DO1FJN (Jan Alte)

main API file for accessing the C2LORA UHF link

*/
#pragma once

#include "C2LORA.h"
#include "localaudio.h"
#include "localrecorder.h"

#include "sx126x.h"
#include "s_spi.h"

#include "stdint.h"
#include "stdbool.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"


#define C2LORA_DVOICE_FRAMELEN_MS     480       // packet length (airtime) / speech data length for a lora transmission
#define C2LORA_WHEADER_FRAMELEN_MS    580       // max value for all rates for timeout

#define C2LORA_FRAME_STARTBYTE        0x33      // an pattern with 4 '1' and 4 '0', inverted (0xCC) for a header, RX decides with or w/o header by compare+count bits 

#define SX126X_CMD_CLEAR_IRQSTA       0x02
#define SX126X_CMD_START_TX           0x83
#define SX126X_CMD_SET_BASEADR        0x8F

#define SX126X_SIZE_PKT_PARAMS        7
#define SX126X_PKT_PARAMS_CMD         0x8C
#define SX126X_PKT_PRE_BYTEPOS        2
#define SX126X_PKT_LEN_BYTEPOS        4

#define C2LORA_MAX_HEADER_SIZE        24      // size of the bytebuffer

#define C2LORA_EVENT_RECEIVE_ACTI     0x0001
#define C2LORA_EVENT_TRANSMIT_FINI    0x0002
#define C2LORA_EVENT_TERMINATE        0x4000
#define C2LORA_EVENT_RECONFIGURE      0x8000

#define C2LORA_EVENT_SX126X_INT       0x10000  // DIO1 based interrupt was fired

#define C2LORA_ALL_EVENTS             (C2LORA_EVENT_RECONFIGURE | C2LORA_EVENT_TERMINATE | C2LORA_EVENT_SX126X_INT)

#define C2LORARX_INTMASK              (SX126X_IRQ_RX_DONE|SX126X_IRQ_PREAMBLE_DETECTED)


typedef enum {
  LS_INACTIVE = 0, LS_IDLE, LS_CALIBRATE,
  TX_SEND_HEADER, TX_SEND_FRAME, TX_FRAME_DONE, TX_FRAME_GAP, TX_LAST_FRAME,
  RX_NOSYNC, RX_PREAMBLE, RX_HEADER, RX_DATA, RX_DONE
} tLoraTxState;


typedef struct {
  uint16_t        bits_header;
  uint16_t        bits_dvoice;   // size if encoded digital voice within the 480ms
  uint16_t        bits_cydata;
  int32_t         min_pkt_end_time;
  int32_t         max_pkt_end_time;
} tLoraStreamPara;


typedef uint32_t (*tLoraCmdFunc) (tLoraStream *lora, void *arg);


struct sLoraStream {
  tsx126x_ctx *   ctx;            // a context used for hardware / SPI related stuff
  const tC2LORAdef *def;          // the C2LORA mode as a pointer to its definition
  tLoraTxState    state;
  uint32_t        frequency;      // frequency for receiving in [Hz]
  int32_t         freq_shift;     // if not zero: transmit frequency is "frequency + freq_shift"
  int16_t         freq_offset;    // calibrated offset for exact output frequency (max +/- 32kHz)
  int8_t          tx_power_dBm;   // transmit power of SX126x device (external PA gain not added; -9 to +22dBm)

  QueueHandle_t   cmd_queue;
  EventGroupHandle_t events;
  TaskHandle_t    task_to_notify;

  tLoraStreamPara p;              // parameter loaded once before stream begins
  gpio_num_t      intr_pin;       // usually the DIO1 output of a SX126x device, used to generate an interupt
  gpio_num_t      rxen_pin;       // for an separate RX switch (LNA) input
  gpio_num_t      txfb_pin;       // usually the DIO2 output of a SX126x device (if availabe)

  uint32_t        frame_cnt;      // nubmer of total speech frames in a transmission (counts up)
  uint32_t        frame_himark;   // max frames to put into the SX126x buffer in advance
  uint8_t         pkt_offset;     // offset within the SX126x data buffer, defined for the actual operation
  uint8_t         pkt_framecnt;   // number of actual frame written or read to/from buffer for actual packet

  union {
    uint8_t       wr_incomplete_byte;
    uint8_t       rd_rsvd;        // no of bytes, programmed within RXdone ISR
  };
  union {
    uint16_t      wr_offset;      // bit-address within the SX126x data buffer, used for write C2
    uint16_t      rd_offset;      // bit-address within the received bytes, start of C2 frame
  };
  union {
    uint16_t      wr_cy_ofs;      // bit-address within the SX126x data buffer, used for write cydata
    uint16_t      rd_cy_ofs;      // bit-address
  };
  int64_t         pkt_start_time; // needed for continuous TX / RX ahead reads
  
  bool            update_pkt_params;
  uint8_t         pkt_params[SX126X_SIZE_PKT_PARAMS+1]; // 7 needed
  uint8_t         header[C2LORA_MAX_HEADER_SIZE];       // max 20 bytes are used
  uint8_t         cydata[32];
  tdvstream *     dva;            // IN/OUT encoded stream, NULL if not in use
  tRecorderHandle *rec;
  uint32_t        udp_stream_id;  // an ID for an UDP-RTP (HAMdLNK) output stream
  tLoraCmdFunc    finish_tx;
};


esp_err_t C2LORA_init_spi(void);

esp_err_t C2LORA_prepare_streamstruct(tLoraStream *lorastream, tC2LORA_mode mode);

esp_err_t C2LORA_init_sx126x(tLoraStream *lora, const tsx126x_config *cfg);

void C2LORA_prepare_cydata(uint8_t *cydata, uint8_t d_length);

bool C2LORA_is_header(bool last_was_header, uint8_t first_byte, int32_t interpkt_dur_us);

esp_err_t C2LORA_set_standby(tLoraStream *lora, sx126x_standby_cfgs_t mode);

esp_err_t C2LORA_set_tx_power(tLoraStream *lora, signed char pwr_dBm);

esp_err_t C2LORA_prepare_transmit(tLoraStream *lora, bool preamble_for_rt_speech);

esp_err_t C2LORA_begin_transmit(tLoraStream *lora);

esp_err_t C2LORA_finish_transmit(tLoraStream *lora);

esp_err_t C2LORA_clear_irq_mask(const tLoraStream *lora);

esp_err_t C2LORA_prepare_receive(tLoraStream *lora, uint32_t *symbol_time, uint32_t *preamble_us, uint32_t *firstdata_us, uint32_t *cydata_us);


sx126x_errors_mask_t SX126x_GetDeviceError(const tsx126x_ctx *ctx);

void SX126x_print_statuscode_deverrs(const void *context);

#if CONFIG_LOG_DEFAULT_LEVEL > 2
#define DEBUG_SX126X_INFOS(ctx)   SX126x_print_statuscode_deverrs(ctx)
#else
#define DEBUG_SX126X_INFOS(ctx)
#endif

esp_err_t C2LORA_read_registers_at(tLoraStream *lora, unsigned short addr);

// in c2lora_uhf:

void      C2LORA_handle_encoded(tdvstream *dva, void *userdata);

esp_err_t C2LORA_task_run_cmd(tLoraStream *lora, tLoraCmdFunc cmd_funct, void *cmd_arg, bool no_wait);

