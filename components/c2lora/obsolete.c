

esp_err_t C2LORA_init_2nd_4rx(void) {
  tLoraStream *lora = &lora_2nd_stream;
  return xTaskCreate(C2LORA_rx_task, "C2LORA_2ndRX", C2LORARX_TASK_STACK_SIZE, lora, C2LORARX_TASK_PRIORITY, NULL)? ESP_OK: ESP_FAIL;
}




static void IRAM_ATTR gpio_rx_handler(void *arg) {
  tLoraStream *lora = (tLoraStream *) arg;
  BaseType_t isAwakeYeah = pdFALSE;
  lora->pkt_start_time = esp_timer_get_time(); // invalid if the SX126x INT was a RXdone, doesn't matter
  xEventGroupSetBitsFromISR(lora->events, C2LORA_EVENT_SX126X_INT, &isAwakeYeah);
  ESP_EARLY_LOGV(TAG, "RX isr");
}


static void C2LORA_rx_task(void *thread_data) {
  esp_err_t err;
  sx126x_hal_status_t s;
  tLoraStream *lora = (tLoraStream *) thread_data;
  TickType_t task_timeout = portMAX_DELAY;
  tdvstream C2LORArx;
  bool rx_is_running = true;

  bool was_repeated;
  tKindOfSender kos;
  char callsign[8];
  char destination[8];
  
  if (lora == NULL) goto c2lora_rx_shutdown;

  if (lora->events == 0) {
    ESP_LOGE(TAG, "RX no events - exit rx thread");
    goto c2lora_rx_shutdown;
  }
  ESP_LOGD(TAG, "RX task init");

  memset(&C2LORArx, 0, sizeof(tdvstream));
  C2LORArx.srate      = 8000;
  C2LORArx.codec_type = 255;
  C2LORArx.flags      = DVSTREAM_FLAG_KEEP_BUFFER;
  C2LORArx.buf        = (tsimple_buffer *) malloc(sizeof(tsimple_buffer));

  memset(callsign, 0, sizeof(callsign));
  memset(destination, 0, sizeof(destination));

  err = gpio_isr_handler_add(lora->intr_pin, gpio_rx_handler, (void*) lora);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "init GPIO ISR failed (%s)", esp_err_to_name(err));
    goto c2lora_rx_shutdown;
  }

#if CONFIG_USE_TEST_BITPATTERN
  // clear it 4 test 
  memset(rx_packet, 0xA5, sizeof(rx_packet));
  sx126x_write_buffer(lora->ctx, 1, rx_packet, 255);
#endif

  xEventGroupClearBits(lora->events, C2LORA_ALL_EVENTS);

  while (rx_is_running) {
    C2LORArx.streamid   = -1;
    if (isSF_CODEC2(lora->def->codec_type)) {
      C2LORArx.codec_type = lora->def->codec_type;
    } else {
      C2LORArx.codec_type = 255;  // some other stuff - not Codec2
    }
    C2LORArx.samples_per_frame = lora->def->dv_frames_per_packet==12? 320: 160;
    C2LORArx.bits_per_frame  = sf_get_subframe_bits(lora->def->codec_type);
    C2LORArx.bytes_per_frame = (C2LORArx.bits_per_frame + 7) >> 3;

    lora->state = RX_NOSYNC;
    lora->dva   = &C2LORArx;
    lora->p.bits_header = lora->def->bytes_per_header << 3;
    lora->p.bits_dvoice = lora->def->dv_frames_per_packet * C2LORArx.bits_per_frame;
    lora->p.bits_cydata = ((lora->def->bytes_per_packet-1) << 3) - lora->p.bits_dvoice; // minus 1 für START

    lora->pkt_params[SX126X_PKT_PRE_BYTEPOS] = lora->def->default_preamble + 1; // should be the max possible preamble
    if (lora->def->firstDV_preamble > lora->pkt_params[SX126X_PKT_PRE_BYTEPOS]) { // work not well
      lora->pkt_params[SX126X_PKT_PRE_BYTEPOS] = lora->def->firstDV_preamble;
    } // fi longer preamble used
    lora->pkt_params[SX126X_PKT_LEN_BYTEPOS] = lora->def->bytes_per_header + lora->def->bytes_per_packet;

    uint32_t symbol_time_us   = c2lora_calc_symboltime(lora->def);
    uint32_t preamble_time_us = c2lora_calc_firstprefetch(lora->def, symbol_time_us, 0, false);
    uint32_t firstdat_time_us = c2lora_calc_firstprefetch(lora->def, symbol_time_us, lora->dva->bytes_per_frame, false);

    uint32_t frame_time_us    = c2lora_calc_rxlength(lora->def, symbol_time_us, lora->dva->bytes_per_frame, 0);

    // Todo calulate no of symbols for cydata length (bytes) and get the needed tickcnt from it...
    TickType_t cyclic_data_to = c2lora_calc_rxlength(lora->def, symbol_time_us, (lora->p.bits_cydata + 7) >> 3, 0);

    ESP_LOGI(TAG, "calculated preamble = %luµs, symboltime %luµs, frame airtime %luµs, cyclic data %luµs", preamble_time_us, symbol_time_us, frame_time_us, cyclic_data_to);

    cyclic_data_to = (cyclic_data_to + 10000) / (configTICK_RATE_HZ * 100);

    if (!sbuf_create(lora->dva->buf, lora->def->dv_frames_per_packet, lora->dva->bytes_per_frame, lora->dva->samples_per_frame)) {
      ESP_LOGW(TAG, "no speech/voice buffer created!");
    } // fi create
    
    sx126x_lock();
    
    ESP_LOGD(TAG, "set dio irq params");
    s = sx126x_set_dio_irq_params(lora->ctx, SX126X_IRQ_PREAMBLE_DETECTED, C2LORARX_INTMASK, 0x00, 0x00);
    if (s) ESP_LOGE(TAG, "set dio irq params");
    DEBUG_SX126X_INFOS(lora->ctx);

    ESP_LOGD(TAG, "set rx/tx fallback mode");
    s = sx126x_set_rx_tx_fallback_mode(lora->ctx, SX126X_FALLBACK_FS);
    if (s) ESP_LOGE(TAG, "set rx/tx fallback mode");
    DEBUG_SX126X_INFOS(lora->ctx);

    ESP_LOGD(TAG, "set rx continuous");
    s = sx126x_set_rx_with_timeout_in_rtc_step(lora->ctx, SX126X_RX_CONTINUOUS);
    if (s) ESP_LOGE(TAG, "set rx continuous");
    DEBUG_SX126X_INFOS(lora->ctx);

    ESP_LOGD(TAG, "clear irq status");
    s = sx126x_clear_irq_status(lora->ctx, C2LORARX_INTMASK);
    if (s) ESP_LOGE(TAG, "clear irq status");
    DEBUG_SX126X_INFOS(lora->ctx);

    // update packet params:
    sx126x_hal_fast_cmd(lora->ctx, lora->pkt_params, SX126X_SIZE_PKT_PARAMS);

    gpio_set_intr_type(lora->intr_pin, GPIO_INTR_POSEDGE);
    if (lora->rxen_pin != GPIO_NUM_NC) {
      gpio_set_level(lora->rxen_pin, 1);
    }
    sx126x_unlock();

    //sx126x_set_lora_symb_nb_timeout(lora->ctx, 4); // setting this results in a non reception!

    lora->frame_cnt = 0;
    ESP_LOGD(TAG, "RX receiving");
        
    uint8_t * rxpacket_ptr = rx_packet;
    uint8_t read_ahead_ofs = 0;
    bool    with_header    = false;

    do {

      EventBits_t rx_bits = xEventGroupWaitBits(lora->events, C2LORA_ALL_EVENTS, pdTRUE, pdFALSE, task_timeout);

//      sx126x_hal_wait_busy(lora->ctx);
      if (rx_bits & C2LORA_EVENT_TERMINATE) {
        rx_is_running = false;
        break;
      }

      if (rx_bits & (C2LORA_EVENT_RECONFIGURE)) break; // break while loop!

      if (rx_bits & C2LORA_EVENT_SX126X_INT) { // DIO1 ISR
        sx126x_irq_mask_t irq_mask;
        
        sx126x_lock();

        //sx126x_get_irq_status(lora->ctx, &irq_mask);
        sx126x_set_dio_irq_params(lora->ctx,  SX126X_IRQ_RX_DONE, C2LORARX_INTMASK, 0x00, 0x00);
        sx126x_get_and_clear_irq_status(lora->ctx, &irq_mask);

        // *** packet receive finished (must processed before PREAMBLE) ***
        if (irq_mask & SX126X_IRQ_RX_DONE) {
          sx126x_irq_mask_t         irq_mask_refresh;
          sx126x_rx_buffer_status_t rxbuffer_status = { 0, 0 };
          sx126x_pkt_status_lora_t  rxpacket_status;
          int rx_len;
          uint8_t cydata_ofs = ((with_header? 8 + lora->p.bits_header: 8) + lora->p.bits_dvoice + 7) >> 3; // Todo check 8nit clipping
          lora->state  = RX_DONE;
          task_timeout = pdMS_TO_TICKS(C2LORA_MAX_RXGAP_MS);
          sx126x_set_dio_irq_params(lora->ctx, SX126X_IRQ_PREAMBLE_DETECTED, C2LORARX_INTMASK, 0x00, 0x00);
          sx126x_get_rx_buffer_status(lora->ctx, &rxbuffer_status);
          
          rx_len = rx_packet + rxbuffer_status.pld_len_in_bytes - rxpacket_ptr;
          if (rx_len > 0) {
            ESP_LOGD(TAG, "last %d bytes of %d", rx_len, rxbuffer_status.pld_len_in_bytes);
            sx126x_read_buffer(lora->ctx, read_ahead_ofs, rxpacket_ptr, rx_len);
          } // fi
#if CONFIG_USE_TEST_BITPATTERN
//          ESP_LOG_BUFFER_HEX_LEVEL(TAG, rx_packet, rxbuffer_status.pld_len_in_bytes, ESP_LOG_DEBUG);
#endif
          for (; lora->pkt_framecnt < lora->def->dv_frames_per_packet; lora->pkt_framecnt++) {
            C2LORA_add_frame_2_stream(lora->dva, rx_packet + (lora->rd_offset >> 3), lora->rd_offset & 7);
            lora->rd_offset += lora->dva->bits_per_frame;
            lora->frame_cnt++;
          } // fi
          sbuf_update_time(lora->dva->buf, xTaskGetTickCount());
#if CONFIG_USE_TEST_BITPATTERN
          C2LORA_log_testpattern_from_buffer(lora->dva);
          if (lora->p.bits_cydata > 0) { // mode 6 don't have cyclic data!
            ESP_LOGD(TAG, "cyclic data (%dbytes):", (lora->p.bits_cydata + 7) >> 3);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, rx_packet + cydata_ofs, (lora->p.bits_cydata + 7) >> 3, ESP_LOG_DEBUG);
          } // fi
#else
          localaudio_handle_stream(lora->dva->streamid);   // trigger audio task to process this output stream
          // ToDo           
          // handle cydata
          C2LORA_handle_rx_cydata(lora, rx_packet + cydata_ofs);
#endif
          ESP_LOGD(TAG, "RX done, buffer:%ubytes @ %02xh: %02xh | %02xh", rxbuffer_status.pld_len_in_bytes, rxbuffer_status.buffer_start_pointer, 
            rx_packet[0], rx_packet[rxbuffer_status.pld_len_in_bytes - (lora->p.bits_cydata >> 3) - 1]);

          s = sx126x_get_lora_pkt_status(lora->ctx, &rxpacket_status);
          
          sx126x_get_and_clear_irq_status(lora->ctx, &irq_mask_refresh);
          irq_mask |= irq_mask_refresh; // add a preample event, if this occured in the RXdone handling time.
          if (s) {
            ESP_LOGE(TAG, "error get packet status");
          } else {
            ESP_LOGD(TAG, "RX rssi %ddBm sig %ddBm, SNR:%ddBm, DVframes:%lu IRQmask:%04xh", rxpacket_status.rssi_pkt_in_dbm, rxpacket_status.signal_rssi_pkt_in_dbm, 
              rxpacket_status.snr_pkt_in_db, lora->frame_cnt, irq_mask);
          }
          //if (rxbuffer_status.pld_len_in_bytes > 0) {
          //  ESP_LOG_BUFFER_HEX(TAG, rx_packet, rxbuffer_status.pld_len_in_bytes);
          //}
          //ESP_LOGI(TAG, "high stack water mark is %u", uxTaskGetStackHighWaterMark(NULL));

          // refresh IRQ-Status (new preable?)
          
          // ??? maskiert? check

        } // fi RXdone

        // *** PREAMBLE detected ***
        if (irq_mask & SX126X_IRQ_PREAMBLE_DETECTED) {
          sx126x_rx_buffer_status_t rxbuffer_status = { 0, 0 };
          lora->state     = RX_PREAMBLE;
          lora->rd_offset = 0;
          lora->pkt_framecnt = 0;
          sx126x_get_rx_buffer_status(lora->ctx, &rxbuffer_status);
//          sx126x_set_dio_irq_params(lora->ctx,  SX126X_IRQ_RX_DONE, C2LORARX_INTMASK, 0x00, 0x00);
          rxpacket_ptr   = rx_packet;  // reset packet position
          read_ahead_ofs = rxbuffer_status.buffer_start_pointer + rxbuffer_status.pld_len_in_bytes;          
#if CONFIG_USE_TEST_BITPATTERN
          memset(rx_packet, 0x22, sizeof(rx_packet)); 
#endif
          if ((lora->dva->streamid == -1) && (lora->dva->codec_type != 255)) {
            int codec2_mode = SF_CODEC2_get_mode(lora->dva->codec_type);
            localaudio_create_codec2_outputstream(lora->dva, codec2_mode, lora->dva->bytes_per_frame, (tfinished_stream_fct)c2lora_rx_finisher);
            if (lora->dva->streamid >= 0) {
               sbuf_update_time(lora->dva->buf, xTaskGetTickCount());
               sbuf_set_steppos(lora->dva->buf, 0);
            } else {
              //sx126x_set_dio_irq_params(lora->ctx, SX126X_IRQ_PREAMBLE_DETECTED, C2LORARX_INTMASK, 0x00, 0x00);
              ESP_LOGE(TAG, "create_audiostream: localaudio denied");
              sx126x_unlock();
              continue;
            }
          } // fi
          int32_t process_time_us = esp_timer_get_time() - lora->pkt_start_time;
          task_timeout = (firstdat_time_us - process_time_us + 10000) / (configTICK_RATE_HZ * 100) + 1;  // run task after a initial delay          
          ESP_LOGD(TAG, "RX preamble, %ldµs since INT", process_time_us);
        } // fi preamble

        sx126x_unlock();

      } else // fi DIO1 INT

      if (rx_bits == 0) {  // do cyclic checking (event timeouts)

        int32_t rx_time = esp_timer_get_time() - lora->pkt_start_time;

        if (rx_time < (preamble_time_us >> 1)) {
          ESP_LOGD(TAG, "awaits RXdone");
          task_timeout = pdMS_TO_TICKS(C2LORA_MAX_RXGAP_MS);
          continue;
        }

/*
        if (lora->state == RX_DONE) { // we are done, no futher sync - disable RX
          task_timeout = portMAX_DELAY;
          lora->state  = RX_NOSYNC;
          ESP_LOGD(TAG, "RX ended");
          localaudio_end_stream(lora->dva, true);
          continue;
        } // fi done with stream
*/
        if (rx_time > (with_header? 760000: 580000)) {
          task_timeout = portMAX_DELAY;
          lora->state  = RX_NOSYNC;
          ESP_LOGW(TAG, "RX timeout, high stack watermark=%u", uxTaskGetStackHighWaterMark(NULL));
          sx126x_lock();
          sx126x_set_dio_irq_params(lora->ctx, SX126X_IRQ_PREAMBLE_DETECTED, C2LORARX_INTMASK, 0x00, 0x00);
          sx126x_unlock();
          localaudio_end_stream(lora->dva, true);
          continue;
        } // fi

        if (lora->pkt_framecnt == lora->def->dv_frames_per_packet) {
          ESP_LOGD(TAG, "RX DV fini, wait 4 RXdone");
          continue;
        }

        int symbols_received = (rx_time - preamble_time_us) / symbol_time_us;        
        int bytes_received   = (symbols_received / (lora->def->mod.cr + 4)) * (lora->def->mod.sf << 2) >> 3;
        //ESP_LOGD("RX>>", "%d s, %dbytes", symbols_received, bytes_received);

        task_timeout = 1;  // run task run - every 10ms look for data

        if (bytes_received <= (int)(rxpacket_ptr - rx_packet)) {
          continue;
        }

        int     rxpkt_free_len = rx_packet + sizeof(rx_packet) - rxpacket_ptr;
        int     rx_byte_length = rxpacket_ptr - rx_packet;
        int  bits_in_rx_packet = rx_byte_length << 3;
        uint8_t read_ahead_len = bytes_received + rx_packet - rxpacket_ptr;
        uint8_t min_bytes_need; // with_header detection

        switch(lora->state) {
        default:  
        case RX_PREAMBLE:
          min_bytes_need = 1; 
          break;
        case RX_HEADER:
          if (with_header) {
            min_bytes_need = lora->def->bytes_per_header - rx_byte_length + 1;
            break;
          }          
          // fall through
        case RX_DATA:
          //min_bytes_need = lora->dva->bytes_per_frame - ((lora->rd_offset & 7)? -1: 0); // ToDo check für != 4          
          min_bytes_need = (lora->dva->bits_per_frame - (bits_in_rx_packet - lora->rd_offset) + 7) >> 3;
          break;
        } // hctiws

        ESP_LOGV(TAG, "RX %d - %d of %d min", rx_byte_length, read_ahead_len, min_bytes_need );
        if (read_ahead_len < min_bytes_need) {
          continue;
        }

        if (read_ahead_len > rxpkt_free_len) read_ahead_len = rxpkt_free_len;
        sx126x_lock();
        sx126x_read_buffer(lora->ctx, read_ahead_ofs, rxpacket_ptr, read_ahead_len);
        sx126x_unlock();
        rxpacket_ptr   += read_ahead_len;
        read_ahead_ofs += read_ahead_len;
        rx_byte_length += read_ahead_len;

        if (lora->state == RX_PREAMBLE) {
          uint8_t pkt_len = lora->def->bytes_per_packet;
          with_header  = C2LORA_is_header(rx_packet[0]);
          if (with_header) {
            pkt_len += lora->def->bytes_per_header;
          }
          sx126x_lock();
          lora->pkt_params[SX126X_PKT_LEN_BYTEPOS] = pkt_len; // correct packet length dependend to the first byte value
          sx126x_hal_fast_cmd(lora->ctx, lora->pkt_params, SX126X_SIZE_PKT_PARAMS);
          lora->state = with_header? RX_HEADER: RX_DATA;
          lora->pkt_framecnt = 0;
          if (with_header) {
            lora->rd_offset = lora->p.bits_header + 8;
            ESP_LOGD(TAG, "RX has header");
          } else {
            lora->rd_offset = 8;
            ESP_LOGD(TAG, "RX new frame");
          }
          sx126x_set_dio_irq_params(lora->ctx, C2LORARX_INTMASK, C2LORARX_INTMASK, 0x00, 0x00);
          sx126x_unlock();
        } // fi determine type of packet

        if (with_header && (lora->state == RX_HEADER)) { 
          // header complete?
          if (((rxpacket_ptr - rx_packet - 1) << 3) >= lora->p.bits_header) {
            lora->state = RX_DATA;
            ESP_LOGD(TAG, "RX header (%d bits) received", lora->p.bits_header);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, rx_packet + 1, rxpacket_ptr - rx_packet - 1, ESP_LOG_DEBUG);
            kos = C2LORA_decode_header(rx_packet + 1, callsign, destination, &was_repeated);
            if (!C2LORA_check_header(rx_packet + 1, lora->def->bytes_per_header)) {
              ESP_LOGW(TAG, "header data are corrupted!");
            }
            ESP_LOGI(TAG, "from: %s to: %s KoS: %s%s", callsign, destination, C2LORA_get_kos_name(kos), was_repeated? " (repeated)": "");
          } // fi header received
        } // fi handle header

        bits_in_rx_packet = rx_byte_length << 3;

        if (bits_in_rx_packet >= (lora->rd_offset + lora->dva->bits_per_frame) ) {
          for (int frame_lastpos = bits_in_rx_packet - lora->dva->bits_per_frame;  (lora->pkt_framecnt < lora->def->dv_frames_per_packet) && (lora->rd_offset <= frame_lastpos);  lora->pkt_framecnt++) { 
            C2LORA_add_frame_2_stream(lora->dva, rx_packet + (lora->rd_offset >> 3), lora->rd_offset & 7);
            lora->rd_offset += lora->dva->bits_per_frame;
            lora->frame_cnt++;
          } // rof multiple data@once
          sbuf_update_time(lora->dva->buf, xTaskGetTickCount());
#if CONFIG_USE_TEST_BITPATTERN
          C2LORA_log_testpattern_from_buffer(lora->dva);
#else
          localaudio_handle_stream(lora->dva->streamid);   // trigger audio task to process this output stream
#endif
        } // fi rx a frame

        if (lora->pkt_framecnt == lora->def->dv_frames_per_packet) {
          task_timeout = cyclic_data_to;
        } // fi done
        
      } // fi task timeout (cyclic fetching data adead of TXdone)

    } while (1); // forever

    sx126x_lock();
    gpio_set_intr_type(lora->intr_pin, GPIO_INTR_DISABLE);
    if (lora->rxen_pin != GPIO_NUM_NC) {
      gpio_set_level(lora->rxen_pin, 0);
    }
    sx126x_set_standby(lora->ctx, SX126X_STANDBY_CFG_XOSC);
    lora->dva   = NULL;
    lora->state = LS_IDLE;
    sx126x_unlock();
    // ToDo achtung: Buffer kann noch in verwendung sein!!!
    sbuf_destroy(C2LORArx.buf);

    // Todo Exit to shutdown

  } // ehliw

c2lora_rx_shutdown:
  ESP_LOGW(TAG, "rx task shutdown.");
  vTaskDelete(NULL);
}

