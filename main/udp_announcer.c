/*
 * udp_announcer.c
 *
 *  Created on: 02.04.2019
 *      Author: Jan Alte
 *          (c) 2019 bentrup Industriesteuerungen
 */

#include "udp_announcer.h"
//include "udp_con.h"
#include "LAN.h"

#include <esp_log.h>
#include <esp_err.h>

#include <string.h>		// needed for size_t below

//#include <lwip/sockets.h>
//#include <lwip/netdb.h>
#include <lwip/udp.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define UDP_ANNOUNCERTASK_PRIOITY     1
#define UDP_ANNOUNCERTASK_STACKSIZE   1536

#define MIN_ANSWER_RATE_TICKS	(30 / portTICK_PERIOD_MS)


static const char *TAG = "UDPan";

static TaskHandle_t hnd = NULL;


void an_answer_funct(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {

  static TickType_t last_tick;
  TickType_t tick = xTaskGetTickCount();
  struct pbuf *answer = (struct pbuf *) arg;

  unsigned int rxlength = p->len; 
  const unsigned char *buffer = p->payload;
  
  char *remoteIP = ipaddr_ntoa(addr);

  if ((rxlength <= 0) || (answer == NULL)) {

    ESP_LOGE(TAG, "UDP request received from %s: no data", remoteIP);      

  } else if ((tick - last_tick) > MIN_ANSWER_RATE_TICKS) {
    last_tick = tick;
    // Update answer:
    switch(buffer[0]) {
    case '?':	// Ask?      
      ESP_LOGI(TAG, "UDP request received from %s: %*.s", remoteIP, rxlength-1, buffer+1);      
      udp_sendto(pcb, answer, addr, port);
      break;
    } // hctiws
  } // fi msg

  pbuf_free(p);

}



static void udp_annoucer_task(void *param) {

  struct udp_pcb *mypcb;
  struct pbuf * answer;
  // Todo ip_addr_t myBindadr;  

  const tAnnouncerParam *Param = (const tAnnouncerParam *)param;
  if (param == NULL) {
    ESP_LOGE(TAG, "no param 4 udp announcer task - exit.");
    vTaskDelete(NULL);
    return;
  }

  mypcb  = udp_new();
  
  answer = pbuf_alloc_reference((void *)Param->json_msg, strnlen(Param->json_msg, 1024), PBUF_ROM);
  if (answer == NULL) {
    ESP_LOGE(TAG, "no memory for UDP pbuf");
    udp_remove(mypcb);
    vTaskDelete(NULL);
    return;
  }

  do {

    ESP_LOGI(TAG, "wait for IP...");
    while (LAN_wait4event(CONNECTED_BIT, -1) == 0);

    err_t res = udp_bind(mypcb, IP_ANY_TYPE, Param->port_listen);
    if (res != ERR_OK) {
      ESP_LOGE(TAG, "can't bind to port - it's in use (%d)", res);
      while (LAN_wait4event(DISCONNECTED_BIT, -1) == 0);
      continue;
    }

    udp_recv(mypcb, an_answer_funct, answer);

    ESP_LOGI(TAG, "ready to reply broadcasts.");

    while (LAN_wait4event(DISCONNECTED_BIT | WIFI_SHUTDOWN_BIT, -1) == 0);
    ESP_LOGI(TAG, "disconnected");
    udp_disconnect(mypcb);

  } while (1);

  udp_remove(mypcb);
  ESP_LOGE(TAG, "abnormal exit.");
  vTaskDelete(NULL);
}



esp_err_t upd_startannoucer(const tAnnouncerParam *Param) {
  xTaskCreate(udp_annoucer_task, "UDPannouncer", UDP_ANNOUNCERTASK_STACKSIZE, (void *)Param, UDP_ANNOUNCERTASK_PRIOITY, &hnd);
  return hnd? ESP_OK: ESP_FAIL;
}


void upd_stopannoucer(void) {
  if (hnd) {
    vTaskDelete(hnd);
  }
}
