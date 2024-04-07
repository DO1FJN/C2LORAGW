/*
 * BlueSPP.c
 *
 *  Created on: 12.03.2019
 *      Author: Jan Alte
 *          (c) 2019 bentrup Industriesteuerungen
 */


#include "BlueSPP.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>	// defines SPP task priority in BT_CONTROLLER_INIT_CONFIG_DEFAULT()

#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#include <esp_bt_device.h>
#include <esp_spp_api.h>


#include <string.h>


#define SPP_TAG			"BT_SPP"

//#include "ATypesGen.h"
#ifndef __weak
#define __weak		__attribute__((weak))
#endif


//#define BT_SERVICE_CLASSOFDEVICE	"080114"
static const esp_bt_cod_t bentrup_ctrl_cod = {
   .reserved_2 = 0,
   .minor      = 5,
   .major      = ESP_BT_COD_MAJOR_DEV_COMPUTER,
   .service    = ESP_BT_COD_SRVC_CAPTURING,
   .reserved_8 = 0
};

static char		My_Device_Name[32];
static uint32_t		SPP_handle;
static esp_bd_addr_t	remote_pairing_addr;


static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);	// fwd
void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);



esp_err_t BlueSPP_init(const char *devicename) {
  esp_err_t ret;
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

  SPP_handle = -1;	// invalid

  snprintf(My_Device_Name, sizeof(My_Device_Name)-1, devicename);
  My_Device_Name[sizeof(My_Device_Name)-1] = 0;

  ESP_LOGD(SPP_TAG, "%s start.", __func__);

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
  bt_cfg.mode = ESP_BT_MODE_CLASSIC_BT;  // newer SDK: bluetooth_mode

  ret = esp_bt_controller_init(&bt_cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
    return ret;
  }
  if ((ret = esp_bt_controller_enable(bt_cfg.mode)) != ESP_OK) { // ESP_BT_MODE_CLASSIC_BT oder ESP_BT_MODE_BTDM
    ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
    return ret;
  }
  if ((ret = esp_bluedroid_init()) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
    return ret;
  }
  if ((ret = esp_bluedroid_enable()) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
    return ret;
  }

  if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
    return ret;
  }
  if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
    return ret;
  }
  if ((ret = esp_spp_init(ESP_SPP_MODE_CB)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
    return ret;
  }
  if ((ret = esp_bt_gap_set_cod(bentrup_ctrl_cod, ESP_BT_INIT_COD)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s gap set CoD failed: %s\n", __func__, esp_err_to_name(ret));
    return ret;
  }

  esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
  esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(uint8_t));

  ESP_LOGD(SPP_TAG, "%s done.", __func__);
  return ESP_OK;
}



void BlueSPP_confirm_passkey(int accept) {
  esp_bt_gap_ssp_confirm_reply(remote_pairing_addr, accept);
}


__weak void BlueSPP_connect_event(uint32_t hnd) {
}

__weak void BlueSPP_disconnect_event(uint32_t old_hnd) {
}

__weak void BlueSPP_HandleData(uint32_t hnd, const void *data, uint16_t len) {
}

__weak unsigned char BlueSPP_GetOwnPIN(char pin[16], int full16digits) {
  pin[0] = '1';
  pin[1] = '2';
  pin[2] = '3';
  pin[3] = '4';
  if (full16digits) {
    ESP_LOGI(SPP_TAG, "Input pin code: 1234 0000 0000 0000");
  } else {
    ESP_LOGI(SPP_TAG, "Input pin code: 1234");
  }
  return full16digits? 16: 4;
}

__weak void BlueSPP_confirm_event(uint32_t match_num) {
}

__weak void BlueSPP_passkey_event(uint32_t passkey) {
}

__weak void BlueSPP_enterkey_event(void) {
}



esp_err_t BlueSPP_send(const void *data, unsigned short len) {
  if (SPP_handle == -1) {
    ESP_LOGE(SPP_TAG, "no handle / SPP not open");
    return -2;
  }
  ESP_LOGI(SPP_TAG, "send to %d, %d bytes", SPP_handle, len);
  return esp_spp_write(SPP_handle, len, (uint8_t *)data);
}




static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  switch (event) {
  case ESP_SPP_INIT_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
    esp_bt_dev_set_device_name(My_Device_Name);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
    break;
  case ESP_SPP_DISCOVERY_COMP_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
    break;
  case ESP_SPP_OPEN_EVT:
    SPP_handle = param->open.handle;
    BlueSPP_connect_event(param->open.handle);
    ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
    break;
  case ESP_SPP_SRV_OPEN_EVT:
    SPP_handle = param->srv_open.handle;
    BlueSPP_connect_event(param->srv_open.handle);
    ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
    break;
  case ESP_SPP_CLOSE_EVT:
    SPP_handle = -1;		// no connection = no handle.
    BlueSPP_disconnect_event(param->close.handle);
    ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
    break;
  case ESP_SPP_START_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
    break;
  case ESP_SPP_CL_INIT_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
    break;

  case ESP_SPP_DATA_IND_EVT:
    ESP_LOGV(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d", param->data_ind.len, param->data_ind.handle);
    BlueSPP_HandleData(param->data_ind.handle, param->data_ind.data, param->data_ind.len);
    break;

  case ESP_SPP_CONG_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
    break;
  case ESP_SPP_WRITE_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT, sta=%d, len=%d", param->write.status, param->write.len);
    break;
  default:
    // BT_SPP: unhandled event: 10 nach INIT und vor START!
    ESP_LOGI(SPP_TAG, "spp-event: %d", event);
    break;
  }
}


void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  esp_bt_pin_code_t	pin_code;
  uint8_t		pin_len;
  switch (event) {
  case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
    ESP_LOGD(SPP_TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
    break;
  case ESP_BT_GAP_AUTH_CMPL_EVT:
    if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
      ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
      esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
    } else {
      ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
    }
    break;
  case ESP_BT_GAP_PIN_REQ_EVT:
    memset(pin_code, 0, sizeof(pin_code));
    pin_len = BlueSPP_GetOwnPIN((char *)&pin_code, param->pin_req.min_16_digit);
    esp_bt_gap_pin_reply(param->pin_req.bda, pin_len > 0, pin_len, pin_code);
    ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT Input Pin: %.16s (%d digits)", (char *)pin_code, pin_len);
    break;

//#ifdef CONFIG_BT_SSP_ENABLE
  case ESP_BT_GAP_CFM_REQ_EVT:
    memcpy(remote_pairing_addr, param->cfm_req.bda, sizeof(esp_bd_addr_t));
    BlueSPP_confirm_event(param->cfm_req.num_val);
    ESP_LOGV(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
    break;
  case ESP_BT_GAP_KEY_NOTIF_EVT:
    BlueSPP_passkey_event(param->key_notif.passkey);
    ESP_LOGV(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
    break;
  case ESP_BT_GAP_KEY_REQ_EVT:
    memcpy(remote_pairing_addr, param->key_req.bda, sizeof(esp_bd_addr_t));
    BlueSPP_enterkey_event();
    ESP_LOGV(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
    break;
  case ESP_BT_GAP_CONFIG_EIR_DATA_EVT:
    ESP_LOGV(SPP_TAG, "ESP_BT_GAP_CONFIG_EIR_DATA_EVT");
    break;
//#endif // CONFIG_BT_SSP_ENABLE
  default:
    ESP_LOGI(SPP_TAG, "unhandled event: %d", event);
    break;
  } //hctiws event
  return;
}


