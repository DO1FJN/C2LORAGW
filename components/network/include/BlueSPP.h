/*
 * BlueSPP.h
 *
 *  Created on: 12.03.2019
 *      Author: jan
 *          (c) 2019 bentrup Industriesteuerungen
 */

#ifndef MAIN_INCLUDE_BLUESPP_H_
#define MAIN_INCLUDE_BLUESPP_H_

#include <esp_err.h>

#define SPP_SERVER_NAME		"WinControl Port"

esp_err_t	BlueSPP_init(const char *devicename);

void		BlueSPP_confirm_passkey(int accept);

esp_err_t	BlueSPP_send(const void *data, unsigned short len);

// weak function that should used:
// unsigned char BlueSPP_GetOwnPIN(unsigned char pin[16], boolean full16digits);
// void BlueSPP_confirm_event(uint32_t match_num)
// void BlueSPP_passkey_event(uint32_t passkey);
// void BlueSPP_enterkey_event(void);
// void BlueSPP_connect_event(uint32_t hnd);
// void BlueSPP_disconnect_event(uint32_t old_hnd);
// void BlueSPP_HandleData(uint32_t hnd, const void *data, uint16_t len);


#endif // MAIN_INCLUDE_BLUESPP_H_
