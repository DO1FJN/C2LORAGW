/*
 * userdata.c
 *
 *  Created on: 31.01.2024
 *      Author: Jan Alte, DO1FJN
 *
 * This file is part of the DV-RPTR application.
 * For general information about this program see "main.c".
 *
 * DV-RPTR app is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The DV-RPTR app is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "HAMdLNK.h"
#include "compiler.h"

#include <string.h>

typedef struct {
  U32		key;
  void *	userdata;
} tUserData;


#ifndef CONFIG_MAX_USERDATA
#define CONFIG_MAX_USERDATA	    8
#endif


static tUserData stream_userdata[CONFIG_MAX_USERDATA];


void hamdlnk_init_userdata(void) {
  memset(stream_userdata, 0, sizeof(stream_userdata));
}

U32 hamdlnk_get_streamuserid(int listener, U32 streamid) {
  return listener << 8 ^streamid;
}


void *hamdlnk_get_streamuserdata(int listener, U32 streamid) {
  U32 key = hamdlnk_get_streamuserid(listener, streamid);
  for (int n=0; n < CONFIG_MAX_USERDATA; n++) {
    if (stream_userdata[n].key == key) return stream_userdata[n].userdata;
  }
  return NULL;
}


void hamdlnk_set_streamuserdata(int listener, U32 streamid, void *userdata) {
  U32 key = hamdlnk_get_streamuserid(listener, streamid);
  for (int n=0; n < CONFIG_MAX_USERDATA; n++) {
    if (stream_userdata[n].key == key) {
      stream_userdata[n].userdata = userdata;
      return;
    }
  }
  for (int n=0; n < CONFIG_MAX_USERDATA; n++) {
    if (stream_userdata[n].key == 0) {
      stream_userdata[n].key = key;
      stream_userdata[n].userdata = userdata;
      return;
    }
  }
}


void hamdlnk_free_streamuserdata(void *userdata) {
  for (int n=0; n < CONFIG_MAX_USERDATA; n++) {
    if (stream_userdata[n].userdata == userdata) {
      stream_userdata[n].userdata = NULL;
      stream_userdata[n].key = 0;
      return;
    }
  }
}
