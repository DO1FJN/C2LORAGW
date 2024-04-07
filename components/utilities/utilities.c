/*
 * utilities.c
 *
 *  Created on: 03.02.2012
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

#include "utilities.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <ctype.h>
#include <limits.h>

/*
char upper_char(const char x) {
  return (char)toupper(x);
}


char lower_char(const char x) {
  return (char)tolower(x);
}

void lower_str(char *strbuffer) {
  while (strbuffer[0] != 0) {
    strbuffer[0] = tolower(strbuffer[0]);
    strbuffer++;
  }
}

*/



int scomp(const char *a, const char *b) {
 if ((a==NULL)&&(b==NULL)) return(0);
 if ((a==NULL)||(b==NULL)) return(-1);
 return( strcmp(a, b) );
}


int scompfirst(const char *arg, const char *testpat) {
 if ((arg==NULL)&&(testpat==NULL)) return(0);
 if ((arg==NULL)||(testpat==NULL)) return(-1);
 return( strncmp(arg, testpat, strlen(testpat)) );
}


char *trim_str(char *s) {
  int epos;
  if (s == NULL) return NULL;
  while ((s[0] != 0) && ((unsigned char)s[0] < 33)) {
    s++;
  }
  epos = strlen(s);
  while (epos > 0) {
    epos--;
    if ((unsigned char)s[epos] > 32) break;
  }
  s[epos+1] = 0;
  return s;
}


char *str_begin(const char *s) {
  char *sx = (char *)s;
  while ((sx[0] != 0) && ((unsigned char)sx[0] < 33)) {
    sx++;
  }
  return sx;
}


int str_index(const char *strx, const char *stringlst[]) {
  int i = 0, tst;
  if (strx == NULL) return(-2);
  while (stringlst[i] != NULL) {
    tst = strcasecmp(strx, stringlst[i]);
    //tst = scomp(strx, stringlst[i]);
    if (tst == 0) return(i);
    i++;
  } //end while
  return(-1);                //not found in list;
}


int str_indexfirst(const char *strx, const char *stringlst[]) {
  int i = 0, tst;
  if (strx == NULL) return(-2);
  while (stringlst[i] != NULL) {
    tst = strncasecmp(strx, stringlst[i], strlen(stringlst[i]));
    //tst = scompfirst(strx, stringlst[i]);
    if (tst == 0) return(i);
    i++;
  } //end while
  return(-1);                //not found in list;
}


int str_getnumber(int *var, const char *numstr) {
  long value;
  char *reststr;
  errno = 0;
  value = strtol(numstr, &reststr, 0);	// auto-base: 0x... for hex-input
  if (errno) return(-1);
#if (INT_MAX < LONG_MAX)
  if (value > INT_MAX) value = INT_MAX; else if (value < INT_MIN) value = INT_MIN;
#endif
  *var = value;
  if ((reststr != NULL) && (reststr[0] != 0)) return(1);	// not complete a number
  return (0);
}


long str_readnum_l(const char *numstr, long default_num) {
  long val;
  errno = 0;
  val = strtol(numstr, NULL, 10);	// dezimal base
  return (errno)? default_num: val;
}


int str_readnum(const char *numstr, int default_num, int min, int max) {
  long val;
  char *tailptr;
  val = strtol(numstr, &tailptr, 10);	// dezimal base
  if ((val < min) || (val > max)) return default_num;
  return (tailptr == numstr)? default_num: val;
}



int str_gettokens(char *tokenlist[], char *strbuf, int listmaxsize) {
  const char *trenner = " \n";	// Nur Leerzeichen, Enter erlaubt!
  int cnt;
  tokenlist[0] = strtok(strbuf, trenner);	// init strtok
  if (tokenlist[0] == NULL) return(0);
  for (cnt=1; cnt<listmaxsize; cnt++) {
    tokenlist[cnt] = strtok(NULL, trenner);	// next
//    printf("%d:%s\n", cnt, tokenlist[cnt]);
    if (tokenlist[cnt] == NULL) return(cnt);
  }
  return(cnt);
}


int str_gettokens_ex(char *tokenlist[], char *strbuf, int listmaxsize, const char *trenner) {
  int cnt;
  tokenlist[0] = strtok(strbuf, trenner);	// init strtok
  if (tokenlist[0] == NULL) return(0);
  for (cnt=1; cnt<listmaxsize; cnt++) {
    tokenlist[cnt] = strtok(NULL, trenner);	// next
    if (tokenlist[cnt] == NULL) return(cnt);
  }
  return(cnt);
}


int str_getvalue(char **valuebuf, char *strbuf) {
  const char *trenner = "=\n";	// Nur Gleich, Enter erlaubt!
  char *res = strtok(strbuf, trenner);	// init strtok
  if (res == NULL) return(-1);
  *valuebuf = strtok(NULL, trenner);
  return(*valuebuf == NULL);
}


int str_getOnOff(const char *onoff_str) {
  const char *on_off_vals[] = { "off", "on", NULL };
  long onOff = str_index(onoff_str, on_off_vals);
  if (onOff < 0) {
    char *tailptr;
    onOff = strtol(onoff_str, &tailptr, 0);	// auto base
    if ((tailptr != onoff_str) && (tailptr[0] == 0)) return (onOff > 0)? 1: 0;
    onOff = 0;
  } // fi
  return onOff;
}


char *str_append(const char *s1, const char *s2) {
  size_t s1_len = strlen(s1);
  size_t s2_len = strlen(s2);
  char *new_s   = malloc(s1_len+s2_len+1);
  if (new_s == NULL) return NULL;
  memcpy(new_s, s1, s1_len);
  memcpy(new_s + s1_len, s2, s2_len);
  new_s[s1_len+s2_len] = 0;
  return new_s;
}


int str_removeCRLF(char *sline, unsigned int sline_size) {
  while ((sline_size > 0) && (sline[sline_size-1] < 32)) {
    sline[sline_size-1] = 0;
    sline_size--;
  }
  return sline_size;
}


void printlist(const char *cmdlist[], const char *helplist[]) {
  int lst_cnt = 0;
  if (helplist == NULL) {
    while (cmdlist[lst_cnt] != NULL) {
      printf("  %s\n", cmdlist[lst_cnt]);
      lst_cnt++;
    } //while
  } else {
    while (cmdlist[lst_cnt] != NULL) {
      if (cmdlist[lst_cnt][strlen(cmdlist[lst_cnt])-1] == '=')  {
        printf("%11s<value> %s\n", cmdlist[lst_cnt], helplist[lst_cnt]);
      } else {
        printf("%10s         %s\n", cmdlist[lst_cnt], helplist[lst_cnt]);
      }
      lst_cnt++;
    } //while
   }
  fflush(stdout);
}



void base64ascii(char *dest, const char sixbits[4], char count) {
  for (unsigned char cnt = 0; cnt < count; cnt++) {
    char chr = sixbits[cnt];
    if (chr < 26)
      dest[0] = 'A' + chr;
    else if (chr < 52)
      dest[0] = 'a' - 26 + chr;
    else if (chr < 62)
      dest[0] = '0' - 52 + chr;
    else switch (chr) {
    case 62: dest[0] = '+'; break;
    case 66: dest[0] = '/'; break;
    default: dest[0] = '='; break;
    } // hctiws
    dest++;
  }
}


int base64(char *dest, int dest_size, const char *s, int s_length) {
  char sixbits[4];
  int colcnt;
  for (colcnt = 0; (s_length > 2) && (dest_size > 3); s_length-=3, s+=3, dest+=4, dest_size-=4) {
    sixbits[0] = s[0] >> 2;
    sixbits[1] = ((s[0] & 0x03) << 4) | ((s[1] & 0xF0) >> 4);
    sixbits[2] = ((s[1] & 0x0F) << 2) | ((s[2] & 0xC0) >> 6);
    sixbits[3] = s[2] & 0x3F;
    base64ascii(dest, sixbits, 4);
    colcnt += 4;
    if (((colcnt & 0x3F) == 0) && (dest_size > 5)) {
      dest[4] = '\r';
      dest[5] = '\n';
      dest += 2;
      dest_size -= 2;
    } // fi CRLF
  } // rof
  if ((s_length > 0) && (dest_size > 1)) {
    sixbits[0] = s[0] >> 2;
    sixbits[1] = (s[0] & 0x03) << 4;
    colcnt += 2;
    if ((s_length > 1) && (dest_size > 2)) {
      sixbits[1] |= (s[1] & 0xF0) >> 4;
      sixbits[2]  = (s[1] & 0x0F) << 2;
      base64ascii(dest, sixbits, 3);
      dest += 3;
      dest_size -= 3;
      colcnt ++;
      if (dest_size > 0) {
	dest[0] = '=';
	dest++;
	dest_size--;
      }
    } else {
      base64ascii(dest, sixbits, 2);
      dest += 2;
      dest_size -= 2;
      if (dest_size > 1) {
      	dest[0] = '=';
      	dest[1] = '=';
      	dest += 2;
      	dest_size -= 2;
      }
    }
  }
  if (dest_size > 0) {
    dest[0] = 0;	// Ende
  }
  return colcnt;
}


int Check_Call(const char *Call, int Call_Length) {
  char z;
  char zcnt = 0, ncnt = 0;
  unsigned int cnt;
  if (Call_Length < 5) return 0;
  for (cnt=0; cnt<6; cnt++) {
    z = toupper(*Call++);
    if (z < ' ') return 0;
    if ((z >= '0')&&(z <= '9')) ncnt++;
    if ((z >= 'A')&&(z <= 'Z')) zcnt++;
  }
  return (ncnt>0)&&(zcnt>3);
}


int Check_Locator(const char *Locator, int Locator_Length) {
  char z;
  char zcnt = 0, ncnt = 0;
  unsigned int cnt;
  if (Locator_Length < 6) return 0;
  for (cnt=0; cnt<6; cnt++) {
    z = toupper(*Locator++);
    if (z < ' ') return 0;
    if ((z >= '0')&&(z <= '9')) ncnt++;
    if ((z >= 'A')&&(z <= 'Z')) zcnt++;
  }
  return (ncnt==2)&&(zcnt==4);
}

// *** ESP32 addons ***

#include <esp_err.h>
#include <nvs_flash.h>


esp_err_t nvs_load_string(unsigned int hnd, const char *key, char *strbuffer, size_t max_size) {
  size_t item_size;
  esp_err_t err = nvs_get_str((nvs_handle)hnd, key, NULL, &item_size);
  if ( (err == ESP_OK) && (item_size > 0) && (item_size <= max_size) ) {
    err = nvs_get_str((nvs_handle)hnd, key, strbuffer, &item_size);
  }
  return err;
}


const char *nvs_get_string(unsigned int hnd, const char *key, size_t max_size) {
  size_t item_size;
  esp_err_t err = nvs_get_str((nvs_handle)hnd, key, NULL, &item_size);
  if (item_size > max_size) item_size = max_size;  
  if (err != ESP_OK) return NULL;
  char *value = malloc(item_size);
  if (value == NULL) return NULL;
  err = nvs_get_str((nvs_handle)hnd, key, value, &item_size);  
  return err == ESP_OK? value: NULL;
}



