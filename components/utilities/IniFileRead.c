/*
 * IniFileRead.c
 *
 *  Created on: 05.02.2018
 *      Author: Jan Alte
 *          (c) 2018,2019 bentrup Industriesteuerungen
 * 
 * Report:
 * 2020-10-15	BugFix config_reader() mit text_size und nasty-last-line fix.
 */

#include "IniFileRead.h"
#include "utilities.h"

#include <ff.h>
#include <string.h>


#define	MAX_TEXTFILE_LINELEN	192


#define COMMALIST_DELIM		",\r\n"

int str_makelist(char *liststr, char *list[], unsigned char size) {
  int n;
  char *tokenPtr, *item;
  item = strtok_r(liststr, COMMALIST_DELIM, &tokenPtr);
  for (n = 0; (item != NULL) && (size > 0); size--, list++, n++) {
    list[0] = trim_str(item);
    item    = strtok_r(NULL, COMMALIST_DELIM, &tokenPtr);
  } // rof
  return n;
}


#define NUMBERLIST_DELIM	", \r\n"

int str_readlistu8(char *liststr, unsigned char *list, unsigned char size, unsigned char min, unsigned char max) {
  int n;
  char *tokenPtr, *item;
  item = strtok_r(liststr, NUMBERLIST_DELIM, &tokenPtr);
  for (n = 0; (item != NULL) && (size > 0); size--, list++, n++) {
    list[0] = str_readnum(item, list[0], min, max);
    item    = strtok_r(NULL, NUMBERLIST_DELIM, &tokenPtr);
  } // rof
  return n;
}


int str_readlist16(char *liststr, short *list, unsigned char size, short min, short max) {
  int n;
  char *tokenPtr, *item;
  item = strtok_r(liststr, NUMBERLIST_DELIM, &tokenPtr);
  for (n = 0; (item != NULL) && (size > 0); size--, list++, n++) {
    list[0] = str_readnum(item, list[0], min, max);
    item    = strtok_r(NULL, NUMBERLIST_DELIM, &tokenPtr);
  } // rof
  return n;
}


char *conf_get(char *line, char **value);

inline char *conf_get(char *line, char **value) {
  char *paraname, *paravalue;
  if (line == NULL) return NULL;
  paraname = strtok_r(line, "=\n", &paravalue);
  if (paraname == NULL) return NULL;
  *value = (paravalue)? trim_str(paravalue): NULL;
  paraname = trim_str(paraname);
  //lower_str(paraname);
  return paraname;
}


void conf_readline(char *line, const char *vnames[], tconfigfunction cfg_handle, void *param) {
  char *iniparaval;
  char *iniparaname = conf_get(line, &iniparaval);
  if (iniparaname != NULL) {
    if (iniparaname[0] == '[') {
      char *sec_end = strchr(iniparaname, ']');
      if (sec_end) sec_end[1] = 0;		// set correct ending
      cfg_handle(-2, iniparaname, param);	// section like [Network]
    } else if ( (iniparaval != NULL) && (iniparaname[0] != '#') ) {
      int inipos = str_index(iniparaname, vnames);
      if (inipos >= 0) cfg_handle(inipos, iniparaval, param);
    } // fi interprets
  } // fi not NULL
}


int config_interpreter(const char *filename, const char *vnames[], tconfigfunction cfg_handle, void *param) {
  FRESULT fop;
  FIL *inifile;
  char *linebuf, *line;
  size_t inilen;

  if (cfg_handle == NULL) return 0;

  inifile = malloc(sizeof(FIL));
  if (inifile == NULL) return -2;

  fop = f_open(inifile, filename, FA_READ | FA_OPEN_EXISTING);
  if (fop != FR_OK) {
    free(inifile);
    return -1;
  }

  linebuf = malloc(MAX_TEXTFILE_LINELEN);
  if (linebuf == NULL) {
    free(inifile);
    return -2;
  }

  do {

    line   = f_gets(linebuf, MAX_TEXTFILE_LINELEN, inifile);
    inilen = (line == NULL)? 0: strnlen(line, MAX_TEXTFILE_LINELEN);

    if (inilen > 0) {
      conf_readline(line, vnames, cfg_handle, param);
    } // fi line not empty

  } while (inilen > 0);
  cfg_handle(-2, "eof", param);
  free(linebuf);
  f_close(inifile);
  free(inifile);
  return 0;
}



unsigned short conf_getline(char **line_ptr, char **text_ptr, unsigned short *length) {
  unsigned short s_len;
  char *start_used_line = *text_ptr;
  for (; (*length > 0) && (start_used_line[0] < 32); start_used_line++, length[0]--);  
  *text_ptr = start_used_line;  
  for (; (length[0] > 0) && ((*text_ptr)[0] >= 32); (*text_ptr)++, length[0]--);
  s_len = (*text_ptr) - start_used_line;
  if (length[0] == 0) {    
    *text_ptr = NULL;
  } else {
    do {
      (*text_ptr)[0] = 0;
      (*text_ptr)++;
      length[0]--;
    } while ((length[0] > 0) && ((*text_ptr)[0] < 32) );
  } // esle
  *line_ptr = start_used_line;  
  return s_len;
}


int config_reader(char *text_buffer, unsigned short text_len, const char *vnames[], tconfigfunction cfg_handle, void *param) {
  char *line, *safetok;
  char nasty_last_line_buf[MAX_TEXTFILE_LINELEN];

  if (cfg_handle == NULL) return 0;

  safetok = text_buffer;
  while ((safetok != NULL) && (text_len > 0)) {
    
    size_t line_len = conf_getline(&line, &safetok, &text_len);
    
    // we are not allowed to write a NULL termination byte behind text_len in this buffer!
    // if the last line isn't terminated, make a copy with \0 character.
    if ((text_len == 0) && (strnlen(line, line_len + 1) > line_len)) {
      memset(nasty_last_line_buf, 0, sizeof(nasty_last_line_buf));
      if (line_len > MAX_TEXTFILE_LINELEN) line_len = MAX_TEXTFILE_LINELEN;
      strncpy(nasty_last_line_buf, line, line_len);
      line = nasty_last_line_buf;
//      printf("!! nasty last line !! ");
    } // fi nasty line
    
//    printf(">>%s<< (%d left)\r\n", line, text_len);
    
    if (line_len > 2) {
      conf_readline(line, vnames, cfg_handle, param);
    } // fi line not empty
  } // ehliw have text

  cfg_handle(-2, "eof", param);
  return 0;
  
}

