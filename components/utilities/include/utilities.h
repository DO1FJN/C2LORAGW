/*
 * utilities.h
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

#ifndef UTILITIES_H_
#define UTILITIES_H_


char	upper_char(const char x);
char	lower_char(const char x) ;

int	scomp(const char *a, const char *b);

int	scompfirst(const char *arg, const char *testpat);

void	lower_str(char *strbuffer);
char	*trim_str(char *s);
char	*str_begin(const char *s);

int	str_index(const char *strx, const char *stringlst[]);

int	str_indexfirst(const char *strx, const char *stringlst[]);

int	str_getnumber(int *var, const char *numstr);

long	str_readnum_l(const char *numstr, long default_num);
int	str_readnum(const char *numstr, int default_num, int min, int max);

int     str_getOnOff(const char *onoff_str);

int	str_gettokens(char *tokenlist[], char *strbuf, int listmaxsize);
int	str_gettokens_ex(char *tokenlist[], char *strbuf, int listmaxsize, const char *trenner);

int	str_getvalue(char **valuebuf, char *strbuf);

char	*str_append(const char *s1, const char *s2);

int	str_removeCRLF(char *sline, unsigned int sline_size);

void	printlist(const char *cmdlist[], const char *helplist[]);

int	base64(char *dest, int dest_size, const char *s, int s_length);

int     Check_Call(const char *Call, int Call_Length);

int     Check_Locator(const char *Locator, int Locator_Length);

#include <esp_err.h>
#include <string.h>


esp_err_t nvs_load_string(unsigned int hnd, const char *key, char *strbuffer, size_t max_size);

const char *nvs_get_string(unsigned int hnd, const char *key, size_t max_size);

#endif // UTILITIES_H_
