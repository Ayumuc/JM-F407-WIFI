/*
 * myfunction.h
 *
 *  Created on: Dec 1, 2021
 *      Author: Administrator
 */

#ifndef MYFUNCTION_H_
#define MYFUNCTION_H_

#include <string.h>
#include <stdio.h>
#include "rtc.h"
#include "boardConfig.h"


void str2hex(char *strarr, char *hexarr, int len);

unsigned char BCD_Decimal(unsigned char bcd);

int decimal_bcd_code(int decimal);








#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>

int mini_vsnprintf(char* buffer, unsigned int buffer_len, const char *fmt, va_list va);
int mini_snprintf(char* buffer, unsigned int buffer_len, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* MYFUNCTION_H_ */
