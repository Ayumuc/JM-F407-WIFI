/*
 * delay.h
 *
 *  Created on: Dec 9, 2021
 *      Author: Administrator
 */

#ifndef DELAY_H_
#define DELAY_H_

#include "FreeRTOS.h"
#include "task.h"

#include "stdint.h"

void delay_us(uint32_t nus);
void for_delay_us(uint32_t nus);

#endif /* DELAY_H_ */
