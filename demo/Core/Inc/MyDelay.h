/*
 *	MyDelay.h
 *
 *  Created on: Aug 6, 2021
 *      Author: tuan
 */
#ifndef __MY_DELAY_H
#define __MY_DELAY_H

#include "stm32f1xx_hal.h"

void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
uint32_t get_tick_ms(void);
uint32_t get_tick_us(void);
void delay_init(void);
#endif
