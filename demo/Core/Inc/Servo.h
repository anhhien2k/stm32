#ifndef __SERVO_H_
#define __SERVO_H_

#include "stm32f1xx_hal.h"
typedef struct
{
	uint8_t angle;
	TIM_HandleTypeDef *htimServo;
	uint32_t chanel;
}Servo;
void servo_write(Servo *sv,uint8_t goc);
void servo_init(Servo *sv,TIM_HandleTypeDef *htim, uint32_t chanel);
uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
#endif
