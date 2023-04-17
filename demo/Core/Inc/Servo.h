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
#endif
