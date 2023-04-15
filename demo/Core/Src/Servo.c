#include "Servo.h"

uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
	return (x - in_min)*(out_max - out_min)/(in_max - in_min)+out_min;
}

void servo_write(Servo *sv,uint8_t goc)
{
	uint16_t ccr = map(goc,0,180,500,2500);
	__HAL_TIM_SetCompare(sv->htimServo,sv->chanel,ccr);
}
void servo_init(Servo *sv,TIM_HandleTypeDef *htim,uint32_t chanel)
{
	sv->htimServo = htim;
	sv->chanel = chanel;
	sv->angle = 0;
	HAL_TIM_PWM_Start(sv->htimServo,sv->chanel);
}
