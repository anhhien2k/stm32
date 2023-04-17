#ifndef HR04_H_
#define HR04_H_

#include "stm32f1xx_hal.h"
#include "main.h"

float hr04_getDistance1(GPIO_TypeDef* port, uint16_t trigPin);

float hr04_getDistance2(GPIO_TypeDef* port, uint16_t trigPin);

#endif
