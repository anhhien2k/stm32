#ifndef __HR04_H_
#define __HR04_H_

#include "main.h"

extern TIM_HandleTypeDef htim3;

#define TriggerDuration 2
#define NUMER_HR   2

extern uint16_t distance, triggerTime, sensor;
extern GPIO_TypeDef *triggerPorts[];
extern uint16_t triggerPins[];
extern GPIO_TypeDef *echoPorts[];
extern uint16_t echoPins[];

void SysTickEnable();
void SysTickDisable();
uint16_t measureDistance(TIM_HandleTypeDef* htim, GPIO_TypeDef *triggerPort, uint16_t triggerPin, GPIO_TypeDef *echoPort, uint16_t echoPin);

//https://www.youtube.com/watch?v=Jmjtri25QWo
#endif /* __HR04_H_ */