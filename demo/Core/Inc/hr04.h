#ifndef __HR04_H_
#define __HR04_H_

#include "main.h"

extern TIM_HandleTypeDef htim2;

#define TriggerDuration 2

extern uint16_t distance, triggerTime, sensor;
extern GPIO_TypeDef *triggerPorts[3];
extern uint16_t triggerPins[3];
extern GPIO_TypeDef *echoPorts[3];
extern uint16_t echoPins[3];

void SysTickEnable();
void SysTickDisable();
uint16_t measureDistance(GPIO_TypeDef *triggerPort, uint16_t triggerPin, GPIO_TypeDef *echoPort, uint16_t echoPin);

//https://www.youtube.com/watch?v=Jmjtri25QWo
#endif /* __HR04_H_ */