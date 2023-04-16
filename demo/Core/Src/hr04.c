#include "hr04.h"

uint16_t distance = 0, triggerTime = 0, sensor = 0;
GPIO_TypeDef *triggerPorts[NUMER_HR] = {GPIOB, GPIOB};
uint16_t triggerPins[NUMER_HR] = {GPIO_PIN_0, GPIO_PIN_1};
GPIO_TypeDef *echoPorts[NUMER_HR] = {GPIOA, GPIOA};
uint16_t echoPins[NUMER_HR] = {GPIO_PIN_6, GPIO_PIN_7};

void SysTickEnable()
{
	__disable_irq();
	SysTick->CTRL |= (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);
	__enable_irq();
}

void SysTickDisable()
{
	__disable_irq();
	SysTick->CTRL &= ~(SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);
	__enable_irq();
}

uint16_t measureDistance(TIM_HandleTypeDef* htim, GPIO_TypeDef *triggerPort, uint16_t triggerPin, GPIO_TypeDef *echoPort, uint16_t echoPin)
{
	if(!HAL_GPIO_ReadPin(echoPort, echoPin))//skip sensor if ECHO pin is still busy
	{
		SysTickDisable();
		HAL_TIM_Base_Start_IT(htim);
		HAL_GPIO_WritePin(triggerPort, triggerPin, GPIO_PIN_SET);
		triggerTime = 0;//reset the variable
		//asm ("nop");//to avoid program freezing
		while(triggerTime < TriggerDuration);
		HAL_GPIO_WritePin(triggerPort, triggerPin, GPIO_PIN_RESET);
		while(!HAL_GPIO_ReadPin(echoPort, echoPin));
		distance = 0;//reset the variable
		while(HAL_GPIO_ReadPin(echoPort, echoPin));
		HAL_TIM_Base_Stop_IT(htim);
		SysTickEnable();
	}else{//give max distance if ECHO pin is still busy
		distance = 500;
	}
	return distance;
}