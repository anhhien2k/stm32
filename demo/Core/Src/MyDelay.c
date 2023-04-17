/*
 *	MyDelay.c
 *
 *  Created on: Aug 6, 2021
 *      Author: tuan
 */
#include "MyDelay.h"

#define DELAY_TIMER					TIM3
#define DELAY_CLK_ENABLE			__HAL_RCC_TIM3_CLK_ENABLE
#define DELAY_IRQn 					TIM3_IRQn
#define DELAY_IRQHandler		TIM3_IRQHandler
#define APB_CLOCK_TIMER					64  //MHz

volatile uint32_t tickUs =0,tickMs =0;
static  TIM_HandleTypeDef htim_delay;
static void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
void DELAY_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim_delay);
	tickUs += 65000;
	tickMs += 65;
}

void delay_ms(uint32_t ms)
{
	uint32_t tickMsCurrent = get_tick_ms();
	while(get_tick_ms() - tickMsCurrent <= ms);
}
void delay_us(uint32_t us)
{
	uint32_t tickUsCurrent = get_tick_us();
	while(get_tick_us() - tickUsCurrent <= us);
}
uint32_t get_tick_ms(void)
{
	return tickMs +__HAL_TIM_GetCounter(&htim_delay)/1000;
}
uint32_t get_tick_us(void)
{
	return tickUs + __HAL_TIM_GetCounter(&htim_delay);
}

void delay_init()
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
	
  DELAY_CLK_ENABLE();
	HAL_NVIC_SetPriority(DELAY_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DELAY_IRQn);
	
  htim_delay.Instance = DELAY_TIMER;
  htim_delay.Init.Prescaler = APB_CLOCK_TIMER - 1;
  htim_delay.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_delay.Init.Period = 65000;
  htim_delay.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim_delay.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim_delay) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim_delay, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim_delay, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
	HAL_TIM_Base_Start_IT(&htim_delay);
}
