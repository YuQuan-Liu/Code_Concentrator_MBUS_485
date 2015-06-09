

#ifndef BSP_H
#define BSP_H

#include "stm32f10x_conf.h"


#define  APP_START_TASK_STK_SIZE      128u

#define  APP_START_TASK_PRIO      3u




void BSP_Init(void);
uint32_t BSP_CPU_ClkFreq(void);
void BSP_LED_Toggle(uint8_t led);
void BSP_USART_Init(void);
void BSP_GPIO_Init(void);
void BSP_NVIC_Init(void);
void BSP_SPI_Init(void);
void BSP_IWDG_Init(void);

#endif