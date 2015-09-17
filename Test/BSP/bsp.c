
#include "bsp.h"

void BSP_Init(void){
  
  //SYSCLK > 48M  two wait state
  FLASH_SetLatency(FLASH_Latency_2);
  
  //the half cycle must used clock < 8M  from the HSI or HSE  not the PLL
  //FLASH_HalfCycleAccessCmd(FLASH_HalfCycleAccess_Enale);
  
  //prefetch buffer switch on/off when sysclk < 24M
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
  
  RCC_DeInit();
  
  //RCC_HSEConfig(RCC_HSE_ON);
  //RCC_WaitForHSEStartUp();
  
  //HSI = 8M
  
  RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_16);  //PLL = HSI / 2 * 16 = 64M
  RCC_PLLCmd(ENABLE);
  
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);    //SYSCLK = 64M
  
  //if the HCLK is not div1 the flash prefetch and half cycle is 
  RCC_HCLKConfig(RCC_SYSCLK_Div1);      //HCLK = 64M
  
  RCC_PCLK1Config(RCC_HCLK_Div2);       //APB1 = 32M
  
  RCC_PCLK2Config(RCC_HCLK_Div1);       //APB2 = 64M
  
  //wait PLL is ready
  while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){
    ;
  }
  
  RCC_LSICmd(ENABLE);
  while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET){
    ;
  }
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
  
  
  BSP_GPIO_Init();
  BSP_NVIC_Init();
  BSP_USART_Init();
  BSP_SPI_Init();
  
}

void BSP_GPIO_Init(void){
  GPIO_InitTypeDef gpio_init;
  EXTI_InitTypeDef exti_init;
  
  //set all the gpio to push pull to low
  gpio_init.GPIO_Pin = GPIO_Pin_All;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&gpio_init);
  GPIO_Init(GPIOB,&gpio_init);
  GPIO_ResetBits(GPIOA,GPIO_Pin_All);
  GPIO_ResetBits(GPIOB,GPIO_Pin_All);
  
  //USART1~~~~~~~~~~~~~~~~~~~~~~~
  /* Configure USART1 Rx as input floating */
  //GPIOA 10
  gpio_init.GPIO_Pin = GPIO_Pin_10;
  gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &gpio_init);  
  
  /* Configure USART1 Tx as alternate function push-pull */
  //GPIOA 9
  gpio_init.GPIO_Pin = GPIO_Pin_9;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &gpio_init);
  
  //USART2~~~~~~~~~~~~~~~~~~~~~~~
  /* Configure USART1 Rx as input floating */
  //GPIOA 3
  gpio_init.GPIO_Pin = GPIO_Pin_3;
  gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &gpio_init);  
  
  /* Configure USART1 Tx as alternate function push-pull */
  //GPIOA 2
  gpio_init.GPIO_Pin = GPIO_Pin_2;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &gpio_init);
  
  //USART3~~~~~~~~~~~~~~~~~~~~~~~~~~~
  /* Configure USART3 Rx as input floating */
  //GPIOB 11
  gpio_init.GPIO_Pin = GPIO_Pin_11;
  gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &gpio_init);  
  
  /* Configure USART3 Tx as alternate function push-pull */
  //GPIOB 10
  gpio_init.GPIO_Pin = GPIO_Pin_10;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &gpio_init);
  
  ///* configure SPI2 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  //SCK
  gpio_init.GPIO_Pin = GPIO_Pin_13;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&gpio_init);
  //MISO
  gpio_init.GPIO_Pin = GPIO_Pin_14;
  gpio_init.GPIO_Mode = GPIO_Mode_IPU;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&gpio_init);
  //MOSI
  gpio_init.GPIO_Pin = GPIO_Pin_15;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&gpio_init);
  //NSS  Not used,used a gpio
  gpio_init.GPIO_Pin = GPIO_Pin_12;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&gpio_init);
  
  //GPIOA
  //GPIOA 1  M590E_POWER_CON  
  //GPIOA 7  DELAY_4
  //GPIOA 8  485_CONTROL
  //GPIOA 15  BEEP   
  
  //GPIOA 6  Relay 485 power  realy5
  //GPIOA 4  ON_OFF
  //GPIOA 5  EMERGOFF  这个现在没用
  /**/
  gpio_init.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_15 | GPIO_Pin_6 | GPIO_Pin_4 | GPIO_Pin_5;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&gpio_init);
  
  //GPIOA 15 default is JTDI
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  
  GPIO_ResetBits(GPIOA,GPIO_Pin_6);
  GPIO_ResetBits(GPIOA,GPIO_Pin_1 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_15);
  
  
  //GPIOB
  //GPIOB 7  LED1
  //GPIOB 9  LED2
  //GPIOB 6  LED3
  //GPIOB 5  LED4
  //GPIOB 0  DELAY_3
  //GPIOB 1  DELAY_2
  //GPIOB 2  DELAY_1
  
  gpio_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_9;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&gpio_init);
  
  GPIO_SetBits(GPIOB,GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_9);
  
  //GPIOC
  //GPIOC 13  DCDC_EN   0 close the mbus power   1 open the mbus power
  
  gpio_init.GPIO_Pin = GPIO_Pin_13;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC,&gpio_init);
  
  GPIO_ResetBits(GPIOC,GPIO_Pin_13);
  
  //GPIOA
  //GPIOA  0  FEEDBACK  
  gpio_init.GPIO_Pin = GPIO_Pin_0;
  gpio_init.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC,&gpio_init);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
  
  /* Configure EXTI0 line */
  exti_init.EXTI_Line = EXTI_Line0;
  exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
  exti_init.EXTI_Trigger = EXTI_Trigger_Falling;  
  exti_init.EXTI_LineCmd = ENABLE;
  EXTI_Init(&exti_init);
}

void BSP_USART_Init(void){
  USART_InitTypeDef usart_init;
  
  /*USART1  485*/
  usart_init.USART_BaudRate = 1200;
  usart_init.USART_WordLength = USART_WordLength_9b;
  usart_init.USART_Parity = USART_Parity_Even;
  usart_init.USART_StopBits = USART_StopBits_1;
  usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  
  USART_Init(USART1, &usart_init);
  USART_Cmd(USART1, ENABLE);
  
  USART_ITConfig(USART1, USART_IT_TC, DISABLE);
  USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  
  /*USART2  mbus*/
  usart_init.USART_BaudRate = 2400;
  usart_init.USART_WordLength = USART_WordLength_9b;
  usart_init.USART_Parity = USART_Parity_Even;
  usart_init.USART_StopBits = USART_StopBits_1;
  usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  
  USART_Init(USART2, &usart_init);
  USART_Cmd(USART2, ENABLE);
  
  USART_ITConfig(USART2, USART_IT_TC, DISABLE);
  USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  
  /*USART3  m590e*/
  usart_init.USART_BaudRate = 115200;
  usart_init.USART_WordLength = USART_WordLength_8b;
  usart_init.USART_Parity = USART_Parity_No;
  usart_init.USART_StopBits = USART_StopBits_1;
  usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  
  USART_Init(USART3, &usart_init);
  USART_Cmd(USART3, ENABLE);
    
  USART_ITConfig(USART3, USART_IT_TC, DISABLE);
  USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

void BSP_NVIC_Init(void){
  NVIC_InitTypeDef nvic_init;
  
  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  /* Enable the USART1 Interrupt */
  nvic_init.NVIC_IRQChannel = USART1_IRQn;
  nvic_init.NVIC_IRQChannelSubPriority = 0;
  nvic_init.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic_init);
  
  /* Enable the USART3 Interrupt */
  nvic_init.NVIC_IRQChannel = USART3_IRQn;
  nvic_init.NVIC_IRQChannelSubPriority = 1;
  nvic_init.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic_init);
  
  /* Enable the USART2 Interrupt */
  nvic_init.NVIC_IRQChannel = USART2_IRQn;
  nvic_init.NVIC_IRQChannelSubPriority = 2;
  nvic_init.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic_init);
  
  /* Enable the EXTI0 Interrupt */
  nvic_init.NVIC_IRQChannel = EXTI0_IRQn;
  nvic_init.NVIC_IRQChannelSubPriority = 3;
  nvic_init.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic_init);
  
  /*
  nvic_init.NVIC_IRQChannel = SPI1_IRQn;
  nvic_init.NVIC_IRQChannelSubPriority = 1;
  nvic_init.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic_init);
  */
  
}

void BSP_SPI_Init(void){
  SPI_InitTypeDef spi_init;
  
  //the APB1 is 32M
  spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  spi_init.SPI_CPHA = SPI_CPHA_2Edge;
  spi_init.SPI_CPOL = SPI_CPOL_High;
  spi_init.SPI_DataSize = SPI_DataSize_8b;
  spi_init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  
  spi_init.SPI_FirstBit = SPI_FirstBit_MSB;
  spi_init.SPI_NSS = SPI_NSS_Soft;
  spi_init.SPI_Mode = SPI_Mode_Master;
  spi_init.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2,&spi_init);
  
  SPI_I2S_ITConfig(SPI2,SPI_I2S_IT_TXE,DISABLE);
  SPI_I2S_ITConfig(SPI2,SPI_I2S_IT_RXNE,DISABLE);
  
  SPI_Cmd(SPI2,ENABLE);
  
}

void BSP_IWDG_Init(void){
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  /* IWDG counter clock: LSI/32 */
  IWDG_SetPrescaler(IWDG_Prescaler_32);   //3276.8ms
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();
}








//the uC OS need it
uint32_t BSP_CPU_ClkFreq(void){
  
  RCC_ClocksTypeDef rcc_clocks;
  
  RCC_GetClocksFreq(&rcc_clocks);
  
  return rcc_clocks.HCLK_Frequency;
  
}
