
#include "stm32f10x_conf.h"
#include "os.h"
#include "serial.h"


extern OS_MEM MEM_ISR;
extern OS_Q Q_Server;
extern OS_SEM SEM_ServerTX;

extern volatile uint8_t q_server_pending;
//m590e
void USART2_Handler(void){
  OS_ERR err;
  uint8_t rx_byte;
  uint8_t *mem_ptr;
  
  //receive the byte
  if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE)){
    rx_byte = USART_ReceiveData(USART2);
    /**/
    if(q_server_pending != 0){
      mem_ptr = OSMemGet(&MEM_ISR,&err);
      *mem_ptr = rx_byte;
      OSQPost((OS_Q *)&Q_Server,
              (void *)mem_ptr,
              1,
              OS_OPT_POST_FIFO,
              &err);
      
      if(err != OS_ERR_NONE){
        asm("NOP");
      }
    }
    
  }
  
  //send the data
  /**/
  if(USART_GetFlagStatus(USART2,USART_FLAG_TC)){
    //It must clear the TC 
    //if not it will stay here 
    USART_ClearITPendingBit(USART2,USART_IT_TC);
    OSSemPost(&SEM_ServerTX,
              OS_OPT_POST_1,
              &err);
    
    if(err != OS_ERR_NONE){
      asm("NOP");
    }
  }
  
}


extern OS_Q Q_Slave;
extern OS_SEM SEM_Slave_485TX;
//485
void USART1_Handler(void){
  OS_ERR err;
  uint8_t rx_byte;
  uint8_t *mem_ptr;
  
  //receive the byte
  if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE)){
    rx_byte = USART_ReceiveData(USART1);
    mem_ptr = OSMemGet(&MEM_ISR,&err);
    *mem_ptr = rx_byte;
    OSQPost((OS_Q *)&Q_Slave,
            (void *)mem_ptr,
            1,
            OS_OPT_POST_FIFO,
            &err);
    
    if(err != OS_ERR_NONE){
      asm("NOP");
    }
  }
  
  //send the data
  if(USART_GetFlagStatus(USART1,USART_FLAG_TC)){
    
    USART_ClearITPendingBit(USART1,USART_IT_TC);
    OSSemPost(&SEM_Slave_485TX,
              OS_OPT_POST_1,
              &err);
    
    if(err != OS_ERR_NONE){
      asm("NOP");
    }
  }
  
}

extern OS_SEM SEM_Slave_mbusTX;
//mbus
void USART3_Handler(void){
  OS_ERR err;
  uint8_t rx_byte;
  uint8_t *mem_ptr;
  
  //receive the byte
  if(USART_GetFlagStatus(USART3,USART_FLAG_RXNE)){
    rx_byte = USART_ReceiveData(USART3);
    mem_ptr = OSMemGet(&MEM_ISR,&err);
    *mem_ptr = rx_byte;
    OSQPost((OS_Q *)&Q_Slave,
            (void *)mem_ptr,
            1,
            OS_OPT_POST_FIFO,
            &err);
    
    if(err != OS_ERR_NONE){
      asm("NOP");
    }
  }
  
  //send the data
  if(USART_GetFlagStatus(USART3,USART_FLAG_TC)){
    
    USART_ClearITPendingBit(USART3,USART_IT_TC);
    OSSemPost(&SEM_Slave_mbusTX,
              OS_OPT_POST_1,
              &err);
    
    if(err != OS_ERR_NONE){
      asm("NOP");
    }
  }
}

extern uint8_t slave_mbus; //0xaa mbus   0xff  485
ErrorStatus Slave_Write(uint8_t * data,uint16_t count){
  uint16_t i;
  CPU_TS ts;
  OS_ERR err;
  
  //send to mbus
  /**/
  if(slave_mbus == 0xAA){
    USART_ITConfig(USART3,USART_IT_TC,ENABLE);
    
    for(i = 0;i < count;i++){
      err = OS_ERR_NONE;
      OSSemPend(&SEM_Slave_mbusTX,
                500,
                OS_OPT_PEND_BLOCKING,
                &ts,
                &err);
      
      
      USART_SendData(USART3,*(data+i));
    }
    
    USART_ITConfig(USART3,USART_IT_TC,DISABLE);
    while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
    
  }
  
  //send to 485
  GPIO_SetBits(GPIOA,GPIO_Pin_8);
  USART_ITConfig(USART1,USART_IT_TC,ENABLE);
  
  for(i = 0;i < count;i++){
    err = OS_ERR_NONE;
    OSSemPend(&SEM_Slave_485TX,
              500,
              OS_OPT_PEND_BLOCKING,
              &ts,
              &err);
    
    USART_SendData(USART1,*(data+i));
  }
  
  
  USART_ITConfig(USART1,USART_IT_TC,DISABLE);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
  /*
  OSTimeDlyHMSM(0,0,0,50,
                    OS_OPT_TIME_HMSM_STRICT,
                    &err);
  */
  GPIO_ResetBits(GPIOA,GPIO_Pin_8);
  return SUCCESS;
}

ErrorStatus Server_Write(uint8_t * data,uint16_t count){
  uint16_t i;
  CPU_TS ts;
  OS_ERR err;
  
  USART_ITConfig(USART2,USART_IT_TC,ENABLE);
  for(i = 0;i < count;i++){
    OSSemPend(&SEM_ServerTX,
              50,
              OS_OPT_PEND_BLOCKING,
              &ts,
              &err);
    if(err != OS_ERR_NONE){
      return ERROR;
    }
    USART_SendData(USART2,*(data+i));
  }
  USART_ITConfig(USART2,USART_IT_TC,DISABLE);
  
  return SUCCESS;
}

ErrorStatus Server_WriteStr(uint8_t * data){
  CPU_TS ts;
  OS_ERR err;
  
  uint8_t * str = data;
  
  USART_ITConfig(USART2,USART_IT_TC,ENABLE);
  USART_ClearITPendingBit(USART2,USART_IT_TC);
  while(*str != '\0'){
    OSSemPend(&SEM_ServerTX,
              50,
              OS_OPT_PEND_BLOCKING,
              &ts,
              &err);
    if(err != OS_ERR_NONE){
      return ERROR;
    }
    USART_SendData(USART2,*str);
    str++;
  }
  
  USART_ITConfig(USART2,USART_IT_TC,DISABLE);
  return SUCCESS;
}

extern volatile uint8_t q_server_pending;
ErrorStatus Server_Post2Queue(FunctionalState NewState){
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  
  if(NewState != DISABLE){
    //CPU_CRITICAL_ENTER();
    q_server_pending = 1;
    //CPU_CRITICAL_EXIT();
  }else{
    //CPU_CRITICAL_ENTER();
    q_server_pending = 0;
    //CPU_CRITICAL_EXIT();
  }
  CPU_CRITICAL_EXIT();
  return SUCCESS;
}


extern volatile uint8_t reading;
ErrorStatus Device_Read(FunctionalState NewState){
  CPU_SR_ALLOC();
  if(NewState != DISABLE){
    CPU_CRITICAL_ENTER();
    reading = 1;
    CPU_CRITICAL_EXIT();
  }else{
    CPU_CRITICAL_ENTER();
    reading = 0;
    CPU_CRITICAL_EXIT();
  }
  return SUCCESS;
}


extern OS_FLAG_GRP FLAG_Event;
void OverLoad(void){
  OS_ERR err;
  if(EXTI_GetITStatus(EXTI_Line14) != RESET)
  {
    OSFlagPost(&FLAG_Event,
               OVERLOAD,
               OS_OPT_POST_FLAG_SET,
               &err);
    
    /* Clear the  EXTI line 9 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line14);
  }
}