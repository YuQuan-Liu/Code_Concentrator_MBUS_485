

#ifndef SERIAL_H
#define SERIAL_H


void USART1_Handler(void);
void USART2_Handler(void);
void USART3_Handler(void);

ErrorStatus Slave_Write(uint8_t * data,uint16_t count);
ErrorStatus Server_Write(uint8_t * data,uint16_t count);
ErrorStatus Server_WriteStr(uint8_t * data);

ErrorStatus Server_Post2Queue(FunctionalState NewState);
ErrorStatus Device_Read(FunctionalState NewState);

/*
Й§ди
*/
#define OVERLOAD (OS_FLAGS)0x0001

void OverLoad(void);

#endif