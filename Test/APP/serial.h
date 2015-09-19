

#ifndef SERIAL_H
#define SERIAL_H


void USART1_Handler(void);
void USART2_Handler(void);
void USART3_Handler(void);

ErrorStatus Slave_Write(uint8_t * data,uint16_t count);
ErrorStatus Server_Write_485(uint8_t * data,uint16_t count);  //to 485
ErrorStatus Server_Write(uint8_t * data,uint16_t count);  //to m590e
ErrorStatus Server_WriteStr(uint8_t * data);  //to m590e

/**
ptr == 0  �ж��в��ŵ�buf��
ptr != 0  �ж��зŵ�ptr��ʼ��buf��
*/
ErrorStatus Server_Post2Buf(uint8_t * ptr);
ErrorStatus Device_Read(FunctionalState NewState);

/*
����
*/
#define OVERLOAD (OS_FLAGS)0x0001

void OverLoad(void);

#endif