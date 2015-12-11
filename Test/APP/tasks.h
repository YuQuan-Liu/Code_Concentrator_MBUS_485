
#ifndef TASK_H
#define TASK_H





void param_config(uint8_t * buf_frame,uint8_t desc);
uint32_t search_cjq(uint8_t * cjqaddr);         //����Flash���Ƿ��Ѱ����˲ɼ���
uint32_t add_cjq(uint8_t * cjqaddr);    //��Ӳɼ���
uint32_t delete_cjq(uint32_t block_cjq);         //ɾ���ɼ���

uint32_t search_meter(uint32_t block_cjq,uint8_t * meteraddr);  //�ڲɼ����²����Ƿ��Ѿ������˱�
uint32_t add_meter(uint32_t block_cjq,uint8_t * meteraddr);
uint32_t delete_meter(uint32_t block_cjq,uint32_t block_meter);

void param_query(uint8_t * buf_frame,uint8_t desc);

void ack_query_mbus(uint8_t desc,uint8_t server_seq_);
void ack_query_test(uint8_t desc,uint8_t server_seq_);
void ack_query_cjq(uint8_t desc,uint8_t server_seq_);
void ack_query_meter(uint8_t metertype,uint8_t * meteraddr,uint8_t desc,uint8_t server_seq_);
void ack_query_addr(uint8_t desc,uint8_t server_seq_);
void ack_query_ip(uint8_t desc,uint8_t server_seq_);
void ack_query_di_seq(uint8_t desc,uint8_t server_seq_);

void meter_read(uint8_t * buf_frame,uint8_t desc);  //����
void meter_control(uint8_t * buf_frame,uint8_t desc);   //���ط�
void meter_read_single(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc);  //ֻ�ܶ���
void meter_send(uint8_t all,uint32_t block_meter_,uint8_t desc);  //all = 1 ����ȫ����  all = 0 ���ͱ���Ӧ�ı�
void meter_open(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc,uint8_t server_seq_);
void meter_close(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc,uint8_t server_seq_);

void device_ack(uint8_t desc,uint8_t server_seq_);  //����ȷ��֡  1 ���͸�M590E������  0 ���͸�485
void device_nack(uint8_t desc,uint8_t server_seq_);  //����ȷ��֡  1 ���͸�M590E������  0 ���͸�485

/*
����ײ��вɼ���  �ײ�ɼ���ʹ��͸��ģʽ
������ֱ�ӷ��ͳ���ָ��
cjq_open   cjq_close  ֻ��4·MBUS�̵������п���
*/
uint8_t cjq_open(uint8_t * cjq_addr,uint32_t block_cjq);
uint8_t cjq_close(uint8_t * cjq_addr,uint32_t block_cjq);

uint8_t relay_485(FunctionalState NewState);
uint8_t mbus_power(FunctionalState NewState);

uint8_t relay_1(FunctionalState NewState);
uint8_t relay_2(FunctionalState NewState);
uint8_t relay_3(FunctionalState NewState);
uint8_t relay_4(FunctionalState NewState);

//tasks
void Task_Slave(void *p_arg);
void Task_Server(void *p_arg);
void Task_DealServer(void *p_arg);
void Task_Connect(void *p_arg);
void Task_Read(void *p_arg);
void Task_HeartBeat(void *p_arg);
void Task_Config(void *p_arg);
void Task_LED1(void *p_arg);
void Task_OverLoad(void *p_arg);


uint8_t check_cs(uint8_t * start,uint16_t len);  //add the cs and return cs
uint8_t check_frame(uint8_t * start);

#endif