
#ifndef TASK_H
#define TASK_H





void param_config(uint8_t * buf_frame,uint8_t desc);
uint32_t search_cjq(uint8_t * cjqaddr);         //查找Flash中是否已包含此采集器
uint32_t add_cjq(uint8_t * cjqaddr);    //添加采集器
uint32_t delete_cjq(uint32_t block_cjq);         //删除采集器

uint32_t search_meter(uint32_t block_cjq,uint8_t * meteraddr);  //在采集器下查找是否已经包含此表
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

void meter_read(uint8_t * buf_frame,uint8_t desc);  //抄表
void meter_control(uint8_t * buf_frame,uint8_t desc);   //开关阀
void meter_read_single(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc);  //只管读表
void meter_send(uint8_t all,uint32_t block_meter_,uint8_t desc);  //all = 1 发送全部表  all = 0 发送表块对应的表
void meter_open(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc,uint8_t server_seq_);
void meter_close(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc,uint8_t server_seq_);

void device_ack(uint8_t desc,uint8_t server_seq_);  //发送确认帧  1 发送给M590E服务器  0 发送给485
void device_nack(uint8_t desc,uint8_t server_seq_);  //发送确认帧  1 发送给M590E服务器  0 发送给485

/*
如果底层有采集器  底层采集器使用透传模式
集中器直接发送抄表指令
cjq_open   cjq_close  只对4路MBUS继电器进行控制
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