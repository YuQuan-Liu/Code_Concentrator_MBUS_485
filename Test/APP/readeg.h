

#ifndef _READ_G_H
#define _READ_G_H

/*抄单个表*/
void meter_single_eg(u8 *deal_ptr);
/*抄采集器*/
void meter_cjq_eg(u8 *deal_ptr);
/*给服务器返回抄到的数据*/
void send_data_eg(u8 metercount,uint8_t desc);
/*给服务器返回采集器超时*/
void send_cjqtimeout_eg(uint8_t desc);
/*
根据采集器 和表地址  去抄单个表
all = 1 表示抄全部表   
all = 0 表示抄单个表
return done  done = 1 表示抄到了表  done = 0 表示超时
*/
u8 read_single_eg(u8 cjq_h,u8 cjq_l,u8 meter_addr,u8 all);

#endif

