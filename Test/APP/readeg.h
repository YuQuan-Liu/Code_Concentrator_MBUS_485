

#ifndef _READ_G_H
#define _READ_G_H

/*��������*/
void meter_single_eg(u8 *deal_ptr);
/*���ɼ���*/
void meter_cjq_eg(u8 *deal_ptr);
/*�����������س���������*/
void send_data_eg(u8 metercount,uint8_t desc);
/*�����������زɼ�����ʱ*/
void send_cjqtimeout_eg(uint8_t desc);
/*
���ݲɼ��� �ͱ��ַ  ȥ��������
all = 1 ��ʾ��ȫ����   
all = 0 ��ʾ��������
return done  done = 1 ��ʾ�����˱�  done = 0 ��ʾ��ʱ
*/
u8 read_single_eg(u8 cjq_h,u8 cjq_l,u8 meter_addr,u8 all);

#endif

