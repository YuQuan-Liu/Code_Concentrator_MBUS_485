


#ifndef __GPRS_H__
#define __GPRS_H__

#define USE_DNS          //ʹ��DNS��ȡIP��ַ   


/*ʹ��ģ��ΪSIM800G*/
#define SIM800G
#define TEST_DNS "\"avenger0422.vicp.cc\""
#define REAL_DNS "\"www.xcxdtech.com\""

/*ʹ��ģ��ΪM590E
#define M590E
#define TEST_DNS "\"avenger0422.vicp.cc\"\r"
#define REAL_DNS "\"www.xcxdtech.com\"\r"
*/

/******************************************************
�������ĵ�ַ   ���ù�֮��ʹ�����ù���  δ���ù�ʹ��Ĭ�ϵ�5700000999
*/

ErrorStatus send_server(uint8_t * data,uint16_t count);

ErrorStatus connect(void);


ErrorStatus Device_Cmd(FunctionalState NewState);

void check_str(uint8_t * start,uint8_t * end);  //��start��ʼ��end�������ַ��� ��ͷ��0x00�滻�ɡ�\r��

void change_connect(uint8_t state);  //�ı����ߵ�״̬

#endif

