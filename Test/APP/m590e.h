
#ifndef M590E_H

#define M590E_H

#define USE_DNS          //ʹ��DNS��ȡIP��ַ                               


#define AT              "AT\r"                                  //�鿴ģ���Ƿ�����
#define ATE             "ATE\r"                                 //�رջ���
#define AT_CCID         "AT+CCID\r"                             //��ȡSIM����ʶ 
#define AT_CSQ          "AT+CSQ\r"                              //��ѯ�ź�ǿ��
#define AT_CREG         "AT+CREG?\r"                            //�������ע��״̬
#define AT_XISP         "AT+XISP=0\r"                           //���ó��ڲ�Э��
#define AT_CGDCONT      "AT+CGDCONT=1,IP,"                      //����APN
#define AT_XGAUTH       "AT+XGAUTH=1,1,"                        //ר���û���֤
#define AT_CGATT        "AT+CGATT=1\r"                          //GPRS����
#define AT_CGATT_       "AT+CGATT?\r"                          //��ѯGPRS����״̬
#define AT_XIIC         "AT+XIIC=1\r"                           //����PPP����
#define AT_XIIC_        "AT+XIIC?\r"                            //���PPP��·״̬
#define AT_DNS          "AT+DNS="                               //DNS��ȡIP
#define AT_TCPSETUP     "AT+TCPSETUP=0,"                        //+ip+port ����TCP����

#define AT_TCPCLOSE     "AT+TCPCLOSE=0\r"                       //�ر���·0
#define AT_TCPSEND      "AT+TCPSEND=0,"                         //����·0 �Ϸ�������

#define AT_CPWROFF      "AT+CPWROFF\r"                          //ģ����ػ�

#define AT_IPSTATUS     "AT+IPSTATUS=0\r"       //��ѯ��·0����·״̬



ErrorStatus at(void);
ErrorStatus ate(void);
ErrorStatus at_ccid(void);
ErrorStatus at_csq(void);
ErrorStatus at_creg(void);
ErrorStatus at_xisp(void);
ErrorStatus at_cgdcont(void);
ErrorStatus at_xgauth(void);
ErrorStatus at_cgatt(void);
ErrorStatus check_cgatt(void);
ErrorStatus at_xiic(void);
ErrorStatus check_xiic(void);
ErrorStatus at_dns(void);
ErrorStatus at_tcpclose(void);
ErrorStatus at_tcpsetup(void);
ErrorStatus M590E_SoftOFF(void);




ErrorStatus send_server(uint8_t * data,uint16_t count);

ErrorStatus connect(void);


ErrorStatus M590E_Cmd(FunctionalState NewState);
uint8_t * M590E_ReadAT_100(uint8_t * buf_server);
uint8_t * M590E_ReadAT_2s(uint8_t * buf_server);
void check_str(uint8_t * start,uint8_t * end);  //��start��ʼ��end�������ַ��� ��ͷ��0x00�滻�ɡ�\r��

void change_connect(uint8_t state);  //�ı����ߵ�״̬
#endif