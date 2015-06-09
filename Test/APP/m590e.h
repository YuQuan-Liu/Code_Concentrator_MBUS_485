
#ifndef M590E_H

#define M590E_H

#define USE_DNS          //使用DNS获取IP地址                               


#define AT              "AT\r"                                  //查看模块是否正常
#define ATE             "ATE\r"                                 //关闭回显
#define AT_CCID         "AT+CCID\r"                             //获取SIM卡标识 
#define AT_CSQ          "AT+CSQ\r"                              //查询信号强度
#define AT_CREG         "AT+CREG?\r"                            //检查网络注册状态
#define AT_XISP         "AT+XISP=0\r"                           //设置成内部协议
#define AT_CGDCONT      "AT+CGDCONT=1,IP,"                      //设置APN
#define AT_XGAUTH       "AT+XGAUTH=1,1,"                        //专网用户认证
#define AT_CGATT        "AT+CGATT=1\r"                          //GPRS附着
#define AT_CGATT_       "AT+CGATT?\r"                          //查询GPRS附着状态
#define AT_XIIC         "AT+XIIC=1\r"                           //建立PPP连接
#define AT_XIIC_        "AT+XIIC?\r"                            //检查PPP链路状态
#define AT_DNS          "AT+DNS="                               //DNS获取IP
#define AT_TCPSETUP     "AT+TCPSETUP=0,"                        //+ip+port 建立TCP连接

#define AT_TCPCLOSE     "AT+TCPCLOSE=0\r"                       //关闭链路0
#define AT_TCPSEND      "AT+TCPSEND=0,"                         //在链路0 上发送数据

#define AT_CPWROFF      "AT+CPWROFF\r"                          //模块软关机

#define AT_IPSTATUS     "AT+IPSTATUS=0\r"       //查询链路0的链路状态



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
void check_str(uint8_t * start,uint8_t * end);  //将start开始的end结束的字符串 开头的0x00替换成‘\r’

void change_connect(uint8_t state);  //改变在线的状态
#endif