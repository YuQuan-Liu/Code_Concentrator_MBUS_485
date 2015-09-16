

#include "stm32f10x_conf.h"
#include "serial.h"
#include "os.h"
#include "lib_str.h"
#include "gprs.h"
#include "spi_flash.h"
#include "stdlib.h"


extern OS_MEM MEM_Buf;
extern OS_MEM MEM_ISR;

extern OS_SEM SEM_Send;      //got the '>'  we can send the data now  可以发送数据
extern OS_SEM SEM_SendOver;      //got the "+TCPSEND:0,"  the data is send over now  发送数据完成
extern OS_SEM SEM_Send_Online;   //发送数据时检测链路状态  "+IPSTATUS:0,CONNECT,TCP"


extern volatile uint8_t connectstate;
extern uint8_t * volatile server_ptr;      //中断中保存M590E 返回来的数据
extern uint8_t * volatile server_ptr_;     //记录中断的开始指针

//get the   APN,APN'username,APN'password,DNS
uint8_t apn[10] = "CMNET\r";                    //其中APN的最后必须是\r\0
uint8_t username[10] = "\"gsm\",";              //apn 的用户名必须为由双引号包围加上分割的，
uint8_t password[10] = "\"1234\"\r";            //apn 的用户名必须为由双引号包围加上结束符\r
//uint8_t dns[25] = "\"www.xcxdtech.com\"\r";     //the server 
uint8_t dns[25] = "\"avenger0422.vicp.cc\"\r";     //the server 
uint8_t ip[17] = "218.28.41.74";                 //the server ip
uint8_t port[8] = ",5555\r";                     //the server port
uint8_t deviceaddr[5] = {0x00,0x00,0x00,0x00,0x00};      //设备地址

uint8_t ip1 = 74;
uint8_t ip2 = 41;
uint8_t ip3 = 28;
uint8_t ip4 = 218;
uint16_t port_ = 3333;
u8 *ats[20]={
	"AT\r",
	"ATE0\r",   //关闭回显
	"AT+CCID\r",  //获取SIM卡标识
	"AT+CSQ\r",   //查询信号强度
	"AT+CREG?\r",  //检查网络注册状态
	"AT+XISP=0\r",  //设置成内部协议
	"AT+CGDCONT=1,\"IP\",\"CMNET\"\r",  //设置APN
	"AT+XGAUTH=1,1,\"GSM\",\"1234\"\r",   //专网用户认证
	"AT+CGATT?\r",  //GPRS附着
	"AT+XIIC=1\r",   //建立PPP连接
	"AT+XIIC?\r",    //检查PPP链路状态
	"AT+DNS=",   //DNS获取IP
	"AT+TCPSETUP=0,",   //+ip+port 建立TCP连接
	"AT+TCPSEND=0,",    //在链路0 上发送数据
	"AT+IPSTATUS=0\r"   //查询链路0的链路状态
};

/*
after Enable   need wait the "+PBREADY"   received the "+PBREADY" M590E is start up
*/

ErrorStatus Device_Cmd(FunctionalState NewState){
  OS_ERR err;
  CPU_TS ts;
  uint8_t cnt = 0;
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  if(NewState != DISABLE){
    buf_server = OSMemGet(&MEM_Buf,&err);
    buf_server_ = buf_server;
    GPIO_SetBits(GPIOB,GPIO_Pin_3);  //on_off 
    OSTimeDlyHMSM(0,0,0,500,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
      
    Mem_Set(buf_server_,0x00,256); //clear the buf
    
    GPIO_SetBits(GPIOA,GPIO_Pin_1); //enable the power
    Server_Post2Buf(buf_server_);   //post to the buf
    
    //wait the "+PBREADY"
    while(cnt < 250){
      OSTimeDlyHMSM(0,0,0,100,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
      if(server_ptr - server_ptr_ > 10){
        check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
        if(Str_Str(buf_server_,"+PBREADY\r\n")){
          OSMemPut(&MEM_Buf,buf_server_,&err);
          
          Server_Post2Buf(0);
          return SUCCESS;
        }
      }
      cnt++;
    }
    
    OSMemPut(&MEM_Buf,buf_server_,&err);
    
    Server_Post2Buf(0);
    return ERROR;
  }else{
    //disable the power
    GPIO_ResetBits(GPIOA,GPIO_Pin_1);
    OSTimeDlyHMSM(0,0,3,0,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
    return SUCCESS;
  }
}

uint8_t * Send_ReadATs(uint8_t *at,uint8_t *buf_server,uint32_t timeout){
  OS_ERR err;
  CPU_TS ts;
  
  Server_WriteStr(at);
  Server_Post2Buf(buf_server);  //接收数据到buf_server
  OSTimeDly(timeout,
             OS_OPT_TIME_DLY,
             &err);
  buf_server = server_ptr;
  
  Server_Post2Buf(0);   //停止接收数据
  return buf_server;
}

void check_str(uint8_t * start,uint8_t * end){
  uint8_t * s;
  s = start;
  while(*s == 0x00 && s < end){
    *s++ = '\r';
  }
}

ErrorStatus at(void){
  
}


ErrorStatus ate(void){
  uint8_t i;
  OS_ERR err;
  
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  buf_server = OSMemGet(&MEM_Buf,&err);
  buf_server_ = buf_server;
  
  for(i = 0;i < 3;i++){
    if(i != 0){
      OSTimeDlyHMSM(0,0,0,200,
                    OS_OPT_TIME_HMSM_STRICT,
                    &err);
    }
    Mem_Set(buf_server_,0x00,256); //clear the buf
    buf_server = Send_ReadATs(ats[1],buf_server_,100);
    check_str(buf_server_,buf_server);    //屏蔽掉数据前的0x00
    if(Str_Str(buf_server_,"OK")){
      OSMemPut(&MEM_Buf,buf_server_,&err);
      return SUCCESS;
    }
  }
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
}


ErrorStatus at_ccid(void){
  uint8_t i;
  OS_ERR err;
  
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  buf_server = OSMemGet(&MEM_Buf,&err);
  buf_server_ = buf_server;
  
  for(i = 0;i < 100;i++){
    if(i != 0){
      OSTimeDlyHMSM(0,0,0,200,
                    OS_OPT_TIME_HMSM_STRICT,
                    &err);
    }
    Mem_Set(buf_server_,0x00,256); //clear the buf
    buf_server = Send_ReadATs(ats[2],buf_server_,100);
    check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
    if(Str_Str(buf_server_,"+CCID")){
      OSMemPut(&MEM_Buf,buf_server_,&err);
      return SUCCESS;
    }
  }
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
}
ErrorStatus at_csq(void){
  uint8_t i;
  OS_ERR err;
  uint8_t good = 0;
  
  uint8_t *rssi_ptr = 0;
  uint8_t rssi = 0;
  uint8_t *ber_ptr = 0;
  uint8_t ber = 0;
  
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  buf_server = OSMemGet(&MEM_Buf,&err);
  buf_server_ = buf_server;
  
  for(i = 0;i < 100;i++){
    if(i != 0){
      OSTimeDlyHMSM(0,0,0,200,
                    OS_OPT_TIME_HMSM_STRICT,
                    &err);
    }
    Mem_Set(buf_server_,0x00,256); //clear the buf
    buf_server = Send_ReadATs(ats[3],buf_server_,100);
    check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
    if(Str_Str(buf_server_,"+CSQ: ")){
      rssi_ptr = Str_Char_N(buf_server_,256,':')+2;
      rssi = atoi(rssi_ptr);
      ber_ptr = Str_Char_N(buf_server_,256,',')+1;
      ber = atoi(ber_ptr);
      if(rssi > 5 && ber != 99){
        good++;
        if(good > 6){
          OSMemPut(&MEM_Buf,buf_server_,&err);
          return SUCCESS;
        }
      }else{
        good = 0;
      }
    }else{
      good = 0;
    }
  }
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
}
ErrorStatus at_creg(void){
  uint8_t i;
  OS_ERR err;
  
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  buf_server = OSMemGet(&MEM_Buf,&err);
  buf_server_ = buf_server;
  
  for(i = 0;i < 100;i++){
    if(i != 0){
      OSTimeDlyHMSM(0,0,0,200,
                    OS_OPT_TIME_HMSM_STRICT,
                    &err);
    }
    Mem_Set(buf_server_,0x00,256); //clear the buf
    buf_server = Send_ReadATs(ats[4],buf_server_,100);
    check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
    if(Str_Str(buf_server_,"+CREG: 0,1") || Str_Str(buf_server_,"+CREG: 0,5")){
      OSMemPut(&MEM_Buf,buf_server_,&err);
      return SUCCESS;
    }
  }
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
}

ErrorStatus at_xisp(void){
  uint8_t i;
  OS_ERR err;
  
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  buf_server = OSMemGet(&MEM_Buf,&err);
  buf_server_ = buf_server;
  
  //for(i = 0;i < 3;i++){
    Mem_Set(buf_server_,0x00,256); //clear the buf
    buf_server = Send_ReadATs(ats[5],buf_server_,100);
    check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
    if(Str_Str(buf_server_,"OK")){
      OSMemPut(&MEM_Buf,buf_server_,&err);
      return SUCCESS;
    }
  //}
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
}


ErrorStatus at_apn(void){
  uint8_t i;
  OS_ERR err;
  
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  buf_server = OSMemGet(&MEM_Buf,&err);
  buf_server_ = buf_server;
  
  //for(i = 0;i < 3;i++){
    Mem_Set(buf_server_,0x00,256); //clear the buf
    buf_server = Send_ReadATs(ats[6],buf_server_,100);
    check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
    if(Str_Str(buf_server_,"OK")){
      OSMemPut(&MEM_Buf,buf_server_,&err);
      return SUCCESS;
    }
  //}
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
}
ErrorStatus at_auth(void){
  uint8_t i;
  OS_ERR err;
  
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  buf_server = OSMemGet(&MEM_Buf,&err);
  buf_server_ = buf_server;
  
  //for(i = 0;i < 3;i++){
    Mem_Set(buf_server_,0x00,256); //clear the buf
    buf_server = Send_ReadATs(ats[7],buf_server_,100);
    check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
    if(Str_Str(buf_server_,"OK")){
      OSMemPut(&MEM_Buf,buf_server_,&err);
      return SUCCESS;
    }
  //}
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
}

ErrorStatus check_cgatt(void){
  uint8_t i;
  OS_ERR err;
  
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  buf_server = OSMemGet(&MEM_Buf,&err);
  buf_server_ = buf_server;
  
  for(i = 0;i < 100;i++){
    if(i != 0){
      OSTimeDlyHMSM(0,0,0,200,
                    OS_OPT_TIME_HMSM_STRICT,
                    &err);
    }
    Mem_Set(buf_server_,0x00,256); //clear the buf
    buf_server = Send_ReadATs(ats[8],buf_server_,100);
    check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
    if(Str_Str(buf_server_,"+CGATT: 1")){
      OSMemPut(&MEM_Buf,buf_server_,&err);
      return SUCCESS;
    }
  }
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
}
ErrorStatus at_xiic(void){
  uint8_t i;
  OS_ERR err;
  
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  buf_server = OSMemGet(&MEM_Buf,&err);
  buf_server_ = buf_server;
  
  for(i = 0;i < 3;i++){
    Mem_Set(buf_server_,0x00,256); //clear the buf
    buf_server = Send_ReadATs(ats[9],buf_server_,100);
    check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
    if(Str_Str(buf_server_,"OK")){
      OSMemPut(&MEM_Buf,buf_server_,&err);
      return SUCCESS;
    }
  }
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
}
ErrorStatus check_xiic(void){
  uint8_t i;
  OS_ERR err;
  
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  buf_server = OSMemGet(&MEM_Buf,&err);
  buf_server_ = buf_server;
  
  for(i = 0;i < 20;i++){
    if(i != 0){
      OSTimeDlyHMSM(0,0,0,200,
                    OS_OPT_TIME_HMSM_STRICT,
                    &err);
    }
    Mem_Set(buf_server_,0x00,256); //clear the buf
    buf_server = Send_ReadATs(ats[10],buf_server_,100);
    check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
    if(Str_Str(buf_server_,"+XIIC:    1")){
      OSMemPut(&MEM_Buf,buf_server_,&err);
      return SUCCESS;
    }
  }
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
}
//现在不使用域名
ErrorStatus at_dns(void){
  uint8_t i;
  OS_ERR err;
  uint8_t * ip_start;
  uint8_t count = 0;
  
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  buf_server = OSMemGet(&MEM_Buf,&err);
  buf_server_ = buf_server;
  
  for(i = 0;i < 100;i++){
    Mem_Set(buf_server_,0x00,256); //clear the buf
    Server_WriteStr(ats[11]);
    Server_WriteStr(dns);
    
    Server_Post2Buf(buf_server);  //接收数据到buf_server
    OSTimeDly(2000,
               OS_OPT_TIME_DLY,
               &err);
    buf_server = server_ptr;
    Server_Post2Buf(0);   //停止接收数据
    
    check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
    if(Str_Str(buf_server_,"\r\n+DNS:OK\r\n")){
      ip_start = Str_Str(buf_server_,"+DNS:");
      while(*(ip_start+5+count) != '\r'){
        ip[count] = *(ip_start+5+count);
        count++;
      }
      ip[count] = '\0';
      
      OSMemPut(&MEM_Buf,buf_server_,&err);
      return SUCCESS;
    }
    OSTimeDlyHMSM(0,0,0,200,
                    OS_OPT_TIME_HMSM_STRICT,
                    &err);
  }
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
}

ErrorStatus at_tcpsetup(void){
  uint8_t i;
  OS_ERR err;
  
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  buf_server = OSMemGet(&MEM_Buf,&err);
  buf_server_ = buf_server;
  
  //for(i = 0;i < 3;i++){
    Mem_Set(buf_server_,0x00,256); //clear the buf
    Server_WriteStr(ats[12]);
    Server_WriteStr(ip);
    Server_WriteStr(port);
    
    Server_Post2Buf(buf_server);  //接收数据到buf_server
    OSTimeDly(2000,
               OS_OPT_TIME_DLY,
               &err);
    buf_server = server_ptr;
    Server_Post2Buf(0);   //停止接收数据
    
    check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
    if(Str_Str(buf_server_,"+TCPSETUP:0,OK")){
      OSMemPut(&MEM_Buf,buf_server_,&err);
      return SUCCESS;
    }
  //}
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
}

ErrorStatus connect(void){
  OS_ERR err;
  if(ate() == ERROR){
    return ERROR;
  }
  if(at_ccid() == ERROR){
    return ERROR;
  }
  if(at_csq() == ERROR){
    return ERROR;
  }
  if(at_creg() == ERROR){
    return ERROR;
  }
  if(at_xisp() == ERROR){
    return ERROR;
  }
  if(at_apn() == ERROR){
    return ERROR;
  }
  if(at_auth() == ERROR){
    return ERROR;
  }
  if(check_cgatt() == ERROR){
    return ERROR;
  }
  if(at_xiic() == ERROR){
    return ERROR;
  }
  if(check_xiic() == ERROR){
    return ERROR;
  }
#ifdef USE_DNS
  if(at_dns() == ERROR){
    return ERROR;
  }
#endif
  if(at_tcpsetup() == ERROR){
    return ERROR;
  }
  //connectstate = 1;
  
  change_connect(1);
  //OSSemPost(&SEM_Connected,
  //          OS_OPT_POST_1,
  //           &err);
  
  //tcpsend(10,"0123456789");
  return SUCCESS;
}

ErrorStatus send_server(uint8_t * data,uint16_t count){
  CPU_TS ts;
  OS_ERR err;
  uint8_t sendcount[5];
  
  //send the data
  sprintf(sendcount,"%d",count);
  
  Server_WriteStr(ats[13]);
  Server_WriteStr(sendcount);
  Server_WriteStr("\r");
  
  OSSemPend(&SEM_Send,
            1000,
            OS_OPT_PEND_BLOCKING,
            &ts,
            &err);
  
  
  if(err != OS_ERR_NONE){
    return ERROR;
  }
  
  Server_Write(data,count);
  Server_Write("\r",1);
  
  OSSemPend(&SEM_SendOver,
            1000,
            OS_OPT_PEND_BLOCKING,
            &ts,
            &err);
  
  if(err != OS_ERR_NONE){
    return ERROR;
  }
  return SUCCESS;
  
}

//改变在线的状态
void change_connect(uint8_t state){
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  connectstate = state;
  CPU_CRITICAL_EXIT();
}  