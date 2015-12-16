#include "stm32f10x_conf.h"
#include "serial.h"
#include "os.h"
#include "lib_str.h"
#include "gprs.h"
#include "spi_flash.h"
#include "stdlib.h"

/**/
//#define _AT_DNS

extern OS_MEM MEM_Buf;
extern OS_MEM MEM_ISR;

extern OS_SEM SEM_Send;      //got the '>'  we can send the data now  可以发送数据
extern OS_SEM SEM_SendOver;      //got the "+TCPSEND:0,"  the data is send over now  发送数据完成
extern OS_SEM SEM_Send_Online;   //发送数据时检测链路状态  "+IPSTATUS:0,CONNECT,TCP"


extern volatile uint8_t connectstate;
extern uint8_t * volatile server_ptr;      //中断中保存M590E 返回来的数据
extern uint8_t * volatile server_ptr_;     //记录中断的开始指针

uint8_t ip[17] = "218.28.41.74";                 //the server ip
uint8_t port[8] = ",3333\r";                     //the server port
uint8_t deviceaddr[5] = {0x99,0x09,0x00,0x00,0x57};      //设备地址

uint8_t ip1 = 218;
uint8_t ip2 = 28;
uint8_t ip3 = 41;
uint8_t ip4 = 74;
uint16_t port_ = 3333;

u8 *ats[20]={
	"AT\r",
	"ATE0\r",   //关闭回显
	"AT+CPIN?\r",  //获取SIM卡标识
	"AT+CSQ\r",   //查询信号强度
	"AT+CREG?\r",  //检查网络注册状态
	"AT+CGATT?\r",  //检查GPRS附着状态
	"AT+CIPMUX=1\r",  //设置成多链路模式
	"AT+CSTT=\"3GWAP\"\r",  //设置APN
        //"AT+CSTT=\"CMNET\"\r",  //设置APN
	"AT+CIICR\r",   //建立PPP连接
	"AT+CIFSR\r",    //获取本地IP地址
	"AT+CIPSTART=0,\"TCP\",",   //+ip+port 建立TCP连接
	"AT+CIPSEND=0,",    //在链路0 上发送数据
};


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

ErrorStatus ate_(void){

  OS_ERR err;
  
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  buf_server = OSMemGet(&MEM_Buf,&err);
  buf_server_ = buf_server;
  
  Mem_Set(buf_server_,0x00,256); //clear the buf
  buf_server = Send_ReadATs(ats[1],buf_server_,100);
  check_str(buf_server_,buf_server);    //屏蔽掉数据前的0x00
  if(Str_Str(buf_server_,"OK")){
    OSMemPut(&MEM_Buf,buf_server_,&err);
    return SUCCESS;
  }
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
}

ErrorStatus at(void){
  
}


/*
检查SIM卡状态
*/
ErrorStatus at_cpin(void){
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
    //the content +CPIN: READY
    if(Str_Str(buf_server_,"READY")){
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
    if(Str_Str(buf_server_,"CSQ")){
      rssi_ptr = Str_Char_N(buf_server_,256,':')+2;
      rssi = atoi(rssi_ptr);
      ber_ptr = Str_Char_N(buf_server_,256,',')+1;
      ber = atoi(ber_ptr);
      //if(rssi > 5 && ber != 99){
      if(rssi > 5){
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
    buf_server = Send_ReadATs(ats[5],buf_server_,100);
    check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
    if(Str_Str(buf_server_,"+CGATT: 1")){
      OSMemPut(&MEM_Buf,buf_server_,&err);
      return SUCCESS;
    }
  }
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
}

//多链路模式
ErrorStatus at_cipmux(void){
  OS_ERR err;
  
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  buf_server = OSMemGet(&MEM_Buf,&err);
  buf_server_ = buf_server;
  
  Mem_Set(buf_server_,0x00,256); //clear the buf
  buf_server = Send_ReadATs(ats[6],buf_server_,100);
  check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
  if(Str_Str(buf_server_,"OK")){
    OSMemPut(&MEM_Buf,buf_server_,&err);
    return SUCCESS;
  }
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
}

ErrorStatus at_apn(void){
  OS_ERR err;
  
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  buf_server = OSMemGet(&MEM_Buf,&err);
  buf_server_ = buf_server;
  
  Mem_Set(buf_server_,0x00,256); //clear the buf
  buf_server = Send_ReadATs(ats[7],buf_server_,100);
  check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
  if(Str_Str(buf_server_,"OK")){
    OSMemPut(&MEM_Buf,buf_server_,&err);
    return SUCCESS;
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
  
  for(i = 0;i < 100;i++){
    Mem_Set(buf_server_,0x00,256); //clear the buf
    buf_server = Send_ReadATs(ats[8],buf_server_,2000);
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
  
  for(i = 0;i < 100;i++){
    if(i != 0){
      OSTimeDlyHMSM(0,0,0,200,
                    OS_OPT_TIME_HMSM_STRICT,
                    &err);
    }
    Mem_Set(buf_server_,0x00,256); //clear the buf
    buf_server = Send_ReadATs(ats[9],buf_server_,100);
    check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
    if(Str_Str(buf_server_,".")){
      OSMemPut(&MEM_Buf,buf_server_,&err);
      return SUCCESS;
    }
  }
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
}

extern uint8_t device_test; //0x00~测试过了~IP   0xFF~未测试~域名（avenger0422.vicp.cc）
ErrorStatus at_tcpsetup(void){
  OS_ERR err;
  
  uint8_t * buf_server = 0;
  uint8_t * buf_server_ = 0;
  
  buf_server = OSMemGet(&MEM_Buf,&err);
  buf_server_ = buf_server;
  
    Mem_Set(buf_server_,0x00,256); //clear the buf
    Server_WriteStr(ats[10]);
    
    //测试为通过连接测试域名  
    //正式使用时连接IP  地址
    if(device_test == 0xFF){
      Server_WriteStr(TEST_DNS);
    }else{
      Server_WriteStr(ip);
    }
    
    Server_WriteStr(port);
    
    Server_Post2Buf(buf_server);  //接收数据到buf_server
    OSTimeDly(3000,
               OS_OPT_TIME_DLY,
               &err);
    buf_server = server_ptr;
    Server_Post2Buf(0);   //停止接收数据
    
    check_str(buf_server_,buf_server);  //屏蔽掉数据前的0x00
    if(Str_Str(buf_server_,"CONNECT OK")){
      OSMemPut(&MEM_Buf,buf_server_,&err);
      return SUCCESS;
    }
  OSMemPut(&MEM_Buf,buf_server_,&err);
  return ERROR;
    
}

/*
the send is the pointer to the data 
the count is the number of the data
*/
extern OS_MUTEX MUTEX_SENDSERVER;    //是否可以发送数据到服务器
ErrorStatus send_server(uint8_t * send,uint16_t count){
  CPU_TS ts;
  OS_ERR err;
  uint8_t sendcount[5];
  
  OSMutexPend(&MUTEX_SENDSERVER,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  
  //send the data
  sprintf(sendcount,"%d",count);
  
  Server_WriteStr(ats[11]);
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
  
  Server_Write(send,count);
  
  OSMutexPost(&MUTEX_SENDSERVER,OS_OPT_POST_NONE,&err);
  /*
  OSSemPend(&SEM_SendOver,
            1000,
            OS_OPT_PEND_BLOCKING,
            &ts,
            &err);
  
  if(err != OS_ERR_NONE){
    return ERROR;
  }*/
  return SUCCESS;
}



ErrorStatus connect(void){
    if(ate_() == ERROR){
        return ERROR;
    }
    if(at_cpin() == ERROR){
        return ERROR;
    }
    if(at_csq() == ERROR){
        return ERROR;
    }
    if(at_creg() == ERROR){
        return ERROR;
    }
    
    if(check_cgatt() == ERROR){
        return ERROR;
    }
	
    if(at_cipmux() == ERROR){
        return ERROR;
    }
    
    //apn
    if(at_apn() == ERROR){
        return ERROR;
    }
    if(at_xiic() == ERROR){
        return ERROR;
    }
    if(check_xiic() == ERROR){
        return ERROR;
    }
    
    if(at_tcpsetup() == ERROR){
        return ERROR;
    }
    
    connectstate = 1;
    
    return SUCCESS;
}


ErrorStatus Device_Cmd(FunctionalState NewState){
  OS_ERR err;
  CPU_TS ts;
  uint8_t cnt = 0;
  
  if(NewState != DISABLE){
    //enable the power
    GPIO_SetBits(GPIOA,GPIO_Pin_1); 
    //low the on_off
    GPIO_ResetBits(GPIOA,GPIO_Pin_4);
    OSTimeDlyHMSM(0,0,1,200,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
    //high the on_off
    GPIO_SetBits(GPIOA,GPIO_Pin_4);
    
    OSTimeDlyHMSM(0,0,3,0,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
    
    while(cnt < 250){
      
      if(ate_() == SUCCESS){
        return SUCCESS;
      }
      
      OSTimeDlyHMSM(0,0,0,100,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
      cnt++;
    }
    
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

//改变在线的状态
void change_connect(uint8_t state){
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  connectstate = state;
  CPU_CRITICAL_EXIT();
}  