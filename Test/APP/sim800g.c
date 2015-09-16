

#include "gprs.h"
#include "bsp.h"

#include "atom.h"
#include "atomsem.h"
#include "atommem.h"
#include "atommutex.h"
#include "atomqueue.h"
#include "string.h"
#include "stdlib.h"
#include "frame.h"
#include "function.h"

/**/
//#define _AT_DNS


u8 *ats[20]={
	"AT\r",
	"ATE0\r",   //关闭回显
	"AT+CPIN?\r",  //获取SIM卡标识
	"AT+CSQ\r",   //查询信号强度
	"AT+CREG?\r",  //检查网络注册状态
	"AT+CGATT?\r",  //检查GPRS附着状态
	"AT+CIPMUX=1\r",  //设置成多链路模式
	"AT+CSTT=\"CMNET\"\r",  //设置APN
	"AT+CIICR\r",   //建立PPP连接
	"AT+CIFSR\r",    //获取本地IP地址
	"AT+CIPSTART=0,\"TCP\",",   //+ip+port 建立TCP连接
	"AT+CIPSEND=0,",    //在链路0 上发送数据
};

uint8_t dns[25] = "\"www.xcxdtech.com\"";     //the server 
uint8_t ip[17] = "218.28.41.74";                 //the server ip
uint8_t port[10] = ",\"3333\"\r";                     //the server port
uint8_t deviceaddr[5] = {0x99,0x09,0x00,0x00,0x57}; //设备地址 standard
uint8_t telnum[12] = "15700000999";		//the addr of the gprs

extern ATOM_SEM sem_send;
extern ATOM_SEM sem_ack;
extern ATOM_SEM sem_sendresult;

extern ATOM_QUEUE server_q;  //receive the data with in \r\n**data**\r\n

extern ATOM_MEM mem_u1;


extern u8 Online;
extern u8 meterdata[600];
extern u8 send_server_result; //根据返回的SEND OK SEND FAIL 设置
extern u8 local_seq;  //本地的序列号


ErrorStatus ate_(void){
	u8 *rcv_ptr = 0;
    u8 err = 0;
    u8 ret = ERROR;
    
    rcv_ptr = atomMemGet(&mem_u1,&err);
    memset(rcv_ptr,0x00,100);
    u1_send(ats[1],strlen(ats[1]));
    u1putbuf(rcv_ptr);
    atomTimerDelay(100);
    u1_check();
    if(strstr((u8 *)rcv_ptr,"OK") > 0){
      ret = SUCCESS;
    }
    u1putbuf(0);
    
    atomMemPut(&mem_u1,rcv_ptr,&err);
    return ret;
}

ErrorStatus at(void){
	u8 *rcv_ptr = 0;
    u8 err = 0;
    u8 ret = ERROR;
    
    rcv_ptr = atomMemGet(&mem_u1,&err);
    memset(rcv_ptr,0x00,100);
    u1_send(ats[0],strlen(ats[0]));
    u1putbuf(rcv_ptr);
    atomTimerDelay(100);
    u1_check();
    if(strstr((u8 *)rcv_ptr,"OK") > 0){
      ret = SUCCESS;
    }
    u1putbuf(0);
    
    atomMemPut(&mem_u1,rcv_ptr,&err);
    return ret;    
}


/*
检查SIM卡状态
*/
ErrorStatus at_cpin(void){
	u8 i = 0;
    u8 *rcv_ptr = 0;
    u8 err = 0;
    u8 ret = ERROR;
    rcv_ptr = atomMemGet(&mem_u1,&err);
    
    for(i = 0;i < 100;i++){
      
      memset(rcv_ptr,0x00,100);
    
      u1_send(ats[2],strlen(ats[2]));
      u1putbuf(rcv_ptr);
      atomTimerDelay(100);
      u1_check();
	  //the content +CPIN: READY
      if(strstr((u8 *)rcv_ptr,"READY") > 0){
        ret = SUCCESS;
        break;
      }
      u1putbuf(0);
      atomTimerDelay(300);
      
    }
    atomMemPut(&mem_u1,rcv_ptr,&err);
    return ret;
    
    
}


ErrorStatus at_csq(void){	
	u8 i = 0;
    u8 good = 0;
    u8 *rcv_ptr = 0;
    u8 err = 0;
    u8 ret = ERROR;
    u8 *rssi_ptr = 0;
    u8 rssi = 0;
    u8 *ber_ptr = 0;
    u8 ber = 0;
    
    rcv_ptr = atomMemGet(&mem_u1,&err);
    
    for(i = 0;i < 200;i++){
      memset(rcv_ptr,0x00,100);
    
      u1_send(ats[3],strlen(ats[3]));
      u1putbuf(rcv_ptr);
      atomTimerDelay(100);
      u1_check();
      if(strstr(rcv_ptr,"CSQ") > 0){
        rssi_ptr = strchr(rcv_ptr,':')+2;
        rssi = atoi(rssi_ptr);
        ber_ptr = strchr(rcv_ptr,',')+1;
        ber = atoi(ber_ptr);
        //if(rssi > 5 && ber != 99){
        if(rssi > 5){
          good++;
          if(good > 6){
            ret = SUCCESS;
            break;
          }
        }else{
          good = 0;
        }
      }else{
        good = 0;
      }
      u1putbuf(0);
      atomTimerDelay(300);
    }
    atomMemPut(&mem_u1,rcv_ptr,&err);
    return ret;
}


ErrorStatus at_creg(void){
	u8 i = 0;
    u8 *rcv_ptr = 0;
    u8 err = 0;
    u8 ret = ERROR;
    
    rcv_ptr = atomMemGet(&mem_u1,&err);
    for(i = 0;i < 100;i++){
      
      memset(rcv_ptr,0x00,100);
    
      u1_send(ats[4],strlen(ats[4]));
      u1putbuf(rcv_ptr);
      atomTimerDelay(100);
      u1_check();
      if(strstr(rcv_ptr,"+CREG: 0,1") > 0 || strstr(rcv_ptr,"+CREG: 0,5") > 0){
        ret = SUCCESS;
        break;
      }
      u1putbuf(0);
      atomTimerDelay(300);
      
    }
    
    atomMemPut(&mem_u1,rcv_ptr,&err);
    return ret;
    
}


ErrorStatus check_cgatt(void){
	u8 i = 0;
    u8 *rcv_ptr = 0;
    u8 err = 0;
    u8 ret = ERROR;
    rcv_ptr = atomMemGet(&mem_u1,&err);
    for(i = 0;i < 100;i++){
      memset(rcv_ptr,0x00,100);
    
      u1_send(ats[5],strlen(ats[5]));
      u1putbuf(rcv_ptr);
      atomTimerDelay(100);
      u1_check();
      if(strstr(rcv_ptr,"+CGATT: 1") > 0){
        ret = SUCCESS;
        break;
      }
      u1putbuf(0);
      atomTimerDelay(300);
      
    }
    atomMemPut(&mem_u1,rcv_ptr,&err);
    return ret;
}

//多链路模式
ErrorStatus at_cipmux(void){
    u8 *rcv_ptr = 0;
    u8 err = 0;
    u8 ret = ERROR;
    
    rcv_ptr = atomMemGet(&mem_u1,&err);
    memset(rcv_ptr,0x00,100);
    
    u1_send(ats[6],strlen(ats[6]));
    u1putbuf(rcv_ptr);
    atomTimerDelay(100);
    u1_check();
    if(strstr(rcv_ptr,"OK") > 0){
      ret = SUCCESS;
    }
    u1putbuf(0);
    
    atomMemPut(&mem_u1,rcv_ptr,&err);
    return ret;
    
}

ErrorStatus at_apn(void){
    u8 ret = ERROR;
    u8 *rcv_ptr = 0;
    u8 err = 0;
    
    rcv_ptr = atomMemGet(&mem_u1,&err);
    memset(rcv_ptr,0x00,100);
    
    u1_send(ats[7],strlen(ats[7]));
    u1putbuf(rcv_ptr);
    atomTimerDelay(100);
    u1_check();
    if(strstr(rcv_ptr,"OK") > 0){
      ret = SUCCESS;
    }
    u1putbuf(0);
    
    atomMemPut(&mem_u1,rcv_ptr,&err);
    return ret;
}



ErrorStatus at_xiic(void){
	u8 i = 0;
    u8 *rcv_ptr = 0;
    u8 err = 0;
    u8 ret = ERROR;
    rcv_ptr = atomMemGet(&mem_u1,&err);
    for(i = 0;i < 100;i++){
      
      memset(rcv_ptr,0x00,100);
    
      u1_send(ats[8],strlen(ats[8]));
      u1putbuf(rcv_ptr);
      atomTimerDelay(2000);
      u1_check();
      if(strstr(rcv_ptr,"OK") > 0){
        ret = SUCCESS;
        break;
      }
      u1putbuf(0);
      atomTimerDelay(300);
      
    }
    atomMemPut(&mem_u1,rcv_ptr,&err);
    return ret;
}


ErrorStatus check_xiic(void){
	u8 i = 0;
    u8 *rcv_ptr = 0;
    u8 err = 0;
    u8 ret = ERROR;
    u8 *ip1_ptr = 0;
    u8 ip1 = 0;
    rcv_ptr = atomMemGet(&mem_u1,&err);
    for(i = 0;i < 100;i++){
      
      memset(rcv_ptr,0x00,100);
    
      u1_send(ats[9],strlen(ats[9]));
      u1putbuf(rcv_ptr);
      atomTimerDelay(100);
      u1_check();
      if(strstr((u8 *)rcv_ptr,".") > 0){
        ret = SUCCESS;
        break;
      }
      u1putbuf(0);
      atomTimerDelay(300);
      
    }
    atomMemPut(&mem_u1,rcv_ptr,&err);
    return ret;
}

ErrorStatus at_tcpsetup(void){
	u8 *rcv_ptr = 0;
    u8 err = 0;
    u8 ret = ERROR;
    
    rcv_ptr = atomMemGet(&mem_u1,&err);
    memset(rcv_ptr,0x00,100);
    
    //for(i = 0;i < 20;i++){
    u1_send(ats[10],strlen(ats[10]));
    u1_send(dns,strlen(dns));
    u1_send(port,strlen(port));
    
    u1putbuf(rcv_ptr);
    atomTimerDelay(3000);
    u1_check();
    if(strstr(rcv_ptr,"CONNECT OK") > 0){
      ret = SUCCESS;
    }
    u1putbuf(0);
    
    atomMemPut(&mem_u1,rcv_ptr,&err);
    return ret;
    
}

/*
the send is the pointer to the data 
the count is the number of the data
the '\r' will send after the data is all send

after this function tim3_timing(500) to receive the data from the m590e(+TCPSEND:0)
*/
extern ATOM_MUTEX sendserver_mutex;
ErrorStatus send_server(uint8_t * send,uint16_t count){
    u8 at_len = 0;
    u8 sendcount[5];
    u8 ret = 0;
    u8 sem_ret = 0;
	
    atomMutexGet(&sendserver_mutex, 0);
	
    memset(sendcount,0,5);
    sprintf(sendcount,"%d",count);
    strcat(sendcount,"\r");
    at_len = strlen(sendcount);
    
    u1_send(ats[11],strlen(ats[11]));
    u1_send(sendcount,at_len);
    
    sem_ret = atomSemGet(&sem_send,1000);
    if(sem_ret == ATOM_OK){
      u1_send(send,count);
    }else{
      _asm("NOP");
    }
	
    sem_ret = atomSemGet(&sem_sendresult,3000);
    if(sem_ret == ATOM_OK){
      if(send_server_result == 1){
        ret = 1;
      }
    }else{
      _asm("NOP");
    }
    atomMutexPut(&sendserver_mutex);
    if(ret){
      return SUCCESS;
    }
    return ERROR;
}



ErrorStatus connectserver(void){
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
    
    Online = 1;
    
    return SUCCESS;
}


void Device_Cmd(FunctionalState NewState){
    u8 cnt = 0;
    u8 simstart = 0;
    if(NewState != DISABLE){
        
        //enable the power
        GPIO_WriteHigh(GPIOB,GPIO_PIN_0);   //give m590e the power
        
        //low the on_off
        GPIO_WriteHigh(GPIOE,GPIO_PIN_7);
        atomTimerDelay(1200);
        //high the on_off
        GPIO_WriteLow(GPIOE,GPIO_PIN_7);
        
        atomTimerDelay(3000);
        
        while(cnt < 100){
          if(ate_() == SUCCESS){
            simstart = 1;
            break;
          }
          cnt++;
          atomTimerDelay(300);
        }
        
        if(simstart == 0){
          _asm("NOP");
        }
        
    }else{
        
        //模块关机
        /*
        //low the on_off
        GPIO_WriteHigh(GPIOE,GPIO_PIN_7);
        atomTimerDelay(2000);
        //high the on_off
        GPIO_WriteLow(GPIOE,GPIO_PIN_7);
        */
        
        //disalbe the power
        GPIO_WriteLow(GPIOB,GPIO_PIN_0);
        atomTimerDelay(3000);
    }
}

/*
void Device_Conf(void){
    //get the telnum
    //get the cjqormeter
    //get the apn
    //get the username
    //get the password
    //get the dns
    
    if(test == 1){
        strcpy(dns,TEST_DNS);
    }
    
    //get the ip
    //get the port
    if(*(u8 *)GPRS_ADDR != 0x00){
        memcpy_((u8 *)GPRS_ADDR,telnum,11);
    }
    memcpy_((u8 *)SLAVE_TYPE_ADDR,&slave,1);
    
}

*/
