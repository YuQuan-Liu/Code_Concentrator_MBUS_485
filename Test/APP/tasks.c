

#include "os.h"
#include "stm32f10x_conf.h"
#include "tasks.h"
#include "lib_str.h"
#include "serial.h"
#include "spi_flash.h"
#include "gprs.h"
#include "frame.h"
#include "frame_188.h"
#include "readeg.h"
//#include "stdlib.h"

extern OS_MEM MEM_Buf;
extern OS_MEM MEM_ISR;

extern OS_Q Q_Slave;            //采集器、表发送过来的数据
extern OS_Q Q_Read;            //抄表任务Queue
extern OS_Q Q_ReadData;        //发送抄表指令后  下层返回抄表数据
extern OS_Q Q_Config;         //配置任务Queue
extern OS_Q Q_Deal;         //处理接收到的服务器发送过来的数据

extern OS_SEM SEM_HeartBeat;    //接收服务器数据Task to HeartBeat Task  接收到心跳的回应
extern OS_SEM SEM_ACKData;     //服务器对数据的ACK
extern OS_SEM SEM_SendOver;      //got the "+TCPSEND:0,"  the data is send over now  发送数据完成
extern OS_SEM SEM_Send;      //got the '>'  we can send the data now  可以发送数据

extern OS_TMR TMR_CJQTIMEOUT;    //打开采集器之后 20分钟超时 自动关闭通道

extern volatile uint8_t reading;
extern volatile uint8_t connectstate; 
extern uint8_t deviceaddr[5];

extern uint8_t slave_mbus; //0xaa mbus   0xff  485   0xBB~采集器

extern uint8_t config_flash[];  //配置处理Flash使用的数组  Sector==4K  需要一个4K的数组
extern uint8_t *meterdata;  //使用海大协议抄表时存放返回的信息  使用config_flash
extern OS_MUTEX MUTEX_CONFIGFLASH;    //是否可以使用 config_flash  4K 数组配置FLASH
extern uint8_t di_seq; //DI0 DI1 顺序   0xAA~DI1在前(千宝通)   0xFF~DI0在前(default)  
extern uint8_t ack_action;  //先应答后操作~0xaa    先操作后应答~0xff
extern uint8_t protocol;  //协议类型 0xFF~188(Default)  1~EG 

uint8_t heart_seq = 0;  //记录心跳的序列号 等待ack
uint8_t data_seq = 0;  //记录数据的序列号 等待ack

uint8_t local_seq = 0;  //本地序列号
uint8_t server_seq = 0;  //服务器端序列号  抄表时  会同步此序列号

uint8_t fe[4] = {0xFE,0xFE,0xFE,0xFE};  //抄表时前面发送的4个0xFE

uint8_t * volatile buf = 0;   //the buf used put the data in 
uint8_t * volatile buf_;       //keep the buf's ptr  used to release the buf
uint8_t start_slave = 0;


uint8_t readingall = 0;   //是否正在抄全部表
uint8_t readingall_progress = 0;  //正在抄全部表的进度情况

void Task_Slave(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  
  uint8_t frame_len = 0;      //the frame's length
  uint8_t data;         //the data get from the queue
  uint8_t * mem_ptr;    //the ptr get from the queue
  uint16_t msg_size;    //the message's size 
  uint8_t header_count; //count 0x68 L L 0x68
  uint8_t header_ok;    //the header is received ok
  uint16_t len1;
  uint16_t len2;
    
  
  while(DEF_TRUE){
    //收到0x68之后  如果200ms 没有收到数据  就认为超时了
    mem_ptr = OSQPend(&Q_Slave,200,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
    
    if(err == OS_ERR_TIMEOUT){
      if(start_slave == 1){
        buf = buf_;
        start_slave = 0;
        frame_len = 0;
      }
      continue;
    }
    
    
    data = *mem_ptr;
    OSMemPut(&MEM_ISR,mem_ptr,&err);
    
    if(buf == 0){
      buf = OSMemGet(&MEM_Buf,
                     &err);
      if(err == OS_ERR_NONE){
        //get the buf
        buf_ = buf;
        Mem_Set(buf_,0x00,256); //clear the buf
      }else{
        //didn't get the buf
        asm("NOP");
        continue;
      }
      
      
    }
    
    if(reading){
      //it is the frame come from the meter
      switch(protocol){
      case 0xFF:
        //188
        if(start_slave == 0){
          if(data == 0x68){
            *buf++ = data;
            frame_len = 0;
            start_slave = 1;
          }
        }else{
          *buf++ = data;
          if((buf-buf_) == 11){
            frame_len = *(buf_+10)+13;
          }
          if(frame_len > 0 && (buf-buf_) >= frame_len){
            //if it is the end of the frame
            if(*(buf-1) == 0x16){
              //check the frame cs
              if(*(buf-2) == check_cs(buf_,frame_len-2)){
                //the frame is ok;
                OSQPost(&Q_ReadData,
                        buf_,
                        frame_len,
                        OS_OPT_POST_FIFO,
                        &err);
                buf_ = 0;
                buf = 0;
                start_slave = 0;
                frame_len = 0;
              }
            }else{
              buf = buf_;
              start_slave = 0;
              frame_len = 0;
            }
          }
        }
        break;
      case 0x01:
        if(start_slave == 0){
          if(data == 0x0E){
            *buf++ = data;
            frame_len = 0;
            start_slave = 1;
          }
        }else{
          *buf++ = data;
          if((buf-buf_) > 8){
            if(*(buf_+2) == 0x0B){
              //the slave is meter
              //post to the reading_q
              frame_len = 9;
              if(check_eor(buf_,9) == 0x00){
                //the frame is ok;
                OSQPost(&Q_ReadData,
                        buf_,
                        frame_len,
                        OS_OPT_POST_FIFO,
                        &err);
                buf_ = 0;
                buf = 0;
                start_slave = 0;
                frame_len = 0;
              }else{
                buf = buf_;
                start_slave = 0;
                frame_len = 0;
              }
            }
            if(*(buf_+2) == 0x0C){
              //the slave is cjq
              if(buf-buf_ > 9){
                //post to the reading_q
                frame_len = 10;
                if(check_eor(buf_,10) == 0x00){
                  //the frame is ok;
                  OSQPost(&Q_ReadData,
                          buf_,
                          frame_len,
                          OS_OPT_POST_FIFO,
                          &err);
                  buf_ = 0;
                  buf = 0;
                  start_slave = 0;
                  frame_len = 0;
                }else{
                  buf = buf_;
                  start_slave = 0;
                  frame_len = 0;
                }
              }
            }
          }
        }
        break;
      }
      
      
    }else{
      //it is the frame come from programmer
      if(start_slave == 0){
        if(data == 0x68){
          *buf++ = data;
          frame_len = 0;
          header_count = 1;
          header_ok = 0;
          start_slave = 1;
        }
      }else{
        *buf++ = data;
        if(header_ok == 0){
          header_count++;
          if(header_count == 6){
            if(*(buf_+5) ==0x68){
              len1 = *(uint16_t *)(buf_+1);
              len2 = *(uint16_t *)(buf_+3);
              if(len1 == len2){
                header_ok = 1;
                frame_len = len1 >> 2;
                frame_len += 8;
              }else{
                //the frame is error
                start_slave = 0;
                frame_len = 0;
                header_count = 0;
                header_ok = 0;
                buf = buf_;
              }
            }else{
              //the frame is error
              start_slave = 0;
              frame_len = 0;
              header_count = 0;
              header_ok = 0;
              buf = buf_;
            }
          }
        }else{
          if(frame_len > 0 && (buf-buf_) >= frame_len){
            //if it is the end of the frame
            if(*(buf-1) == 0x16){
              //check the frame cs
              if(*(buf-2) == check_cs(buf_+6,frame_len-8)){
                //the frame is ok;
                switch(*(buf_+AFN_POSITION)){
                  case AFN_CONFIG:
                  case AFN_QUERY:
                    *buf = 0x00;//标识这一帧数据是来自485的
                    OSQPost(&Q_Config,
                            buf_,frame_len,
                            OS_OPT_POST_FIFO,
                            &err);
                    break;
                  case AFN_CONTROL:
                  case AFN_CURRENT:
                    *buf = 0x00;//标识这一帧数据是来自485的
                    OSQPost(&Q_Read,
                            buf_,frame_len,
                            OS_OPT_POST_FIFO,
                            &err);
                    break;
                }
                buf_ = 0;
                buf = 0;
                start_slave = 0;
                frame_len = 0;
                header_count = 0;
                header_ok = 0;
              }else{
                buf = buf_;
                start_slave = 0;
                frame_len = 0;
                header_count = 0;
                header_ok = 0;
              }
            }else{
              buf = buf_;
              start_slave = 0;
              frame_len = 0;
              header_count = 0;
              header_ok = 0;
            }
          }
        }
      }
    }
  }
}


uint8_t check_cs(uint8_t * start,uint16_t len){
  uint16_t i;
  uint8_t cs = 0;
  for(i = 0;i < len;i++){
    cs += *(start+i);
  }
  return cs;
}


uint8_t check_eor(uint8_t * start,uint16_t len){
  uint16_t i;
  uint8_t cs = 0;
  for(i = 0;i < len;i++){
    cs ^= *(start+i);
  }
  return cs;
}


uint8_t * volatile server_ptr = 0;      //中断中保存M590E 返回来的数据
uint8_t * volatile server_ptr_ = 0;     //记录中断的开始指针

void Task_Server(void *p_arg){
  OS_ERR err;
  
  uint8_t * buf_server_task = 0;
  uint8_t * buf_server_task_ = 0;
  
  while(DEF_TRUE){
    if(connectstate == 1){
      if(buf_server_task == 0){
        buf_server_task = OSMemGet(&MEM_Buf,
                       &err);
        
        if(err == OS_ERR_NONE){
          //get the buf
          buf_server_task_ = buf_server_task;
          Mem_Set(buf_server_task_,0x00,256); //clear the buf
          Server_Post2Buf(buf_server_task_);
        }else{
          //didn't get the buf
          asm("NOP");
          continue;
        }
      }
      
      if(server_ptr - server_ptr_ > 0){
        OSTimeDly(4,
             OS_OPT_TIME_DLY,
             &err);
        
        buf_server_task = server_ptr;
        check_str(buf_server_task_,buf_server_task);  //屏蔽掉数据前的0x00
        if(Str_Str(buf_server_task_,"\n>")){
          
          buf_server_task = buf_server_task_;
          Mem_Set(buf_server_task_,0x00,256); //clear the buf
          Server_Post2Buf(buf_server_task_);
          
          OSSemPost(&SEM_Send,
                    OS_OPT_POST_1,
                    &err);
          continue;
        }
        
        if(Str_Str(buf_server_task_,"RECEIVE")){
          //oh it's the data 
          OSQPost(&Q_Deal,
                  buf_server_task_,
                  buf_server_task-buf_server_task_,
                  OS_OPT_POST_FIFO,
                  &err);
          buf_server_task = 0;
          Server_Post2Buf(0);
          continue;
        }
        
        //CLOSED
        if(Str_Str(buf_server_task_,"CLOSE")){
          
          buf_server_task = buf_server_task_;
          Mem_Set(buf_server_task_,0x00,256); //clear the buf
          
          buf_server_task = 0;
          buf_server_task_ = 0;
          Server_Post2Buf(0);
          change_connect(0);
          continue;
        }
        //+PDP: DEACT\r\n
        if(Str_Str(buf_server_task_,"DEACT")){
          
          buf_server_task = buf_server_task_;
          Mem_Set(buf_server_task_,0x00,256); //clear the buf
          
          buf_server_task = 0;
          buf_server_task_ = 0;
          Server_Post2Buf(0);
          change_connect(0);
          continue;
        }
        
        if(Str_Str(buf_server_task_,"Error")){
          
          buf_server_task = buf_server_task_;
          Mem_Set(buf_server_task_,0x00,256); //clear the buf
          
          buf_server_task = 0;
          buf_server_task_ = 0;
          Server_Post2Buf(0);
          change_connect(0);
          continue;
        }
        
        //don't know what's that
        buf_server_task = buf_server_task_;
        Mem_Set(buf_server_task_,0x00,256); //clear the buf
        Server_Post2Buf(buf_server_task_);
        
      }else{
        OSTimeDly(4,
             OS_OPT_TIME_DLY,
             &err);
      }
    }else{
      
      if(buf_server_task != 0){
        Mem_Set(buf_server_task_,0x00,256); //clear the buf
        OSMemPut(&MEM_Buf,buf_server_task_,&err);
        
        buf_server_task = 0;
        buf_server_task_ = 0;
        
        Server_Post2Buf(0);
      }
      
      Device_Cmd(DISABLE);
      Device_Cmd(ENABLE);
      connect();
    }
  }
  
}



void Task_DealServer(void *p_arg){
  CPU_TS ts;
  OS_ERR err;
  uint8_t * buf_copy = 0;
  uint8_t * buf_ptr_ = 0;
  uint16_t msg_size = 0;
  
  uint8_t * start = 0;
  uint8_t server_seq_ = 0;
  uint16_t len = 0;
  
  while(DEF_TRUE){
    /*
    \r\n+TCPRECV:0,**,0x68 L L 0x68 C A     \r\n
    */
    buf_ptr_ = OSQPend(&Q_Deal,
            0,
            OS_OPT_PEND_BLOCKING,
            &msg_size,
            &ts,
            &err);
    
    start = Str_Str(buf_ptr_,"\r\n\x68") + 2;
    
    //check the frame
    len = check_frame(start);
    
      if(len){
        //the frame is ok
        switch(*(start+AFN_POSITION)){
        case AFN_ACK:
          //the ack of the server
            
          server_seq_ = *(start+SEQ_POSITION) & 0x0F;  //获得该帧的序列号
          
          if(server_seq_ == heart_seq){
            OSSemPost(&SEM_HeartBeat,
                    OS_OPT_POST_1,
                    &err);
          }else{
            if(server_seq_ == data_seq){
              OSSemPost(&SEM_ACKData,
                        OS_OPT_POST_1,
                        &err);
            }else{
              //抛弃此应答帧
            }
          }
          break;
        case AFN_LINK_TEST:
          //never will come to here
          break;
        case AFN_CONFIG:
        case AFN_QUERY:
          buf_copy = OSMemGet(&MEM_Buf,&err);
          if(buf_copy != 0){
            Mem_Copy(buf_copy,start,len);
            *(buf_copy + len) = 0x01;  //标识这一帧来自服务器
            OSQPost(&Q_Config,buf_copy,len,OS_OPT_POST_1,&err);
          }
          break;
        case AFN_CONTROL:
        case AFN_CURRENT:
          
          server_seq_ = *(start + SEQ_POSITION) & 0x0F;
          if(*(start+FN_POSITION) == 0x05){
            //匹配序列号
            server_seq = server_seq_;
            device_ack(0x01,server_seq_);
          }else{
            if(server_seq != server_seq_){
              //新的抄表指令  ack & read
              buf_copy = OSMemGet(&MEM_Buf,&err);
              if(buf_copy != 0){
                Mem_Copy(buf_copy,start,len);
                *(buf_copy + len) = 0x01;  //标识这一帧来自服务器
              }
              server_seq = server_seq_;
              device_ack(0x01,server_seq_);
              OSQPost(&Q_Read,buf_copy,len,OS_OPT_POST_1,&err);
            }else{
              device_ack(0x01,server_seq_);
            }
          }
          
          
          break;
        case AFN_HISTORY:
          //don't support
          break;
          
        }
      }
    OSMemPut(&MEM_Buf,buf_ptr_,&err);
  }
}

/*
if the frame is ok  return the length of the frame
if the frame is error return 0;
*/
uint8_t check_frame(uint8_t * start){
  uint16_t len1;
  uint16_t len2;
  
  uint8_t * s;
  s = start;
  len1 = *(uint16_t *)(s + 1);
  len2 = *(uint16_t *)(s + 3);
  
  if(len1 == len2){
    len1 = len1 >> 2;
    if(*(s+len1 + 6) == check_cs(s+6,len1) && *(s+len1+7) == 0x16){
      return len1+8;
    }
  }
  return 0;
}

void addSEQ(void){
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  local_seq++;
  local_seq = local_seq & 0x0F;
  CPU_CRITICAL_EXIT();
}

void Task_HeartBeat(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint8_t * buf_frame;
  uint8_t heart_ack = 0;
  uint8_t i;
  uint8_t beat[17];
  
  while(DEF_TRUE){
    buf_frame = beat;
    *buf_frame++ = FRAME_HEAD;
    //buf_frame_16 = (uint16_t *)buf_frame;
    *buf_frame++ = 0x27;//(9 << 2) | 0x03;
    *buf_frame++ = 0x00;
    *buf_frame++ = 0x27;//(9 << 2) | 0x03;
    *buf_frame++ = 0x00;
    //buf_frame = (uint8_t *)buf_frame_16;
    *buf_frame++ = FRAME_HEAD;
    
    *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_START | START_FUN_TEST;
    /**/
    *buf_frame++ = deviceaddr[0];
    *buf_frame++ = deviceaddr[1];
    *buf_frame++ = deviceaddr[2];
    *buf_frame++ = deviceaddr[3];
    *buf_frame++ = deviceaddr[4];
    
    *buf_frame++ = AFN_LINK_TEST;
    *buf_frame++ = ZERO_BYTE |SINGLE | CONFIRM | local_seq;
    heart_seq = local_seq;
    addSEQ();
    *buf_frame++ = FN_HEARTBEAT;
    
    *buf_frame++ = check_cs(beat+6,9);
    *buf_frame++ = FRAME_END;
    if(connectstate){
      for(i = 0;connectstate && i < 3;i++){
        heart_ack = 0;
        if(send_server(beat,17)){
          OSSemPend(&SEM_HeartBeat,
                    5000,
                    OS_OPT_PEND_BLOCKING,
                    &ts,
                    &err);
          if(err == OS_ERR_NONE){
            heart_ack = 1;
            break;
          }
        }
      }
      if(heart_ack){
        OSTimeDly(120000,
                    OS_OPT_TIME_DLY,
                    &err);
      }else{
        change_connect(0);
      }
    }else{
      OSTimeDly(100,
                    OS_OPT_TIME_DLY,
                    &err);
    }
  }
}

void power_cmd(FunctionalState NewState){
  if(NewState != DISABLE){
    //打开电源
    if(slave_mbus == 0xAA){
      mbus_power(ENABLE);
    }else{
      relay_485(ENABLE);
    }
  }else{
    //关闭电源
    if(slave_mbus == 0xAA){
      mbus_power(DISABLE);
    }else{
      relay_485(DISABLE);
    }
  }
}

void Task_Read(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint16_t msg_size;
  uint8_t * buf_frame;
  
  while(DEF_TRUE){
    buf_frame = OSQPend(&Q_Read,
                        0,
                        OS_OPT_PEND_BLOCKING,
                        &msg_size,
                        &ts,
                        &err);
    switch(protocol){
    case 0xFF:
      //188
      switch(*(buf_frame+AFN_POSITION)){
        case AFN_CONTROL:
          power_cmd(ENABLE);
          meter_control(buf_frame,*(buf_frame+msg_size));
          power_cmd(DISABLE);
          break;
        case AFN_CURRENT:
          power_cmd(ENABLE);
          meter_read_188(buf_frame,*(buf_frame+msg_size));
          power_cmd(DISABLE);
          break;
      }
      break;
    case 0x01:
      //EG 
      power_cmd(ENABLE);
      meter_read_eg(buf_frame,*(buf_frame+msg_size));
      power_cmd(DISABLE);
      break;
    }
    
    
    
    OSMemPut(&MEM_Buf,buf_frame,&err);
  }
}

uint8_t mbus_power(FunctionalState NewState){
  /**/
  OS_ERR err;
  if(NewState != DISABLE){
    
    GPIO_SetBits(GPIOA,GPIO_Pin_0);
    OSTimeDly(1500,
                  OS_OPT_TIME_DLY,
                  &err);
  }else{
    GPIO_ResetBits(GPIOA,GPIO_Pin_0);
  }
  return 1;
}

uint8_t relay_485(FunctionalState NewState){
  /**/
  OS_ERR err;
  if(NewState != DISABLE){
   GPIO_SetBits(GPIOB,GPIO_Pin_1);
   OSTimeDly(600,
                  OS_OPT_TIME_DLY,
                  &err);
  }else{
    GPIO_ResetBits(GPIOB,GPIO_Pin_1);
  }
  return 1;
}

uint8_t relay_1(FunctionalState NewState){
  OS_ERR err;
  if(NewState != DISABLE){
   GPIO_SetBits(GPIOA,GPIO_Pin_15);
   OSTimeDly(3000,
                  OS_OPT_TIME_DLY,
                  &err);
  }else{
    GPIO_ResetBits(GPIOA,GPIO_Pin_15);
  }
  return 1;
}
uint8_t relay_2(FunctionalState NewState){
  OS_ERR err;
  if(NewState != DISABLE){
   GPIO_SetBits(GPIOB,GPIO_Pin_3);
   OSTimeDly(3000,
                  OS_OPT_TIME_DLY,
                  &err);
  }else{
    GPIO_ResetBits(GPIOB,GPIO_Pin_3);
  }
  return 1;
}
uint8_t relay_3(FunctionalState NewState){
  OS_ERR err;
  if(NewState != DISABLE){
   GPIO_SetBits(GPIOB,GPIO_Pin_4);
   OSTimeDly(3000,
                  OS_OPT_TIME_DLY,
                  &err);
  }else{
    GPIO_ResetBits(GPIOB,GPIO_Pin_4);
  }
  return 1;
}
uint8_t relay_4(FunctionalState NewState){
  OS_ERR err;
  if(NewState != DISABLE){
   GPIO_SetBits(GPIOB,GPIO_Pin_5);
   OSTimeDly(3000,
                  OS_OPT_TIME_DLY,
                  &err);
  }else{
    GPIO_ResetBits(GPIOB,GPIO_Pin_5);
  }
  return 1;
}

/*
抄海大协议表
*/
void meter_read_eg(uint8_t * buf_frame,uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  //获取config_flash的使用权
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //获取MUTEX过程中 出错了...
    //return 0xFFFFFF;
    return;
  }
  meterdata = config_flash; //将表返回的所有信息存放在config_flash
  
  switch (*(buf_frame + DATA_POSITION)){
    case 0xAA:
      meter_single_eg(buf_frame);
      send_data_eg(1,desc);
    break;
    case 0x00:
      meter_cjq_eg(buf_frame);
      if(meterdata[2] != 0xFF){
        send_data_eg(*(buf_frame + DATA_POSITION + 3),desc);
      }else{
        send_cjqtimeout_eg(desc);
      }
      
    break;
  }
  
  OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
}

/*
如果采集器地址为 FF FF FF FF FF FF 表示集中器下面直接接的表

抄表和配置时：
先发送打开指定采集器透传功能  接收到回应之后
可以发送抄表配置指令
指定时间内没有收到返回的表的指令  重复3次

关掉指定采集器的透传功能 

*/
void meter_read_188(uint8_t * buf_frame,uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  uint32_t block_cjq = 0;   //cjq block 地址
  uint32_t block_meter = 0;  //meter block 地址
  
  uint16_t cjq_count = 0;
  uint16_t cjqmeter_count = 0;
  
  uint16_t i = 0;
  uint16_t j = 0;
  
  uint8_t cjq_addr[6];
  uint8_t meter_addr[7];
  uint8_t meter_type = 0;
  
  uint8_t meter_fount = 0;
  //查询是否有这个表
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
  if(cjq_count == 0){
    //没有采集器。。。
    return;
  }
  
  Device_Read(ENABLE);
  if(Mem_Cmp(buf_frame+16,"\xFF\xFF\xFF\xFF\xFF\xFF\xFF",7) == DEF_YES){
    //抄全部表
    readingall = 1;
    /**/
    for(i = 0;i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      
      if(cjq_open(cjq_addr,block_cjq) == 0){
        //没有打开采集器
        OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
      
        if(err != OS_ERR_NONE){
          //获取MUTEX过程中 出错了...
          //return 0xFFFFFF;
          return;
        }
        for(j=0;j < cjqmeter_count;j++){
          sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
          *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x80;
          
          //将配置好的Flash块重新写入到Flash中。
          sFLASH_EraseSector((block_meter/0x1000)*0x1000);
          sFLASH_WriteBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
          
          sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
        }
        
        OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
      }else{
        for(j=0;j < cjqmeter_count;j++){
          sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
          sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
          
          meter_read_single(meter_addr,block_meter,meter_type,desc);
          sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
        }
        cjq_close(cjq_addr,block_cjq);
      }
      
      readingall_progress = (i+1)/cjq_count;  //抄全部表时主动上报的进度  当前第几个采集器/采集器数量
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+3,3);
    }
    readingall = 0;
    meter_send(1,0,desc);
  }else{
    //抄单个表
    for(i = 0;meter_fount == 0 && i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      for(j=0;j < cjqmeter_count;j++){
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
        
        if(Mem_Cmp(buf_frame + 16,meter_addr,7) == DEF_YES && meter_type == *(buf_frame + 15)){
          //找到这个表了。。。
          if(cjq_open(cjq_addr,block_cjq) == 0){
            //没有打开采集器
            OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
          
            if(err != OS_ERR_NONE){
              //获取MUTEX过程中 出错了...
              //return 0xFFFFFF;
              return;
            }
            sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
            *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x80;
            
            //将配置好的Flash块重新写入到Flash中。
            sFLASH_EraseSector((block_meter/0x1000)*0x1000);
            sFLASH_WriteBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
            
            OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
          }else{
            meter_read_single(meter_addr,block_meter,meter_type,desc);
            //send the data;
            meter_send(0,block_meter,desc);
            cjq_close(cjq_addr,block_cjq);
          }
          
          meter_fount = 1; 
          break;
        }
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
      }
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+3,3);
    }
  }
  Device_Read(DISABLE);
}

//只管读表
void meter_read_single(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc){
    OS_ERR err;
    CPU_TS ts;
    uint8_t buf_frame_[16];
    uint8_t read[4];
    uint8_t half[4];
    uint8_t i = 0;
    uint8_t j = 0;
    uint16_t msg_size = 0;
    uint8_t * buf_readdata = 0;
    uint8_t success = 0;
    uint8_t st_l = 0;
    uint8_t st_h = 0;
    uint8_t * buf_frame = 0;
    
    buf_frame = buf_frame_;
    *buf_frame++ = FRAME_HEAD;
    *buf_frame++ = meter_type;
    for(i=0;i<7;i++){
      *buf_frame++ = *(meter_addr + i);
    }
    *buf_frame++ = 0x01; //C
    *buf_frame++ = 0x03; //len
    if(di_seq == 0xFF){
      //默认低位在前
      *buf_frame++ = DATAFLAG_RD_L;
      *buf_frame++ = DATAFLAG_RD_H;
    }else{
      //千宝通使用的顺序。。。大表使用
      *buf_frame++ = DATAFLAG_RD_H;
      *buf_frame++ = DATAFLAG_RD_L;
    }
    
    *buf_frame++ = 0x01;
    *buf_frame++ = check_cs(buf_frame_,11+3);
    *buf_frame++ = FRAME_END;
    
    for(i =0;i<4;i++){
      read[i] = 0x00;
      half[i] = 0x00;
    }
    
    for(i = 0;success == 0 && i < 3;i++){
      Slave_Write(fe,4);
      Slave_Write(buf_frame_,13+3);
      buf_readdata = OSQPend(&Q_ReadData,1200,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
      if(err != OS_ERR_NONE){
        if(i==2){
          //读表失败  存flash  
          success = 0;
        }
        continue;
      }
      //接收到正确的数据
      //判断表地址
      if(Mem_Cmp(meter_addr,buf_readdata+2,7) == DEF_YES){
        success = 1;
        //获取ST
        st_l = *(buf_readdata + 31);
        st_h = *(buf_readdata + 32);
        for(j = 0;j < 4;j++){
          read[j] = *(buf_readdata + 14 + j);
          half[j] = *(buf_readdata + 19 + j);
        }
      }
      
      OSMemPut(&MEM_Buf,buf_readdata,&err);
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  
    if(err != OS_ERR_NONE){
      //获取MUTEX过程中 出错了...
      //return 0xFFFFFF;
      return;
    }
    
    sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
    if(success == 0){
      //读表失败  存flash  return nack
      *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x40;
    }else{
      Mem_Copy(config_flash+block_meter%0x1000 + 22,&st_l,1);  //记录st信息
      Mem_Copy(config_flash+block_meter%0x1000 + 23,&st_h,1);  //记录st信息
      Mem_Copy(config_flash+block_meter%0x1000 + 14,read,4);        //读数
      Mem_Copy(config_flash+block_meter%0x1000 + 24,half,4);        //半位
    }
    
    //将配置好的Flash块重新写入到Flash中。
    sFLASH_EraseSector((block_meter/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
}

//all = 1 发送全部表  all = 0 发送表块对应的表
void meter_send(uint8_t all,uint32_t block_meter_,uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  uint8_t * buf_frame = 0;
  uint8_t * buf_frame_ = 0;
  uint16_t * buf_frame_16 = 0;
  uint16_t cjqmeter_count = 0;
  uint16_t allmeter_count = 0;
  uint16_t cjq_count = 0;
  
  uint16_t times = 0;
  uint8_t remain = 0;
  uint16_t times_ = 0;      //一共要发送多少帧
  uint16_t times_count = 0; //发送了多少帧了
  
  
  uint8_t meter_addr[7];
  uint8_t meter_read[4];
  uint32_t block_cjq = 0;
  uint32_t block_meter = 0;
  
  uint8_t meter_type = 0;   //表的类型
  uint8_t st_l = 0; //表的状态
  uint8_t st_h = 0; //表的状态
  
  uint16_t i = 0;       //计数采集器
  uint16_t j = 0;       //计数采集器下的表
  uint8_t k = 0;        //计数copy采集器、表地址
  
  uint16_t meter_count = 0;  //计数一帧中的数据体个数
  uint16_t meter_count_ = 0;    //保持一帧中数据体的个数
  uint8_t header = 0;   //一帧的帧头是否已准备
  
  uint16_t len = 0; 
  
  block_meter = block_meter_;
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  
  buf_frame_ = buf_frame;
  
  if(all == 0){
    sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
    sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
    sFLASH_ReadBuffer((uint8_t *)&meter_read,block_meter+14,4);
    sFLASH_ReadBuffer((uint8_t *)&st_l,block_meter+22,1);
    sFLASH_ReadBuffer((uint8_t *)&st_h,block_meter+23,1);
    
    *buf_frame++ = FRAME_HEAD;
    buf_frame_16 = (uint16_t *)buf_frame;
    *buf_frame_16++ = 0x73;//((9+14+5) << 2) | 0x03;
    *buf_frame_16++ = 0x73;//((9+14+5) << 2) | 0x03;
    buf_frame = (uint8_t *)buf_frame_16;
    *buf_frame++ = FRAME_HEAD;
    
    *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    *buf_frame++ = deviceaddr[0];
    *buf_frame++ = deviceaddr[1];
    *buf_frame++ = deviceaddr[2];
    *buf_frame++ = deviceaddr[3];
    *buf_frame++ = deviceaddr[4];
    
    *buf_frame++ = AFN_CURRENT;
    *buf_frame++ = ZERO_BYTE |SINGLE | local_seq;
    *buf_frame++ = FN_CURRENT_METER;
    
    data_seq = local_seq;
    addSEQ();
    
    *buf_frame++ = 0x00;
    *buf_frame++ = 0x00;
    *buf_frame++ = 0x00;
    *buf_frame++ = 0x00;
    *buf_frame++ = meter_type;
    
    
    for(i=0;i<7;i++){
      *buf_frame++ = meter_addr[i];
    }
    *buf_frame++ = 0x01;
    for(i=0;i<4;i++){
      *buf_frame++ = meter_read[i];
    }
    *buf_frame++ = st_l;
    *buf_frame++ = st_h;
    
    *buf_frame++ = check_cs(buf_frame_+6,28);
    *buf_frame++ = FRAME_END;
    
    if(desc){
      //to m590e
      
      
      for(k = 0;k < 3;k++){
        send_server(buf_frame_,36);
        OSSemPend(&SEM_ACKData,
                  5000,
                  OS_OPT_PEND_BLOCKING,
                  &ts,
                  &err);
        if(err == OS_ERR_NONE){
          break;
        }
      }
    }else{
      //to 485
      Server_Write_485(buf_frame_,36);
    }
  }else{
    //全部表
    sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
    sFLASH_ReadBuffer((uint8_t *)&allmeter_count,sFLASH_METER_COUNT,2);
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
    
    times = allmeter_count/10;
    remain = allmeter_count%10;
    times_count = 0;
    times_ = times;
    if(remain > 0){
      times_ = times_ + 1;
    }
    
    for(i = 0;i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      for(j=0;j < cjqmeter_count;j++){
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
        sFLASH_ReadBuffer((uint8_t *)&meter_read,block_meter+14,4);
        sFLASH_ReadBuffer((uint8_t *)&st_l,block_meter+22,1);
        sFLASH_ReadBuffer((uint8_t *)&st_h,block_meter+23,1);
        
        if(header == 0){
          header = 1;
          times_count++;
          if(times_ == 1){
            //单帧
            meter_count = allmeter_count;
            meter_count_ = meter_count;
            
            len = ((9+14*meter_count_+5) << 2) | 0x03;    //+1  because of  *buf_frame++ = metertype;
            
          }else{
            //多帧
            if(times_count == 1){
              //首帧
              meter_count = 10;
              meter_count_ = meter_count;
              
              len = ((9+14*meter_count_+5) << 2) | 0x03;
            }else{
              if(times_count == times_){
                //尾帧
                if(remain == 0){
                  meter_count = 10; 
                  meter_count_ = meter_count;
                }else{
                  meter_count = remain;
                  meter_count_ = meter_count;
                }
                
                len = ((9+14*meter_count_+5) << 2) | 0x03;
              }else{
                //中间帧
                meter_count = 10;
                meter_count_ = meter_count;
                
                len = ((9+14*meter_count_+5) << 2) | 0x03;
              }
            }
          }
          
          *buf_frame++ = FRAME_HEAD;
          buf_frame_16 = (uint16_t *)buf_frame;
          *buf_frame_16++ = len;    //+1  because of  *buf_frame++ = metertype;
          *buf_frame_16++ = len;
          buf_frame = (uint8_t *)buf_frame_16;
          *buf_frame++ = FRAME_HEAD;
              
          *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
          /**/
          *buf_frame++ = deviceaddr[0];
          *buf_frame++ = deviceaddr[1];
          *buf_frame++ = deviceaddr[2];
          *buf_frame++ = deviceaddr[3];
          *buf_frame++ = deviceaddr[4];
          
          *buf_frame++ = AFN_CURRENT;
          if(times_ == 1){
            //单帧
            *buf_frame++ = ZERO_BYTE |SINGLE | CONFIRM| local_seq;
          }else{
            //多帧
            if(times_count == 1){
              //首帧
              *buf_frame++ = ZERO_BYTE |MUL_FIRST | CONFIRM| local_seq;
            }else{
              if(times_count == times_){
                //尾帧
                *buf_frame++ = ZERO_BYTE |MUL_LAST | CONFIRM| local_seq;
              }else{
                //中间帧
                *buf_frame++ = ZERO_BYTE |MUL_MIDDLE | CONFIRM| local_seq;
              }
            }
          }
          data_seq = local_seq;
          addSEQ();
          
          *buf_frame++ = FN_CURRENT_METER;
          
          buf_frame_16 = (uint16_t *)buf_frame;
          *buf_frame_16++ = times_;    //总共多少帧
          *buf_frame_16++ = times_count;  //第几帧
          buf_frame = (uint8_t *)buf_frame_16;
          *buf_frame++ = meter_type;
        }
        
        
        
        for(k=0;k<7;k++){
          *buf_frame++ = meter_addr[k];
        }
        *buf_frame++ = 0x01;
        for(k=0;k<4;k++){
          *buf_frame++ = meter_read[k];
        }
        *buf_frame++ = st_l;
        *buf_frame++ = st_h;
        
        meter_count--;
        if(meter_count == 0){
          header = 0;   //为下一帧做准备
          //发送这一帧
          
          *buf_frame++ = check_cs(buf_frame_+6,9+14*meter_count_+5);
          *buf_frame++ = FRAME_END;
          
          
          if(desc){
            //to m590e
            for(k = 0;k < 3;k++){
              send_server(buf_frame_,17+14*meter_count_+5);
              OSSemPend(&SEM_ACKData,
                        5000,
                        OS_OPT_PEND_BLOCKING,
                        &ts,
                        &err);
              if(err == OS_ERR_NONE){
                break;
              }
            }
          }else{
            //to 485
            Server_Write_485(buf_frame_,17+14*meter_count_+5);
          }
          OSTimeDly(100,
                    OS_OPT_TIME_DLY,
                    &err);
          
          buf_frame = buf_frame_;
        }
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
      }
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+3,3);
    }
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
}

void meter_control(uint8_t * buf_frame,uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  uint32_t block_cjq = 0;   //cjq block 地址
  uint32_t block_meter = 0;  //meter block 地址
  
  uint16_t cjq_count = 0;
  uint16_t cjqmeter_count = 0;
  
  uint16_t i = 0;
  uint16_t j = 0;
  
  uint8_t cjq_addr[6];
  uint8_t meter_addr[7];
  uint8_t meter_type = 0;
  
  uint8_t meter_fount = 0;
  uint8_t server_seq_ = 0;  
  //查询是否有这个表
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
  if(cjq_count == 0){
    //没有采集器。。。
    //todo 回应 NACK
    return;
  }
  
  server_seq_ = *(buf_frame + SEQ_POSITION) & 0x0F;
  if(*(buf_frame + FN_POSITION) == FN_CLEAN){
    //send ack;
    device_ack(desc,server_seq_);
    meter_clean();
  }else{
    Device_Read(ENABLE);
    for(i = 0;meter_fount == 0 && i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      
      for(j=0;j < cjqmeter_count;j++){
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
        
        if(Mem_Cmp(buf_frame + 16,meter_addr,7) == DEF_YES && meter_type == *(buf_frame + 15)){
          //找到这个表了。。。
          
          if(cjq_open(cjq_addr,block_cjq)==0){
            //没有打开采集器
              OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
            
              if(err != OS_ERR_NONE){
                //获取MUTEX过程中 出错了...
                //return 0xFFFFFF;
                return;
              }
              sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
              *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x80;
              
              //将配置好的Flash块重新写入到Flash中。
              sFLASH_EraseSector((block_meter/0x1000)*0x1000);
              sFLASH_WriteBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
              
              OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
          }else{
            switch(*(buf_frame + FN_POSITION)){
            case FN_OPEN:
              meter_open(meter_addr,block_meter,meter_type,desc,server_seq_);
              break;
            case FN_CLOSE:
              meter_close(meter_addr,block_meter,meter_type,desc,server_seq_);
              break;
            }
            
            cjq_close(cjq_addr,block_cjq);
          }
          meter_fount = 1; 
          break;
        }
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
      }
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+3,3);
    }
    Device_Read(DISABLE);
  }
}

uint8_t cjq_isopen = 0;

uint8_t cjq_open(uint8_t * cjq_addr,uint32_t block_cjq){
    uint8_t buf_frame_[17];
    uint8_t * buf_frame = 0;
    uint8_t i;
    uint16_t msg_size;
    uint8_t * buf_readdata;
    uint8_t success = 0;
    uint8_t st_l;
    OS_ERR err;
    CPU_TS ts;
    
    if(slave_mbus == 0xAA){
      //mbus ~~~
      if(cjq_isopen == cjq_addr[0]){
        OSTmrStart(&TMR_CJQTIMEOUT,&err);
      }else{
        relay_1(DISABLE);
        relay_2(DISABLE);
        relay_3(DISABLE);
        relay_4(DISABLE);
        switch (cjq_addr[0]){
          case 1:
            relay_1(ENABLE);
            cjq_isopen = 1;
            break;
          case 2:
            relay_2(ENABLE);
            cjq_isopen = 2;
            break;
          case 3:
            relay_3(ENABLE);
            cjq_isopen = 3;
            break;
          case 4:
            relay_4(ENABLE);
            cjq_isopen = 4;
            break;
        }
        OSTmrStart(&TMR_CJQTIMEOUT,&err);
      }
      return 1;
    }
    
    if(slave_mbus == 0xBB){
      //采集器 ~~~
      buf_frame = buf_frame_;
      *buf_frame++ = FRAME_HEAD;
      *buf_frame++ = 0xA0;  //采集器标志
      for(i=0;i<6;i++){
        *buf_frame++ = cjq_addr[i];
      }
      *buf_frame++ = 0x00;  //采集器最高位
      *buf_frame++ = 0x04; //C
      *buf_frame++ = 0x04; //len
      *buf_frame++ = DATAFLAG_WC_L;
      *buf_frame++ = DATAFLAG_WC_H;
      *buf_frame++ = 0x01;
      *buf_frame++ = OPEN_CJQ;
      *buf_frame++ = check_cs(buf_frame_,11+4);
      *buf_frame++ = FRAME_END;
      
      for(i = 0;success == 0 && i < 2;i++){
        Slave_Write(buf_frame_,13+4);
        buf_readdata = OSQPend(&Q_ReadData,5000,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
        if(err != OS_ERR_NONE){
          if(i==1){
            //开采集器失败  存flash  return nack
            success = 0;
          }
          continue;
        }
        //接收到正确的数据
        //判断表地址
        if(Mem_Cmp(cjq_addr,buf_readdata+2,6) == DEF_YES){
          //获取ST
          st_l = *(buf_readdata + 14);
          if((st_l & 0x03) == 0x00){
            //opened  return ack
            success = 1;
          }else{
            //开采集器失败  存flash  return nack
            success = 0;
          }
        }
        OSMemPut(&MEM_Buf,buf_readdata,&err);
      }
      if(success == 0){
        //开采集器失败  存flash  return nack
        OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
      
        if(err != OS_ERR_NONE){
          //获取MUTEX过程中 出错了...
          //return 0xFFFFFF;
          return success;
        }
        
        sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
        //读表失败  存flash  return nack
        *(config_flash+block_cjq%0x1000 + 23) = 0x01;
        
        //将配置好的Flash块重新写入到Flash中。
        sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        
        OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
      }
      return success;
    }
    return 1; //如果底层直接接了485的188协议的表  直接返回1
}

uint8_t cjq_close(uint8_t * cjq_addr,uint32_t block_cjq){
    uint8_t buf_frame_[17];
    uint8_t * buf_frame = 0;
    uint8_t i;
    uint16_t msg_size;
    uint8_t * buf_readdata;
    uint8_t success = 0;
    uint8_t st_l;
    OS_ERR err;
    CPU_TS ts;
    if(slave_mbus == 0xAA){
      //mbus ~~~
      relay_1(DISABLE);
      relay_2(DISABLE);
      relay_3(DISABLE);
      relay_4(DISABLE);
      cjq_isopen = 0;
      OSTmrStop(&TMR_CJQTIMEOUT,OS_OPT_TMR_NONE,0,&err);
      return 1;
    }
    if(slave_mbus == 0xBB){
      //采集器 ~~~
      buf_frame = buf_frame_;
      *buf_frame++ = FRAME_HEAD;
      *buf_frame++ = 0xA0;  //采集器标志
      for(i=0;i<6;i++){
        *buf_frame++ = cjq_addr[i];
      }
      *buf_frame++ = 0x00;  //采集器最高位
      *buf_frame++ = 0x04; //C
      *buf_frame++ = 0x04; //len
      *buf_frame++ = DATAFLAG_WC_L;
      *buf_frame++ = DATAFLAG_WC_H;
      *buf_frame++ = 0x01;
      *buf_frame++ = CLOSE_CJQ;
      *buf_frame++ = check_cs(buf_frame_,11+4);
      *buf_frame++ = FRAME_END;
      
      for(i = 0;success == 0 && i < 2;i++){
        Slave_Write(buf_frame_,13+4);
        buf_readdata = OSQPend(&Q_ReadData,4000,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
        if(err != OS_ERR_NONE){
          if(i==1){
            //关采集器失败  存flash  return nack
            success = 0;
          }
          continue;
        }
        //接收到正确的数据
        //判断表地址
        if(Mem_Cmp(cjq_addr,buf_readdata+2,6) == DEF_YES){
          //获取ST
          st_l = *(buf_readdata + 14);
          if((st_l & 0x03) == 0x02){
            //closed  return ack
            success = 1;
          }else{
            //开采集器失败  存flash  return nack
            success = 0;
          }
        }
        OSMemPut(&MEM_Buf,buf_readdata,&err);
      }
    }
    return success;
}

void cjq_timeout(void *p_tmr,void *p_arg){
  OS_ERR err;
  //关闭电源
  power_cmd(DISABLE);
  //关闭通道
  //cjq_close();
  relay_1(DISABLE);
  relay_2(DISABLE);
  relay_3(DISABLE);
  relay_4(DISABLE);
  cjq_isopen = 0;
  OSTmrStop(&TMR_CJQTIMEOUT,OS_OPT_TMR_NONE,0,&err);
}


void meter_open(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc,uint8_t server_seq_){
    OS_ERR err;
    CPU_TS ts;
    uint8_t buf_frame_[17];
    uint8_t i = 0;
    uint16_t msg_size = 0;
    uint8_t * buf_readdata = 0;
    uint8_t success = 0;
    uint8_t st_l = 0;
    uint8_t st_h = 0;
    uint8_t * buf_frame = 0;
    
    buf_frame = buf_frame_;
    *buf_frame++ = FRAME_HEAD;
    *buf_frame++ = meter_type;
    for(i=0;i<7;i++){
      *buf_frame++ = *(meter_addr + i);
    }
    *buf_frame++ = 0x04; //C
    *buf_frame++ = 0x04; //len
    
    if(di_seq == 0xFF){
      //默认低位在前
      *buf_frame++ = DATAFLAG_WV_L;
      *buf_frame++ = DATAFLAG_WV_H;
    }else{
      // 骏普阀控表使用  有点
      *buf_frame++ = DATAFLAG_WV_H;
      *buf_frame++ = DATAFLAG_WV_L;
    }
    
    *buf_frame++ = 0x01;
    *buf_frame++ = OPEN_VALVE;
    *buf_frame++ = check_cs(buf_frame_,11+4);
    *buf_frame++ = FRAME_END;
    
    for(i = 0;success == 0 && i < 2;i++){
      Slave_Write(fe,4);
      Slave_Write(buf_frame_,13+4);
      buf_readdata = OSQPend(&Q_ReadData,15000,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
      if(err != OS_ERR_NONE){
        if(i==1){
          //开阀失败  存flash  return nack
          success = 0;
        }
        continue;
      }
      //接收到正确的数据
      if(ack_action != 0xff){
        OSMemPut(&MEM_Buf,buf_readdata,&err);
        OSTimeDly(12000,
                  OS_OPT_TIME_DLY,
                  &err);
        success = 1;
      }else{
        //判断表地址
        if(Mem_Cmp(meter_addr,buf_readdata+2,7) == DEF_YES){
          //获取ST
          st_l = *(buf_readdata + 14);
          st_h = *(buf_readdata + 15);
          if((st_l & 0x03) == 0x00){
            //opened  return ack
            success = 1;
          }else{
            //开阀失败  存flash  return nack
            success = 0;
          }
        }
        
        OSMemPut(&MEM_Buf,buf_readdata,&err);
      }
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  
    if(err != OS_ERR_NONE){
      //获取MUTEX过程中 出错了...
      //return 0xFFFFFF;
      return;
    }
    sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
    if(success == 0){
      //开阀失败  存flash  return nack
      //Mem_Copy(configflash + 22,"\x43",1);  //超时
      if((st_l & 0x03) == 0x03){
        //阀门返回异常
        *(config_flash+block_meter%0x1000 + 22) = st_l;
        *(config_flash+block_meter%0x1000 + 23) = st_h;
      }else{
        //超时
        *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x40;
      }
      //device_nack(desc,server_seq_);
    }else{
      //开
      if(ack_action != 0xff){
        *(config_flash+block_meter%0x1000 + 22) = 0x00;
        *(config_flash+block_meter%0x1000 + 23) = 0x00;
      }else{
        *(config_flash+block_meter%0x1000 + 22) = st_l;
        *(config_flash+block_meter%0x1000 + 23) = st_h;
      }
      device_ack(desc,server_seq_);   //return nack;
    }
    
    //将配置好的Flash块重新写入到Flash中。
    sFLASH_EraseSector((block_meter/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    
}

void meter_close(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc,uint8_t server_seq_){
    OS_ERR err;
    CPU_TS ts;
    uint8_t buf_frame_[17];
    uint8_t i = 0;
    uint16_t msg_size = 0;
    uint8_t * buf_readdata = 0;
    uint8_t success = 0;
    uint8_t st_l = 0;
    uint8_t st_h = 0;
    uint8_t * buf_frame = 0;  
  
    buf_frame = buf_frame_;
    *buf_frame++ = FRAME_HEAD;
    *buf_frame++ = meter_type;
    for(i=0;i<7;i++){
      *buf_frame++ = meter_addr[i];
    }
    *buf_frame++ = 0x04; //C
    *buf_frame++ = 0x04; //len
    
    if(di_seq == 0xFF){
      //默认低位在前
      *buf_frame++ = DATAFLAG_WV_L;
      *buf_frame++ = DATAFLAG_WV_H;
    }else{
      // 骏普阀控表使用  有点
      *buf_frame++ = DATAFLAG_WV_H;
      *buf_frame++ = DATAFLAG_WV_L;
    }
    
    *buf_frame++ = 0x01;
    *buf_frame++ = CLOSE_VALVE;
    *buf_frame++ = check_cs(buf_frame_,11+4);
    *buf_frame++ = FRAME_END;
    
    for(i = 0;success == 0 && i < 2;i++){
      Slave_Write(fe,4);
      Slave_Write(buf_frame_,13+4);
      buf_readdata = OSQPend(&Q_ReadData,15000,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
      if(err != OS_ERR_NONE){
        if(i==1){
          //开阀失败  存flash  return nack
          success = 0;
        }
        continue;
      }
      //接收到正确的数据
      if(ack_action != 0xff){
        OSMemPut(&MEM_Buf,buf_readdata,&err);
        OSTimeDly(12000,
                  OS_OPT_TIME_DLY,
                  &err);
        success = 1;
      }else{
        //判断表地址
        if(Mem_Cmp(meter_addr,buf_readdata+2,7) == DEF_YES){
          //获取ST
          st_l = *(buf_readdata + 14);
          st_h = *(buf_readdata + 15);
          //对于关阀在ST中的理解   D0 D1中只要有一个为1即关  
          //me：协议中指的是D1 = 1
          //骏普：D0 = 1 为关
          if((st_l & 0x03) == 0x03){
            //开阀失败  存flash  return nack
            success = 0;
          }else{
            if((st_l & 0x02) == 0x02 || (st_l & 0x01) == 0x01){
              //opened  return ack
              success = 1;
            }else{
              //开阀失败  存flash  return nack
              success = 0;
            }
          }
        }
        OSMemPut(&MEM_Buf,buf_readdata,&err);
      }
    }
    
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  
    if(err != OS_ERR_NONE){
      //获取MUTEX过程中 出错了...
      //return 0xFFFFFF;
      return;
    }
    sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
    
    if(success == 0){
      //开阀失败  存flash  return nack
      //Mem_Copy(configflash + 22,"\x43",1);  //超时
      if((st_l & 0x03) == 0x03){
        //开阀失败  存flash  return nack
        //阀门坏
        *(config_flash+block_meter%0x1000 + 22) = st_l;
        *(config_flash+block_meter%0x1000 + 23) = st_h;
      }else{
        //超时
        *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x40;
      }
      //device_nack(desc,server_seq_);  //return nack;
    }else{
      //关阀
      if(ack_action != 0xff){
        *(config_flash+block_meter%0x1000 + 22) = 0x01;
        *(config_flash+block_meter%0x1000 + 23) = 0x00;
      }else{
        *(config_flash+block_meter%0x1000 + 22) = st_l;
        *(config_flash+block_meter%0x1000 + 23) = st_h;
      }
      device_ack(desc,server_seq_);   //return ack;
    }
    
    //将配置好的Flash块重新写入到Flash中。
    sFLASH_EraseSector((block_meter/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
}

/*
首先读表操作（从Flash中提取）  获取阀门当前状态

*/
void meter_clean(void){
    
  OS_ERR err;
  uint32_t block_cjq = 0;   //cjq block 地址
  uint32_t block_meter = 0;  //meter block 地址
  
  uint16_t cjq_count = 0;
  uint16_t cjqmeter_count = 0;
  
  uint16_t i = 0;
  uint16_t j = 0;
  
  uint8_t cjq_addr[6];
  uint8_t meter_addr[7];
  uint8_t meter_type = 0;
  
  uint8_t buf_frame_[17];
  uint8_t z = 0;
  uint8_t * buf_frame = 0; 
  
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
  if(cjq_count == 0){
    //没有采集器。。。
    return;
  }
  
  Device_Read(ENABLE);
  for(i = 0;i < cjq_count;i++){
    sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
    sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
    sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
    
    if(cjq_open(cjq_addr,block_cjq) == 0){
      //没有打开采集器
      //do nothing;
    }else{
      for(j=0;j < cjqmeter_count;j++){
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
        
        //meter_read_single(meter_addr,block_meter,meter_type,desc);
        buf_frame = buf_frame_;
        *buf_frame++ = FRAME_HEAD;
        *buf_frame++ = meter_type;
        for(z=0;z<7;z++){
          *buf_frame++ = meter_addr[z];
        }
        *buf_frame++ = 0x04; //C
        *buf_frame++ = 0x04; //len
        
        if(di_seq == 0xFF){
          //默认低位在前
          *buf_frame++ = DATAFLAG_WV_L;
          *buf_frame++ = DATAFLAG_WV_H;
        }else{
          // 骏普阀控表使用  有点
          *buf_frame++ = DATAFLAG_WV_H;
          *buf_frame++ = DATAFLAG_WV_L;
        }
        
        *buf_frame++ = 0x01;
        *buf_frame++ = CLEAN_VALVE;
        *buf_frame++ = check_cs(buf_frame_,11+4);
        *buf_frame++ = FRAME_END;
        
        Slave_Write(fe,4);
        Slave_Write(buf_frame_,13+4);
        
        //获取下一个表
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
        
        //延时12s之后执行下一个操作。
        OSTimeDly(12000,
                  OS_OPT_TIME_DLY,
                  &err);
      }
      cjq_close(cjq_addr,block_cjq);
    }
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+3,3);
  }
  Device_Read(DISABLE);
  
}

//config and query the parameter
void Task_Config(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint16_t msg_size;
  uint8_t * buf_frame;
  
  while(DEF_TRUE){
    buf_frame = OSQPend(&Q_Config,
                        0,
                        OS_OPT_PEND_BLOCKING,
                        &msg_size,
                        &ts,
                        &err);
    switch(*(buf_frame+AFN_POSITION)){
      case AFN_CONFIG:
        param_config(buf_frame,*(buf_frame+msg_size));
        break;
      case AFN_QUERY:
        param_query(buf_frame,*(buf_frame+msg_size));
        break;
    }
    OSMemPut(&MEM_Buf,buf_frame,&err);
  }
}

extern uint8_t ip[17];                 //the server ip
extern uint8_t port[8];  
extern uint8_t ip1;
extern uint8_t ip2;
extern uint8_t ip3;
extern uint8_t ip4;
extern uint16_t port_;
extern uint8_t device_test; //0x00~测试过了~www.xcxdtech.com   0xFF~未测试~avenger0422.vicp.cc

void param_config(uint8_t * buf_frame,uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  uint8_t ip_port_[6];
  
  uint16_t i = 0;
  uint16_t cjq_count = 0;
  uint32_t block_cjq = 0;   //cjq block 地址
  uint32_t block_cjq_next = 0;   //cjq block 地址删除时  先查出来下一个的地址 然后在删除
  uint32_t block_meter = 0;  //meter block 地址
  uint8_t server_seq_ = *(buf_frame + SEQ_POSITION) & 0x0F;
  
  switch(*(buf_frame + FN_POSITION)){
  case FN_IP_PORT:
    ip1 = *(buf_frame + DATA_POSITION + 3);
    ip2 = *(buf_frame + DATA_POSITION + 2);
    ip3 = *(buf_frame + DATA_POSITION + 1);
    ip4 = *(buf_frame + DATA_POSITION);
    port_ = *((uint16_t *)(buf_frame + DATA_POSITION +4));
    
    //ip
    Mem_Clr(ip,17);
    Mem_Clr(ip_port_,6);
    sprintf(ip_port_,"%d",*(buf_frame + DATA_POSITION + 3));
    Str_Cat(ip,ip_port_);
    Str_Cat(ip,".");
    
    Mem_Clr(ip_port_,6);
    sprintf(ip_port_,"%d",*(buf_frame + DATA_POSITION + 2));
    Str_Cat(ip,ip_port_);
    Str_Cat(ip,".");
    
    Mem_Clr(ip_port_,6);
    sprintf(ip_port_,"%d",*(buf_frame + DATA_POSITION + 1));
    Str_Cat(ip,ip_port_);
    Str_Cat(ip,".");
    
    Mem_Clr(ip_port_,6);
    sprintf(ip_port_,"%d",*(buf_frame + DATA_POSITION));
    Str_Cat(ip,ip_port_);
    
    
    //port
    Mem_Clr(port,8);
    Mem_Clr(ip_port_,6);
    sprintf(ip_port_,"%d",*((uint16_t *)(buf_frame + DATA_POSITION +4)));
    Str_Cat(port,",");
    Str_Cat(port,ip_port_);
    Str_Cat(port,"\r");
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    
    if(err != OS_ERR_NONE){
      //获取MUTEX过程中 出错了...
      //return 0xFFFFFF;
      return;
    }
    //处理Config Flash 块
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_CON_IP - sFLASH_CON_START_ADDR),ip,17);
    Mem_Copy(config_flash + (sFLASH_CON_PORT - sFLASH_CON_START_ADDR),port,8);
    
    Mem_Copy(config_flash + (sFLASH_CON_IP1 - sFLASH_CON_START_ADDR),&ip1,1);
    Mem_Copy(config_flash + (sFLASH_CON_IP2 - sFLASH_CON_START_ADDR),&ip2,1);
    Mem_Copy(config_flash + (sFLASH_CON_IP3 - sFLASH_CON_START_ADDR),&ip3,1);
    Mem_Copy(config_flash + (sFLASH_CON_IP4 - sFLASH_CON_START_ADDR),&ip4,1);
    Mem_Copy(config_flash + (sFLASH_CON_PORT_ - sFLASH_CON_START_ADDR),&port_,2);
    
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    device_ack(desc,server_seq_);
    
    break;
  case FN_ADDR:
    Mem_Clr(deviceaddr,5);
    deviceaddr[0] = *(buf_frame + DATA_POSITION);
    deviceaddr[1] = *(buf_frame + DATA_POSITION + 1);
    deviceaddr[2] = *(buf_frame + DATA_POSITION + 2);
    deviceaddr[3] = *(buf_frame + DATA_POSITION + 3);
    deviceaddr[4] = *(buf_frame + DATA_POSITION + 4);  //与新天协议有出入 协议第5位默认为0x00
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    
    if(err != OS_ERR_NONE){
      //获取MUTEX过程中 出错了...
      //return 0xFFFFFF;
      return;
    }
    //处理Config Flash 块
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_DEVICE_ADDR - sFLASH_CON_START_ADDR),deviceaddr,5);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    
    device_ack(desc,server_seq_);
    
    break;
  case FN_METER:
    block_cjq = search_cjq(buf_frame + DATA_POSITION + 10);
    block_meter = search_meter(block_cjq,buf_frame + DATA_POSITION + 3);
    if(block_cjq != 0xFFFFFF){
      if(*(buf_frame + DATA_POSITION + 16) == 0x00){
        //删除表
        if(block_meter == 0xFFFFFF){
          //没有这个表 do nothing
        }else{
          //有这个表  删除这个表
          if(delete_meter(block_cjq,block_meter) == 0xFFFFFF){
            return;
          }
        }
        device_ack(desc,server_seq_);
      }
      
      if(*(buf_frame + DATA_POSITION + 16) == 0x01){
        //添加表
        if(block_meter == 0xFFFFFF){
          //没有这个表 添加
          if(add_meter(block_cjq,buf_frame + DATA_POSITION + 3) == 0xFFFFFF){
            return;
          }
        }else{
          //有这个表 do nothing
        }
        device_ack(desc,server_seq_);
      }
    }
    
    break;
  case FN_CJQ:
    if(*(buf_frame + DATA_POSITION) == 0xAA){
      //删除全部采集器  即清空集中器中的采集器表信息
      sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
      
      for(i = 0;i < cjq_count;i++){
        sFLASH_ReadBuffer((uint8_t *)&block_cjq_next,block_cjq+3,3);
        
        if(delete_cjq(block_cjq) == 0xFFFFFF){
          return;
        }
        block_cjq = block_cjq_next;
      }
      device_ack(desc,server_seq_);
    }
    
    if(*(buf_frame + DATA_POSITION) == 0x55){
      //添加
      if(search_cjq(buf_frame + DATA_POSITION + 1) == 0xFFFFFF){
        //添加这个采集器
        if(add_cjq(buf_frame + DATA_POSITION + 1) == 0xFFFFFF){
          return;
        }
      }else{
        //已经有这个采集器了
      }
      device_ack(desc,server_seq_);
    }
    break;
  case FN_MBUS:
    if(*(buf_frame + DATA_POSITION) == 0xAA){
      //the slave is mbus
      slave_mbus = 0xAA;
    }
    
    if(*(buf_frame + DATA_POSITION) == 0xBB){
      //the slave is 采集器
      slave_mbus = 0xBB;
    }
    
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      //the slave is 485
      slave_mbus = 0xFF;
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  
    if(err != OS_ERR_NONE){
      //获取MUTEX过程中 出错了...
      //return 0xFFFFFF;
      return;
    }
    //处理Config Flash 块
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_METER_MBUS - sFLASH_CON_START_ADDR),&slave_mbus,1);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    
    device_ack(desc,server_seq_);
    break;
  case FN_DI_SEQ:
    if(*(buf_frame + DATA_POSITION) == 0xAA){
      //千宝通使用的大表模块
      di_seq = 0xAA;
    }
    
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      //默认
      di_seq = 0xFF;
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    if(err != OS_ERR_NONE){
      //获取MUTEX过程中 出错了...
      //return 0xFFFFFF;
      return;
    }
    //处理Config Flash 块
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_READMETER_DI_SEQ - sFLASH_CON_START_ADDR),&di_seq,1);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    
    device_ack(desc,server_seq_);
    break;
  case FN_ACK_ACTION:
    if(*(buf_frame + DATA_POSITION) == 0xAA){
      //骏普阀控表模式
      ack_action = 0xAA;
    }
    
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      //默认
      ack_action = 0xFF;
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    if(err != OS_ERR_NONE){
      //获取MUTEX过程中 出错了...
      //return 0xFFFFFF;
      return;
    }
    //处理Config Flash 块
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_ACK_ACTION - sFLASH_CON_START_ADDR),&ack_action,1);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    
    device_ack(desc,server_seq_);
    break;
  case FN_PROTOCOL:
    if(*(buf_frame + DATA_POSITION) == 0x01){
      //EG protocol
      protocol = 0x01;
    }else{
      //188协议
      protocol = 0xFF;
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    if(err != OS_ERR_NONE){
      //获取MUTEX过程中 出错了...
      //return 0xFFFFFF;
      return;
    }
    //处理Config Flash 块
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_PROTOCOL - sFLASH_CON_START_ADDR),&protocol,1);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    
    device_ack(desc,server_seq_);
    break;
  case FN_ERASE:
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
      if(err != OS_ERR_NONE){
        //获取MUTEX过程中 出错了...
        //return 0xFFFFFF;
        return;
      }
      //处理Config Flash 块
      sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
      di_seq = 0xFF;
      Mem_Copy(config_flash + (sFLASH_POOL_INIT - sFLASH_CON_START_ADDR),&di_seq,1);
      sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
      
      device_ack(desc,server_seq_);
      *((uint8_t *)0) = 0x00;  //迫使系统重启
    }
    break;
  case FN_RESET:
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      device_ack(desc,server_seq_);
      *((uint8_t *)0) = 0x00;  //迫使系统重启
    }
    break;
  }
  
}

uint32_t search_meter(uint32_t block_cjq,uint8_t * meteraddr){
  uint32_t block_current = 0;
  uint16_t meter_count = 0;
  uint8_t meter_addr[7];
  uint16_t i;
  
  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+18,2);
  sFLASH_ReadBuffer((uint8_t *)&block_current,block_cjq+12,3);
  
  for(i = 0;i < meter_count;i++){
    sFLASH_ReadBuffer(meter_addr,block_current+6,7);
    if(Mem_Cmp(meteraddr,meter_addr,7) == DEF_YES){
      
      return block_current;
    }
    sFLASH_ReadBuffer((uint8_t *)&block_current,block_current+3,3);
  }
  return 0xFFFFFF;
}

uint32_t add_meter(uint32_t block_cjq,uint8_t * meteraddr){
  uint32_t block_last = 0;
  uint32_t block_new = 0;
  uint16_t meter_count = 0;
  uint16_t meter_all = 0;
  uint32_t meter_read = 0;
  OS_ERR err;
  CPU_TS ts;
  
  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+18,2);
  sFLASH_ReadBuffer((uint8_t *)&meter_all,sFLASH_METER_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_last,block_cjq+15,3);
  
  meter_count++;  //采集器下的表数量++
  meter_all++;  //所有表数量++
  
  block_new = GetFlash();  
  if(block_new == 0xFFFFFF){
    return 0xFFFFFF;
  }
  sFLASH_WritePage(meteraddr,block_new + 6,7);  //表地址
  sFLASH_WritePage((uint8_t *)(meteraddr - 3),block_new + 13,1);  //表类型
  sFLASH_WritePage((uint8_t *)&meter_read,block_new + 14,4);  //表读数  meter_read = 0
  sFLASH_WritePage((uint8_t *)&meter_read,block_new + 22,2);  //表状态  meter_read = 0
  
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //获取MUTEX过程中 出错了...
    return 0xFFFFFF;
  }
  
  if(block_last == 0xFFFFFF){
    //first meter
    sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
    
    //采集器表的开始和结尾都指向新添加的表的块
    Mem_Copy(config_flash+block_cjq%0x1000 + 12,(uint8_t *)&block_new,3);
    Mem_Copy(config_flash+block_cjq%0x1000 + 15,(uint8_t *)&block_new,3);
    Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);  //采集器下的数目++
    
    //将配置好的Flash块重新写入到Flash中。
    sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
  }else{
    //将采集器表的结尾指向新添加的表的块
    sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
    Mem_Copy(config_flash+block_cjq%0x1000 + 15,(uint8_t *)&block_new,3);
    Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);  //采集器下的数目++
    sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
    //原来最后一个表的下一个表指向新添加的表的块
    sFLASH_WritePage((uint8_t *)&block_new,block_last + 3,3);  
    //新添加的表的块的上一个表 指向原来的最后一个表
    sFLASH_WritePage((uint8_t *)&block_last,block_new + 18,3);
    
  }
  
  sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
  Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
  sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
  
  OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
  return block_new;
}

uint32_t delete_meter(uint32_t block_cjq,uint32_t block_meter){
  uint32_t block_after = 0;
  uint32_t block_before = 0;
  uint16_t meter_count = 0;
  uint16_t meter_all = 0;
  OS_ERR err;
  CPU_TS ts;
  
  
  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+18,2);
  sFLASH_ReadBuffer((uint8_t *)&meter_all,sFLASH_METER_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_after,block_meter+3,3);
  sFLASH_ReadBuffer((uint8_t *)&block_before,block_meter+18,3);
  
  meter_count--;
  meter_all--;
  
  PutFlash(block_meter);
  
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  
  if(err != OS_ERR_NONE){
    //获取MUTEX过程中 出错了...
    return 0xFFFFFF;
  }
  
  if(meter_count == 0){
    //采集器下唯一的表  block_before、block_after  都为0xFFFFFF
    sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
    Mem_Copy(config_flash+block_cjq%0x1000 + 12,(uint8_t *)&block_after,3);
    Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);
    Mem_Copy(config_flash+block_cjq%0x1000 + 15,(uint8_t *)&block_before,3);
    sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
    //更新全部表数目
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
  }else{
    if(block_before == 0xFFFFFF || block_after == 0xFFFFFF){
      //要删除的这个是第一个  或者是最后一个
      if(block_before == 0xFFFFFF){
        //修改后一个的before 为block_before
        sFLASH_ReadBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
        Mem_Copy(config_flash+block_after%0x1000 + 18,(uint8_t *)&block_before,3);
        sFLASH_EraseSector((block_after/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //修改采集器第一个表地址为  block_after  更新采集器表数目
        sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
        Mem_Copy(config_flash+block_cjq%0x1000 + 12,(uint8_t *)&block_after,3);
        Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);
        sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //更新全部表数目
        sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
        sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      }
      if(block_after == 0xFFFFFF){
        //修改前一个的next 为block_after
        sFLASH_ReadBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
        Mem_Copy(config_flash+block_before%0x1000 + 3,(uint8_t *)&block_after,3);
        sFLASH_EraseSector((block_before/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //修改采集器的最后一个表地址为  block_before  更新采集器表数目
        sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
        Mem_Copy(config_flash+block_cjq%0x1000 + 15,(uint8_t *)&block_before,3);
        Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);
        sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //更新全部表数目
        sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
        sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      }
    }else{
      //要删除的这个在中间
      //修改前一个的next 为block_after
      sFLASH_ReadBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
      Mem_Copy(config_flash+block_before%0x1000 + 3,(uint8_t *)&block_after,3);
      sFLASH_EraseSector((block_before/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //修改后一个的before 为block_before
      sFLASH_ReadBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
      Mem_Copy(config_flash+block_after%0x1000 + 18,(uint8_t *)&block_before,3);
      sFLASH_EraseSector((block_after/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //更新采集器表数目
      sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
      Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);
      sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //更新全部表数目
      sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
      Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
      sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    }
  }
  OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
  return block_meter;
  
}

//有采集器  返回此采集器的block地址   没有则返回0xFFFFFF
uint32_t search_cjq(uint8_t * cjqaddr){
  uint32_t block_current = 0;
  uint16_t cjq_count;
  uint8_t cjq_addr[6];
  
  uint16_t i;
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_current,sFLASH_CJQ_Q_START,3);
  
  for(i = 0;i < cjq_count;i++){
    
    sFLASH_ReadBuffer(cjq_addr,block_current+6,6);
    if(Mem_Cmp(cjqaddr,cjq_addr,6) == DEF_YES){
      
      return block_current;
    }
    
    sFLASH_ReadBuffer((uint8_t *)&block_current,block_current+3,3);
  }
  return 0xFFFFFF;
}

uint32_t add_cjq(uint8_t * cjqaddr){
  
  uint32_t block_last = 0;
  uint32_t block_new = 0;
  uint16_t cjq_count = 0;
  uint16_t meter_count = 0;
  OS_ERR err;
  CPU_TS ts;
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_last,sFLASH_CJQ_Q_LAST,3);
  
  cjq_count++;  //采集器数量++
  
  //获取一个flash块  并配置相应信息
  block_new = GetFlash();  
  sFLASH_WritePage(cjqaddr,block_new + 6,6);  //采集器地址
  sFLASH_WritePage((uint8_t *)&meter_count,block_new + 18,2);  //采集器表数  (uint8_t *)0 是地址0x00000000处的值。
  //第一块表和最后一块表都指向了0xFFFFFF
  
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //获取MUTEX过程中 出错了...
    return 0xFFFFFF;
  }
  sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
  
  if(block_last == 0xFFFFFF){
    //this is the first cjq
    //cjq Q 的开始和结尾都指向新添加的采集器块
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
    Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
  }else{
    //不是第一个
    //将cjq Q 的结尾指向新添加的采集器块
    Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
    //将原来最后一个采集器的下一个采集器指向新添加的采集器块
    sFLASH_WritePage((uint8_t *)&block_new,block_last + 3,3);
    //将新添加的采集器块中的上一个采集器  指向原来的最后一个采集器
    sFLASH_WritePage((uint8_t *)&block_last,block_new + 20,3);  //上一个采集器
  }
  
  sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
  OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
  
  return block_new;
}

uint32_t delete_cjq(uint32_t block_cjq){
  uint16_t meter_count = 0;
  uint32_t block_meter = 0;
  uint32_t block_meter_next = 0;
  uint32_t block_after = 0;  //下一个采集器
  uint32_t block_before = 0;  //上一个采集器
  uint16_t cjq_count = 0;
  uint16_t i = 0;
  OS_ERR err;
  CPU_TS ts;
  
  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+18,2);
  sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
  sFLASH_ReadBuffer((uint8_t *)&block_after,block_cjq+3,3);
  sFLASH_ReadBuffer((uint8_t *)&block_before,block_cjq+20,3);
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  
  cjq_count--;
  
  //将这个采集器下的表Q清空  将采集器删除
  for(i = 0;i < meter_count;i++){
    
    sFLASH_ReadBuffer((uint8_t *)&block_meter_next,block_meter+3,3);  //获取下一个表的block地址
    if(delete_meter(block_cjq,block_meter) == 0xFFFFFF){
      continue;
    }
    block_meter = block_meter_next;
  }
  //将采集器删除
  PutFlash(block_cjq);
  
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //获取MUTEX过程中 出错了...
    return 0xFFFFFF;
  }
  
  if(cjq_count == 0){
    //这个采集器是唯一的一个   block_before、block_after  都为0xFFFFFF
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_after,3);
    Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_before,3);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
  }else{
    if(block_before == 0xFFFFFF || block_after == 0xFFFFFF){
      //要删除的这个是第一个  或者是最后一个
      if(block_before == 0xFFFFFF){
        //修改后一个的before 为block_before
        sFLASH_ReadBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
        Mem_Copy(config_flash+block_after%0x1000 + 20,(uint8_t *)&block_before,3);
        sFLASH_EraseSector((block_after/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //修改采集器Q的start 为block_after  更新采集器数量
        sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(config_flash + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_after,3);
        Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
        sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      }
      
      if(block_after == 0xFFFFFF){
        //修改前一个的next 为block_after
        sFLASH_ReadBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
        Mem_Copy(config_flash+block_before%0x1000 + 3,(uint8_t *)&block_after,3);
        sFLASH_EraseSector((block_before/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //修改采集器Q的end 为block_before  更新采集器数量
        sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
        Mem_Copy(config_flash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_before,3);
        sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      }
      
    }else{
      //要删除的这个在中间
      //修改前一个的next 为block_after
      sFLASH_ReadBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
      Mem_Copy(config_flash+block_before%0x1000 + 3,(uint8_t *)&block_after,3);
      sFLASH_EraseSector((block_before/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //修改后一个的before 为block_before
      sFLASH_ReadBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //读取所在Sector
      Mem_Copy(config_flash+block_after%0x1000 + 20,(uint8_t *)&block_before,3);
      sFLASH_EraseSector((block_after/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //修改control blocks
      sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
      Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
      sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    }
  }
  OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
  return block_cjq;
}

void param_query(uint8_t * buf_frame,uint8_t desc){
  uint8_t server_seq_ = *(buf_frame + SEQ_POSITION) & 0x0F;
  switch(*(buf_frame + FN_POSITION)){
  case FN_IP_PORT:
    ack_query_ip(desc,server_seq_);
    break;
  case FN_ADDR: 
    ack_query_addr(desc,server_seq_);
    break;
  case FN_METER:
    ack_query_meter(*(buf_frame + DATA_POSITION),buf_frame + DATA_POSITION + 1,desc,server_seq_);
    break;
  case FN_CJQ:
    ack_query_cjq(desc,server_seq_);
    break;
  case FN_MBUS:
    ack_query_mbus(desc,server_seq_);
    break;
  case FN_DI_SEQ:
    ack_query_di_seq(desc,server_seq_);
    break;
  case FN_ACK_ACTION:
    ack_query_ack_action(desc,server_seq_);
    break;
  case FN_PROTOCOL:
    ack_query_protocol(desc,server_seq_);
    break;
  }
}

void Task_LED(void *p_arg){
  OS_ERR err;
  uint8_t cnt = 0;
  uint8_t readingbeat[17];  //抄全部表时的心跳
  uint8_t *buf_frame = 0;
  
  while(DEF_TRUE){
    //LED2
    if(reading == 0){
      GPIO_SetBits(GPIOB,GPIO_Pin_8);
      OSTimeDly(1000,
                    OS_OPT_TIME_DLY,
                    &err);
      GPIO_ResetBits(GPIOB,GPIO_Pin_8);
      OSTimeDly(1000,
                    OS_OPT_TIME_DLY,
                    &err);
      cnt = 0;
    }else{
      GPIO_SetBits(GPIOB,GPIO_Pin_8);
      OSTimeDly(300,
                    OS_OPT_TIME_DLY,
                    &err);
      GPIO_ResetBits(GPIOB,GPIO_Pin_8);
      OSTimeDly(300,
                    OS_OPT_TIME_DLY,
                    &err);
      if(readingall){
        cnt++;
        if(cnt >= 15){
          cnt = 0;
          buf_frame = readingbeat;
          *buf_frame++ = FRAME_HEAD;
          //buf_frame_16 = (uint16_t *)buf_frame;
          *buf_frame++ = 0x27;//(9 << 2) | 0x03;
          *buf_frame++ = 0x00;
          *buf_frame++ = 0x27;//(9 << 2) | 0x03;
          *buf_frame++ = 0x00;
          //buf_frame = (uint8_t *)buf_frame_16;
          *buf_frame++ = FRAME_HEAD;
          
          *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_START | START_FUN_REQ1;
          /**/
          *buf_frame++ = deviceaddr[0];
          *buf_frame++ = deviceaddr[1];
          *buf_frame++ = deviceaddr[2];
          *buf_frame++ = deviceaddr[3];
          *buf_frame++ = deviceaddr[4];
          
          *buf_frame++ = AFN_FAKE;
          *buf_frame++ = ZERO_BYTE |SINGLE | local_seq;
          *buf_frame++ = readingall_progress;//FN_HEARTBEAT;
          
          *buf_frame++ = check_cs(readingbeat+6,9);
          *buf_frame++ = FRAME_END;
          send_server(readingbeat,17);
        }
      }
    }
  }
}

void device_ack(uint8_t desc,uint8_t server_seq_){
  uint8_t ack[17];
  uint8_t * buf_frame;
  
  buf_frame = ack;
  *buf_frame++ = FRAME_HEAD;
  *buf_frame++ = 0x27;//(9 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = 0x27;//(9 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_ACK;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_ACK;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_ACK;
  
  *buf_frame++ = check_cs(ack+6,9);
  *buf_frame++ = FRAME_END;
  
  if(desc){
    //to m590e
    send_server(ack,17);
  }else{
    //to 485
    Server_Write_485(ack,17);
  }
  
}

void device_nack(uint8_t desc,uint8_t server_seq_){
  uint8_t nack[17];
  uint8_t * buf_frame;
  
  buf_frame = nack;
  *buf_frame++ = FRAME_HEAD;
  *buf_frame++ = 0x27;//(9 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = 0x27;//(9 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_ACK;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_ACK;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_NACK;
  
  *buf_frame++ = check_cs(nack+6,9);
  *buf_frame++ = FRAME_END;
  
  if(desc){
    //to m590e
    send_server(nack,17);
  }else{
    //to 485
    Server_Write_485(nack,17);
  }
  
}

void ack_query_cjq(uint8_t desc,uint8_t server_seq_){
  OS_ERR err;
  uint8_t * buf_frame;
  uint8_t * buf_frame_;
  uint16_t * buf_frame_16;
  
  uint16_t cjq_count;
  uint32_t block_cjq;
  uint16_t i;
  uint8_t cjq_addr[6];
  uint8_t j;
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  buf_frame_ = buf_frame;
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  
  *buf_frame++ = FRAME_HEAD;
  buf_frame_16 = (uint16_t *)buf_frame;
  *buf_frame_16++ = ((9+cjq_count*6) << 2) | 0x03;
  *buf_frame_16++ = ((9+cjq_count*6) << 2) | 0x03;
  buf_frame = (uint8_t *)buf_frame_16;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_QUERY;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_CJQ;
  
  for(i = 0;i < cjq_count;i++){
    sFLASH_ReadBuffer(cjq_addr,block_cjq+6,6);
    for(j = 0;j < 6;j++){
      *buf_frame++ = cjq_addr[j];
    }
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+3,3);
  }
  
  *buf_frame++ = check_cs(buf_frame_+6,9+cjq_count*6);
  *buf_frame++ = FRAME_END;
  
  
  if(desc){
    //to m590e
    send_server(buf_frame_,17+cjq_count*6);
  }else{
    //to 485
    Server_Write_485(buf_frame_,17+cjq_count*6);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
  
}

void ack_query_meter(uint8_t metertype,uint8_t * meteraddr,uint8_t desc,uint8_t server_seq_){
  OS_ERR err;
  uint8_t * buf_frame;
  uint8_t * buf_frame_;
  uint16_t * buf_frame_16;
  uint16_t cjqmeter_count;
  uint16_t allmeter_count;
  uint16_t cjq_count;
  
  uint16_t times;
  uint8_t remain;
  uint16_t times_;      //一共要发送多少帧
  uint16_t times_count; //发送了多少帧了
  
  uint16_t len = 0;  //当前帧的数据长度
  
  uint32_t block_cjq;
  uint32_t block_meter;
  
  uint8_t cjq_addr[6];
  uint8_t meter_addr[7];
  uint8_t meter_type;   //表的类型
  uint8_t meter_status; //表的状态
  uint8_t meter_fount = 0;  //是否找到了这个表
  
  uint16_t i = 0;       //计数采集器
  uint16_t j = 0;       //计数采集器下的表
  uint8_t k = 0;        //计数copy采集器、表地址
  
  uint16_t meter_count = 0;  //计数一帧中的数据体个数
  uint16_t meter_count_ = 0;    //保持一帧中数据体的个数
  uint8_t header = 0;   //一帧的帧头是否已准备
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&allmeter_count,sFLASH_METER_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
  if(cjq_count == 0){
    //没有采集器。。。
    return;
  }
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  
  buf_frame_ = buf_frame;
  
  if(Mem_Cmp(meteraddr,"\xFF\xFF\xFF\xFF\xFF\xFF\xFF",7) == DEF_YES){
    //全部表
    times = allmeter_count/10;
    remain = allmeter_count%10;
    times_count = 0;
    times_ = times;
    if(remain > 0){
      times_ = times_ + 1;
    }
    
    for(i = 0;i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      for(j=0;j < cjqmeter_count;j++){
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
        sFLASH_ReadBuffer((uint8_t *)&meter_status,block_meter+21,1);
        
        if(header == 0){
          header = 1;
          times_count++;
          *buf_frame++ = FRAME_HEAD;
          if(times_ == 1){
            //单帧
            meter_count = allmeter_count;
            meter_count_ = meter_count;
            len = ((9+17*meter_count_+1) << 2) | 0x03;    //+1  because of  *buf_frame++ = metertype;
            
          }else{
            //多帧
            if(times_count == 1){
              //首帧
              meter_count = 10;
              meter_count_ = meter_count;
              len = ((9+17*meter_count_+1) << 2) | 0x03;
              
            }else{
              if(times_count == times_){
                //尾帧
                if(remain == 0){
                  meter_count = 10; 
                  meter_count_ = meter_count;
                }else{
                  meter_count = remain;
                  meter_count_ = meter_count;
                }
                len = ((9+17*meter_count_+1) << 2) | 0x03;
                
              }else{
                //中间帧
                meter_count = 10;
                meter_count_ = meter_count;
                len = ((9+17*meter_count_+1) << 2) | 0x03;
                
              }
            }
          }
          
          buf_frame_16 = (uint16_t *)buf_frame;
          *buf_frame_16++ = len;
          *buf_frame_16++ = len;
          buf_frame = (uint8_t *)buf_frame_16;
          *buf_frame++ = FRAME_HEAD;
          
          *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
          *buf_frame++ = deviceaddr[0];
          *buf_frame++ = deviceaddr[1];
          *buf_frame++ = deviceaddr[2];
          *buf_frame++ = deviceaddr[3];
          *buf_frame++ = deviceaddr[4];
              
          *buf_frame++ = AFN_QUERY;
          
          if(times_ == 1){
            //单帧
            *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
          }else{
            //多帧
            if(times_count == 1){
              //首帧
              *buf_frame++ = ZERO_BYTE |MUL_FIRST | server_seq_;
            }else{
              if(times_count == times_){
                //尾帧
                *buf_frame++ = ZERO_BYTE |MUL_LAST | server_seq_;
              }else{
                //中间帧
               *buf_frame++ = ZERO_BYTE |MUL_MIDDLE | server_seq_;
              }
            }
          }
          
          *buf_frame++ = FN_METER;
          *buf_frame++ = metertype;
        }
        
        *buf_frame++ = 0x00;
        *buf_frame++ = 0x00;
        for(k=0;k<7;k++){
          *buf_frame++ = meter_addr[k];
        }
        for(k=0;k<6;k++){
          *buf_frame++ = cjq_addr[k];
        }
        *buf_frame++ = 0x00;
        *buf_frame++ = 0x01;
        
        meter_count--;
        if(meter_count == 0){
          header = 0;   //为下一帧做准备
          //发送这一帧
          
          *buf_frame++ = check_cs(buf_frame_+6,9+17*meter_count_+1);
          *buf_frame++ = FRAME_END;
          
          if(desc){
            //to m590e
            send_server(buf_frame_,17+17*meter_count_+1);
          }else{
            //to 485
            Server_Write_485(buf_frame_,17+17*meter_count_+1);
          }
          OSTimeDly(100,
                    OS_OPT_TIME_DLY,
                    &err);
          
          buf_frame = buf_frame_;
        }
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
      }
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+3,3);
    }
  }else{
    //单个表
    for(i = 0;meter_fount == 0 && i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      for(j=0;j < cjqmeter_count;j++){
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
        
        if(Mem_Cmp(meteraddr,meter_addr,7) == DEF_YES && meter_type == metertype){
          //找到这个表了。。。
          sFLASH_ReadBuffer((uint8_t *)&meter_status,block_meter+21,1);
          meter_fount = 1; 
          break;
        }
        
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
      }
    }
    
    if(meter_fount){
      //找到这个表了
      *buf_frame++ = FRAME_HEAD;
      *buf_frame++ = 0x6F;//((9+17+1) << 2) | 0x03;
      *buf_frame++ = 0x00;
      *buf_frame++ = 0x6F;//((9+17+1) << 2) | 0x03;
      *buf_frame++ = 0x00;
      *buf_frame++ = FRAME_HEAD;
      
      *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
      /**/
      *buf_frame++ = deviceaddr[0];
      *buf_frame++ = deviceaddr[1];
      *buf_frame++ = deviceaddr[2];
      *buf_frame++ = deviceaddr[3];
      *buf_frame++ = deviceaddr[4];
      
      *buf_frame++ = AFN_QUERY;
      *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
      *buf_frame++ = FN_METER;
      
      *buf_frame++ = metertype;
      *buf_frame++ = 0x00;
      *buf_frame++ = 0x00;
      for(i=0;i<7;i++){
        *buf_frame++ = meter_addr[i];
      }
      for(i=0;i<6;i++){
        *buf_frame++ = cjq_addr[i];
      }
      *buf_frame++ = 0x00;
      *buf_frame++ = 0x01;
      
      *buf_frame++ = check_cs(buf_frame_+6,27);
      *buf_frame++ = FRAME_END;
      
      if(desc){
        //to m590e
        send_server(buf_frame_,35);
      }else{
        //to 485
        Server_Write_485(buf_frame_,35);
      }
    }
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
}

void ack_query_addr(uint8_t desc,uint8_t server_seq_){
  OS_ERR err;
  uint8_t * buf_frame;
  uint8_t * buf_frame_;
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  buf_frame_ = buf_frame;
  *buf_frame++ = FRAME_HEAD;
  *buf_frame++ = 0x27;//(9 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = 0x27;//(9 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_QUERY;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_ADDR;
  
  *buf_frame++ = check_cs(buf_frame_+6,9);
  *buf_frame++ = FRAME_END;
  
  
  if(desc){
    //to m590e
    send_server(buf_frame_,17);
  }else{
    //to 485
    Server_Write_485(buf_frame_,17);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
}

void ack_query_ip(uint8_t desc,uint8_t server_seq_){
  OS_ERR err;
  uint8_t * buf_frame = 0;
  uint8_t * buf_frame_ = 0;
  uint16_t * buf_frame_16 = 0;
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  buf_frame_ = buf_frame;
  *buf_frame++ = FRAME_HEAD;
  *buf_frame++ = 0x3F;//(15 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = 0x3F;//(15 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_QUERY;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_IP_PORT;
  
  *buf_frame++ = ip4;
  *buf_frame++ = ip3;
  *buf_frame++ = ip2;
  *buf_frame++ = ip1;
  
  buf_frame_16 = (uint16_t *)buf_frame;
  *buf_frame_16++ = port_;
  buf_frame = (uint8_t *)buf_frame_16;
  
  *buf_frame++ = check_cs(buf_frame_+6,15);
  *buf_frame++ = FRAME_END;
  
  
  if(desc){
    //to m590e
    send_server(buf_frame_,23);
  }else{
    //to 485
    Server_Write_485(buf_frame_,23);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
}

void ack_query_mbus(uint8_t desc,uint8_t server_seq_){
  OS_ERR err;
  uint8_t * buf_frame = 0;
  uint8_t * buf_frame_ = 0;
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  buf_frame_ = buf_frame;
  *buf_frame++ = FRAME_HEAD;
  *buf_frame++ = 0x2B;//(10 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = 0x2B;//(10 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_QUERY;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_MBUS;
  
  *buf_frame++ = slave_mbus;
  *buf_frame++ = check_cs(buf_frame_+6,10);
  *buf_frame++ = FRAME_END;
  
  
  if(desc){
    //to m590e
    send_server(buf_frame_,18);
  }else{
    //to 485
    Server_Write_485(buf_frame_,18);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
}

void ack_query_di_seq(uint8_t desc,uint8_t server_seq_){
  OS_ERR err;
  uint8_t * buf_frame = 0;
  uint8_t * buf_frame_ = 0;
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  buf_frame_ = buf_frame;
  *buf_frame++ = FRAME_HEAD;
  *buf_frame++ = 0x2B;//(10 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = 0x2B;//(10 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_QUERY;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_DI_SEQ;
  
  *buf_frame++ = di_seq;
  *buf_frame++ = check_cs(buf_frame_+6,10);
  *buf_frame++ = FRAME_END;
  
  
  if(desc){
    //to m590e
    send_server(buf_frame_,18);
  }else{
    //to 485
    Server_Write_485(buf_frame_,18);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
}


void ack_query_ack_action(uint8_t desc,uint8_t server_seq_){
  OS_ERR err;
  uint8_t * buf_frame = 0;
  uint8_t * buf_frame_ = 0;
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  buf_frame_ = buf_frame;
  *buf_frame++ = FRAME_HEAD;
  *buf_frame++ = 0x2B;//(10 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = 0x2B;//(10 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_QUERY;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_ACK_ACTION;
  
  *buf_frame++ = ack_action;
  *buf_frame++ = check_cs(buf_frame_+6,10);
  *buf_frame++ = FRAME_END;
  
  
  if(desc){
    //to m590e
    send_server(buf_frame_,18);
  }else{
    //to 485
    Server_Write_485(buf_frame_,18);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
}


void ack_query_protocol(uint8_t desc,uint8_t server_seq_){
  OS_ERR err;
  uint8_t * buf_frame = 0;
  uint8_t * buf_frame_ = 0;
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  buf_frame_ = buf_frame;
  *buf_frame++ = FRAME_HEAD;
  *buf_frame++ = 0x2B;//(10 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = 0x2B;//(10 << 2) | 0x03;
  *buf_frame++ = 0x00;
  *buf_frame++ = FRAME_HEAD;
  
  *buf_frame++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
  /**/
  *buf_frame++ = deviceaddr[0];
  *buf_frame++ = deviceaddr[1];
  *buf_frame++ = deviceaddr[2];
  *buf_frame++ = deviceaddr[3];
  *buf_frame++ = deviceaddr[4];
  
  *buf_frame++ = AFN_QUERY;
  *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
  *buf_frame++ = FN_PROTOCOL;
  
  *buf_frame++ = protocol;
  *buf_frame++ = check_cs(buf_frame_+6,10);
  *buf_frame++ = FRAME_END;
  
  
  if(desc){
    //to m590e
    send_server(buf_frame_,18);
  }else{
    //to 485
    Server_Write_485(buf_frame_,18);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
}

extern OS_FLAG_GRP FLAG_Event;
void Task_OverLoad(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  
  while(DEF_TRUE){
    
    OSFlagPend(&FLAG_Event,
                OVERLOAD,
                0,
                OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME+OS_OPT_PEND_BLOCKING,
                &ts,
                &err);
    
    OSTimeDly(500,
                  OS_OPT_TIME_DLY,
                  &err);
    if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)){
      //enable the beep
      GPIO_SetBits(GPIOC,GPIO_Pin_14);
      //disable the mbus power
      GPIO_ResetBits(GPIOA,GPIO_Pin_0);
      //Light the LED3
      
      while(DEF_TRUE){
        GPIO_SetBits(GPIOB,GPIO_Pin_7);
        OSTimeDly(100,
                  OS_OPT_TIME_DLY,
                  &err);
        GPIO_ResetBits(GPIOB,GPIO_Pin_7);
        OSTimeDly(100,
                  OS_OPT_TIME_DLY,
                  &err);
      }
    }
    
  }
}