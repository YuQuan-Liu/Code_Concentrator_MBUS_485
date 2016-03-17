

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

extern OS_Q Q_Slave;            //�ɼ��������͹���������
extern OS_Q Q_Read;            //��������Queue
extern OS_Q Q_ReadData;        //���ͳ���ָ���  �²㷵�س�������
extern OS_Q Q_Config;         //��������Queue
extern OS_Q Q_Deal;         //������յ��ķ��������͹���������

extern OS_SEM SEM_HeartBeat;    //���շ���������Task to HeartBeat Task  ���յ������Ļ�Ӧ
extern OS_SEM SEM_ACKData;     //�����������ݵ�ACK
extern OS_SEM SEM_SendOver;      //got the "+TCPSEND:0,"  the data is send over now  �����������
extern OS_SEM SEM_Send;      //got the '>'  we can send the data now  ���Է�������

extern OS_TMR TMR_CJQTIMEOUT;    //�򿪲ɼ���֮�� 20���ӳ�ʱ �Զ��ر�ͨ��

extern volatile uint8_t reading;
extern volatile uint8_t connectstate; 
extern uint8_t deviceaddr[5];

extern uint8_t slave_mbus; //0xaa mbus   0xff  485   0xBB~�ɼ���

extern uint8_t config_flash[];  //���ô���Flashʹ�õ�����  Sector==4K  ��Ҫһ��4K������
extern uint8_t *meterdata;  //ʹ�ú���Э�鳭��ʱ��ŷ��ص���Ϣ  ʹ��config_flash
extern OS_MUTEX MUTEX_CONFIGFLASH;    //�Ƿ����ʹ�� config_flash  4K ��������FLASH
extern uint8_t di_seq; //DI0 DI1 ˳��   0xAA~DI1��ǰ(ǧ��ͨ)   0xFF~DI0��ǰ(default)  
extern uint8_t ack_action;  //��Ӧ������~0xaa    �Ȳ�����Ӧ��~0xff
extern uint8_t protocol;  //Э������ 0xFF~188(Default)  1~EG 

uint8_t heart_seq = 0;  //��¼���������к� �ȴ�ack
uint8_t data_seq = 0;  //��¼���ݵ����к� �ȴ�ack

uint8_t local_seq = 0;  //�������к�
uint8_t server_seq = 0;  //�����������к�  ����ʱ  ��ͬ�������к�

uint8_t fe[4] = {0xFE,0xFE,0xFE,0xFE};  //����ʱǰ�淢�͵�4��0xFE

uint8_t * volatile buf = 0;   //the buf used put the data in 
uint8_t * volatile buf_;       //keep the buf's ptr  used to release the buf
uint8_t start_slave = 0;


uint8_t readingall = 0;   //�Ƿ����ڳ�ȫ����
uint8_t readingall_progress = 0;  //���ڳ�ȫ����Ľ������

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
    //�յ�0x68֮��  ���200ms û���յ�����  ����Ϊ��ʱ��
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
                    *buf = 0x00;//��ʶ��һ֡����������485��
                    OSQPost(&Q_Config,
                            buf_,frame_len,
                            OS_OPT_POST_FIFO,
                            &err);
                    break;
                  case AFN_CONTROL:
                  case AFN_CURRENT:
                    *buf = 0x00;//��ʶ��һ֡����������485��
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


uint8_t * volatile server_ptr = 0;      //�ж��б���M590E ������������
uint8_t * volatile server_ptr_ = 0;     //��¼�жϵĿ�ʼָ��

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
        check_str(buf_server_task_,buf_server_task);  //���ε�����ǰ��0x00
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
            
          server_seq_ = *(start+SEQ_POSITION) & 0x0F;  //��ø�֡�����к�
          
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
              //������Ӧ��֡
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
            *(buf_copy + len) = 0x01;  //��ʶ��һ֡���Է�����
            OSQPost(&Q_Config,buf_copy,len,OS_OPT_POST_1,&err);
          }
          break;
        case AFN_CONTROL:
        case AFN_CURRENT:
          
          server_seq_ = *(start + SEQ_POSITION) & 0x0F;
          if(*(start+FN_POSITION) == 0x05){
            //ƥ�����к�
            server_seq = server_seq_;
            device_ack(0x01,server_seq_);
          }else{
            if(server_seq != server_seq_){
              //�µĳ���ָ��  ack & read
              buf_copy = OSMemGet(&MEM_Buf,&err);
              if(buf_copy != 0){
                Mem_Copy(buf_copy,start,len);
                *(buf_copy + len) = 0x01;  //��ʶ��һ֡���Է�����
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
    //�򿪵�Դ
    if(slave_mbus == 0xAA){
      mbus_power(ENABLE);
    }else{
      relay_485(ENABLE);
    }
  }else{
    //�رյ�Դ
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
������Э���
*/
void meter_read_eg(uint8_t * buf_frame,uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  //��ȡconfig_flash��ʹ��Ȩ
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //��ȡMUTEX������ ������...
    //return 0xFFFFFF;
    return;
  }
  meterdata = config_flash; //�����ص�������Ϣ�����config_flash
  
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
����ɼ�����ַΪ FF FF FF FF FF FF ��ʾ����������ֱ�ӽӵı�

���������ʱ��
�ȷ��ʹ�ָ���ɼ���͸������  ���յ���Ӧ֮��
���Է��ͳ�������ָ��
ָ��ʱ����û���յ����صı��ָ��  �ظ�3��

�ص�ָ���ɼ�����͸������ 

*/
void meter_read_188(uint8_t * buf_frame,uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  uint32_t block_cjq = 0;   //cjq block ��ַ
  uint32_t block_meter = 0;  //meter block ��ַ
  
  uint16_t cjq_count = 0;
  uint16_t cjqmeter_count = 0;
  
  uint16_t i = 0;
  uint16_t j = 0;
  
  uint8_t cjq_addr[6];
  uint8_t meter_addr[7];
  uint8_t meter_type = 0;
  
  uint8_t meter_fount = 0;
  //��ѯ�Ƿ��������
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
  if(cjq_count == 0){
    //û�вɼ���������
    return;
  }
  
  Device_Read(ENABLE);
  if(Mem_Cmp(buf_frame+16,"\xFF\xFF\xFF\xFF\xFF\xFF\xFF",7) == DEF_YES){
    //��ȫ����
    readingall = 1;
    /**/
    for(i = 0;i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      
      if(cjq_open(cjq_addr,block_cjq) == 0){
        //û�д򿪲ɼ���
        OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
      
        if(err != OS_ERR_NONE){
          //��ȡMUTEX������ ������...
          //return 0xFFFFFF;
          return;
        }
        for(j=0;j < cjqmeter_count;j++){
          sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
          *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x80;
          
          //�����úõ�Flash������д�뵽Flash�С�
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
      
      readingall_progress = (i+1)/cjq_count;  //��ȫ����ʱ�����ϱ��Ľ���  ��ǰ�ڼ����ɼ���/�ɼ�������
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+3,3);
    }
    readingall = 0;
    meter_send(1,0,desc);
  }else{
    //��������
    for(i = 0;meter_fount == 0 && i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      for(j=0;j < cjqmeter_count;j++){
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
        
        if(Mem_Cmp(buf_frame + 16,meter_addr,7) == DEF_YES && meter_type == *(buf_frame + 15)){
          //�ҵ�������ˡ�����
          if(cjq_open(cjq_addr,block_cjq) == 0){
            //û�д򿪲ɼ���
            OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
          
            if(err != OS_ERR_NONE){
              //��ȡMUTEX������ ������...
              //return 0xFFFFFF;
              return;
            }
            sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
            *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x80;
            
            //�����úõ�Flash������д�뵽Flash�С�
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

//ֻ�ܶ���
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
      //Ĭ�ϵ�λ��ǰ
      *buf_frame++ = DATAFLAG_RD_L;
      *buf_frame++ = DATAFLAG_RD_H;
    }else{
      //ǧ��ͨʹ�õ�˳�򡣡������ʹ��
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
          //����ʧ��  ��flash  
          success = 0;
        }
        continue;
      }
      //���յ���ȷ������
      //�жϱ��ַ
      if(Mem_Cmp(meter_addr,buf_readdata+2,7) == DEF_YES){
        success = 1;
        //��ȡST
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
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    
    sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
    if(success == 0){
      //����ʧ��  ��flash  return nack
      *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x40;
    }else{
      Mem_Copy(config_flash+block_meter%0x1000 + 22,&st_l,1);  //��¼st��Ϣ
      Mem_Copy(config_flash+block_meter%0x1000 + 23,&st_h,1);  //��¼st��Ϣ
      Mem_Copy(config_flash+block_meter%0x1000 + 14,read,4);        //����
      Mem_Copy(config_flash+block_meter%0x1000 + 24,half,4);        //��λ
    }
    
    //�����úõ�Flash������д�뵽Flash�С�
    sFLASH_EraseSector((block_meter/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
}

//all = 1 ����ȫ����  all = 0 ���ͱ���Ӧ�ı�
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
  uint16_t times_ = 0;      //һ��Ҫ���Ͷ���֡
  uint16_t times_count = 0; //�����˶���֡��
  
  
  uint8_t meter_addr[7];
  uint8_t meter_read[4];
  uint32_t block_cjq = 0;
  uint32_t block_meter = 0;
  
  uint8_t meter_type = 0;   //�������
  uint8_t st_l = 0; //���״̬
  uint8_t st_h = 0; //���״̬
  
  uint16_t i = 0;       //�����ɼ���
  uint16_t j = 0;       //�����ɼ����µı�
  uint8_t k = 0;        //����copy�ɼ��������ַ
  
  uint16_t meter_count = 0;  //����һ֡�е����������
  uint16_t meter_count_ = 0;    //����һ֡��������ĸ���
  uint8_t header = 0;   //һ֡��֡ͷ�Ƿ���׼��
  
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
    //ȫ����
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
            //��֡
            meter_count = allmeter_count;
            meter_count_ = meter_count;
            
            len = ((9+14*meter_count_+5) << 2) | 0x03;    //+1  because of  *buf_frame++ = metertype;
            
          }else{
            //��֡
            if(times_count == 1){
              //��֡
              meter_count = 10;
              meter_count_ = meter_count;
              
              len = ((9+14*meter_count_+5) << 2) | 0x03;
            }else{
              if(times_count == times_){
                //β֡
                if(remain == 0){
                  meter_count = 10; 
                  meter_count_ = meter_count;
                }else{
                  meter_count = remain;
                  meter_count_ = meter_count;
                }
                
                len = ((9+14*meter_count_+5) << 2) | 0x03;
              }else{
                //�м�֡
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
            //��֡
            *buf_frame++ = ZERO_BYTE |SINGLE | CONFIRM| local_seq;
          }else{
            //��֡
            if(times_count == 1){
              //��֡
              *buf_frame++ = ZERO_BYTE |MUL_FIRST | CONFIRM| local_seq;
            }else{
              if(times_count == times_){
                //β֡
                *buf_frame++ = ZERO_BYTE |MUL_LAST | CONFIRM| local_seq;
              }else{
                //�м�֡
                *buf_frame++ = ZERO_BYTE |MUL_MIDDLE | CONFIRM| local_seq;
              }
            }
          }
          data_seq = local_seq;
          addSEQ();
          
          *buf_frame++ = FN_CURRENT_METER;
          
          buf_frame_16 = (uint16_t *)buf_frame;
          *buf_frame_16++ = times_;    //�ܹ�����֡
          *buf_frame_16++ = times_count;  //�ڼ�֡
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
          header = 0;   //Ϊ��һ֡��׼��
          //������һ֡
          
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
  uint32_t block_cjq = 0;   //cjq block ��ַ
  uint32_t block_meter = 0;  //meter block ��ַ
  
  uint16_t cjq_count = 0;
  uint16_t cjqmeter_count = 0;
  
  uint16_t i = 0;
  uint16_t j = 0;
  
  uint8_t cjq_addr[6];
  uint8_t meter_addr[7];
  uint8_t meter_type = 0;
  
  uint8_t meter_fount = 0;
  uint8_t server_seq_ = 0;  
  //��ѯ�Ƿ��������
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
  if(cjq_count == 0){
    //û�вɼ���������
    //todo ��Ӧ NACK
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
          //�ҵ�������ˡ�����
          
          if(cjq_open(cjq_addr,block_cjq)==0){
            //û�д򿪲ɼ���
              OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
            
              if(err != OS_ERR_NONE){
                //��ȡMUTEX������ ������...
                //return 0xFFFFFF;
                return;
              }
              sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
              *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x80;
              
              //�����úõ�Flash������д�뵽Flash�С�
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
      //�ɼ��� ~~~
      buf_frame = buf_frame_;
      *buf_frame++ = FRAME_HEAD;
      *buf_frame++ = 0xA0;  //�ɼ�����־
      for(i=0;i<6;i++){
        *buf_frame++ = cjq_addr[i];
      }
      *buf_frame++ = 0x00;  //�ɼ������λ
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
            //���ɼ���ʧ��  ��flash  return nack
            success = 0;
          }
          continue;
        }
        //���յ���ȷ������
        //�жϱ��ַ
        if(Mem_Cmp(cjq_addr,buf_readdata+2,6) == DEF_YES){
          //��ȡST
          st_l = *(buf_readdata + 14);
          if((st_l & 0x03) == 0x00){
            //opened  return ack
            success = 1;
          }else{
            //���ɼ���ʧ��  ��flash  return nack
            success = 0;
          }
        }
        OSMemPut(&MEM_Buf,buf_readdata,&err);
      }
      if(success == 0){
        //���ɼ���ʧ��  ��flash  return nack
        OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
      
        if(err != OS_ERR_NONE){
          //��ȡMUTEX������ ������...
          //return 0xFFFFFF;
          return success;
        }
        
        sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
        //����ʧ��  ��flash  return nack
        *(config_flash+block_cjq%0x1000 + 23) = 0x01;
        
        //�����úõ�Flash������д�뵽Flash�С�
        sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        
        OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
      }
      return success;
    }
    return 1; //����ײ�ֱ�ӽ���485��188Э��ı�  ֱ�ӷ���1
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
      //�ɼ��� ~~~
      buf_frame = buf_frame_;
      *buf_frame++ = FRAME_HEAD;
      *buf_frame++ = 0xA0;  //�ɼ�����־
      for(i=0;i<6;i++){
        *buf_frame++ = cjq_addr[i];
      }
      *buf_frame++ = 0x00;  //�ɼ������λ
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
            //�زɼ���ʧ��  ��flash  return nack
            success = 0;
          }
          continue;
        }
        //���յ���ȷ������
        //�жϱ��ַ
        if(Mem_Cmp(cjq_addr,buf_readdata+2,6) == DEF_YES){
          //��ȡST
          st_l = *(buf_readdata + 14);
          if((st_l & 0x03) == 0x02){
            //closed  return ack
            success = 1;
          }else{
            //���ɼ���ʧ��  ��flash  return nack
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
  //�رյ�Դ
  power_cmd(DISABLE);
  //�ر�ͨ��
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
      //Ĭ�ϵ�λ��ǰ
      *buf_frame++ = DATAFLAG_WV_L;
      *buf_frame++ = DATAFLAG_WV_H;
    }else{
      // ���շ��ر�ʹ��  �е�
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
          //����ʧ��  ��flash  return nack
          success = 0;
        }
        continue;
      }
      //���յ���ȷ������
      if(ack_action != 0xff){
        OSMemPut(&MEM_Buf,buf_readdata,&err);
        OSTimeDly(12000,
                  OS_OPT_TIME_DLY,
                  &err);
        success = 1;
      }else{
        //�жϱ��ַ
        if(Mem_Cmp(meter_addr,buf_readdata+2,7) == DEF_YES){
          //��ȡST
          st_l = *(buf_readdata + 14);
          st_h = *(buf_readdata + 15);
          if((st_l & 0x03) == 0x00){
            //opened  return ack
            success = 1;
          }else{
            //����ʧ��  ��flash  return nack
            success = 0;
          }
        }
        
        OSMemPut(&MEM_Buf,buf_readdata,&err);
      }
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  
    if(err != OS_ERR_NONE){
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
    if(success == 0){
      //����ʧ��  ��flash  return nack
      //Mem_Copy(configflash + 22,"\x43",1);  //��ʱ
      if((st_l & 0x03) == 0x03){
        //���ŷ����쳣
        *(config_flash+block_meter%0x1000 + 22) = st_l;
        *(config_flash+block_meter%0x1000 + 23) = st_h;
      }else{
        //��ʱ
        *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x40;
      }
      //device_nack(desc,server_seq_);
    }else{
      //��
      if(ack_action != 0xff){
        *(config_flash+block_meter%0x1000 + 22) = 0x00;
        *(config_flash+block_meter%0x1000 + 23) = 0x00;
      }else{
        *(config_flash+block_meter%0x1000 + 22) = st_l;
        *(config_flash+block_meter%0x1000 + 23) = st_h;
      }
      device_ack(desc,server_seq_);   //return nack;
    }
    
    //�����úõ�Flash������д�뵽Flash�С�
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
      //Ĭ�ϵ�λ��ǰ
      *buf_frame++ = DATAFLAG_WV_L;
      *buf_frame++ = DATAFLAG_WV_H;
    }else{
      // ���շ��ر�ʹ��  �е�
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
          //����ʧ��  ��flash  return nack
          success = 0;
        }
        continue;
      }
      //���յ���ȷ������
      if(ack_action != 0xff){
        OSMemPut(&MEM_Buf,buf_readdata,&err);
        OSTimeDly(12000,
                  OS_OPT_TIME_DLY,
                  &err);
        success = 1;
      }else{
        //�жϱ��ַ
        if(Mem_Cmp(meter_addr,buf_readdata+2,7) == DEF_YES){
          //��ȡST
          st_l = *(buf_readdata + 14);
          st_h = *(buf_readdata + 15);
          //���ڹط���ST�е����   D0 D1��ֻҪ��һ��Ϊ1����  
          //me��Э����ָ����D1 = 1
          //���գ�D0 = 1 Ϊ��
          if((st_l & 0x03) == 0x03){
            //����ʧ��  ��flash  return nack
            success = 0;
          }else{
            if((st_l & 0x02) == 0x02 || (st_l & 0x01) == 0x01){
              //opened  return ack
              success = 1;
            }else{
              //����ʧ��  ��flash  return nack
              success = 0;
            }
          }
        }
        OSMemPut(&MEM_Buf,buf_readdata,&err);
      }
    }
    
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  
    if(err != OS_ERR_NONE){
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    sFLASH_ReadBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
    
    if(success == 0){
      //����ʧ��  ��flash  return nack
      //Mem_Copy(configflash + 22,"\x43",1);  //��ʱ
      if((st_l & 0x03) == 0x03){
        //����ʧ��  ��flash  return nack
        //���Ż�
        *(config_flash+block_meter%0x1000 + 22) = st_l;
        *(config_flash+block_meter%0x1000 + 23) = st_h;
      }else{
        //��ʱ
        *(config_flash+block_meter%0x1000 + 22) = (*(config_flash+block_meter%0x1000 + 22)) | 0x40;
      }
      //device_nack(desc,server_seq_);  //return nack;
    }else{
      //�ط�
      if(ack_action != 0xff){
        *(config_flash+block_meter%0x1000 + 22) = 0x01;
        *(config_flash+block_meter%0x1000 + 23) = 0x00;
      }else{
        *(config_flash+block_meter%0x1000 + 22) = st_l;
        *(config_flash+block_meter%0x1000 + 23) = st_h;
      }
      device_ack(desc,server_seq_);   //return ack;
    }
    
    //�����úõ�Flash������д�뵽Flash�С�
    sFLASH_EraseSector((block_meter/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_meter/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
}

/*
���ȶ����������Flash����ȡ��  ��ȡ���ŵ�ǰ״̬

*/
void meter_clean(void){
    
  OS_ERR err;
  uint32_t block_cjq = 0;   //cjq block ��ַ
  uint32_t block_meter = 0;  //meter block ��ַ
  
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
    //û�вɼ���������
    return;
  }
  
  Device_Read(ENABLE);
  for(i = 0;i < cjq_count;i++){
    sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
    sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
    sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
    
    if(cjq_open(cjq_addr,block_cjq) == 0){
      //û�д򿪲ɼ���
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
          //Ĭ�ϵ�λ��ǰ
          *buf_frame++ = DATAFLAG_WV_L;
          *buf_frame++ = DATAFLAG_WV_H;
        }else{
          // ���շ��ر�ʹ��  �е�
          *buf_frame++ = DATAFLAG_WV_H;
          *buf_frame++ = DATAFLAG_WV_L;
        }
        
        *buf_frame++ = 0x01;
        *buf_frame++ = CLEAN_VALVE;
        *buf_frame++ = check_cs(buf_frame_,11+4);
        *buf_frame++ = FRAME_END;
        
        Slave_Write(fe,4);
        Slave_Write(buf_frame_,13+4);
        
        //��ȡ��һ����
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
        
        //��ʱ12s֮��ִ����һ��������
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
extern uint8_t device_test; //0x00~���Թ���~www.xcxdtech.com   0xFF~δ����~avenger0422.vicp.cc

void param_config(uint8_t * buf_frame,uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  uint8_t ip_port_[6];
  
  uint16_t i = 0;
  uint16_t cjq_count = 0;
  uint32_t block_cjq = 0;   //cjq block ��ַ
  uint32_t block_cjq_next = 0;   //cjq block ��ַɾ��ʱ  �Ȳ������һ���ĵ�ַ Ȼ����ɾ��
  uint32_t block_meter = 0;  //meter block ��ַ
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
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    //����Config Flash ��
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
    deviceaddr[4] = *(buf_frame + DATA_POSITION + 4);  //������Э���г��� Э���5λĬ��Ϊ0x00
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    
    if(err != OS_ERR_NONE){
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    //����Config Flash ��
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
        //ɾ����
        if(block_meter == 0xFFFFFF){
          //û������� do nothing
        }else{
          //�������  ɾ�������
          if(delete_meter(block_cjq,block_meter) == 0xFFFFFF){
            return;
          }
        }
        device_ack(desc,server_seq_);
      }
      
      if(*(buf_frame + DATA_POSITION + 16) == 0x01){
        //��ӱ�
        if(block_meter == 0xFFFFFF){
          //û������� ���
          if(add_meter(block_cjq,buf_frame + DATA_POSITION + 3) == 0xFFFFFF){
            return;
          }
        }else{
          //������� do nothing
        }
        device_ack(desc,server_seq_);
      }
    }
    
    break;
  case FN_CJQ:
    if(*(buf_frame + DATA_POSITION) == 0xAA){
      //ɾ��ȫ���ɼ���  ����ռ������еĲɼ�������Ϣ
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
      //���
      if(search_cjq(buf_frame + DATA_POSITION + 1) == 0xFFFFFF){
        //�������ɼ���
        if(add_cjq(buf_frame + DATA_POSITION + 1) == 0xFFFFFF){
          return;
        }
      }else{
        //�Ѿ�������ɼ�����
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
      //the slave is �ɼ���
      slave_mbus = 0xBB;
    }
    
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      //the slave is 485
      slave_mbus = 0xFF;
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  
    if(err != OS_ERR_NONE){
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    //����Config Flash ��
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_METER_MBUS - sFLASH_CON_START_ADDR),&slave_mbus,1);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    
    device_ack(desc,server_seq_);
    break;
  case FN_DI_SEQ:
    if(*(buf_frame + DATA_POSITION) == 0xAA){
      //ǧ��ͨʹ�õĴ��ģ��
      di_seq = 0xAA;
    }
    
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      //Ĭ��
      di_seq = 0xFF;
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    if(err != OS_ERR_NONE){
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    //����Config Flash ��
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_READMETER_DI_SEQ - sFLASH_CON_START_ADDR),&di_seq,1);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
    
    device_ack(desc,server_seq_);
    break;
  case FN_ACK_ACTION:
    if(*(buf_frame + DATA_POSITION) == 0xAA){
      //���շ��ر�ģʽ
      ack_action = 0xAA;
    }
    
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      //Ĭ��
      ack_action = 0xFF;
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    if(err != OS_ERR_NONE){
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    //����Config Flash ��
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
      //188Э��
      protocol = 0xFF;
    }
    
    OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
    if(err != OS_ERR_NONE){
      //��ȡMUTEX������ ������...
      //return 0xFFFFFF;
      return;
    }
    //����Config Flash ��
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
        //��ȡMUTEX������ ������...
        //return 0xFFFFFF;
        return;
      }
      //����Config Flash ��
      sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
      di_seq = 0xFF;
      Mem_Copy(config_flash + (sFLASH_POOL_INIT - sFLASH_CON_START_ADDR),&di_seq,1);
      sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
      
      device_ack(desc,server_seq_);
      *((uint8_t *)0) = 0x00;  //��ʹϵͳ����
    }
    break;
  case FN_RESET:
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      device_ack(desc,server_seq_);
      *((uint8_t *)0) = 0x00;  //��ʹϵͳ����
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
  
  meter_count++;  //�ɼ����µı�����++
  meter_all++;  //���б�����++
  
  block_new = GetFlash();  
  if(block_new == 0xFFFFFF){
    return 0xFFFFFF;
  }
  sFLASH_WritePage(meteraddr,block_new + 6,7);  //���ַ
  sFLASH_WritePage((uint8_t *)(meteraddr - 3),block_new + 13,1);  //������
  sFLASH_WritePage((uint8_t *)&meter_read,block_new + 14,4);  //�����  meter_read = 0
  sFLASH_WritePage((uint8_t *)&meter_read,block_new + 22,2);  //��״̬  meter_read = 0
  
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //��ȡMUTEX������ ������...
    return 0xFFFFFF;
  }
  
  if(block_last == 0xFFFFFF){
    //first meter
    sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
    
    //�ɼ�����Ŀ�ʼ�ͽ�β��ָ������ӵı�Ŀ�
    Mem_Copy(config_flash+block_cjq%0x1000 + 12,(uint8_t *)&block_new,3);
    Mem_Copy(config_flash+block_cjq%0x1000 + 15,(uint8_t *)&block_new,3);
    Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);  //�ɼ����µ���Ŀ++
    
    //�����úõ�Flash������д�뵽Flash�С�
    sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
  }else{
    //���ɼ�����Ľ�βָ������ӵı�Ŀ�
    sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
    Mem_Copy(config_flash+block_cjq%0x1000 + 15,(uint8_t *)&block_new,3);
    Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);  //�ɼ����µ���Ŀ++
    sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
    //ԭ�����һ�������һ����ָ������ӵı�Ŀ�
    sFLASH_WritePage((uint8_t *)&block_new,block_last + 3,3);  
    //����ӵı�Ŀ����һ���� ָ��ԭ�������һ����
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
    //��ȡMUTEX������ ������...
    return 0xFFFFFF;
  }
  
  if(meter_count == 0){
    //�ɼ�����Ψһ�ı�  block_before��block_after  ��Ϊ0xFFFFFF
    sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
    Mem_Copy(config_flash+block_cjq%0x1000 + 12,(uint8_t *)&block_after,3);
    Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);
    Mem_Copy(config_flash+block_cjq%0x1000 + 15,(uint8_t *)&block_before,3);
    sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
    sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
    
    //����ȫ������Ŀ
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
  }else{
    if(block_before == 0xFFFFFF || block_after == 0xFFFFFF){
      //Ҫɾ��������ǵ�һ��  ���������һ��
      if(block_before == 0xFFFFFF){
        //�޸ĺ�һ����before Ϊblock_before
        sFLASH_ReadBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
        Mem_Copy(config_flash+block_after%0x1000 + 18,(uint8_t *)&block_before,3);
        sFLASH_EraseSector((block_after/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //�޸Ĳɼ�����һ�����ַΪ  block_after  ���²ɼ�������Ŀ
        sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
        Mem_Copy(config_flash+block_cjq%0x1000 + 12,(uint8_t *)&block_after,3);
        Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);
        sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //����ȫ������Ŀ
        sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
        sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      }
      if(block_after == 0xFFFFFF){
        //�޸�ǰһ����next Ϊblock_after
        sFLASH_ReadBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
        Mem_Copy(config_flash+block_before%0x1000 + 3,(uint8_t *)&block_after,3);
        sFLASH_EraseSector((block_before/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //�޸Ĳɼ��������һ�����ַΪ  block_before  ���²ɼ�������Ŀ
        sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
        Mem_Copy(config_flash+block_cjq%0x1000 + 15,(uint8_t *)&block_before,3);
        Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);
        sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //����ȫ������Ŀ
        sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
        sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      }
    }else{
      //Ҫɾ����������м�
      //�޸�ǰһ����next Ϊblock_after
      sFLASH_ReadBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
      Mem_Copy(config_flash+block_before%0x1000 + 3,(uint8_t *)&block_after,3);
      sFLASH_EraseSector((block_before/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //�޸ĺ�һ����before Ϊblock_before
      sFLASH_ReadBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
      Mem_Copy(config_flash+block_after%0x1000 + 18,(uint8_t *)&block_before,3);
      sFLASH_EraseSector((block_after/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //���²ɼ�������Ŀ
      sFLASH_ReadBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
      Mem_Copy(config_flash+block_cjq%0x1000 + 18,(uint8_t *)&meter_count,2);
      sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_cjq/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //����ȫ������Ŀ
      sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
      Mem_Copy(config_flash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
      sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
    }
  }
  OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
  return block_meter;
  
}

//�вɼ���  ���ش˲ɼ�����block��ַ   û���򷵻�0xFFFFFF
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
  
  cjq_count++;  //�ɼ�������++
  
  //��ȡһ��flash��  ��������Ӧ��Ϣ
  block_new = GetFlash();  
  sFLASH_WritePage(cjqaddr,block_new + 6,6);  //�ɼ�����ַ
  sFLASH_WritePage((uint8_t *)&meter_count,block_new + 18,2);  //�ɼ�������  (uint8_t *)0 �ǵ�ַ0x00000000����ֵ��
  //��һ�������һ���ָ����0xFFFFFF
  
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //��ȡMUTEX������ ������...
    return 0xFFFFFF;
  }
  sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
  
  if(block_last == 0xFFFFFF){
    //this is the first cjq
    //cjq Q �Ŀ�ʼ�ͽ�β��ָ������ӵĲɼ�����
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
    Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
  }else{
    //���ǵ�һ��
    //��cjq Q �Ľ�βָ������ӵĲɼ�����
    Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
    //��ԭ�����һ���ɼ�������һ���ɼ���ָ������ӵĲɼ�����
    sFLASH_WritePage((uint8_t *)&block_new,block_last + 3,3);
    //������ӵĲɼ������е���һ���ɼ���  ָ��ԭ�������һ���ɼ���
    sFLASH_WritePage((uint8_t *)&block_last,block_new + 20,3);  //��һ���ɼ���
  }
  
  sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
  OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
  
  return block_new;
}

uint32_t delete_cjq(uint32_t block_cjq){
  uint16_t meter_count = 0;
  uint32_t block_meter = 0;
  uint32_t block_meter_next = 0;
  uint32_t block_after = 0;  //��һ���ɼ���
  uint32_t block_before = 0;  //��һ���ɼ���
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
  
  //������ɼ����µı�Q���  ���ɼ���ɾ��
  for(i = 0;i < meter_count;i++){
    
    sFLASH_ReadBuffer((uint8_t *)&block_meter_next,block_meter+3,3);  //��ȡ��һ�����block��ַ
    if(delete_meter(block_cjq,block_meter) == 0xFFFFFF){
      continue;
    }
    block_meter = block_meter_next;
  }
  //���ɼ���ɾ��
  PutFlash(block_cjq);
  
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //��ȡMUTEX������ ������...
    return 0xFFFFFF;
  }
  
  if(cjq_count == 0){
    //����ɼ�����Ψһ��һ��   block_before��block_after  ��Ϊ0xFFFFFF
    sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_after,3);
    Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    Mem_Copy(config_flash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_before,3);
    sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
  }else{
    if(block_before == 0xFFFFFF || block_after == 0xFFFFFF){
      //Ҫɾ��������ǵ�һ��  ���������һ��
      if(block_before == 0xFFFFFF){
        //�޸ĺ�һ����before Ϊblock_before
        sFLASH_ReadBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
        Mem_Copy(config_flash+block_after%0x1000 + 20,(uint8_t *)&block_before,3);
        sFLASH_EraseSector((block_after/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //�޸Ĳɼ���Q��start Ϊblock_after  ���²ɼ�������
        sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(config_flash + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_after,3);
        Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
        sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      }
      
      if(block_after == 0xFFFFFF){
        //�޸�ǰһ����next Ϊblock_after
        sFLASH_ReadBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
        Mem_Copy(config_flash+block_before%0x1000 + 3,(uint8_t *)&block_after,3);
        sFLASH_EraseSector((block_before/0x1000)*0x1000);
        sFLASH_WriteBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
        //�޸Ĳɼ���Q��end Ϊblock_before  ���²ɼ�������
        sFLASH_ReadBuffer(config_flash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(config_flash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
        Mem_Copy(config_flash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_before,3);
        sFLASH_EraseWritePage(config_flash,sFLASH_CON_START_ADDR,256);
      }
      
    }else{
      //Ҫɾ����������м�
      //�޸�ǰһ����next Ϊblock_after
      sFLASH_ReadBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
      Mem_Copy(config_flash+block_before%0x1000 + 3,(uint8_t *)&block_after,3);
      sFLASH_EraseSector((block_before/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_before/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //�޸ĺ�һ����before Ϊblock_before
      sFLASH_ReadBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);  //��ȡ����Sector
      Mem_Copy(config_flash+block_after%0x1000 + 20,(uint8_t *)&block_before,3);
      sFLASH_EraseSector((block_after/0x1000)*0x1000);
      sFLASH_WriteBuffer(config_flash,(block_after/0x1000)*0x1000,sFLASH_SECTOR_SIZE);
      //�޸�control blocks
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
  uint8_t readingbeat[17];  //��ȫ����ʱ������
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
  uint16_t times_;      //һ��Ҫ���Ͷ���֡
  uint16_t times_count; //�����˶���֡��
  
  uint16_t len = 0;  //��ǰ֡�����ݳ���
  
  uint32_t block_cjq;
  uint32_t block_meter;
  
  uint8_t cjq_addr[6];
  uint8_t meter_addr[7];
  uint8_t meter_type;   //�������
  uint8_t meter_status; //���״̬
  uint8_t meter_fount = 0;  //�Ƿ��ҵ��������
  
  uint16_t i = 0;       //�����ɼ���
  uint16_t j = 0;       //�����ɼ����µı�
  uint8_t k = 0;        //����copy�ɼ��������ַ
  
  uint16_t meter_count = 0;  //����һ֡�е����������
  uint16_t meter_count_ = 0;    //����һ֡��������ĸ���
  uint8_t header = 0;   //һ֡��֡ͷ�Ƿ���׼��
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&allmeter_count,sFLASH_METER_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
  if(cjq_count == 0){
    //û�вɼ���������
    return;
  }
  
  buf_frame = OSMemGet(&MEM_Buf,&err);
  if(buf_frame == 0){
    return;
  }
  
  buf_frame_ = buf_frame;
  
  if(Mem_Cmp(meteraddr,"\xFF\xFF\xFF\xFF\xFF\xFF\xFF",7) == DEF_YES){
    //ȫ����
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
            //��֡
            meter_count = allmeter_count;
            meter_count_ = meter_count;
            len = ((9+17*meter_count_+1) << 2) | 0x03;    //+1  because of  *buf_frame++ = metertype;
            
          }else{
            //��֡
            if(times_count == 1){
              //��֡
              meter_count = 10;
              meter_count_ = meter_count;
              len = ((9+17*meter_count_+1) << 2) | 0x03;
              
            }else{
              if(times_count == times_){
                //β֡
                if(remain == 0){
                  meter_count = 10; 
                  meter_count_ = meter_count;
                }else{
                  meter_count = remain;
                  meter_count_ = meter_count;
                }
                len = ((9+17*meter_count_+1) << 2) | 0x03;
                
              }else{
                //�м�֡
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
            //��֡
            *buf_frame++ = ZERO_BYTE |SINGLE | server_seq_;
          }else{
            //��֡
            if(times_count == 1){
              //��֡
              *buf_frame++ = ZERO_BYTE |MUL_FIRST | server_seq_;
            }else{
              if(times_count == times_){
                //β֡
                *buf_frame++ = ZERO_BYTE |MUL_LAST | server_seq_;
              }else{
                //�м�֡
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
          header = 0;   //Ϊ��һ֡��׼��
          //������һ֡
          
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
    //������
    for(i = 0;meter_fount == 0 && i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      for(j=0;j < cjqmeter_count;j++){
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
        
        if(Mem_Cmp(meteraddr,meter_addr,7) == DEF_YES && meter_type == metertype){
          //�ҵ�������ˡ�����
          sFLASH_ReadBuffer((uint8_t *)&meter_status,block_meter+21,1);
          meter_fount = 1; 
          break;
        }
        
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
      }
    }
    
    if(meter_fount){
      //�ҵ��������
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