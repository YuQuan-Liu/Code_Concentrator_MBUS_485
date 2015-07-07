

#include "os.h"
#include "stm32f10x_conf.h"
#include "tasks.h"
#include "lib_str.h"
#include "serial.h"
#include "spi_flash.h"
#include "m590e.h"
#include "frame.h"
#include "frame_188.h"

//#include "stdlib.h"

extern OS_MEM MEM_Buf;
extern OS_MEM MEM_ISR;

extern OS_Q Q_Server;        //���������͹���������
extern OS_Q Q_Slave;            //�ɼ��������͹���������
extern OS_Q Q_Read;             //��������Queue
extern OS_Q Q_ReadData;        //���ͳ���ָ���  �²㷵�س�������
extern OS_Q Q_Config;         //��������Queue
extern OS_Q Q_Deal;         //������յ��ķ��������͹���������

extern OS_SEM SEM_HeartBeat;    //���շ���������Task to HeartBeat Task  ���յ������Ļ�Ӧ
extern OS_SEM SEM_Restart;      //HeartBeat Task to Connection Task  ����ģ��
extern OS_SEM SEM_Connected;   //m590e online  
extern OS_SEM SEM_Send_Online;   //��������ʱ�����·״̬  "+IPSTATUS:0,CONNECT,TCP"
extern OS_SEM SEM_SendOver;      //got the "+TCPSEND:0,"  the data is send over now  �����������
extern OS_SEM SEM_Send;      //got the '>'  we can send the data now  ���Է�������

extern OS_TMR TMR_Server;
extern OS_TMR TMR_Slave;
extern OS_TMR TMR_Server_200;      //200ms

extern volatile uint8_t reading;
extern volatile uint8_t connectstate; 
extern uint8_t deviceaddr[5];

extern uint8_t slave_mbus; //0xaa mbus   0xff  485


uint8_t * buf = 0;   //the buf used put the data in 
uint8_t * buf_;       //keep the buf's ptr  used to release the buf
uint8_t start_slave = 0;
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
    if(buf == 0){
      buf = OSMemGet(&MEM_Buf,
                     &err);
      buf_ = buf;
      Mem_Set(buf_,0x00,256); //clear the buf
      if(err != OS_ERR_NONE){
        asm("NOP");
        //didn't get the buf
      }
    }
    
    mem_ptr = OSQPend(&Q_Slave,0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
    data = *mem_ptr;
    OSMemPut(&MEM_ISR,mem_ptr,&err);
    
    if(reading){
      //it is the frame come from the meter
      if(start_slave == 0){
        if(data == 0x68){
          *buf++ = data;
          frame_len = 0;
          start_slave = 1;
          //start the tmr_slave
          OSTmrStart(&TMR_Slave,
                     &err);
          if(err != OS_ERR_NONE){
            asm("NOP");
          }
        }else{
          //do nothing
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
              asm("NOP");
              OSTmrStop(&TMR_Slave,
                          OS_OPT_TMR_NONE,
                          0,
                          &err);
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
          }
        }
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
          //start the tmr_slave
          OSTmrStart(&TMR_Slave,
                     &err);
          if(err != OS_ERR_NONE){
            asm("NOP");
          }
        }else{
          //do nothing
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
                asm("NOP");
                OSTmrStop(&TMR_Slave,
                          OS_OPT_TMR_NONE,
                          0,
                          &err);
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

void Tmr_SlaveCallBack(OS_TMR *p_tmr, void *p_arg){
  
  buf = buf_;
  start_slave = 0;
  
}



uint8_t check_cs(uint8_t * start,uint16_t len){
  uint16_t i;
  uint8_t cs = 0;
  for(i = 0;i < len;i++){
    cs += *(start+i);
  }
  return cs;
}



volatile uint8_t q_server_pending = 0;  //ֻ����Ҫ���շ���������ʱ��������Q_Server��post
void Task_Server(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint8_t data = 0;
  uint8_t * mem_ptr = 0;
  uint16_t msg_size = 0;
  
  uint8_t * buf_server_task = 0;
  uint8_t * buf_server_task_ = 0;
  
  while(DEF_TRUE){
    
    OSSemPend(&SEM_Connected,
              0,
              OS_OPT_PEND_BLOCKING,
              &ts,
              &err);
    while(connectstate == 1){
      if(q_server_pending == 0){
        buf_server_task = 0;
        buf_server_task_ = 0;
        Server_Post2Queue(ENABLE);
      }
      
      if(buf_server_task == 0){
        buf_server_task = OSMemGet(&MEM_Buf,
                       &err);
        buf_server_task_ = buf_server_task;
        Mem_Set(buf_server_task_,0x00,256); //clear the buf
        if(err != OS_ERR_NONE){
          asm("NOP");
          //didn't get the buf
        }
      }
      mem_ptr = OSQPend(&Q_Server,0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
      data = *mem_ptr;
      OSMemPut(&MEM_ISR,mem_ptr,&err);
      
      
      OSTmrStart(&TMR_Server_200,&err);
      while(OSTmrStateGet(&TMR_Server_200,&err) == OS_TMR_STATE_RUNNING){
        mem_ptr = OSQPend(&Q_Server,10,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
        if(mem_ptr != 0){
          data = *mem_ptr;
          OSMemPut(&MEM_ISR,mem_ptr,&err);
          *buf_server_task++ = data;
          if((buf_server_task-buf_server_task_) == 250){
            //the buf is full
            break;
          }
        }
      }
      check_str(buf_server_task_,buf_server_task);  //���ε�����ǰ��0x00
      if(Str_Str(buf_server_task_,"\n>")){
        OSSemPost(&SEM_Send,
                  OS_OPT_POST_1,
                  &err);
        buf_server_task = buf_server_task_;
        Mem_Set(buf_server_task_,0x00,256); //clear the buf
        continue;
      }
      
      if(Str_Str(buf_server_task_,"+IPSTATUS:0,CONNECT,TCP,")){
        OSSemPost(&SEM_Send_Online,
                  OS_OPT_POST_1,
                  &err);
        buf_server_task = buf_server_task_;
        Mem_Set(buf_server_task_,0x00,256); //clear the buf
        continue;
      }
      
      if(Str_Str(buf_server_task_,"+IPSTATUS:0,DISCONNECT")){
        Mem_Set(buf_server_task_,0x00,256); //clear the buf
        OSMemPut(&MEM_Buf,buf_server_task_,&err);
        
        buf_server_task = 0;
        buf_server_task_ = 0;
        //connectstate = 0;
        change_connect(0);
        Server_Post2Queue(DISABLE);
        //OSSemPost(&SEM_Restart,
        //          OS_OPT_POST_1,
        //          &err);
        break;
      }
      
      if(Str_Str(buf_server_task_,"+TCPSEND:0,") && !Str_Str(buf_server_task_,"+TCPSEND:0,-1")){
        OSSemPost(&SEM_SendOver,
                  OS_OPT_POST_1,
                  &err);
        buf_server_task = buf_server_task_;
        Mem_Set(buf_server_task_,0x00,256); //clear the buf
        continue;
      }
      
      if(Str_Str(buf_server_task_,"+TCPRECV:0")){
        //oh it's the data 
        OSQPost(&Q_Deal,
                buf_server_task_,
                buf_server_task-buf_server_task_,
                OS_OPT_POST_FIFO,
                &err);
        buf_server_task = 0;
        continue;
      }
      
      //don't know what's that
      buf_server_task = buf_server_task_;
      Mem_Set(buf_server_task_,0x00,256); //clear the buf
      
    }
  }
  
}

void Tmr_ServerCallBack(OS_TMR *p_tmr, void *p_arg){
  //do nothing
}

void Task_DealServer(void *p_arg){
  CPU_TS ts;
  OS_ERR err;
  uint8_t * buf_copy = 0;
  uint8_t * buf_ptr_ = 0;
  uint16_t msg_size = 0;
  
  uint8_t * start = 0;
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
    
    start = Str_Str(buf_ptr_,",\x68") + 1;
    //end = Str_Str(buf_ptr_,"\x16\r\n");  �����п���Ҫ����0x00��
    
    //check the frame
    len = check_frame(start);
    /*
    if(*(start+CON_POSITION) == 0xCA){
      //����1������  ����Ӧ�ò�����ȷ�ϵ���·���䡣
      //����������ʱ  �������ظ�ȷ��  ���������ߡ�
      device_ack(1);
    }else{
      
    }
    */
      if(len){
        //the frame is ok
        switch(*(start+AFN_POSITION)){
        case AFN_ACK:
          if(*(start+CON_POSITION) == SLAVE_FUN_TEST){
            //the ack of the heart beat
            OSSemPost(&SEM_HeartBeat,
                      OS_OPT_POST_1,
                      &err);
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
          buf_copy = OSMemGet(&MEM_Buf,&err);
          if(buf_copy != 0){
            Mem_Copy(buf_copy,start,len);
            *(buf_copy + len) = 0x01;  //��ʶ��һ֡���Է�����
            OSQPost(&Q_Read,buf_copy,len,OS_OPT_POST_1,&err);
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

void Task_Connect(void *p_arg){
  uint8_t fail_count = 0;
  CPU_TS ts;
  OS_ERR err;
  uint8_t needdisable = 0;
  
  while(DEF_TRUE){
    
    while(connectstate == 0){
      M590E_Cmd(DISABLE);
      M590E_Cmd(ENABLE);
      while(connect() == ERROR){
        fail_count++;
        if(fail_count%3 == 0){
          fail_count = 0;
          M590E_Cmd(DISABLE);
          M590E_Cmd(ENABLE);
        }
      }
    }
    OSTimeDlyHMSM(0,1,0,0,
                 OS_OPT_TIME_HMSM_STRICT,
                 &err);
  }
}

void Task_HeartBeat(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint8_t * buf_frame;
  //uint16_t * buf_frame_16;
  uint8_t i;
  uint8_t beat[17];
  
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
  *buf_frame++ = ZERO_BYTE |SINGLE | CONFIRM;
  *buf_frame++ = FN_HEARTBEAT;
  
  *buf_frame++ = check_cs(beat+6,9);
  *buf_frame++ = FRAME_END;
  
  while(DEF_TRUE){
    if(connectstate == 0){
      OSTimeDlyHMSM(0,0,10,0,
                    OS_OPT_TIME_HMSM_STRICT,
                    &err);
    }else{
      
      send_server(beat,17);
      OSSemPend(&SEM_HeartBeat,
                2000,
                OS_OPT_PEND_BLOCKING,
                &ts,
                &err);
      if(err == OS_ERR_NONE){
        OSTimeDlyHMSM(0,2,0,0,
                OS_OPT_TIME_HMSM_STRICT,
                &err);
      }else{
        change_connect(0);
      }
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
    //�򿪵�Դ
    if(slave_mbus == 0xAA){
      mbus_power(ENABLE);
    }else{
      relay_485(ENABLE);
    }
    
    switch(*(buf_frame+AFN_POSITION)){
      case AFN_CONTROL:
        meter_control(buf_frame,*(buf_frame+msg_size));
        break;
      case AFN_CURRENT:
        meter_read(buf_frame,*(buf_frame+msg_size));
        break;
    }
    //�رյ�Դ
    if(slave_mbus == 0xAA){
      mbus_power(DISABLE);
    }else{
      relay_485(DISABLE);
    }
    
    OSMemPut(&MEM_Buf,buf_frame,&err);
  }
}

uint8_t mbus_power(FunctionalState NewState){
  /**/
  OS_ERR err;
  if(NewState != DISABLE){
    GPIO_SetBits(GPIOA,GPIO_Pin_12);
    OSTimeDlyHMSM(0,0,1,500,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
  }else{
    GPIO_ResetBits(GPIOA,GPIO_Pin_12);
  }
  
}

uint8_t relay_485(FunctionalState NewState){
  /**/
  OS_ERR err;
  if(NewState != DISABLE){
   GPIO_SetBits(GPIOA,GPIO_Pin_7);
   OSTimeDlyHMSM(0,0,0,600,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
  }else{
    GPIO_ResetBits(GPIOA,GPIO_Pin_7);
  }
  
}

uint8_t relay_1(FunctionalState NewState){
  OS_ERR err;
  if(NewState != DISABLE){
   GPIO_SetBits(GPIOB,GPIO_Pin_2);
   OSTimeDlyHMSM(0,0,2,0,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
  }else{
    GPIO_ResetBits(GPIOB,GPIO_Pin_2);
  }
}
uint8_t relay_2(FunctionalState NewState){
  OS_ERR err;
  if(NewState != DISABLE){
   GPIO_SetBits(GPIOB,GPIO_Pin_1);
   OSTimeDlyHMSM(0,0,2,0,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
  }else{
    GPIO_ResetBits(GPIOB,GPIO_Pin_1);
  }
}
uint8_t relay_3(FunctionalState NewState){
  OS_ERR err;
  if(NewState != DISABLE){
   GPIO_SetBits(GPIOB,GPIO_Pin_0);
   OSTimeDlyHMSM(0,0,2,0,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
  }else{
    GPIO_ResetBits(GPIOB,GPIO_Pin_0);
  }
}
uint8_t relay_4(FunctionalState NewState){
  OS_ERR err;
  if(NewState != DISABLE){
   GPIO_SetBits(GPIOA,GPIO_Pin_4);
   OSTimeDlyHMSM(0,0,2,0,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
  }else{
    GPIO_ResetBits(GPIOA,GPIO_Pin_4);
  }
}

/*
����ɼ�����ַΪ FF FF FF FF FF FF ��ʾ����������ֱ�ӽӵı�

���������ʱ��
�ȷ��ʹ�ָ���ɼ���͸������  ���յ���Ӧ֮��
���Է��ͳ�������ָ��
ָ��ʱ����û���յ����صı��ָ��  �ظ�3��

�ص�ָ���ɼ�����͸������ 

*/
void meter_read(uint8_t * buf_frame,uint8_t desc){
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
  uint8_t meter_status[2];
  
  uint8_t meter_fount = 0;
  uint8_t * configflash = 0;
  //��ѯ�Ƿ��������
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
  if(cjq_count == 0){
    //û�вɼ���������
    return;
  }
  
  //return ack
  if(Mem_Cmp(buf_frame+16,"\xFF\xFF\xFF\xFF\xFF\xFF\xFF",7) == DEF_YES){
    //��ȫ����
    for(i = 0;i < cjq_count;i++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
      sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
      
      //�򿪲ɼ���͸��   ����ֱ�ӽӱ�ʱ  �ɼ����ĵ�ַΪ"\xFF\xFF\xFF\xFF\xFF\xFF"
      if(Mem_Cmp(cjq_addr,"\xFF\xFF\xFF\xFF\xFF\xFF",6) == DEF_NO){
        if(cjq_open(cjq_addr,block_cjq) == 0){
          //�ɼ���û�д򿪣�
          //��¼��ǰ�ɼ��������б�  �ɼ�����ʱ
          configflash = OSMemGet(&MEM_Buf,&err);
          for(j=0;j < cjqmeter_count;j++){
            sFLASH_ReadBuffer(configflash,block_meter,256);
            *(configflash + 22) = (*(configflash + 22)) | 0x80;
            sFLASH_EraseWritePage(configflash,block_meter,256);
            
            sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
          }
          OSMemPut(&MEM_Buf,configflash,&err);
          continue;
        }
      }
      
      Device_Read(ENABLE);
      for(j=0;j < cjqmeter_count;j++){
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
        
        meter_read_single(meter_addr,block_meter,meter_type,desc);
        
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);
      }
      Device_Read(DISABLE);
      
      if(Mem_Cmp(cjq_addr,"\xFF\xFF\xFF\xFF\xFF\xFF",6) == DEF_NO){
        //�رղɼ���͸��
        cjq_close(cjq_addr,block_cjq);
      }
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+3,3);
    }
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
          
          //�򿪲ɼ���͸��   ����ֱ�ӽӱ�ʱ  �ɼ����ĵ�ַΪ"\xFF\xFF\xFF\xFF\xFF\xFF"
          if(Mem_Cmp(cjq_addr,"\xFF\xFF\xFF\xFF\xFF\xFF",6) == DEF_NO){
            if(cjq_open(cjq_addr,block_cjq) == 0){
              //�ɼ���û�д򿪣�
              //��¼��ǰ��ɼ�����ʱ
              //continue;
              configflash = OSMemGet(&MEM_Buf,&err);
              sFLASH_ReadBuffer(configflash,block_meter,256);
              *(configflash + 22) = (*(configflash + 22)) | 0x80;
              sFLASH_EraseWritePage(configflash,block_meter,256);
              OSMemPut(&MEM_Buf,configflash,&err);
              //device_nack(desc);
              
              meter_fount = 1; 
              break;
            }
          }
          Device_Read(ENABLE);
          meter_read_single(meter_addr,block_meter,meter_type,desc);
          Device_Read(DISABLE);
          //send the data;
          meter_send(0,block_meter,desc);
          
          if(Mem_Cmp(cjq_addr,"\xFF\xFF\xFF\xFF\xFF\xFF",6) == DEF_NO){
            //�رղɼ���͸��
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
    uint8_t * configflash = 0;
    uint8_t * buf_frame = 0;
    
    buf_frame = buf_frame_;
    *buf_frame++ = FRAME_HEAD;
    *buf_frame++ = meter_type;
    for(i=0;i<7;i++){
      *buf_frame++ = *(meter_addr + i);
    }
    *buf_frame++ = 0x01; //C
    *buf_frame++ = 0x03; //len
    *buf_frame++ = DATAFLAG_RD_L;
    *buf_frame++ = DATAFLAG_RD_H;
    *buf_frame++ = 0x01;
    *buf_frame++ = check_cs(buf_frame_,11+3);
    *buf_frame++ = FRAME_END;
    
    for(i =0;i<4;i++){
      read[i] = 0xFE;
    }
    
    for(i = 0;success == 0 && i < 3;i++){
      Slave_Write(read,4);
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
        for(j = 0;j < 4;j++){
          read[j] = *(buf_readdata + 14 + j);
          half[j] = *(buf_readdata + 19 + j);
        }
      }
      
      OSMemPut(&MEM_Buf,buf_readdata,&err);
    }
    configflash = OSMemGet(&MEM_Buf,&err);
    sFLASH_ReadBuffer(configflash,block_meter,256);
    if(success == 0){
      //����ʧ��  ��flash  return nack
      //Mem_Copy(configflash + 22,"\x40",1);  //��ʱ
      *(configflash + 22) = (*(configflash + 22)) | 0x40;
      sFLASH_EraseWritePage(configflash,block_meter,256);
    }else{
      Mem_Copy(configflash + 22,&st_l,1);  //��¼st��Ϣ
      Mem_Copy(configflash + 14,read,4);        //����
      Mem_Copy(configflash + 24,half,4);        //��λ
      sFLASH_EraseWritePage(configflash,block_meter,256);
    }
    OSMemPut(&MEM_Buf,configflash,&err);
}

//all = 1 ����ȫ����  all = 0 ���ͱ���Ӧ�ı�
void meter_send(uint8_t all,uint32_t block_meter_,uint8_t desc){
  OS_ERR err;
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
  uint8_t meter_status = 0; //���״̬
  
  uint16_t i = 0;       //�����ɼ���
  uint16_t j = 0;       //�����ɼ����µı�
  uint8_t k = 0;        //����copy�ɼ��������ַ
  
  uint16_t meter_count = 0;  //����һ֡�е����������
  uint16_t meter_count_ = 0;    //����һ֡��������ĸ���
  uint8_t header = 0;   //һ֡��֡ͷ�Ƿ���׼��
  
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
    sFLASH_ReadBuffer((uint8_t *)&meter_status,block_meter+22,1);
    
    *buf_frame++ = FRAME_HEAD;
    buf_frame_16 = (uint16_t *)buf_frame;
    *buf_frame_16++ = 0x63;//((9+14+1) << 2) | 0x03;
    *buf_frame_16++ = 0x63;//((9+14+1) << 2) | 0x03;
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
    *buf_frame++ = ZERO_BYTE |SINGLE ;
    *buf_frame++ = FN_CURRENT_METER;
    
    *buf_frame++ = meter_type;
    
    for(i=0;i<7;i++){
      *buf_frame++ = meter_addr[i];
    }
    *buf_frame++ = 0x01;
    for(i=0;i<4;i++){
      *buf_frame++ = meter_read[i];
    }
    *buf_frame++ = meter_status;
    *buf_frame++ = 0x00;
    
    *buf_frame++ = check_cs(buf_frame_+6,24);
    *buf_frame++ = FRAME_END;
    
    if(desc){
      //to m590e
      send_server(buf_frame_,32);
    }else{
      //to 485
      Slave_Write(buf_frame_,32);
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
        sFLASH_ReadBuffer((uint8_t *)&meter_status,block_meter+22,1);
        
        if(header == 0){
          header = 1;
          times_count++;
          if(times_ == 1){
            //��֡
            meter_count = allmeter_count;
            meter_count_ = meter_count;
            
            *buf_frame++ = FRAME_HEAD;
            buf_frame_16 = (uint16_t *)buf_frame;
            *buf_frame_16++ = ((9+14*meter_count_+1) << 2) | 0x03;    //+1  because of  *buf_frame++ = metertype;
            *buf_frame_16++ = ((9+14*meter_count_+1) << 2) | 0x03;
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
            *buf_frame++ = ZERO_BYTE |SINGLE ;
            *buf_frame++ = FN_CURRENT_METER;
            
            *buf_frame++ = meter_type;
            
          }else{
            //��֡
            if(times_count == 1){
              //��֡
              meter_count = 10;
              meter_count_ = meter_count;
              
              *buf_frame++ = FRAME_HEAD;
              buf_frame_16 = (uint16_t *)buf_frame;
              *buf_frame_16++ = ((9+14*meter_count_+1) << 2) | 0x03;
              *buf_frame_16++ = ((9+14*meter_count_+1) << 2) | 0x03;
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
              *buf_frame++ = ZERO_BYTE |MUL_FIRST ;
              *buf_frame++ = FN_CURRENT_METER;
              
              *buf_frame++ = meter_type;
              
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
                
                *buf_frame++ = FRAME_HEAD;
                buf_frame_16 = (uint16_t *)buf_frame;
                *buf_frame_16++ = ((9+14*meter_count_+1) << 2) | 0x03;
                *buf_frame_16++ = ((9+14*meter_count_+1) << 2) | 0x03;
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
                *buf_frame++ = ZERO_BYTE |MUL_LAST ;
                *buf_frame++ = FN_CURRENT_METER;
                
                *buf_frame++ = meter_type;
                
              }else{
                //�м�֡
                meter_count = 10;
                meter_count_ = meter_count;
                
                *buf_frame++ = FRAME_HEAD;
                buf_frame_16 = (uint16_t *)buf_frame;
                *buf_frame_16++ = ((9+14*meter_count_+1) << 2) | 0x03;
                *buf_frame_16++ = ((9+14*meter_count_+1) << 2) | 0x03;
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
                *buf_frame++ = ZERO_BYTE |MUL_MIDDLE ;
                *buf_frame++ = FN_CURRENT_METER;
                
                *buf_frame++ = meter_type;
              }
            }
          }
        }
        
        for(k=0;k<7;k++){
          *buf_frame++ = meter_addr[k];
        }
        *buf_frame++ = 0x01;
        for(k=0;k<4;k++){
          *buf_frame++ = meter_read[k];
        }
        *buf_frame++ = meter_status;
        *buf_frame++ = 0x00;
        
        meter_count--;
        if(meter_count == 0){
          header = 0;   //Ϊ��һ֡��׼��
          //������һ֡
          
          *buf_frame++ = check_cs(buf_frame_+6,9+14*meter_count_+1);
          *buf_frame++ = FRAME_END;
          
          if(desc){
            //to m590e
            send_server(buf_frame_,17+14*meter_count_+1);
          }else{
            //to 485
            Slave_Write(buf_frame_,17+14*meter_count_+1);
          }
          
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
  
  uint32_t block_cjq = 0;   //cjq block ��ַ
  uint32_t block_meter = 0;  //meter block ��ַ
  
  uint16_t cjq_count = 0;
  uint16_t cjqmeter_count = 0;
  
  uint16_t i = 0;
  uint16_t j = 0;
  
  uint8_t cjq_addr[6];
  uint8_t meter_addr[7];
  uint8_t meter_type = 0;
  uint8_t meter_status[2];
  
  uint8_t meter_fount = 0;
  uint8_t * configflash = 0;
  //��ѯ�Ƿ��������
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
  if(cjq_count == 0){
    //û�вɼ���������
    //todo ��Ӧ NACK
    return;
  }
  for(i = 0;meter_fount == 0 && i < cjq_count;i++){
    sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+6,6);
    sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
    sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+18,2);
    
    for(j=0;j < cjqmeter_count;j++){
      sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+6,7);
      sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+13,1);
      
      if(Mem_Cmp(buf_frame + 16,meter_addr,7) == DEF_YES && meter_type == *(buf_frame + 15)){
        //�ҵ�������ˡ�����
        
        //�򿪲ɼ���͸��   ����ֱ�ӽӱ�ʱ  �ɼ����ĵ�ַΪ"\xFF\xFF\xFF\xFF\xFF\xFF"
        if(Mem_Cmp(cjq_addr,"\xFF\xFF\xFF\xFF\xFF\xFF",6) == DEF_NO){
          if(cjq_open(cjq_addr,block_cjq) == 0){
            //�ɼ���û�д򿪣� ����nack 
            //��¼��ǰ��ɼ�����ʱ  todo...
            
            configflash = OSMemGet(&MEM_Buf,&err);
            sFLASH_ReadBuffer(configflash,block_meter,256);
            *(configflash + 22) = (*(configflash + 22)) | 0x80;
            sFLASH_EraseWritePage(configflash,block_meter,256);
            OSMemPut(&MEM_Buf,configflash,&err);
            //device_nack(desc);
            
            meter_fount = 1; 
            break;
          }
        }
        
        sFLASH_ReadBuffer((uint8_t *)&meter_status,block_meter+22,2);
        
        switch(*(buf_frame + FN_POSITION)){
        case FN_OPEN:
          meter_open(meter_addr,block_meter,meter_type,desc);
          break;
        case FN_CLOSE:
          meter_close(meter_addr,block_meter,meter_type,desc);
          break;
        }
        
        if(Mem_Cmp(cjq_addr,"\xFF\xFF\xFF\xFF\xFF\xFF",6) == DEF_NO){
          //�رղɼ���͸��
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


uint8_t cjq_open(uint8_t * cjq_addr,uint32_t block_cjq){
  
    OS_ERR err;
    CPU_TS ts;
    uint8_t buf_frame_[17];
    uint8_t i = 0;
    uint16_t msg_size = 0;
    uint8_t * buf_readdata = 0;
    uint8_t success = 0;
    uint8_t st_l = 0;
    uint8_t * configflash = 0;
    uint8_t * buf_frame = 0;
    
    
    if(slave_mbus == 0xAA){
      //mbus ~~~
      switch (cjq_addr[0]){
        case 1:
          relay_1(ENABLE);
          break;
        case 2:
          relay_2(ENABLE);
          break;
        case 3:
          relay_3(ENABLE);
          break;
        case 4:
          relay_4(ENABLE);
          break;
      }
      return 1;
    }else{
      //485 ~~~
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
      
      Device_Read(ENABLE);
      for(i = 0;success == 0 && i < 3;i++){
        Slave_Write(buf_frame_,13+4);
        buf_readdata = OSQPend(&Q_ReadData,3000,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
        if(err != OS_ERR_NONE){
          if(i==2){
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
        configflash = OSMemGet(&MEM_Buf,&err);
        sFLASH_ReadBuffer(configflash,block_cjq,256);
        Mem_Copy(configflash + 23,"\x01",1);  //��ʱ
        sFLASH_EraseWritePage(configflash,block_cjq,256);
        
        OSMemPut(&MEM_Buf,configflash,&err);
        
      }
      Device_Read(DISABLE);
      
      return success;
    }
}

uint8_t cjq_close(uint8_t * cjq_addr,uint32_t block_cjq){
  
    OS_ERR err;
    CPU_TS ts;
    uint8_t buf_frame_[17];
    uint8_t i;
    uint16_t msg_size;
    uint8_t * buf_readdata;
    uint8_t success = 0;
    uint8_t st_l;
    uint8_t * configflash;
    uint8_t * buf_frame;
    
    if(slave_mbus == 0xAA){
      //mbus ~~~
      switch (cjq_addr[0]){
        case 1:
          relay_1(DISABLE);
          break;
        case 2:
          relay_2(DISABLE);
          break;
        case 3:
          relay_3(DISABLE);
          break;
        case 4:
          relay_4(DISABLE);
          break;
      }
      return 1;
    }else{
      //485 ~~~
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
      
      Device_Read(ENABLE);
      for(i = 0;success == 0 && i < 3;i++){
        Slave_Write(buf_frame_,13+4);
        buf_readdata = OSQPend(&Q_ReadData,3000,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
        if(err != OS_ERR_NONE){
          if(i==2){
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
            //�زɼ���ʧ��  ��flash  return nack
            success = 0;
          }
        }
        
        OSMemPut(&MEM_Buf,buf_readdata,&err);
        
      }
      
      if(success == 0){
        //�زɼ���ʧ��  ��flash  return nack
        configflash = OSMemGet(&MEM_Buf,&err);
        sFLASH_ReadBuffer(configflash,block_cjq,256);
        Mem_Copy(configflash + 23,"\x01",1);  //��ʱ
        sFLASH_EraseWritePage(configflash,block_cjq,256);
        
        OSMemPut(&MEM_Buf,configflash,&err);
        
      }
      Device_Read(DISABLE);
      
      return success;
    }
    
}

void meter_open(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc){
    OS_ERR err;
    CPU_TS ts;
    uint8_t buf_frame_[17];
    uint8_t fe[4];
    uint8_t i = 0;
    uint16_t msg_size = 0;
    uint8_t * buf_readdata = 0;
    uint8_t success = 0;
    uint8_t st_l = 0;
    uint8_t * configflash = 0;
    uint8_t * buf_frame = 0;
    
    buf_frame = buf_frame_;
    *buf_frame++ = FRAME_HEAD;
    *buf_frame++ = meter_type;
    for(i=0;i<7;i++){
      *buf_frame++ = *(meter_addr + i);
    }
    *buf_frame++ = 0x04; //C
    *buf_frame++ = 0x04; //len
    *buf_frame++ = DATAFLAG_WV_L;
    *buf_frame++ = DATAFLAG_WV_H;
    *buf_frame++ = 0x01;
    *buf_frame++ = OPEN_VALVE;
    *buf_frame++ = check_cs(buf_frame_,11+4);
    *buf_frame++ = FRAME_END;
    
    for(i=0;i<4;i++){
      fe[i] = 0xFE;
    }
    
    Device_Read(ENABLE);
    for(i = 0;success == 0 && i < 3;i++){
      Slave_Write(fe,4);
      Slave_Write(buf_frame_,13+4);
      buf_readdata = OSQPend(&Q_ReadData,20000,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
      if(err != OS_ERR_NONE){
        if(i==2){
          //����ʧ��  ��flash  return nack
          success = 0;
        }
        continue;
      }
      //���յ���ȷ������
      
      //�жϱ��ַ
      if(Mem_Cmp(meter_addr,buf_readdata+2,7) == DEF_YES){
        //��ȡST
        st_l = *(buf_readdata + 14);
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
    configflash = OSMemGet(&MEM_Buf,&err);
    sFLASH_ReadBuffer(configflash,block_meter,256);
    if(success == 0){
      //����ʧ��  ��flash  return nack
      //Mem_Copy(configflash + 22,"\x43",1);  //��ʱ
      *(configflash + 22) = (*(configflash + 22)) | 0x40;
      sFLASH_EraseWritePage(configflash,block_meter,256);
      
      device_nack(desc);
    }else{
      Mem_Copy(configflash + 22,"\x00",1);  //��
      sFLASH_EraseWritePage(configflash,block_meter,256);
      
      device_ack(desc);   //return nack;
    }
    OSMemPut(&MEM_Buf,configflash,&err);
    Device_Read(DISABLE);
}

void meter_close(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc){
    OS_ERR err;
    CPU_TS ts;
    uint8_t buf_frame_[17];
    uint8_t fe[4];
    uint8_t i = 0;
    uint16_t msg_size = 0;
    uint8_t * buf_readdata = 0;
    uint8_t success = 0;
    uint8_t st_l = 0;
    uint8_t * configflash = 0;
    uint8_t * buf_frame = 0;  
  
    buf_frame = buf_frame_;
    *buf_frame++ = FRAME_HEAD;
    *buf_frame++ = meter_type;
    for(i=0;i<7;i++){
      *buf_frame++ = meter_addr[i];
    }
    *buf_frame++ = 0x04; //C
    *buf_frame++ = 0x04; //len
    *buf_frame++ = DATAFLAG_WV_L;
    *buf_frame++ = DATAFLAG_WV_H;
    *buf_frame++ = 0x01;
    *buf_frame++ = CLOSE_VALVE;
    *buf_frame++ = check_cs(buf_frame_,11+4);
    *buf_frame++ = FRAME_END;
    
    for(i=0;i<4;i++){
      fe[i] = 0xFE;
    }
    
    Device_Read(ENABLE);
    for(i = 0;success == 0 && i < 3;i++){
      Slave_Write(fe,4);
      Slave_Write(buf_frame_,13+4);
      buf_readdata = OSQPend(&Q_ReadData,20000,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
      if(err != OS_ERR_NONE){
        if(i==2){
          //����ʧ��  ��flash  return nack
          success = 0;
        }
        continue;
      }
      //���յ���ȷ������
      //�жϱ��ַ
      if(Mem_Cmp(meter_addr,buf_readdata+2,7) == DEF_YES){
        //��ȡST
        st_l = *(buf_readdata + 14);
        if((st_l & 0x02) == 0x02){
          //opened  return ack
          success = 1;
        }else{
          //����ʧ��  ��flash  return nack
          success = 0;
        }
      }
      
      OSMemPut(&MEM_Buf,buf_readdata,&err);
      
    }
    
    configflash = OSMemGet(&MEM_Buf,&err);
    sFLASH_ReadBuffer(configflash,block_meter,256);
    
    if(success == 0){
      //����ʧ��  ��flash  return nack
      //Mem_Copy(configflash + 22,"\x43",1);  //��ʱ
      *(configflash + 22) = (*(configflash + 22)) | 0x40;
      sFLASH_EraseWritePage(configflash,block_meter,256);
      
      device_nack(desc);  //return nack;
    }else{
      Mem_Copy(configflash + 22,"\x02",1);  //�ط�
      sFLASH_EraseWritePage(configflash,block_meter,256);
      
      device_ack(desc);  //return ack;
    }
    
    OSMemPut(&MEM_Buf,configflash,&err);
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

void param_config(uint8_t * buf_frame,uint8_t desc){
  OS_ERR err;
  uint8_t ip_port_[6];
  uint8_t * configflash;
  
  uint32_t block_cjq = 0;   //cjq block ��ַ
  uint32_t block_meter = 0;  //meter block ��ַ
  
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
    
    configflash = OSMemGet(&MEM_Buf,&err);
    
    sFLASH_ReadBuffer(configflash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(configflash + (sFLASH_CON_IP - sFLASH_CON_START_ADDR),ip,17);
    Mem_Copy(configflash + (sFLASH_CON_PORT - sFLASH_CON_START_ADDR),port,8);
    
    Mem_Copy(configflash + (sFLASH_CON_IP1 - sFLASH_CON_START_ADDR),&ip1,1);
    Mem_Copy(configflash + (sFLASH_CON_IP2 - sFLASH_CON_START_ADDR),&ip2,1);
    Mem_Copy(configflash + (sFLASH_CON_IP3 - sFLASH_CON_START_ADDR),&ip3,1);
    Mem_Copy(configflash + (sFLASH_CON_IP4 - sFLASH_CON_START_ADDR),&ip4,1);
    Mem_Copy(configflash + (sFLASH_CON_PORT_ - sFLASH_CON_START_ADDR),&port_,2);
    
    sFLASH_EraseWritePage(configflash,sFLASH_CON_START_ADDR,256);
    
    OSMemPut(&MEM_Buf,configflash,&err);
    
    device_ack(desc);
    
    break;
  case FN_ADDR:
    Mem_Clr(deviceaddr,5);
    deviceaddr[0] = *(buf_frame + DATA_POSITION);
    deviceaddr[1] = *(buf_frame + DATA_POSITION + 1);
    deviceaddr[2] = *(buf_frame + DATA_POSITION + 2);
    deviceaddr[3] = *(buf_frame + DATA_POSITION + 3);
    deviceaddr[4] = 0x00;//*(buf_frame + DATA_POSITION + 4);  Э���5λĬ��Ϊ0x00
    
    configflash = OSMemGet(&MEM_Buf,&err);
    
    sFLASH_ReadBuffer(configflash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(configflash + (sFLASH_DEVICE_ADDR - sFLASH_CON_START_ADDR),deviceaddr,5);
    sFLASH_EraseWritePage(configflash,sFLASH_CON_START_ADDR,256);
    
    OSMemPut(&MEM_Buf,configflash,&err);
    
    device_ack(desc);
    
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
          delete_meter(block_cjq,block_meter);
          device_ack(desc);
        }
      }
      
      if(*(buf_frame + DATA_POSITION + 16) == 0x01){
        //��ӱ�
        if(block_meter == 0xFFFFFF){
          //û������� ���
          add_meter(block_cjq,buf_frame + DATA_POSITION + 3);
          device_ack(desc);
        }else{
          //������� do nothing
        }
      }
    }
    
    break;
  case FN_CJQ:
    if(*(buf_frame + DATA_POSITION) == 0xAA){
      //ɾ������ɼ���
      block_cjq = search_cjq(buf_frame + DATA_POSITION + 1);
      if(block_cjq == 0xFFFFFF){
        //û������ɼ��� do nothing
      }else{
        //������ɼ���  ������ɼ����µı�Q���  ���ɼ���ɾ��
        delete_cjq(block_cjq);
        device_ack(desc);
      }
    }
    
    if(*(buf_frame + DATA_POSITION) == 0x55){
      //���
      if(search_cjq(buf_frame + DATA_POSITION + 1) == 0xFFFFFF){
        //�������ɼ���
        add_cjq(buf_frame + DATA_POSITION + 1);
        device_ack(desc);
      }else{
        //�Ѿ�������ɼ�����
      }
    }
    break;
  case FN_MBUS:
    if(*(buf_frame + DATA_POSITION) == 0xAA){
      //the slave is mbus
      slave_mbus = 0xAA;
    }
    
    if(*(buf_frame + DATA_POSITION) == 0xFF){
      //the slave is 485
      slave_mbus = 0xFF;
    }
    
    configflash = OSMemGet(&MEM_Buf,&err);
    
    sFLASH_ReadBuffer(configflash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(configflash + (sFLASH_METER_MBUS - sFLASH_CON_START_ADDR),&slave_mbus,1);
    sFLASH_EraseWritePage(configflash,sFLASH_CON_START_ADDR,256);
    
    OSMemPut(&MEM_Buf,configflash,&err);
    
    device_ack(desc);
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
  uint8_t * configflash = 0;
  uint16_t meter_count = 0;
  uint16_t meter_all = 0;
  uint32_t meter_read = 0;
  OS_ERR err;
  
  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+18,2);
  sFLASH_ReadBuffer((uint8_t *)&meter_all,sFLASH_METER_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_last,block_cjq+15,3);
  
  meter_count++;  //�ɼ����µı�����++
  meter_all++;  //���б�����++
  
  block_new = GetFlash();  
  sFLASH_WritePage(meteraddr,block_new + 6,7);  //���ַ
  sFLASH_WritePage((uint8_t *)(meteraddr - 3),block_new + 13,1);  //������
  sFLASH_WritePage((uint8_t *)&meter_read,block_new + 14,4);  //�����  meter_read = 0
  sFLASH_WritePage((uint8_t *)&meter_read,block_new + 22,2);  //��״̬  meter_read = 0
  
  configflash = OSMemGet(&MEM_Buf,&err);
    
  if(block_last == 0xFFFFFF){
    //first meter
    //�ɼ�����Ŀ�ʼ�ͽ�β��ָ������ӵı�Ŀ�
    sFLASH_WritePage((uint8_t *)&block_new,block_cjq + 12,3);  
    sFLASH_WritePage((uint8_t *)&block_new,block_cjq + 15,3);  
    
    sFLASH_ReadBuffer(configflash,block_cjq,256);  
    Mem_Copy(configflash + 18,(uint8_t *)&meter_count,2);  //�ɼ����µ���Ŀ++
    sFLASH_EraseWritePage(configflash,block_cjq,256);
  }else{
    //���ɼ�����Ľ�βָ������ӵı�Ŀ�
    sFLASH_ReadBuffer(configflash,block_cjq,256);
    Mem_Copy(configflash + 15,(uint8_t *)&block_new,3);
    Mem_Copy(configflash + 18,(uint8_t *)&meter_count,2);  //�ɼ����µ���Ŀ++
    sFLASH_EraseWritePage(configflash,block_cjq,256);
    
    //ԭ�����һ�������һ����ָ������ӵı�Ŀ�
    sFLASH_WritePage((uint8_t *)&block_new,block_last + 3,3);  
    //����ӵı�Ŀ����һ���� ָ��ԭ�������һ����
    sFLASH_WritePage((uint8_t *)&block_last,block_new + 18,3);
    
  }
  
  sFLASH_ReadBuffer(configflash,sFLASH_CON_START_ADDR,256);
  Mem_Copy(configflash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
  sFLASH_EraseWritePage(configflash,sFLASH_CON_START_ADDR,256);
  OSMemPut(&MEM_Buf,configflash,&err);
  
  return block_new;
}

uint32_t delete_meter(uint32_t block_cjq,uint32_t block_meter){
  uint32_t block_after = 0;
  uint32_t block_before = 0;
  uint8_t * configflash = 0;
  uint16_t meter_count = 0;
  uint16_t meter_all = 0;
  OS_ERR err;
  
  
  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+18,2);
  sFLASH_ReadBuffer((uint8_t *)&meter_all,sFLASH_METER_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_after,block_meter+3,3);
  sFLASH_ReadBuffer((uint8_t *)&block_before,block_meter+18,3);
  
  meter_count--;
  meter_all--;
  
  PutFlash(block_meter);
  configflash = OSMemGet(&MEM_Buf,&err);
  
  if(meter_count == 0){
    //�ɼ�����Ψһ�ı�  block_before��block_after  ��Ϊ0xFFFFFF
    sFLASH_ReadBuffer(configflash,block_cjq,256);
    Mem_Copy(configflash + 12,(uint8_t *)&block_after,3);
    Mem_Copy(configflash + 18,(uint8_t *)&meter_count,2);
    Mem_Copy(configflash + 15,(uint8_t *)&block_before,3);
    sFLASH_EraseWritePage(configflash,block_cjq,256);
    
    //����ȫ������Ŀ
    sFLASH_ReadBuffer(configflash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(configflash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
    sFLASH_EraseWritePage(configflash,sFLASH_CON_START_ADDR,256);
  }else{
    if(block_before == 0xFFFFFF || block_after == 0xFFFFFF){
      //Ҫɾ��������ǵ�һ��  ���������һ��
      if(block_before == 0xFFFFFF){
        //�޸ĺ�һ����before Ϊblock_before
        sFLASH_ReadBuffer(configflash,block_after,256);
        Mem_Copy(configflash + 18,(uint8_t *)&block_before,3);
        sFLASH_EraseWritePage(configflash,block_after,256);
        //�޸Ĳɼ�����һ�����ַΪ  block_after  ���²ɼ�������Ŀ
        sFLASH_ReadBuffer(configflash,block_cjq,256);
        Mem_Copy(configflash + 12,(uint8_t *)&block_after,3);
        Mem_Copy(configflash + 18,(uint8_t *)&meter_count,2);
        sFLASH_EraseWritePage(configflash,block_cjq,256);
        //����ȫ������Ŀ
        sFLASH_ReadBuffer(configflash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(configflash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
        sFLASH_EraseWritePage(configflash,sFLASH_CON_START_ADDR,256);
      }
      if(block_after == 0xFFFFFF){
        //�޸�ǰһ����next Ϊblock_after
        sFLASH_ReadBuffer(configflash,block_before,256);
        Mem_Copy(configflash + 3,(uint8_t *)&block_after,3);
        sFLASH_EraseWritePage(configflash,block_before,256);
        //�޸Ĳɼ��������һ�����ַΪ  block_before  ���²ɼ�������Ŀ
        sFLASH_ReadBuffer(configflash,block_cjq,256);
        Mem_Copy(configflash + 15,(uint8_t *)&block_before,3);
        Mem_Copy(configflash + 18,(uint8_t *)&meter_count,2);
        sFLASH_EraseWritePage(configflash,block_cjq,256);
        //����ȫ������Ŀ
        sFLASH_ReadBuffer(configflash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(configflash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
        sFLASH_EraseWritePage(configflash,sFLASH_CON_START_ADDR,256);
      }
    }else{
      //Ҫɾ����������м�
      //�޸�ǰһ����next Ϊblock_after
      sFLASH_ReadBuffer(configflash,block_before,256);
      Mem_Copy(configflash + 3,(uint8_t *)&block_after,3);
      sFLASH_EraseWritePage(configflash,block_before,256);
      //�޸ĺ�һ����before Ϊblock_before
      sFLASH_ReadBuffer(configflash,block_after,256);
      Mem_Copy(configflash + 18,(uint8_t *)&block_before,3);
      sFLASH_EraseWritePage(configflash,block_after,256);
      
      sFLASH_ReadBuffer(configflash,block_cjq,256);
      Mem_Copy(configflash + 18,(uint8_t *)&meter_count,2);
      sFLASH_EraseWritePage(configflash,block_cjq,256);
      
      //�޸�control blocks
      sFLASH_ReadBuffer(configflash,sFLASH_CON_START_ADDR,256);
      Mem_Copy(configflash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
      sFLASH_EraseWritePage(configflash,sFLASH_CON_START_ADDR,256);
    }
  }
  OSMemPut(&MEM_Buf,configflash,&err);
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
  uint8_t * configflash = 0;
  uint16_t cjq_count = 0;
  uint16_t meter_count = 0;
  OS_ERR err;
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_last,sFLASH_CJQ_Q_LAST,3);
  
  cjq_count++;  //�ɼ�������++
  
  //��ȡһ��flash��  ��������Ӧ��Ϣ
  block_new = GetFlash();  
  sFLASH_WritePage(cjqaddr,block_new + 6,6);  //�ɼ�����ַ
  sFLASH_WritePage((uint8_t *)&meter_count,block_new + 18,2);  //�ɼ�������  (uint8_t *)0 �ǵ�ַ0x00000000����ֵ��
  //��һ�������һ���ָ����0xFFFFFF
  
  configflash = OSMemGet(&MEM_Buf,&err);
    
  sFLASH_ReadBuffer(configflash,sFLASH_CON_START_ADDR,256);
  
  if(block_last == 0xFFFFFF){
    //this is the first cjq
    //cjq Q �Ŀ�ʼ�ͽ�β��ָ������ӵĲɼ�����
    Mem_Copy(configflash + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
    Mem_Copy(configflash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    Mem_Copy(configflash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
  }else{
    //���ǵ�һ��
    //��cjq Q �Ľ�βָ������ӵĲɼ�����
    Mem_Copy(configflash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    Mem_Copy(configflash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
    //��ԭ�����һ���ɼ�������һ���ɼ���ָ������ӵĲɼ�����
    sFLASH_WritePage((uint8_t *)&block_new,block_last + 3,3);
    //������ӵĲɼ������е���һ���ɼ���  ָ��ԭ�������һ���ɼ���
    sFLASH_WritePage((uint8_t *)&block_last,block_new + 20,3);  //��һ���ɼ���
  }
  
  sFLASH_EraseWritePage(configflash,sFLASH_CON_START_ADDR,256);
  OSMemPut(&MEM_Buf,configflash,&err);
  
  return block_new;
}

uint32_t delete_cjq(uint32_t block_cjq){
  uint16_t meter_count = 0;
  uint16_t meter_all = 0;
  uint32_t block_meter = 0;
  uint32_t block_after = 0;  //��һ���ɼ���
  uint32_t block_before = 0;  //��һ���ɼ���
  uint16_t cjq_count = 0;
  uint16_t i = 0;
  OS_ERR err;
  uint8_t * configflash = 0;
  
  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+18,2);
  sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+12,3);
  sFLASH_ReadBuffer((uint8_t *)&block_after,block_cjq+3,3);
  sFLASH_ReadBuffer((uint8_t *)&block_before,block_cjq+20,3);
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&meter_all,sFLASH_METER_COUNT,2);
  
  cjq_count--;
  
  //������ɼ����µı�Q���  ���ɼ���ɾ��
  for(i = 0;i < meter_count;i++){
    
    PutFlash(block_meter);
    
    sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+3,3);  //��ȡ��һ�����block��ַ
  }
  //���ɼ���ɾ��
  PutFlash(block_cjq);
  
  configflash = OSMemGet(&MEM_Buf,&err);
  
  //����ȫ������Ŀ
  meter_all = meter_all - meter_count;
  sFLASH_ReadBuffer(configflash,sFLASH_CON_START_ADDR,256);
  Mem_Copy(configflash + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
  sFLASH_EraseWritePage(configflash,sFLASH_CON_START_ADDR,256);
  
  if(cjq_count == 0){
    //����ɼ�����Ψһ��һ��   block_before��block_after  ��Ϊ0xFFFFFF
    sFLASH_ReadBuffer(configflash,sFLASH_CON_START_ADDR,256);
    Mem_Copy(configflash + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_after,3);
    Mem_Copy(configflash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    Mem_Copy(configflash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_before,3);
    sFLASH_EraseWritePage(configflash,sFLASH_CON_START_ADDR,256);
  }else{
    if(block_before == 0xFFFFFF || block_after == 0xFFFFFF){
      //Ҫɾ��������ǵ�һ��  ���������һ��
      if(block_before == 0xFFFFFF){
        //�޸ĺ�һ����before Ϊblock_before
        sFLASH_ReadBuffer(configflash,block_after,256);
        Mem_Copy(configflash + 20,(uint8_t *)&block_before,3);
        sFLASH_EraseWritePage(configflash,block_after,256);
        
        sFLASH_ReadBuffer(configflash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(configflash + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_after,3);
        Mem_Copy(configflash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
        sFLASH_EraseWritePage(configflash,sFLASH_CON_START_ADDR,256);
      }
      
      if(block_after == 0xFFFFFF){
        //�޸�ǰһ����next Ϊblock_after
        sFLASH_ReadBuffer(configflash,block_before,256);
        Mem_Copy(configflash + 3,(uint8_t *)&block_after,3);
        sFLASH_EraseWritePage(configflash,block_before,256);
        
        sFLASH_ReadBuffer(configflash,sFLASH_CON_START_ADDR,256);
        Mem_Copy(configflash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
        Mem_Copy(configflash + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_before,3);
        sFLASH_EraseWritePage(configflash,sFLASH_CON_START_ADDR,256);
      }
      
    }else{
      //Ҫɾ����������м�
      //�޸�ǰһ����next Ϊblock_after
      sFLASH_ReadBuffer(configflash,block_before,256);
      Mem_Copy(configflash + 3,(uint8_t *)&block_after,3);
      sFLASH_EraseWritePage(configflash,block_before,256);
      //�޸ĺ�һ����before Ϊblock_before
      sFLASH_ReadBuffer(configflash,block_after,256);
      Mem_Copy(configflash + 20,(uint8_t *)&block_before,3);
      sFLASH_EraseWritePage(configflash,block_after,256);
      
      //�޸�control blocks
      sFLASH_ReadBuffer(configflash,sFLASH_CON_START_ADDR,256);
      Mem_Copy(configflash + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
      sFLASH_EraseWritePage(configflash,sFLASH_CON_START_ADDR,256);
    }
  }
  OSMemPut(&MEM_Buf,configflash,&err);
  return block_cjq;
}

void param_query(uint8_t * buf_frame,uint8_t desc){
  switch(*(buf_frame + FN_POSITION)){
  case FN_IP_PORT:
    ack_query_ip(desc);
    break;
  case FN_ADDR: 
    ack_query_addr(desc);
    break;
  case FN_METER:
    ack_query_meter(*(buf_frame + DATA_POSITION),buf_frame + DATA_POSITION + 1,desc);
    break;
  case FN_CJQ:
    ack_query_cjq(desc);
    break;
  case FN_MBUS:
    ack_query_mbus(desc);
    break;
  }
}

extern uint8_t dns[];
extern OS_SEM SEM_ServerTX;
void Task_LED1(void *p_arg){
  OS_ERR err;
  //CPU_TS ts;
  
  //uint8_t * configflash;
  //uint8_t y;
  //uint8_t *yy;
  
  while(DEF_TRUE){
    if(reading == 0){
      GPIO_SetBits(GPIOB,GPIO_Pin_8);
      OSTimeDlyHMSM(0,0,1,0,
                    OS_OPT_TIME_HMSM_STRICT,
                    &err);
      GPIO_ResetBits(GPIOB,GPIO_Pin_8);
      OSTimeDlyHMSM(0,0,1,0,
                    OS_OPT_TIME_HMSM_STRICT,
                    &err);
    }else{
      GPIO_SetBits(GPIOB,GPIO_Pin_8);
      OSTimeDlyHMSM(0,0,0,500,
                    OS_OPT_TIME_HMSM_STRICT,
                    &err);
      GPIO_ResetBits(GPIOB,GPIO_Pin_8);
      OSTimeDlyHMSM(0,0,0,500,
                    OS_OPT_TIME_HMSM_STRICT,
                    &err);
    }
  }
}

void device_ack(uint8_t desc){
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
  *buf_frame++ = ZERO_BYTE |SINGLE ;
  *buf_frame++ = FN_ACK;
  
  *buf_frame++ = check_cs(ack+6,9);
  *buf_frame++ = FRAME_END;
  
  if(desc){
    //to m590e
    send_server(ack,17);
  }else{
    //to 485
    Slave_Write(ack,17);
  }
  
}

void device_nack(uint8_t desc){
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
  *buf_frame++ = ZERO_BYTE |SINGLE ;
  *buf_frame++ = FN_NACK;
  
  *buf_frame++ = check_cs(nack+6,9);
  *buf_frame++ = FRAME_END;
  
  if(desc){
    //to m590e
    send_server(nack,17);
  }else{
    //to 485
    Slave_Write(nack,17);
  }
  
}

void ack_query_cjq(uint8_t desc){
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
  *buf_frame++ = ZERO_BYTE |SINGLE ;
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
    Slave_Write(buf_frame_,17+cjq_count*6);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
  
}

void ack_query_meter(uint8_t metertype,uint8_t * meteraddr,uint8_t desc){
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
          if(times_ == 1){
            //��֡
            meter_count = allmeter_count;
            meter_count_ = meter_count;
            
            *buf_frame++ = FRAME_HEAD;
            buf_frame_16 = (uint16_t *)buf_frame;
            *buf_frame_16++ = ((9+17*meter_count_+1) << 2) | 0x03;    //+1  because of  *buf_frame++ = metertype;
            *buf_frame_16++ = ((9+17*meter_count_+1) << 2) | 0x03;
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
            *buf_frame++ = ZERO_BYTE |SINGLE ;
            *buf_frame++ = FN_METER;
            
            *buf_frame++ = metertype;
            
          }else{
            //��֡
            if(times_count == 1){
              //��֡
              meter_count = 10;
              meter_count_ = meter_count;
              
              *buf_frame++ = FRAME_HEAD;
              buf_frame_16 = (uint16_t *)buf_frame;
              *buf_frame_16++ = ((9+17*meter_count_+1) << 2) | 0x03;
              *buf_frame_16++ = ((9+17*meter_count_+1) << 2) | 0x03;
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
              *buf_frame++ = ZERO_BYTE |MUL_FIRST ;
              *buf_frame++ = FN_METER;
              
              *buf_frame++ = metertype;
              
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
                
                *buf_frame++ = FRAME_HEAD;
                buf_frame_16 = (uint16_t *)buf_frame;
                *buf_frame_16++ = ((9+17*meter_count_+1) << 2) | 0x03;
                *buf_frame_16++ = ((9+17*meter_count_+1) << 2) | 0x03;
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
                *buf_frame++ = ZERO_BYTE |MUL_LAST ;
                *buf_frame++ = FN_METER;
                
                *buf_frame++ = metertype;
                
              }else{
                //�м�֡
                meter_count = 10;
                meter_count_ = meter_count;
                
                *buf_frame++ = FRAME_HEAD;
                buf_frame_16 = (uint16_t *)buf_frame;
                *buf_frame_16++ = ((9+17*meter_count_+1) << 2) | 0x03;
                *buf_frame_16++ = ((9+17*meter_count_+1) << 2) | 0x03;
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
                *buf_frame++ = ZERO_BYTE |MUL_MIDDLE ;
                *buf_frame++ = FN_METER;
                
                *buf_frame++ = metertype;
              }
            }
          }
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
            Slave_Write(buf_frame_,17+17*meter_count_+1);
          }
          
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
      *buf_frame++ = ZERO_BYTE |SINGLE ;
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
        Slave_Write(buf_frame_,35);
      }
    }
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
}

void ack_query_addr(uint8_t desc){
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
  *buf_frame++ = ZERO_BYTE |SINGLE ;
  *buf_frame++ = FN_ADDR;
  
  *buf_frame++ = check_cs(buf_frame_+6,9);
  *buf_frame++ = FRAME_END;
  
  
  if(desc){
    //to m590e
    send_server(buf_frame_,17);
  }else{
    //to 485
    Slave_Write(buf_frame_,17);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
}

void ack_query_ip(uint8_t desc){
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
  *buf_frame++ = ZERO_BYTE |SINGLE ;
  *buf_frame++ = FN_IP_PORT;
  
  *buf_frame++ = ip1;
  *buf_frame++ = ip2;
  *buf_frame++ = ip3;
  *buf_frame++ = ip4;
  
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
    Slave_Write(buf_frame_,23);
  }
  
  OSMemPut(&MEM_Buf,buf_frame_,&err);
  
}

void ack_query_mbus(uint8_t desc){
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
  *buf_frame++ = ZERO_BYTE |SINGLE ;
  *buf_frame++ = FN_MBUS;
  
  *buf_frame++ = slave_mbus;
  *buf_frame++ = check_cs(buf_frame_+6,10);
  *buf_frame++ = FRAME_END;
  
  
  if(desc){
    //to m590e
    send_server(buf_frame_,18);
  }else{
    //to 485
    Slave_Write(buf_frame_,18);
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
    
    OSTimeDlyHMSM(0,0,0,200,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
    if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14)){
      //enable the beep
      GPIO_SetBits(GPIOA,GPIO_Pin_15);
      //disable the mbus power
      GPIO_ResetBits(GPIOA,GPIO_Pin_12);
    }
    
  }
}