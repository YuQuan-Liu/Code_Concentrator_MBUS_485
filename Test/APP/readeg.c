#include "os.h"
#include "stm32f10x_conf.h"
#include "tasks.h"
#include "serial.h"
#include "readeg.h"
#include "frame.h"

extern uint8_t *meterdata;  //使用海大协议抄表时存放返回的信息  使用config_flash

extern uint8_t data_seq;  //记录数据的序列号 等待ack
extern uint8_t local_seq;  //本地序列号
extern OS_SEM SEM_ACKData;     //服务器对数据的ACK
extern OS_MEM MEM_Buf;
extern OS_Q Q_ReadData;        //发送抄表指令后  下层返回抄表数据
extern uint8_t deviceaddr[5];
extern uint8_t slave_mbus; //0xaa mbus   0xff  485   0xBB~采集器



//抄单个表
void meter_single_eg(u8 *deal_ptr){
  
  uint8_t cjq_h = *(deal_ptr + DATA_POSITION + 1);
  uint8_t cjq_l = *(deal_ptr + DATA_POSITION + 2);
  uint8_t meter_addr = *(deal_ptr + DATA_POSITION + 3);
  
  meterdata[0] = cjq_h;
  meterdata[1] = cjq_l;
  meterdata[2] = 0x00;
  read_single_eg(cjq_h,cjq_l,meter_addr,0);
}

//抄全部表
void meter_cjq_eg(u8 *deal_ptr){
  OS_ERR err;
  CPU_TS ts;
  u8 i = 0;
  u16 tmr_count = 0;
  u8 *buf_readdata = 0;
  u8 cjq_h = *(deal_ptr + DATA_POSITION + 1);
  u8 cjq_l = *(deal_ptr + DATA_POSITION + 2);
  u8 allmeters = *(deal_ptr + DATA_POSITION + 3);
  u8 meter_addr = 0;
  uint16_t msg_size = 0;
  u8 toslave[10];
  u8 done = 0;
  
  switch(slave_mbus){
    case 0xBB:
      //采集器
      //测试前3快表
      for(i = 1;i <= 3;i++){
        done = read_single_eg(cjq_h,cjq_l,i,0);
        if(done){
          break;
        }
      }
      
      if(done){
        //采集器没问题  可以去抄采集器
        memset(meterdata,0x00,600);
        meterdata[0] = cjq_h;
        meterdata[1] = cjq_l;
        meterdata[2] = allmeters;
        toslave[0] = 0x0E;
        toslave[1] = 0x0D;
        toslave[2] = 0x0C;
        toslave[3] = 0x01;
        toslave[4] = allmeters;
        toslave[5] = cjq_h;
        toslave[6] = cjq_l;
        toslave[7] = 0xFF;
        toslave[8] = 0xFF;
        toslave[9] = 0x0E^toslave[4]^toslave[5]^toslave[6];
        
        Slave_Write(toslave,10);
        
        while(tmr_count < 65){
          buf_readdata = OSQPend(&Q_ReadData,1000,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
          if(err != OS_ERR_NONE){
            tmr_count++;
          }else{
            //check the addr
            meter_addr = *(buf_readdata + 4);
            if(meter_addr > 0 && meter_addr <200){
              meterdata[3*meter_addr] = meter_addr;
              meterdata[3*meter_addr+1] = *(buf_readdata + 7);
              meterdata[3*meter_addr+2] = *(buf_readdata + 8);
            }
            OSMemPut(&MEM_Buf,buf_readdata,&err);
            if(meter_addr == allmeters){
              break;
            }
          }
        }
        if(tmr_count == 65){
          //sorry 采集器故障
          meterdata[0] = cjq_h;
          meterdata[1] = cjq_l;
          meterdata[2] = 0xFF;
        }
      }else{
        //sorry 采集器故障
        meterdata[0] = cjq_h;
        meterdata[1] = cjq_l;
        meterdata[2] = 0xFF;
      }
      
    break;
    default:
      
      meterdata[0] = cjq_h;
      meterdata[1] = cjq_l;
      meterdata[2] = 0x00;  
      
      for(i = 1;i <= allmeters;i++){
        read_single_eg(cjq_h,cjq_l,i,1);
      }
    break;
  }
}

/*
根据采集器 和表地址  去抄单个表
all = 1 表示抄全部表   
all = 0 表示抄单个表
return done  done = 1 表示抄到了表  done = 0 表示超时
*/
u8 read_single_eg(u8 cjq_h,u8 cjq_l,u8 meter_addr,u8 all){
  OS_ERR err;
  CPU_TS ts;
  u8 i = 0;
  uint16_t msg_size = 0;
  u8 *buf_readdata = 0;
  
  u8 toslave[10];
  u8 done = 0;
  
  switch(slave_mbus){
    case 0xBB:
      //采集器
      toslave[0] = 0x0E;
      toslave[1] = 0x0D;
      toslave[2] = 0x0C;
      toslave[3] = 0x02;
      toslave[4] = meter_addr;
      toslave[5] = cjq_h;
      toslave[6] = cjq_l;
      toslave[7] = 0xFF;
      toslave[8] = 0xFF;
      toslave[9] = 0x0D^toslave[4]^toslave[5]^toslave[6];
    break;
    default:
      //表
      toslave[0] = 0x0E;
      toslave[1] = 0x0D;
      toslave[2] = 0x0B;
      toslave[3] = 0x02;
      toslave[4] = meter_addr;
      toslave[5] = 0xAA;
      toslave[6] = 0xAA;
      toslave[7] = 0xAA;
      toslave[8] = 0xA0^toslave[4];
    break;
  }
  
  for(i = 0;i< 3;i++){
    if(slave_mbus == 0xBB){
      //底层采集器
      Slave_Write(toslave,10);
      buf_readdata = OSQPend(&Q_ReadData,3000,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
      if(err != OS_ERR_NONE){
        continue;
      }
      //check the addr
      if(*(buf_readdata + 4)== meter_addr){
        if(all){
          meterdata[3*meter_addr] = meter_addr;
          meterdata[3*meter_addr+1] = *(buf_readdata + 7);
          meterdata[3*meter_addr+2] = *(buf_readdata + 8);
        }else{
          meterdata[3] = meter_addr;
          meterdata[4] = *(buf_readdata + 7);
          meterdata[5] = *(buf_readdata + 8);
        }
        
      }
      OSMemPut(&MEM_Buf,buf_readdata,&err);
      done = 1;
      break;
    }else{
      //底层表  0xAA
      Slave_Write(toslave,9);
      buf_readdata = OSQPend(&Q_ReadData,1000,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
      if(err != OS_ERR_NONE){
        continue;
      }
      //check the addr
      if(*(buf_readdata + 4)== meter_addr){
        if(all){
          meterdata[3*meter_addr] = meter_addr;
          meterdata[3*meter_addr+1] = *(buf_readdata + 6);
          meterdata[3*meter_addr+2] = *(buf_readdata + 7);
        }else{
          meterdata[3] = meter_addr;
          meterdata[4] = *(buf_readdata + 6);
          meterdata[5] = *(buf_readdata + 7);
        }
        
      }
      OSMemPut(&MEM_Buf,buf_readdata,&err);
      done = 1;
      break;
    }
  }
  if(!done){
    if(all){
      meterdata[3*meter_addr] = meter_addr;
      meterdata[3*meter_addr+1] = 0xBB;
      meterdata[3*meter_addr+2] = 0xBB;
    }else{
      meterdata[3] = meter_addr;
      meterdata[4] = 0xBB;
      meterdata[5] = 0xBB;
    }
  }
  return done;
}

void send_data_eg(u8 metercount,uint8_t desc){
  //需要有确认实现
  OS_ERR err;
  CPU_TS ts;
  uint8_t times = metercount/20;
  uint8_t remain = metercount%20;
  uint8_t times_  = 0;
  uint8_t k = 0;  //计数这个帧中的数据
  uint8_t i = 0;   //计数这是第几帧
  uint8_t j = 0;   //计数发送的次数
  uint8_t *buf_ptr = 0;
  uint8_t *buf_ptr_ = 0;
  uint16_t len = 0;
  //是否得到服务器的ack  没有则停止发送 3次都没收到 肯定异常了
  uint8_t ack = 0;   
  
  buf_ptr = OSMemGet(&MEM_Buf,&err);
  if(buf_ptr != 0){
    buf_ptr_ = buf_ptr;
    times_ = times;
    if(remain > 0){
      times_ = times + 1;
    }
    for(i = 0;i<times_;i++){
      buf_ptr = buf_ptr_;
      *buf_ptr++ = FRAME_HEAD;
      if(times_ == 1){
        //只有这一帧
        len = ((metercount*3+12)<<2) |0x03;
      }else{
        if(i == 0){
          //头帧
          len = 0x123;  //(72<<2) |0x03
        }else{
          if(i == times_-1){
            //尾帧
            if(remain > 0){
              len = ((remain*3+12)<<2) |0x03;
            }else{
              len = 0x123;  //(72<<2) |0x03
            }
            
          }else{
            //中间帧
            len = 0x123;  //(72<<2) |0x03
          }
        }
      }
      *buf_ptr++ = len & 0xFF;//(9 << 2) | 0x03;
      *buf_ptr++ = len >> 8;
      *buf_ptr++ = len & 0xFF;//(9 << 2) | 0x03;
      *buf_ptr++ = len >> 8;
      *buf_ptr++ = FRAME_HEAD;
      
      *buf_ptr++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
      *buf_ptr++ = deviceaddr[0];
      *buf_ptr++ = deviceaddr[1];
      *buf_ptr++ = deviceaddr[2];
      *buf_ptr++ = deviceaddr[3];
      *buf_ptr++ = deviceaddr[4];
      
      *buf_ptr++ = AFN_CURRENT;
      if(times_ == 1){
        //只有这一帧
        *buf_ptr++ = ZERO_BYTE |SINGLE | CONFIRM | local_seq;
      }else{
        if(i == 0){
          //头帧
          *buf_ptr++ = ZERO_BYTE |MUL_FIRST |CONFIRM | local_seq;
        }else{
          if(i == times_-1){
            //尾帧
            *buf_ptr++ = ZERO_BYTE |MUL_LAST |CONFIRM | local_seq;
          }else{
            //中间帧
            *buf_ptr++ = ZERO_BYTE |MUL_MIDDLE |CONFIRM | local_seq;
          }
        }
      }
      
      data_seq = local_seq;
      addSEQ();
      
      *buf_ptr++ = FN_CURRENT_METER;
      *buf_ptr++ = meterdata[0];
      *buf_ptr++ = meterdata[1];
      *buf_ptr++ = meterdata[2];
      if(times_ == 1){
        //只有这一帧
        for(k = 1;k <= metercount;k++){
          *buf_ptr++ = meterdata[3*k];
          *buf_ptr++ = meterdata[3*k+1];
          *buf_ptr++ = meterdata[3*k+2];
        }
      }else{
        if(i == 0){
          //头帧
          for(k = 1;k <= 20;k++){
            *buf_ptr++ = meterdata[3*k];
            *buf_ptr++ = meterdata[3*k+1];
            *buf_ptr++ = meterdata[3*k+2];
          }
        }else{
          if(i == times_-1){
            //尾帧
            if(remain > 0){
              for(k = 1;k <= remain;k++){
                *buf_ptr++ = meterdata[3*k+i*60];
                *buf_ptr++ = meterdata[3*k+1+i*60];
                *buf_ptr++ = meterdata[3*k+2+i*60];
              }
            }else{
              for(k = 1;k <= 20;k++){
                *buf_ptr++ = meterdata[3*k+i*60];
                *buf_ptr++ = meterdata[3*k+1+i*60];
                *buf_ptr++ = meterdata[3*k+2+i*60];
              }
            }
            
          }else{
            //中间帧
            for(k = 1;k <= 20;k++){
              *buf_ptr++ = meterdata[3*k+i*60];
              *buf_ptr++ = meterdata[3*k+1+i*60];
              *buf_ptr++ = meterdata[3*k+2+i*60];
            }
          }
        }
      }
      *buf_ptr++ = check_cs(buf_ptr_+6,buf_ptr-buf_ptr_-6);  
      *buf_ptr++ = FRAME_END;
      if(desc){
        for(j = 0;j<3;j++){
          ack = 0;
          send_server(buf_ptr_,buf_ptr-buf_ptr_);
          OSSemPend(&SEM_ACKData,
                    5000,
                    OS_OPT_PEND_BLOCKING,
                    &ts,
                    &err);
          if(err == OS_ERR_NONE){
            ack = 1;
            break;
          }
        }
        /**/
        if(ack == 0){
          //3次都没收到 肯定异常了
          break;
        }
      }else{
        Server_Write_485(buf_ptr_,buf_ptr-buf_ptr_);
      }
    }
    OSMemPut(&MEM_Buf,buf_ptr_,&err);
  }
}

void send_cjqtimeout_eg(uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  uint8_t j = 0;   //计数发送的次数
  uint8_t *buf_ptr = 0;
  uint8_t *buf_ptr_ = 0;
  
  buf_ptr = OSMemGet(&MEM_Buf,&err);
  if(buf_ptr != 0){
    buf_ptr_ = buf_ptr;
    *buf_ptr++ = FRAME_HEAD;
    *buf_ptr++ = 0x33;//(12 << 2) | 0x03;
    *buf_ptr++ = 0x00;
    *buf_ptr++ = 0x33;//(12 << 2) | 0x03;
    *buf_ptr++ = 0x00;
    *buf_ptr++ = FRAME_HEAD;
    
    *buf_ptr++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    *buf_ptr++ = deviceaddr[0];
    *buf_ptr++ = deviceaddr[1];
    *buf_ptr++ = deviceaddr[2];
    *buf_ptr++ = deviceaddr[3];
    *buf_ptr++ = deviceaddr[4];
    
    *buf_ptr++ = AFN_CURRENT;
    *buf_ptr++ = ZERO_BYTE |SINGLE | CONFIRM | local_seq;
    
    data_seq = local_seq;
    addSEQ();
      
    *buf_ptr++ = FN_CURRENT_METER;
    *buf_ptr++ = meterdata[0];
    *buf_ptr++ = meterdata[1];
    *buf_ptr++ = meterdata[2];
    
    *buf_ptr++ = check_cs(buf_ptr_+6,12);  
    *buf_ptr++ = FRAME_END;
    
    if(desc){
      for(j = 0;j<3;j++){
        send_server(buf_ptr_,buf_ptr-buf_ptr_);
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
      Server_Write_485(buf_ptr_,buf_ptr-buf_ptr_);
    }
    
    
    OSMemPut(&MEM_Buf,buf_ptr_,&err);
  }
}
