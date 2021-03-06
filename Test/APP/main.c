


#include "includes.h"
#include "tasks.h"



//OS_TCBs
OS_TCB  TCB_Start;
CPU_STK STK_Start[APP_START_TASK_STK_SIZE];

OS_TCB TCB_LED;
CPU_STK STK_LED[APP_START_TASK_STK_SIZE];

//处理服务器发送过来的数据
OS_TCB TCB_Server;
CPU_STK STK_Server[APP_START_TASK_STK_SIZE];

//处理采集器、表发送过来的数据
OS_TCB TCB_Slave;
CPU_STK STK_Slave[APP_START_TASK_STK_SIZE];

//设置任务
OS_TCB TCB_Config;
CPU_STK STK_Config[APP_START_TASK_STK_SIZE*3];

//抄表任务
OS_TCB TCB_Read;
CPU_STK STK_Read[APP_START_TASK_STK_SIZE*3];

//the heartbeat task
OS_TCB TCB_HeartBeat;
CPU_STK STK_HeartBeat[APP_START_TASK_STK_SIZE];

//task deal the data "+TCPRECV"
OS_TCB TCB_DealServer;
CPU_STK STK_DealServer[APP_START_TASK_STK_SIZE];

//task deal the overload
OS_TCB TCB_OverLoad;
CPU_STK STK_OverLoad[APP_START_TASK_STK_SIZE];


//OS_MEMs
//receive the data from the ISR    put the data in the mem to deal buf
OS_MEM MEM_Buf;
uint8_t mem_buf[6][256];

//be used in the ISR   get the data from the usart*  post it to the deal task
OS_MEM MEM_ISR;
uint8_t mem_isr[30][4];

//配置处理Flash使用的数组  Sector==4K  需要一个4K的数组
uint8_t config_flash[0x1000];
uint8_t *meterdata;  //使用海大协议抄表时存放返回的信息  使用config_flash

//OS_MUTEXs;
OS_MUTEX MUTEX_CONFIGFLASH;    //是否可以使用 config_flash  4K 数组配置FLASH
OS_MUTEX MUTEX_SENDSERVER;    //是否可以发送数据到服务器

//OS_SEMs ;

OS_SEM SEM_ServerTX;    //往服务器发送数据
OS_SEM SEM_Slave_485TX;     //往采集器、表发送数据
OS_SEM SEM_Slave_mbusTX;

OS_SEM SEM_HeartBeat;    //接收服务器数据Task to HeartBeat Task  接收到心跳的回应
OS_SEM SEM_ACKData;    //服务器对数据的ACK
OS_SEM SEM_Send;      //got the '>'  we can send the data now  可以发送数据
OS_SEM SEM_SendOver;      //got the "+TCPSEND:0,"  the data is send over now  发送数据完成

//OS_Qs
OS_Q Q_Slave;            //采集器、表发送过来的数据
OS_Q Q_Read;             //抄表任务Queue
OS_Q Q_ReadData;        //发送抄表指令后  下层返回抄表数据
OS_Q Q_Config;         //配置任务Queue
OS_Q Q_Deal;         //处理接收到的服务器发送过来的数据


//OS_FLAG
OS_FLAG_GRP FLAG_Event;

//OS_TMR
OS_TMR TMR_CJQTIMEOUT;    //打开采集器通道之后 20分钟超时 自动关闭通道


volatile uint8_t connectstate = 0;       //0 didn't connect to the server   1 connect to the server
volatile uint8_t reading = 0;   //0 didn't reading meters    1  reading meters

uint8_t ack_action = 0xff;  //先应答后操作~0xaa    先操作后应答~0xff
uint8_t slave_mbus = 0xaa; //0xaa~mbus   0xff~485   0xbb~采集器
uint8_t di_seq; //DI0 DI1 顺序   0xAA~DI1在前(千宝通)   0xFF~DI0在前(default)  
uint8_t protocol = 0xFF;  //协议类型 0xFF~188(Default)  1~EG 

void TaskStart(void *p_arg);
void TaskCreate(void);
void ObjCreate(void);


int main (void){
  OS_ERR err;
  CPU_IntDis();
  
  OSInit(&err);
  
  OSTaskCreate((OS_TCB  *)&TCB_Start,
               (CPU_CHAR *)"START",
               (OS_TASK_PTR )TaskStart,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO,
               (CPU_STK *)&STK_Start[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  
  OSStart(&err);
}

void TaskStart(void *p_arg){
  CPU_INT32U cnts;
  CPU_INT32U cpu_clk_freq;
  OS_ERR err;
  uint32_t flashid;
  uint8_t * buf;////
  uint8_t test = 0;////
  uint8_t test_write = 0;////
  uint32_t sectionaddr = 0x1FF000;////
  
  BSP_Init();
  
  CPU_Init();
  
  cpu_clk_freq = BSP_CPU_ClkFreq();
  cnts = cpu_clk_freq / (CPU_INT32U)OS_CFG_TICK_RATE_HZ;
  OS_CPU_SysTickInit(cnts);
  
  while(DEF_TRUE){
    //check the w25x16 是否存在
    
    flashid = sFLASH_ReadID();
    if(FLASH_ID == flashid){
      break;
    }
    OSTimeDly(100,
              OS_OPT_TIME_DLY,
              &err);
  }
  
  sFLASH_PoolInit();
  
  TaskCreate();
  ObjCreate();
  
  if(test){
    while(1){
      asm("NOP");
      sFLASH_ReadBuffer(config_flash,sectionaddr,0x1000);
      if(test_write){
        sFLASH_EraseWritePage(buf,sectionaddr,256);
      }
    }
  }
  //Open the IWDG;
  BSP_IWDG_Init();
  
  while(DEF_TRUE){
    /* Reload IWDG counter */
    IWDG_ReloadCounter();
    
    //LED1
    GPIO_SetBits(GPIOB,GPIO_Pin_9);
    OSTimeDly(1000,
                  OS_OPT_TIME_DLY,
                  &err);
    GPIO_ResetBits(GPIOB,GPIO_Pin_9);
    OSTimeDly(1000,
                  OS_OPT_TIME_DLY,
                  &err);
  }
}

void TaskCreate(void){
  OS_ERR err;
  
  /*the data come from slave */
  OSTaskCreate((OS_TCB  *)&TCB_Slave,
               (CPU_CHAR *)"USART1",
               (OS_TASK_PTR )Task_Slave,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 1,
               (CPU_STK *)&STK_Slave[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  /*the data come from the server */
  OSTaskCreate((OS_TCB  *)&TCB_Server,
               (CPU_CHAR *)"USART2",
               (OS_TASK_PTR )Task_Server,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 2,
               (CPU_STK *)&STK_Server[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  //OS_CFG_TICK_TASK_PRIO == 6 = APP_START_TASK_PRIO + 3
  
  /*deal the server data */
  OSTaskCreate((OS_TCB  *)&TCB_DealServer,
               (CPU_CHAR *)"deal",
               (OS_TASK_PTR )Task_DealServer,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 4,
               (CPU_STK *)&STK_DealServer[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  /*read meter */
  OSTaskCreate((OS_TCB  *)&TCB_Read,
               (CPU_CHAR *)"read",
               (OS_TASK_PTR )Task_Read,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 5,
               (CPU_STK *)&STK_Read[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE*3/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE*3,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  /*heart beat */
  OSTaskCreate((OS_TCB  *)&TCB_HeartBeat,
               (CPU_CHAR *)"heart",
               (OS_TASK_PTR )Task_HeartBeat,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 6,
               (CPU_STK *)&STK_HeartBeat[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  /*config */
  OSTaskCreate((OS_TCB  *)&TCB_Config,
               (CPU_CHAR *)"config",
               (OS_TASK_PTR )Task_Config,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 7,
               (CPU_STK *)&STK_Config[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE*3/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE*3,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  //blink the led 1
  /**/
  OSTaskCreate((OS_TCB  *)&TCB_LED,
               (CPU_CHAR *)"LED",
               (OS_TASK_PTR )Task_LED,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 8,
               (CPU_STK *)&STK_LED[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
  
  //overload
  /**/
  OSTaskCreate((OS_TCB  *)&TCB_OverLoad,
               (CPU_CHAR *)"OverLoad",
               (OS_TASK_PTR )Task_OverLoad,
               (void *) 0,
               (OS_PRIO )APP_START_TASK_PRIO + 9,
               (CPU_STK *)&STK_OverLoad[0],
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE/10,
               (CPU_STK_SIZE)APP_START_TASK_STK_SIZE,
               (OS_MSG_QTY) 0u,
               (OS_TICK) 0u,
               (void *) 0,
               (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
}

void ObjCreate(void){
  OS_ERR err;
  
  //OS_MEM
  OSMemCreate((OS_MEM *)&MEM_Buf,
              (CPU_CHAR *)"frame",
              (void *)&mem_buf[0][0],
              (OS_MEM_QTY)6,
              (OS_MEM_SIZE)256,
              (OS_ERR *)&err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSMemCreate((OS_MEM *)&MEM_ISR,
              (CPU_CHAR *)"isr",
              (void *)&mem_isr[0][0],
              (OS_MEM_QTY)30,
              (OS_MEM_SIZE)4,
              (OS_ERR *)&err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  //OS_MUTEX;
  //OS_MUTEX MUTEX_CONFIGFLASH;    //是否可以使用 config_flash  4K 数组配置FLASH
  OSMutexCreate(&MUTEX_CONFIGFLASH,"",&err);
  OSMutexCreate(&MUTEX_SENDSERVER,"",&err);
  
  //OS_SEM
  OSSemCreate(&SEM_ServerTX,
              "u3_tx",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSSemCreate(&SEM_Slave_485TX,
              "u1_tx",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSSemCreate(&SEM_Slave_mbusTX,
              "u2_tx",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSSemCreate(&SEM_HeartBeat,
              "heart",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSSemCreate(&SEM_ACKData,
              "ackdata",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSSemCreate(&SEM_Send,
              "tcpsend",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSSemCreate(&SEM_SendOver,
              "sendover",
              0,
              &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  //OS_Q
  //data from slave
  OSQCreate(&Q_Slave,
            "slave",
            4,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_Read,
            "read",
            4,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_ReadData,
            "readdata",
            4,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_Config,
            "config",
            4,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  OSQCreate(&Q_Deal,
            "deal",
            4,
            &err);
  if(err != OS_ERR_NONE){
    return;
  }
  
  
  //OS_FLAGS
  OSFlagCreate(&FLAG_Event,
               "",
               (OS_FLAGS)0,
               &err);
  
  //OS_TMRs
  OSTmrCreate(&TMR_CJQTIMEOUT,
              "",
              12000,
              0,
              OS_OPT_TMR_ONE_SHOT,
              cjq_timeout,
              0,
              &err);
}



