
#ifndef FRAME_H

#define FRAME_H

#define FRAME_HEAD              0x68
#define FRAME_END               0x16

#define ZERO_BYTE               0x00

//CON
#define DIR_TO_DEVICE           0x00
#define DIR_TO_SERVER           0x80

#define PRM_START               0x40
#define PRM_SLAVE               0x00


#define START_FUN_RESET         0x01
#define START_FUN_TEST          0x09
#define START_FUN_REQ1          0x0A
#define START_FUN_REQ2          0x0B


#define SLAVE_FUN_ACK           0x00
#define SLAVE_FUN_DATA          0x08
#define SLAVE_FUN_NODATA        0x09
#define SLAVE_FUN_TEST          0x0B




//AFN
#define AFN_ACK                     0x00
#define AFN_LINK_TEST               0x02
#define AFN_CONFIG                  0x03
#define AFN_CONTROL                 0x04
#define AFN_QUERY                   0x0A
#define AFN_CURRENT                 0x0B
#define AFN_HISTORY                 0x0C
#define AFN_FAKE                 0x0F



//SEQ
#define SINGLE                  0x60
#define MUL_MIDDLE              0x00
#define MUL_LAST                0x20
#define MUL_FIRST               0x40

#define CONFIRM                 0x10
#define NOCONFIRM               0x00


//Fn

//Fn  AFN == 0x00
#define FN_ACK                          1
#define FN_NACK                         2

//Fn LINK_TEST   AFN == 0x02
#define FN_HEARTBEAT                    3

//Fn  AFN == 0x03 (0x0A)
#define FN_IP_PORT                      2
#define FN_ADDR                         3
#define FN_METER                        6
#define FN_CJQ                          7
#define FN_MBUS                         12  //底层表的类型   是否打开MBUS部分  发送信息
#define FN_DI_SEQ                       14  //DI0 DI1 顺序   0xAA~DI1在前(千宝通)   0xFF~DI0在前(default)   只在抄表时使用
#define FN_ERASE                        15  //将FLASH清空  重新初始化
#define FN_RESET                        16  //重启系统
#define FN_ACK_ACTION                   17  //先应答后操作~0xaa    先操作后应答~0xff

//Fn  AFN == 0x04
#define FN_CLOSE                      2
#define FN_OPEN                        3
#define FN_CLEAN  4   //执行一次开关阀操作  防止生锈  阀门清洗

//Fn AFN == 0x0B
#define FN_CURRENT_METER                4
#define FN_READ_CJQ                     5

//POSITION  the byte in the frame
#define CON_POSITION                    6
#define AFN_POSITION                    12
#define SEQ_POSITION                    13
#define FN_POSITION                     14
#define DATA_POSITION                   15



#define FIXED_LEN                       9   // C A AFN SEQ FN = 9byte
#define FRAME_MIN_LEN                   17  //0x68 L L 0x68 (6)  FIXED_LEN (9)  cs 0x16 (2)
#define FRAME_MAX_LEN                   256 


#endif