

#ifndef FRAME_H_188

#define FRAME_H_188


#define   STARTFLAG         0x68
#define   METERTYPE         0x10

//read control
#define   CTR_READDATA      0x01
#define   CTR_READKEYV      0X09
#define   CTR_READADDR      0x03

#define   CTR_ACKDATA       0x81
#define   CTR_ACKKEYV       0x89
#define   CTR_ACKADDR       0x83

#define   CTR_NACKDATA      0x0C1
#define   CTR_NACKKEYV      0x0C9
#define   CTR_NACKADDR      0x0C3

//write control
#define   CTR_WRITEDATA     0x04
#define   CTR_WRITEADDR     0x15
#define   CTR_WRITESYNDATA  0x16

#define   CTR_ACKWD         0x84
#define   CTR_ACKWA         0x95
#define   CTR_ACKWSD        0x96

#define   CTR_NACKWD        0xC4
#define   CTR_NACKWA        0xD5
#define   CTR_NACKWSD       0xD6

//read DI0 DI1
//read data
#define   DATAFLAG_RD_H     0x90
#define   DATAFLAG_RD_L     0x1F
//read addr
#define   DATAFLAG_RA_H     0x81
#define   DATAFLAG_RA_L     0x0A
//read key version
#define   DATAFLAG_RK_H     0x81
#define   DATAFLAG_RK_L     0x06

//write DI0 DI1
//write new key
#define   DATAFLAG_WK_H     0xA0
#define   DATAFLAG_WK_L     0x14
//write out company
#define   DATAFLAG_WO_H     0xA0
#define   DATAFLAG_WO_L     0x19
//write addr
#define   DATAFLAG_WA_H     0xA0
#define   DATAFLAG_WA_L     0x18
//write meter start num
#define   DATAFLAG_WS_H     0xA0
#define   DATAFLAG_WS_L     0x16
//write need encrypt
#define   DATAFLAG_WE_H     0xA0
#define   DATAFLAG_WE_L     0x99
//write not out company
#define   DATAFLAG_WNO_H     0xA0
#define   DATAFLAG_WNO_L     0x98
//write i am bad
#define   DATAFLAG_WB_H     0xA0
#define   DATAFLAG_WB_L     0x97
//write to iap
#define   DATAFLAG_WI_H     0xA0
#define   DATAFLAG_WI_L     0x96
//write i am good
#define   DATAFLAG_WG_H     0xA0
#define   DATAFLAG_WG_L     0x95
//write cjq
#define   DATAFLAG_WC_H     0xA0
#define   DATAFLAG_WC_L     0x17
//write valve
#define   DATAFLAG_WV_H     0xA0
#define   DATAFLAG_WV_L     0x17
#define   OPEN_VALVE  0x55
#define   CLOSE_VALVE   0x99

#define   OPEN_CJQ  0x55
#define   CLOSE_CJQ   0x99


#define  ST_H              0x00
#define  ST_L              0x00


#endif