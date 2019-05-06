#ifndef MACROS_H 
#define MACROS_H 

#define ENABLE_CPU_ATTACH
#define DEBUG_PRINT 0
#define DEBUG_RADIO 0

#define EVENT_RX_SYMBOL 0
#define EVENT_CROPPED 1
#define EVENT_ZF 2
#define EVENT_DEMUL 3


#define TASK_CROP 0
#define TASK_ZF 1
#define TASK_DEMUL 2

// TASK & SOCKET thread number 
#define TASK_THREAD_NUM 1
#define SOCKET_THREAD_NUM 4

#define CORR_THRESHOLD    0x4
#define CORR_RST          0x0
#define CORR_SCNT         0x8
#define CORR_CONF         60
#define RF_RST_REG        48
#define TDD_CONF_REG      120
#define SCH_ADDR_REG      136
#define SCH_MODE_REG      140
#define TX_GAIN_CTRL      88

#define MAX_FRAME_INC 2000
#define TIME_DELTA 20 //ms 

#endif
