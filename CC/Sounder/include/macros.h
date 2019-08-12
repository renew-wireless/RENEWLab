#ifndef MACROS_H 
#define MACROS_H 

#define DEBUG_PRINT 0
#define DEBUG_RADIO 0
#define DEBUG_PLOT 0

#define EVENT_RX_SYMBOL 0

#define TASK_RECORD 0

// TASK & SOCKET thread number 
#define TASK_THREAD_NUM 1
#define RX_THREAD_NUM 4

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

// Triggers - Used for Samples Offset Calibration
#define FPGA_IRIS30_TRIGGERS 44
#define FPGA_IRIS30_INCR_TIME (1 << 2)
#define FPGA_IRIS30_DECR_TIME (1 << 3)


#endif
