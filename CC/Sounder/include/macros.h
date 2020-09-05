#ifndef MACROS_H
#define MACROS_H

#ifdef USE_UHD
static constexpr bool kUseUHD = true;
#else
static constexpr bool kUseUHD = false;
#endif

#define DEBUG_PRINT 0
#define DEBUG_RADIO 0
#define DEBUG_PLOT 0

#define EVENT_RX_SYMBOL 0

#define TASK_RECORD 0

// TASK & SOCKET thread number
#define TASK_THREAD_NUM 1
#define RX_THREAD_NUM 4

#define MAX_FRAME_INC 2000
#define TIME_DELTA 40 //ms
#define SETTLE_TIME_MS 1

#endif
