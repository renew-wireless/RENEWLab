#ifndef MACROS_H
#define MACROS_H

#include <atomic>
#include <vector>

#ifdef USE_SOAPYUHD
static constexpr bool kUseSoapyUHD = true;
#else
static constexpr bool kUseSoapyUHD = false;
#endif

#ifdef USE_UHD
static constexpr bool kUsePureUHD = true;
#else
static constexpr bool kUsePureUHD = false;
#endif

static constexpr size_t kStreamContinuous = 1;
static constexpr size_t kStreamEndBurst = 2;
static constexpr size_t kDsDimsNum = 5;
static constexpr size_t kDsDimSymbol = 2;

// buffer length of each rx thread
static constexpr size_t kSampleBufferFrameNum = 80;
static constexpr size_t kQueueSize = 36;

static constexpr bool kDebugPrint = false;
static constexpr bool kDebugRadio = false;
static constexpr bool kDebugPlot = false;
#define DEBUG_PRINT (0)
#define DEBUG_RADIO (0)
#define DEBUG_PLOT (0)

// TASK & SOCKET thread number
#define RECORDER_THREAD_NUM (1)
#define RX_THREAD_NUM (4)

#define MAX_FRAME_INC (2000)
#define TIME_DELTA_MS (40)  //ms
#define SETTLE_TIME_MS (1)
#define UHD_INIT_TIME_SEC (3)  // radio init time for UHD devices
#define BEACON_INTERVAL (20)   // frames

enum SchedulerEventType {
  kEventRxSymbol = 0,
  kEventTxSymbol = 1,
  kTaskRecord = 2,
  kTaskRead = 3,
  kThreadTermination = 4
};

enum NodeType { kBS = 0, kClient = 1 };

// each thread has a SampleBuffer
struct SampleBuffer {
  std::vector<char> buffer;
  std::atomic_int* pkt_buf_inuse;
};

struct Packet {
  uint32_t frame_id;
  uint32_t slot_id;
  uint32_t cell_id;
  uint32_t ant_id;
  short data[];
  Packet(int f, int s, int c, int a)
      : frame_id(f), slot_id(s), cell_id(c), ant_id(a) {}
};

struct Event_data {
  SchedulerEventType event_type;
  NodeType node_type;
  int frame_id;
  int slot_id;
  int ant_id;
  size_t buff_size;
  int offset;
  SampleBuffer* buffer;
};

#endif
