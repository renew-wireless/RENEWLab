/** @file receiver.h
  * @brief Declaration file for the Receiver class.
  * Copyright (c) 2018-2022, Rice University
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
  * ----------------------------------------------------------
  * Handles received samples from massive-mimo base station 
  *----------------------------------------------------------
*/
#ifndef DATARECEIVER_H_
#define DATARECEIVER_H_

#include <pthread.h>

#include <complex>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(USE_UHD)
#include "BaseRadioSetUHD.h"
#include "ClientRadioSetUHD.h"
#else
#include "BaseRadioSet.h"
#include "ClientRadioSet.h"
#endif
#include "concurrentqueue.h"
#include "config.h"
#include "macros.h"

class ReceiverException : public std::runtime_error {
 public:
  ReceiverException()
      : std::runtime_error("Receiver could not be setup correctly!") {}
  explicit ReceiverException(const std::string& message)
      : std::runtime_error(message) {}
};

class Receiver {
 public:
  // use for create pthread
  struct ReceiverContext {
    Receiver* ptr;
    SampleBuffer* buffer;
    size_t core_id;
    size_t tid;
  };

 public:
  Receiver(Config* config, moodycamel::ConcurrentQueue<Event_data>* in_queue,
           std::vector<moodycamel::ConcurrentQueue<Event_data>*> tx_queue,
           std::vector<moodycamel::ProducerToken*> tx_ptoks,
           std::vector<moodycamel::ConcurrentQueue<Event_data>*> cl_tx_queue,
           std::vector<moodycamel::ProducerToken*> cl_tx_ptoks);
  ~Receiver();

  std::vector<pthread_t> startRecvThreads(SampleBuffer* rx_buffer,
                                          size_t n_rx_threads,
                                          SampleBuffer* tx_buffer,
                                          unsigned in_core_id = 0);
  void completeRecvThreads(const std::vector<pthread_t>& recv_thread);
  std::vector<pthread_t> startClientThreads(SampleBuffer* rx_buffer,
                                            SampleBuffer* tx_buffer,
                                            unsigned in_core_id = 0);
  void go();
  static void* loopRecv_launch(void* in_context);
  void loopRecv(int tid, int core_id, SampleBuffer* rx_buffer);
  void baseTxBeacon(int radio_id, int cell, int frame_id, long long base_time);
  int baseTxData(int radio_id, int cell, int frame_id, long long base_time);
  void notifyPacket(NodeType node_type, int frame_id, int slot_id, int ant_id,
                    int buff_size, int offset = 0);
  static void* clientTxRx_launch(void* in_context);
  void clientTxRx(int tid);
  void clientSyncTxRx(int tid, int core_id, SampleBuffer* rx_buffer);
  ssize_t syncSearch(const std::complex<int16_t>* check_data,
                     size_t search_window);

  float estimateCFO(const std::vector<std::complex<int16_t>>& sync_buff,
                    int sync_index);
  void initBuffers();
  void clientTxPilots(size_t user_id, long long base_time);
  int clientTxData(int tid, int frame_id, long long base_time);
  ssize_t clientSyncBeacon(size_t radio_id, size_t sample_window);
  void clientAdjustRx(size_t radio_id, size_t discard_samples);

 private:
  Config* config_;

#if defined(USE_UHD)
  ClientRadioSetUHD* client_radio_set_;
  BaseRadioSetUHD* base_radio_set_;
#else
  ClientRadioSet* client_radio_set_;
  BaseRadioSet* base_radio_set_;
#endif

  size_t thread_num_;
  // pointer of message_queue_
  moodycamel::ConcurrentQueue<Event_data>* message_queue_;
  std::vector<moodycamel::ConcurrentQueue<Event_data>*> tx_queue_;
  std::vector<moodycamel::ProducerToken*> tx_ptoks_;
  std::vector<moodycamel::ConcurrentQueue<Event_data>*> cl_tx_queue_;
  std::vector<moodycamel::ProducerToken*> cl_tx_ptoks_;

  // Data buffers
  SampleBuffer* cl_tx_buffer_;
  SampleBuffer* bs_tx_buffer_;
  std::vector<void*> pilotbuffA_;
  std::vector<void*> pilotbuffB_;
  std::vector<void*> zeros_;
  size_t txTimeDelta_;
  size_t txFrameDelta_;
};

#endif  // DATARECEIVER_H_
