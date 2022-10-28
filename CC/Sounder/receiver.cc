/*
 Copyright (c) 2018-2022, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
----------------------------------------------------------
 Handles received samples from massive-mimo base station 
----------------------------------------------------------
*/

#include "include/receiver.h"

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <climits>
#include <random>

#include "SoapySDR/Time.hpp"
#if defined(USE_UHD)
#include "include/ClientRadioSetUHD.h"
#else
#include "include/ClientRadioSet.h"
#endif
#include "include/comms-lib.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"

//Default to detect the beacon on first channel
static constexpr size_t kSyncDetectChannel = 0;
static constexpr float kBeaconDetectWindowScaler = 2.33f;
static constexpr bool kEnableCfo = false;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

Receiver::Receiver(
    Config* config, moodycamel::ConcurrentQueue<Event_data>* in_queue,
    std::vector<moodycamel::ConcurrentQueue<Event_data>*> tx_queue,
    std::vector<moodycamel::ProducerToken*> tx_ptoks,
    std::vector<moodycamel::ConcurrentQueue<Event_data>*> cl_tx_queue,
    std::vector<moodycamel::ProducerToken*> cl_tx_ptoks)
    : config_(config),
      message_queue_(in_queue),
      tx_queue_(tx_queue),
      tx_ptoks_(tx_ptoks),
      cl_tx_queue_(cl_tx_queue),
      cl_tx_ptoks_(cl_tx_ptoks) {
  /* initialize random seed: */
  srand(time(NULL));

  MLPD_TRACE("Receiver Construction - CL present: %d, BS Present: %d\n",
             config_->client_present(), config_->bs_present());
  try {
#if defined(USE_UHD)
    this->client_radio_set_ =
        config_->client_present() ? new ClientRadioSetUHD(config_) : nullptr;
    this->base_radio_set_ =
        config_->bs_present() ? new BaseRadioSetUHD(config_) : nullptr;
#else
    this->client_radio_set_ =
        config_->client_present() ? new ClientRadioSet(config_) : nullptr;
    this->base_radio_set_ =
        config_->bs_present() ? new BaseRadioSet(config_) : nullptr;
#endif
  } catch (std::exception& e) {
    throw ReceiverException(e.what());
  }

  MLPD_TRACE("Receiver Construction -- number radios %zu\n",
             config_->num_bs_sdrs_all());

  if (((this->base_radio_set_ != nullptr) &&
       (this->base_radio_set_->getRadioNotFound())) ||
      ((this->client_radio_set_ != nullptr) &&
       (this->client_radio_set_->getRadioNotFound()))) {
    if (this->base_radio_set_ != nullptr) {
      MLPD_WARN("Invalid Base Radio Setup: %d\n",
                this->base_radio_set_ == nullptr);
      this->base_radio_set_->radioStop();
      delete this->base_radio_set_;
    }
    if (this->client_radio_set_ != nullptr) {
      MLPD_WARN("Invalid Client Radio Setup: %d\n",
                this->client_radio_set_ == nullptr);
      this->client_radio_set_->radioStop();
      delete this->client_radio_set_;
    }
    throw ReceiverException("Invalid Radio Setup");
  }

  this->initBuffers();
  MLPD_TRACE("Construction complete\n");
}

Receiver::~Receiver() {
  MLPD_TRACE("Radio Set cleanup, Base: %d, Client: %d\n",
             this->base_radio_set_ == nullptr,
             this->client_radio_set_ == nullptr);
  if (this->base_radio_set_ != nullptr) {
    this->base_radio_set_->radioStop();
    delete this->base_radio_set_;
  }
  if (this->client_radio_set_ != nullptr) {
    this->client_radio_set_->radioStop();
    delete this->client_radio_set_;
  }

  for (auto memory : zeros_) {
    free(memory);
  }
  zeros_.clear();
}

void Receiver::initBuffers() {
  size_t frameTimeLen = config_->samps_per_frame();
  txFrameDelta_ = config_->getTxFrameDelta();
  txTimeDelta_ = txFrameDelta_ * frameTimeLen;

  zeros_.resize(2);
  pilotbuffA_.resize(2);
  pilotbuffB_.resize(2);
  for (auto& memory : zeros_) {
    memory = calloc(config_->samps_per_slot(), sizeof(int16_t) * 2);
    if (memory == NULL) {
      throw std::runtime_error("Error allocating memory");
    }
  }
  pilotbuffA_.at(0) = config_->pilot_ci16().data();
  if (config_->cl_sdr_ch() == 2) {
    pilotbuffA_.at(1) = zeros_.at(0);
    pilotbuffB_.at(1) = config_->pilot_ci16().data();
    pilotbuffB_.at(0) = zeros_.at(1);
  }
}

std::vector<pthread_t> Receiver::startClientThreads(SampleBuffer* rx_buffer,
                                                    SampleBuffer* tx_buffer,
                                                    unsigned in_core_id) {
  cl_tx_buffer_ = tx_buffer;
  std::vector<pthread_t> client_threads;
  if (config_->client_present() == true) {
    client_threads.resize(config_->num_cl_sdrs());
    for (unsigned int i = 0; i < config_->num_cl_sdrs(); i++) {
      pthread_t cl_thread_;
      // record the thread id
      ReceiverContext* context = new ReceiverContext;
      context->ptr = this;
      context->core_id = in_core_id;
      context->tid = i;
      context->buffer = rx_buffer;
      // start socket thread
      if (pthread_create(&cl_thread_, NULL, Receiver::clientTxRx_launch,
                         context) != 0) {
        MLPD_ERROR(
            "Socket client thread create failed in start client "
            "threads");
        throw std::runtime_error(
            "Socket client thread create failed "
            "in start client threads");
      }
      client_threads[i] = cl_thread_;
    }
  }
  return client_threads;
}

std::vector<pthread_t> Receiver::startRecvThreads(SampleBuffer* rx_buffer,
                                                  size_t n_rx_threads,
                                                  SampleBuffer* tx_buffer,
                                                  unsigned in_core_id) {
  assert(rx_buffer[0].buffer.size() != 0);
  thread_num_ = n_rx_threads;
  bs_tx_buffer_ = tx_buffer;
  std::vector<pthread_t> created_threads;
  created_threads.resize(this->thread_num_);
  for (size_t i = 0; i < this->thread_num_; i++) {
    // record the thread id
    ReceiverContext* context = new ReceiverContext;
    context->ptr = this;
    context->core_id = in_core_id;
    context->tid = i;
    context->buffer = rx_buffer;
    // start socket thread
    if (pthread_create(&created_threads.at(i), NULL, Receiver::loopRecv_launch,
                       context) != 0) {
      MLPD_ERROR("Socket recv thread create failed");
      throw std::runtime_error("Socket recv thread create failed");
    }
  }
  sleep(1);
  pthread_cond_broadcast(&cond);
  go();
  return created_threads;
}

void Receiver::completeRecvThreads(const std::vector<pthread_t>& recv_thread) {
  for (std::vector<pthread_t>::const_iterator it = recv_thread.begin();
       it != recv_thread.end(); ++it) {
    pthread_join(*it, NULL);
  }
}

void Receiver::go() {
  if (this->base_radio_set_ != NULL) {
    this->base_radio_set_->radioStart();  // hardware trigger
  }
}

void Receiver::baseTxBeacon(int radio_id, int cell, int frame_id,
                            long long base_time) {
  // prepare BS beacon in host buffer
  std::vector<void*> beaconbuff(2);
  if (config_->beam_sweep() == true) {
    size_t beacon_frame_slot = frame_id % config_->num_bs_antennas_all();
    for (size_t ch = 0; ch < config_->bs_sdr_ch(); ++ch) {
      size_t cell_radio_id = radio_id + config_->n_bs_sdrs_agg().at(cell);
      size_t cell_ant_id = cell_radio_id * config_->bs_sdr_ch();
      int hdmd = CommsLib::hadamard2(beacon_frame_slot, cell_ant_id);
      beaconbuff.at(ch) = hdmd == -1 ? config_->neg_beacon_ci16().data()
                                     : config_->beacon_ci16().data();
    }
  } else {
    if (config_->beacon_radio() == (size_t)radio_id) {
      size_t bcn_ch = config_->beacon_channel();
      beaconbuff.at(bcn_ch) = config_->beacon_ci16().data();
      if (config_->bs_sdr_ch() > 1) beaconbuff.at(1 - bcn_ch) = zeros_.at(0);
    } else {  // set both channels to zeros
      for (size_t ch = 0; ch < config_->bs_sdr_ch(); ++ch)
        beaconbuff.at(ch) = zeros_.at(ch);
    }
  }
  int r_tx;
  r_tx = this->base_radio_set_->radioTx(
      radio_id, cell, beaconbuff.data(), kStreamEndBurst,
      base_time);  // assume beacon is first slot

  if (r_tx != (int)config_->samps_per_slot())
    std::cerr << "BAD Transmit(" << r_tx << "/" << config_->samps_per_slot()
              << ") at Time " << base_time << ", frame count " << frame_id
              << std::endl;
}

int Receiver::baseTxData(int radio_id, int cell, int frame_id,
                         long long base_time) {
  int num_samps = config_->samps_per_slot();
  size_t packetLength = sizeof(Packet) + config_->getPacketDataLength();
  size_t tx_buffer_size = bs_tx_buffer_[radio_id].buffer.size() / packetLength;
  int flagsTxData;
  std::vector<void*> dl_txbuff(2);
  Event_data event;
  if (tx_queue_.at(radio_id)->try_dequeue_from_producer(*tx_ptoks_.at(radio_id),
                                                        event) == true) {
    assert(event.event_type == kEventTxSymbol);
    assert(event.ant_id == radio_id);
    size_t cur_offset = event.offset;
    long long txFrameTime =
        base_time + (event.frame_id - frame_id) * config_->samps_per_frame();
    if (config_->bs_hw_framer() == false)
      this->baseTxBeacon(radio_id, cell, event.frame_id, txFrameTime);
    for (size_t s = 0; s < config_->dl_slot_per_frame(); s++) {
      for (size_t ch = 0; ch < config_->bs_sdr_ch(); ++ch) {
        char* cur_ptr_buffer =
            bs_tx_buffer_[radio_id].buffer.data() + (cur_offset * packetLength);
        Packet* pkt = reinterpret_cast<Packet*>(cur_ptr_buffer);
        assert(pkt->slot_id == config_->dl_slots().at(cell).at(s));
        assert(pkt->ant_id == config_->bs_sdr_ch() * radio_id + ch);
        dl_txbuff.at(ch) = pkt->data;
        cur_offset = (cur_offset + 1) % tx_buffer_size;
      }
      long long txTime = 0;
      if (kUseSoapyUHD == true || kUsePureUHD == true ||
          config_->bs_hw_framer() == false) {
        txTime = txFrameTime +
                 config_->dl_slots().at(radio_id).at(s) * num_samps -
                 config_->tx_advance(radio_id);
      } else {
        //size_t frame_id = (size_t)(base_time >> 32);
        txTime = ((size_t)event.frame_id << 32) |
                 (config_->dl_slots().at(cell).at(s) << 16);
      }
      if ((kUsePureUHD == true || kUseSoapyUHD == true) &&
          s < (config_->dl_slot_per_frame() - 1))
        flagsTxData = kStreamContinuous;  // HAS_TIME
      else
        flagsTxData = kStreamEndBurst;  // HAS_TIME & END_BURST, fixme
      int r;
      r = this->base_radio_set_->radioTx(radio_id, cell, dl_txbuff.data(),
                                         flagsTxData, txTime);

      if (r < num_samps) {
        MLPD_WARN("BAD Write: %d/%d\n", r, num_samps);
      }
    }
    bs_tx_buffer_[radio_id]
        .pkt_buf_inuse[event.frame_id % kSampleBufferFrameNum] = 0;
    return 0;
  }
  return -1;
}

void Receiver::notifyPacket(NodeType node_type, int frame_id, int slot_id,
                            int ant_id, int buff_size, int offset) {
  moodycamel::ProducerToken local_ptok(*message_queue_);
  Event_data new_frame;
  new_frame.event_type = kEventRxSymbol;
  new_frame.frame_id = frame_id;
  new_frame.slot_id = slot_id;
  new_frame.ant_id = ant_id;
  new_frame.node_type = node_type;
  new_frame.buff_size = buff_size;
  new_frame.offset = offset;
  if (message_queue_->enqueue(local_ptok, new_frame) == false) {
    MLPD_ERROR("New frame message enqueue failed\n");
    throw std::runtime_error("New frame message enqueue failed");
  }
}

void* Receiver::loopRecv_launch(void* in_context) {
  ReceiverContext* context = (ReceiverContext*)in_context;
  auto me = context->ptr;
  auto tid = context->tid;
  auto core_id = context->core_id;
  auto buffer = context->buffer;
  delete context;
  me->loopRecv(tid, core_id, buffer);
  return 0;
}

void Receiver::loopRecv(int tid, int core_id, SampleBuffer* rx_buffer) {
  if (config_->core_alloc() == true) {
    MLPD_INFO("Pinning rx thread %d to core %d\n", tid, core_id + tid);
    if (pin_to_core(core_id + tid) != 0) {
      MLPD_ERROR("Pin rx thread %d to core %d failed\n", tid, core_id + tid);
      throw std::runtime_error("Pin rx thread to core failed");
    }
  }

  // Use mutex to sychronize data receiving across threads
  if (config_->internal_measurement() ||
      ((config_->num_cl_sdrs() > 0) && (config_->num_bs_sdrs_all() > 0))) {
    pthread_mutex_lock(&mutex);
    MLPD_INFO("Recv Thread %d: waiting for release\n", tid);
    pthread_cond_wait(&cond, &mutex);
    pthread_mutex_unlock(&mutex);  // unlocking for all other threads
  }

  // use token to speed up
  moodycamel::ProducerToken local_ptok(*message_queue_);

  const size_t num_channels = config_->bs_channel().length();
  size_t packetLength = sizeof(Packet) + config_->getPacketDataLength();
  int buffer_chunk_size = rx_buffer[0].buffer.size() / packetLength;
  int bs_tx_buff_size = kSampleBufferFrameNum * config_->slot_per_frame();

  // handle two channels at each radio
  // this is assuming buffer_chunk_size is at least 2
  std::atomic_int* pkt_buf_inuse = rx_buffer[tid].pkt_buf_inuse;
  char* buffer = rx_buffer[tid].buffer.data();

  size_t num_radios = config_->num_bs_sdrs_all();  //config_->n_bs_sdrs()[0]
  std::vector<size_t> radio_ids_in_thread;
  if (config_->internal_measurement() && config_->ref_node_enable()) {
    if (tid == 0)
      radio_ids_in_thread.push_back(config_->cal_ref_sdr_id());
    else
      // FIXME: Does this work in multi-cell case?
      for (size_t it = 0; it < config_->num_bs_sdrs_all(); it++)
        if (it != config_->cal_ref_sdr_id()) radio_ids_in_thread.push_back(it);
  } else {
    size_t radio_start = (tid * num_radios) / thread_num_;
    size_t radio_end = ((tid + 1) * num_radios) / thread_num_;
    for (size_t it = radio_start; it < radio_end; it++)
      radio_ids_in_thread.push_back(it);
  }
  MLPD_INFO("Receiver thread %d has %zu radios\n", tid,
            radio_ids_in_thread.size());
  MLPD_TRACE(
      " -- %d - radio start: %zu, end: %zu, total radios %zu, thread: %zu\n",
      tid, radio_ids_in_thread.front(), radio_ids_in_thread.back(), num_radios,
      thread_num_);

  // prepare BS beacon in host buffer
  std::vector<void*> beaconbuff(2);
  void* zeroes_memory = calloc(config_->samps_per_slot(), sizeof(int16_t) * 2);

  if (zeroes_memory == NULL) {
    throw std::runtime_error("Memory allocation error");
  }

  MLPD_SYMBOL(
      "Process %d -- Loop Rx Allocated memory at: %p, approx size: %lu\n", tid,
      zeroes_memory, (sizeof(int16_t) * 2) * config_->samps_per_slot());
  beaconbuff.at(0u) = config_->beacon_ci16().data();
  beaconbuff.at(1u) = zeroes_memory;

  long long rxTimeBs(0);

  // read rx_offset to align the FPGA time of the BS
  // by performing dummy readStream()
  std::vector<std::complex<int16_t>> samp_buffer0(config_->samps_per_frame(),
                                                  0);
  std::vector<std::complex<int16_t>> samp_buffer1(config_->samps_per_frame(),
                                                  0);
  std::vector<void*> samp_buffer(2);
  samp_buffer[0] = samp_buffer0.data();
  if (num_channels == 2) samp_buffer[1] = samp_buffer1.data();

  int cell = 0;
  // for UHD device, the first pilot should not have an END_BURST flag
  if (kUseSoapyUHD == true || kUsePureUHD == true ||
      config_->bs_hw_framer() == false) {
    // For multi-USRP BS perform dummy radioRx to avoid initial late packets
    int bs_sync_ret = -1;
    MLPD_INFO("Sync BS host and FPGA timestamp for thread %d\n", tid);
    for (auto& it : radio_ids_in_thread) {
      // Find cell this USRP belongs to..,
      for (size_t i = 0; i <= config_->num_cells(); i++) {
        if (it < config_->n_bs_sdrs_agg().at(i)) {
          cell = i - 1;
          break;
        }
      }
      size_t radio_id = it - config_->n_bs_sdrs_agg().at(cell);
      bs_sync_ret = -1;
      while (bs_sync_ret < 0) {
        bs_sync_ret =
            this->base_radio_set_->radioRx(radio_id, cell, samp_buffer.data(),
                                           config_->samps_per_slot(), rxTimeBs);
      }
    }
  }

  int cursor = 0;
  size_t frame_id = 0;
  size_t slot_id = 0;
  size_t ant_id = 0;
  cell = 0;
  MLPD_INFO("Start BS main recv loop in thread %d\n", tid);
  while (config_->running() == true) {
    // Global updates of frame and slot IDs for USRPs
    if (kUseSoapyUHD == true || kUsePureUHD == true ||
        config_->bs_hw_framer() == false) {
      if (slot_id == config_->slot_per_frame()) {
        slot_id = 0;
        frame_id++;
      }
    }

    // Receive data
    for (auto& it : radio_ids_in_thread) {
      Packet* pkt[num_channels];
      void* samp[num_channels];

      // Find cell this board belongs to...
      for (size_t i = 0; i <= config_->num_cells(); i++) {
        if (it < config_->n_bs_sdrs_agg().at(i)) {
          cell = i - 1;
          break;
        }
      }

      size_t radio_id = it - config_->n_bs_sdrs_agg().at(cell);

      size_t num_packets =
          config_->internal_measurement() &&
                  radio_id == config_->cal_ref_sdr_id() &&
                  config_->ref_node_enable()
              ? 1
              : num_channels;  // receive only on one channel at the ref antenna

      // Set buffer status(es) to full; fail if full already
      for (size_t ch = 0; ch < num_packets; ++ch) {
        int bit = 1 << (cursor + ch) % sizeof(std::atomic_int);
        int offs = (cursor + ch) / sizeof(std::atomic_int);
        int old = std::atomic_fetch_or(&pkt_buf_inuse[offs], bit);  // now full
        // if buffer was full, exit
        if ((old & bit) != 0) {
          MLPD_ERROR("thread %d buffer full\n", tid);
          throw std::runtime_error("Thread %d buffer full\n");
        }
        // Reserved until marked empty by consumer
      }

      // Receive data into buffers
      for (size_t ch = 0; ch < num_packets; ++ch) {
        pkt[ch] = (Packet*)(buffer + (cursor + ch) * packetLength);
        samp[ch] = pkt[ch]->data;
      }
      if (num_packets != num_channels)
        samp[num_channels - 1] = std::vector<char>(packetLength).data();

      assert(this->base_radio_set_ != NULL);

      ant_id = radio_id * num_channels;

      if (kUseSoapyUHD == true || kUsePureUHD == true ||
          config_->bs_hw_framer() == false) {
        int rx_len = config_->samps_per_slot();
        int r;

        // only write received pilot or data into samp
        // otherwise use samp_buffer as a dummy buffer
        if (config_->isPilot(frame_id, slot_id) ||
            config_->isUlData(frame_id, slot_id))
          r = this->base_radio_set_->radioRx(radio_id, cell, samp, rxTimeBs);
        else
          r = this->base_radio_set_->radioRx(radio_id, cell, samp_buffer.data(),
                                             rxTimeBs);

        if (r < 0) {
          config_->running(false);
          break;
        }
        if (r != rx_len) {
          std::cerr << "BAD Receive(" << r << "/" << rx_len << ") at Time "
                    << rxTimeBs << ", frame count " << frame_id << std::endl;
        }

        // schedule all TX slot
        if (slot_id == 0) {
          // schedule downlink slots
          if (config_->dl_data_slot_present() == true) {
            while (-1 != baseTxData(radio_id, cell, frame_id, rxTimeBs))
              ;
            this->notifyPacket(kBS, frame_id + this->txFrameDelta_, 0, radio_id,
                               bs_tx_buff_size);  // Notify new frame
          } else {
            this->baseTxBeacon(radio_id, cell, frame_id,
                               rxTimeBs + txTimeDelta_);
          }  // end if config_->dul_data_slot_present()
        }
        if (!config_->isPilot(frame_id, slot_id) &&
            !config_->isUlData(frame_id, slot_id)) {
          for (size_t ch = 0; ch < num_packets; ++ch) {
            const int bit = 1 << (cursor + ch) % sizeof(std::atomic_int);
            const int offs = (cursor + ch) / sizeof(std::atomic_int);
            const int old =
                std::atomic_fetch_and(&pkt_buf_inuse[offs], ~bit);  // now empty
            // if buffer was empty, exit
            if ((old & bit) != bit) {
              MLPD_ERROR("thread %d freed buffer when already free\n", tid);
              throw std::runtime_error("buffer empty during free\n");
            }
            // Reserved until marked empty by consumer
          }
          continue;
        }

      } else {
        long long frameTime;
        if (this->base_radio_set_->radioRx(radio_id, cell, samp, frameTime) <
            0) {
          config_->running(false);
          break;
        }

        frame_id = (size_t)(frameTime >> 32);
        slot_id = (size_t)((frameTime >> 16) & 0xFFFF);

        if (config_->internal_measurement() && config_->ref_node_enable()) {
          size_t beacon_slot = config_->num_cl_antennas() > 0 ? 1 : 0;
          if (radio_id == config_->cal_ref_sdr_id()) {
            ant_id = slot_id - beacon_slot;
            slot_id = 0;  // downlink reciprocal pilot
          } else {
            slot_id -= config_->cal_ref_sdr_id();  // uplink pilots
          }
        } else if (config_->internal_measurement() &&
                   !config_->ref_node_enable()) {
          // Mapping (compress schedule to eliminate Gs)
          size_t adv = int(slot_id / (config_->guard_mult() * num_channels));
          slot_id = slot_id - ((config_->guard_mult() - 1) * 2 * adv);
        } else if (config_->getClientId(frame_id, slot_id) ==
                   0) {  // first received pilot
          if (config_->dl_data_slot_present() == true) {
            while (-1 != baseTxData(radio_id, cell, frame_id, frameTime))
              ;
            this->notifyPacket(kBS, frame_id + this->txFrameDelta_, 0, radio_id,
                               bs_tx_buff_size);  // Notify new frame
          }
        }
      }

#if DEBUG_PRINT
      for (size_t ch = 0; ch < num_packets; ++ch) {
        printf(
            "receive thread %d, frame %zu, slot %zu, cell %zu, ant "
            "%zu samples: %d %d %d %d %d %d %d %d ...\n",
            tid, frame_id, slot_id, cell, ant_id + ch, pkt[ch]->data[1],
            pkt[ch]->data[2], pkt[ch]->data[3], pkt[ch]->data[4],
            pkt[ch]->data[5], pkt[ch]->data[6], pkt[ch]->data[7],
            pkt[ch]->data[8]);
      }
#endif

      for (size_t ch = 0; ch < num_packets; ++ch) {
        // new (pkt[ch]) Packet(frame_id, slot_id, 0, ant_id + ch);
        new (pkt[ch]) Packet(frame_id, slot_id, cell, ant_id + ch);
        // push kEventRxSymbol event into the queue
        this->notifyPacket(kBS, frame_id, slot_id, ant_id + ch,
                           buffer_chunk_size, cursor + tid * buffer_chunk_size);
        cursor++;
        cursor %= buffer_chunk_size;
      }
    }

    // for UHD device update slot_id on host
    if (kUseSoapyUHD == true || kUsePureUHD == true ||
        config_->bs_hw_framer() == false) {
      slot_id++;
    }
  }
  MLPD_SYMBOL("Process %d -- Loop Rx Freed memory at: %p\n", tid,
              zeroes_memory);
  free(zeroes_memory);
}

void* Receiver::clientTxRx_launch(void* in_context) {
  ReceiverContext* context = (ReceiverContext*)in_context;
  auto me = context->ptr;
  auto tid = context->tid;
  auto core_id = context->core_id;
  auto buffer = context->buffer;
  delete context;
  if (me->config_->hw_framer())
    me->clientTxRx(tid);
  else
    me->clientSyncTxRx(tid, core_id, buffer);
  return 0;
}

void Receiver::clientTxRx(int tid) {
  size_t tx_slots = config_->cl_ul_slots().at(tid).size();
  size_t rxSyms = config_->cl_dl_slots().at(tid).size();
  int txStartSym = config_->cl_ul_slots().at(tid).empty()
                       ? 0
                       : config_->cl_ul_slots().at(tid).at(0);
  int NUM_SAMPS = config_->samps_per_slot();

  if (config_->core_alloc() == true) {
    int core =
        tid + 1 + config_->bs_rx_thread_num() + config_->recorder_thread_num();
    MLPD_INFO("Pinning client TxRx thread %d to core %d\n", tid, core);
    if (pin_to_core(core) != 0) {
      MLPD_ERROR("Pin client TxRx thread %d to core %d failed in client txrx\n",
                 tid, core);
      throw std::runtime_error(
          "Pin client TxRx thread to core failed in client txr");
    }
  }

  std::vector<std::complex<int16_t>> buffs(NUM_SAMPS, 0);
  std::vector<void*> rxbuff(2);
  rxbuff[0] = buffs.data();
  rxbuff[1] = buffs.data();

  std::vector<void*> ul_txbuff(2);
  for (size_t ch = 0; ch < config_->cl_sdr_ch(); ch++) {
    ul_txbuff.at(ch) =
        std::calloc(config_->samps_per_slot(), sizeof(int16_t) * 2);
  }
  //size_t slot_byte_size = config_->samps_per_slot() * sizeof(int16_t) * 2;
  //if (tx_slots > 0) {
  //  size_t txIndex = tid * config_->cl_sdr_ch();
  //  for (size_t ch = 0; ch < config_->cl_sdr_ch(); ch++) {
  //    std::memcpy(ul_txbuff.at(ch),
  //                config_->txdata_time_dom().at(txIndex + ch).data(),
  //                slot_byte_size);
  //  }
  //  MLPD_INFO("%zu uplink slots will be sent per frame...\n", tx_slots);
  //}

  int all_trigs = 0;
  struct timespec tv, tv2;
  clock_gettime(CLOCK_MONOTONIC, &tv);

  assert(client_radio_set_ != NULL);

  while (config_->running() == true) {
    clock_gettime(CLOCK_MONOTONIC, &tv2);
    double diff =
        ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec)) / 1e9;
    if ((config_->frame_mode() != "free_running") && (diff > 2)) {
      int total_trigs;
      total_trigs = client_radio_set_->triggers(tid);

      std::cout << "new triggers: " << total_trigs - all_trigs
                << ", total: " << total_trigs << std::endl;
      all_trigs = total_trigs;
      tv = tv2;
    }
    // receiver loop
    long long rxTime(0);
    long long txTime(0);
    long long firstRxTime(0);
    bool receiveErrors = false;
    for (size_t i = 0; i < rxSyms; i++) {
      int r;
      r = client_radio_set_->radioRx(tid, rxbuff.data(), NUM_SAMPS, rxTime);
      if (r == NUM_SAMPS) {
        if (i == 0) firstRxTime = rxTime;
      } else {
        std::cerr << "waiting for receive frames... " << std::endl;
        receiveErrors = true;
        break;
      }
    }
    if (receiveErrors == false) {
      // transmit loop
      txTime = firstRxTime & 0xFFFFFFFF00000000;
      txTime += ((long long)this->txFrameDelta_ << 32);
      txTime += ((long long)txStartSym << 16);
      //printf("rxTime %llx, txTime %llx \n", firstRxTime, txTime);
      for (size_t i = 0; i < tx_slots; i++) {
        int r;
        r = client_radio_set_->radioTx(tid, ul_txbuff.data(), NUM_SAMPS, 1,
                                       txTime);
        if (r == NUM_SAMPS) {
          txTime += 0x10000;
        }
      }
    }  // end receiveErrors == false)
  }    // end while config_->running() == true)
  for (size_t ch = 0; ch < config_->cl_sdr_ch(); ch++) {
    std::free(ul_txbuff.at(ch));
  }
}

void Receiver::clientTxPilots(size_t user_id, long long base_time) {
  // for UHD device, the first pilot should not have an END_BURST flag
  int flags = (((kUsePureUHD == true || kUseSoapyUHD == true) &&
                (config_->cl_sdr_ch() == 2)))
                  ? kStreamContinuous
                  : kStreamEndBurst;
  int num_samps = config_->samps_per_slot();
  long long txTime = base_time +
                     config_->cl_pilot_slots().at(user_id).at(0) * num_samps -
                     config_->tx_advance(user_id);
  int r;
  r = client_radio_set_->radioTx(user_id, pilotbuffA_.data(), num_samps, flags,
                                 txTime);

  if (r < num_samps) {
    MLPD_WARN("BAD Write: %d/%d\n", r, num_samps);
  }
  if (config_->cl_sdr_ch() == 2) {
    txTime = base_time +
             config_->cl_pilot_slots().at(user_id).at(1) * num_samps -
             config_->tx_advance(user_id);
    r = client_radio_set_->radioTx(user_id, pilotbuffB_.data(), num_samps,
                                   kStreamEndBurst, txTime);

    if (r < num_samps) {
      MLPD_WARN("BAD Write: %d/%d\n", r, num_samps);
    }
  }
}

int Receiver::clientTxData(int tid, int frame_id, long long base_time) {
  size_t tx_slots = config_->cl_ul_slots().at(tid).size();
  int num_samps = config_->samps_per_slot();
  size_t packetLength = sizeof(Packet) + config_->getPacketDataLength();
  size_t tx_buffer_size = cl_tx_buffer_[tid].buffer.size() / packetLength;
  int flagsTxUlData;
  std::vector<void*> ul_txbuff(2);
  Event_data event;
  if (cl_tx_queue_.at(tid)->try_dequeue_from_producer(*cl_tx_ptoks_.at(tid),
                                                      event) == true) {
    assert(event.event_type == kEventTxSymbol);
    assert(event.ant_id == tid);
    size_t cur_offset = event.offset;
    long long txFrameTime =
        base_time + (event.frame_id - frame_id) * config_->samps_per_frame();
    clientTxPilots(tid,
                   txFrameTime);  // assuming pilot is always sent before data

    for (size_t s = 0; s < tx_slots; s++) {
      for (size_t ch = 0; ch < config_->cl_sdr_ch(); ++ch) {
        char* cur_ptr_buffer =
            cl_tx_buffer_[tid].buffer.data() + (cur_offset * packetLength);
        Packet* pkt = reinterpret_cast<Packet*>(cur_ptr_buffer);
        assert(pkt->slot_id == config_->cl_ul_slots().at(tid).at(s));
        assert(pkt->ant_id == config_->cl_sdr_ch() * tid + ch);
        ul_txbuff.at(ch) = pkt->data;
        cur_offset = (cur_offset + 1) % tx_buffer_size;
      }
      long long txTime = txFrameTime +
                         config_->cl_ul_slots().at(tid).at(s) * num_samps -
                         config_->tx_advance(tid);
      if ((kUsePureUHD || kUseSoapyUHD) && s < (tx_slots - 1)) {
        flagsTxUlData = kStreamContinuous;  // HAS_TIME
      } else {
        flagsTxUlData = kStreamEndBurst;  // HAS_TIME & END_BURST, fixme
      }
      int r;
      r = client_radio_set_->radioTx(tid, ul_txbuff.data(), num_samps,
                                     flagsTxUlData, txTime);
      if (r < num_samps) {
        MLPD_WARN("BAD Write: %d/%d\n", r, num_samps);
      }
    }
    cl_tx_buffer_[tid].pkt_buf_inuse[event.frame_id % kSampleBufferFrameNum] =
        0;
    return 0;
  }
  return -1;
}

ssize_t Receiver::syncSearch(const std::complex<int16_t>* check_data,
                             size_t search_window) {
  ssize_t sync_index(-1);
  assert(search_window <= config_->samps_per_frame());
#if defined(__x86_64__)
  sync_index = CommsLib::find_beacon_avx(check_data, config_->gold_cf32(),
                                         search_window);
#else
  sync_index = CommsLib::find_beacon(check_data, search_window);
#endif
  return sync_index;
}

float Receiver::estimateCFO(const std::vector<std::complex<int16_t>>& sync_buff,
                            int sync_index) {
  float cfo_phase_est = 0;
  const int beacon_start = sync_index - config_->beacon_size();
  const int beacon_half_size = config_->beacon_size() / 2;
  std::vector<std::complex<float>> beacon0(beacon_half_size, 0.0f);
  std::vector<std::complex<float>> beacon1(beacon_half_size, 0.0f);
  for (int i = 0; i < beacon_half_size; i++) {
    const size_t beacon0_id = i + beacon_start;
    const size_t beacon1_id = i + beacon_start + beacon_half_size;
    beacon0.at(i) =
        std::complex<float>(sync_buff[beacon0_id].real() / SHRT_MAX,
                            sync_buff[beacon0_id].imag() / SHRT_MAX);
    beacon1.at(i) =
        std::complex<float>(sync_buff[beacon1_id].real() / SHRT_MAX,
                            sync_buff[beacon1_id].imag() / SHRT_MAX);
  }
  const auto cfo_mult = CommsLib::complex_mult_avx(beacon1, beacon0, true);
  float phase = 0.0f;
  float prev_phase = 0.0f;
  for (size_t i = 0; i < cfo_mult.size(); i++) {
    phase = std::arg(cfo_mult.at(i));
    float unwrapped_phase = 0;
    if (i == 0) {
      unwrapped_phase = phase;
    } else {
      float diff = phase - prev_phase;
      if (diff > M_PI)
        diff = diff - 2 * M_PI;
      else if (diff < -M_PI)
        diff = diff + 2 * M_PI;
      unwrapped_phase = prev_phase + diff;
    }
    prev_phase = phase;
    cfo_phase_est += unwrapped_phase;
  }
  cfo_phase_est /= (M_PI * cfo_mult.size() * config_->beacon_size());
  return cfo_phase_est;
}

void Receiver::clientSyncTxRx(int tid, int core_id, SampleBuffer* rx_buffer) {
  if (config_->core_alloc() == true) {
    const int core = tid + core_id;

    MLPD_INFO("Pinning client synctxrx thread %d to core %d\n", tid, core);
    if (pin_to_core(core) != 0) {
      MLPD_ERROR("Pin client synctxrx thread %d to core %d failed\n", tid,
                 core);
      throw std::runtime_error("Failed to Pin client synctxrx thread to core");
    }
  }

  MLPD_INFO("Scheduling TX: %zu Frames (%lf ms) in the future!\n",
            this->txFrameDelta_,
            this->txFrameDelta_ * (1e3 * config_->getFrameDurationSec()));

  const size_t samples_per_slot = config_->samps_per_slot();
  const size_t num_rx_buffs = config_->cl_sdr_ch();
  std::vector<std::vector<std::complex<int16_t>>> samplemem(
      num_rx_buffs, std::vector<std::complex<int16_t>>(
                        samples_per_slot, std::complex<int16_t>(0, 0)));

  std::vector<void*> rxbuff;
  for (size_t ch = 0; ch < num_rx_buffs; ch++) {
    rxbuff.push_back(samplemem.at(ch).data());
  }

  const size_t ant_id = tid * config_->cl_sdr_ch();
  // use token to speed up
  moodycamel::ProducerToken local_ptok(*message_queue_);

  char* buffer = nullptr;
  std::atomic_int* pkt_buf_inuse = nullptr;
  int buffer_chunk_size = 0;
  int buffer_id = tid + config_->bs_rx_thread_num();
  size_t packetLength = sizeof(Packet) + config_->getPacketDataLength();
  if (config_->cl_dl_slots().at(0).empty() == false) {
    buffer_chunk_size = rx_buffer[buffer_id].buffer.size() / packetLength;
    // handle two channels at each radio
    // this is assuming buffer_chunk_size is at least 2
    pkt_buf_inuse = rx_buffer[buffer_id].pkt_buf_inuse;
    buffer = rx_buffer[buffer_id].buffer.data();
  }

  // tx_buffer info
  size_t tx_buffer_size = 0;
  if (config_->ul_data_slot_present() == true) {
    tx_buffer_size = cl_tx_buffer_[tid].buffer.size() / packetLength;
  }

  // For USRP clients skip UHD_INIT_TIME_SEC to avoid late packets
  if (kUsePureUHD == true || kUseSoapyUHD == true) {
    auto start_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsed_seconds{0.0};

    while (elapsed_seconds.count() < UHD_INIT_TIME_SEC) {
      long long ignore_time;
      client_radio_set_->radioRx(tid, rxbuff.data(), samples_per_slot,
                                 ignore_time);
      elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::steady_clock::now() - start_time);
    }
    std::printf("Wait duration %3.2f seconds \n", elapsed_seconds.count());
  }

  //-------------------- New sync
  const size_t beacon_detect_window =
      static_cast<size_t>(static_cast<float>(config_->samps_per_slot()) *
                          kBeaconDetectWindowScaler);
  size_t sync_count = 0;
  constexpr size_t kTargetSyncCount = 2;
  assert(config_->samps_per_frame() >= beacon_detect_window);
  while ((sync_count < kTargetSyncCount) && config_->running()) {
    const ssize_t sync_index = clientSyncBeacon(tid, beacon_detect_window);
    if (sync_index >= 0) {
      const ssize_t adjust =
          sync_index - (config_->beacon_size() + config_->prefix());
      const size_t alignment_samples =
          config_->samps_per_frame() - beacon_detect_window;
      MLPD_INFO(
          "clientSyncTxRx [%d]: Beacon detected sync_index: %ld, rx sample "
          "offset: %ld, window %zu, samples in frame %zu, alignment removal "
          "%zu\n",
          tid, sync_index, adjust, beacon_detect_window,
          config_->samps_per_frame(), alignment_samples);

      //By definition alignment_samples + adjust must be > 0;
      if (static_cast<ssize_t>(alignment_samples) + adjust < 0) {
        throw std::runtime_error("Unexpected alignment");
      }
      clientAdjustRx(tid, alignment_samples + adjust);
      sync_count++;
    } else if (config_->running()) {
      MLPD_WARN(
          "clientSyncTxRx [%d]: Beacon could not be detected sync_index: %ld\n",
          tid, sync_index);
      throw std::runtime_error("rx sample offset is less than 0");
    }
  }

  // Main client read/write loop.
  size_t frame_id = 0;
  size_t buffer_offset = 0;
  //sync on the first beacon after initial detection
  bool resync = true;
  bool resync_enable = (config_->frame_mode() == "continuous_resync");
  size_t resync_retry_cnt(0);
  size_t resync_retry_max(100);
  size_t resync_success(0);
  // TODO: measure CFO from the first beacon and apply here
  const size_t max_cfo = 100;  // in ppb, For Iris
  const size_t resync_period = static_cast<size_t>(
      std::floor(1e9 / (max_cfo * config_->samps_per_frame())));
  size_t last_resync = frame_id;
  MLPD_INFO("Start main client txrx loop... tid=%d with resync period of %zu\n",
            tid, resync_period);
  long long rx_beacon_time(0);
  //Always decreases the requested rx samples
  size_t beacon_adjust = 0;

  while (config_->running() == true) {
    if (config_->max_frame() > 0 && frame_id >= config_->max_frame()) {
      config_->running(false);
      break;
    }
    //Slot 0 / Beacon...
    const int request_samples = samples_per_slot - beacon_adjust;
    const int rx_status = client_radio_set_->radioRx(
        tid, rxbuff.data(), request_samples, rx_beacon_time);
    beacon_adjust = 0;
    if (rx_status < 0) {
      MLPD_ERROR("Rx status reporting error %d, exiting\n", rx_status);
      config_->running(false);
      break;
    }
    if (config_->ul_data_slot_present() == true) {
      // Notify new frame
      this->notifyPacket(kClient, frame_id + this->txFrameDelta_, 0, tid,
                         tx_buffer_size);
    }

    if ((frame_id - last_resync) >= resync_period) {
      resync = resync_enable;
      last_resync = frame_id;
      MLPD_TRACE("Enable resyncing at frame %zu\n", frame_id);
    }
    if (resync == true) {
      ssize_t sync_index =
          this->syncSearch(reinterpret_cast<std::complex<int16_t>*>(
                               rxbuff.at(kSyncDetectChannel)),
                           request_samples);
      if (sync_index >= 0) {
        const int new_rx_offset =
            sync_index - (config_->beacon_size() + config_->prefix());
        //Adjust tx time
        rx_beacon_time += new_rx_offset;
        resync = false;
        resync_retry_cnt = 0;
        resync_success++;
        MLPD_INFO(
            "Re-syncing success at frame %zu with offset: %d, after %zu tries, "
            "index: %ld, tid %d\n",
            frame_id, new_rx_offset, resync_retry_cnt + 1, sync_index, tid);

        if (kEnableCfo && (sync_index >= 0)) {
          const auto cfo_phase_est =
              estimateCFO(samplemem.at(kSyncDetectChannel), sync_index);
          MLPD_INFO("Client %d Estimated CFO (Hz): %f\n", tid,
                    cfo_phase_est * config_->rate());
        }

        //Offset Alignment logic
        if (new_rx_offset < 0) {
          beacon_adjust = (-1 * new_rx_offset);
        } else if (new_rx_offset > 0) {
          const size_t discard_samples = new_rx_offset;
          //throw away samples to get back in alignment, could combine with the next beacon but would need bigger buffers
          clientAdjustRx(tid, discard_samples);
        }
      } else {
        resync_retry_cnt++;

        if (resync_retry_cnt > resync_retry_max) {
          MLPD_WARN(
              "Exceeded resync retry limit (%zu) for client %d reached after "
              "%zu resync successes at frame: %zu.  Stopping!\n",
              resync_retry_max, tid, resync_success, frame_id);
          resync = false;
          resync_retry_cnt = 0;
          config_->running(false);
          break;
        }
      }
    }
    // schedule all TX slot
    // config_->tx_advance() needs calibration based on SDR model and sampling rate
    if (config_->ul_data_slot_present() == true) {
      int tx_return = 0;
      while (tx_return >= 0) {
        tx_return = this->clientTxData(tid, frame_id, rx_beacon_time);
      }
    } else {
      if (config_->cl_pilot_slots().at(tid).size() > 0) {
        this->clientTxPilots(tid, rx_beacon_time + txTimeDelta_);
      }
    }  // end if config_->ul_data_slot_present()

    //Beacon + Tx Complete, process the rest of the slots
    for (size_t slot_id = 1; slot_id < config_->slot_per_frame(); slot_id++) {
      int rx_data_status;
      long long rx_data_time;
      if (config_->isDlData(tid, slot_id)) {
        // Set buffer status(es) to full; fail if full already
        for (size_t ch = 0; ch < config_->cl_sdr_ch(); ++ch) {
          const int bit = 1 << (buffer_offset + ch) % sizeof(std::atomic_int);
          const int offs = (buffer_offset + ch) / sizeof(std::atomic_int);
          const int old =
              std::atomic_fetch_or(&pkt_buf_inuse[offs], bit);  // now full
          // if buffer was full, exit
          if ((old & bit) != 0) {
            MLPD_ERROR("thread %d buffer full\n", tid);
            throw std::runtime_error("Thread %d buffer full\n");
          }
          // Reserved until marked empty by consumer
        }

        // Receive data into buffers
        std::vector<Packet*> pkts(config_->cl_sdr_ch());
        std::vector<void*> dl_slot_samp(config_->cl_sdr_ch());
        for (size_t ch = 0; ch < config_->cl_sdr_ch(); ++ch) {
          pkts.at(ch) = reinterpret_cast<Packet*>(
              buffer + (buffer_offset + ch) * packetLength);
          dl_slot_samp.at(ch) = pkts.at(ch)->data;
        }

        rx_data_status = this->client_radio_set_->radioRx(
            tid, dl_slot_samp.data(), samples_per_slot, rx_data_time);
        for (size_t ch = 0; ch < config_->cl_sdr_ch(); ++ch) {
          new (pkts.at(ch)) Packet(frame_id, slot_id, 0, ant_id + ch);
          // push kEventRxSymbol event into the queue
          this->notifyPacket(kClient, frame_id, slot_id, ant_id + ch,
                             buffer_chunk_size,
                             buffer_offset + buffer_id * buffer_chunk_size);
          buffer_offset++;
          buffer_offset %= buffer_chunk_size;
        }
      } else {
        //Not dl data so we throw it away
        rx_data_status = this->client_radio_set_->radioRx(
            tid, rxbuff.data(), samples_per_slot, rx_data_time);
      }
      if (rx_data_status < 0) {
        MLPD_ERROR(
            "Rx status reporting error %d during frame %zu , slot %zu, "
            "exiting\n",
            rx_data_status, frame_id, slot_id);
        config_->running(false);
        break;
      } else if (rx_data_status != static_cast<int>(samples_per_slot)) {
        MLPD_WARN("BAD Receive(%d/%zu) at Time %lld, frame count %zu\n",
                  rx_data_status, samples_per_slot, rx_data_time, frame_id);
      }
    }  // end for
    frame_id++;
  }  // end while
}

//Blocking function for beacon detected or exit()
ssize_t Receiver::clientSyncBeacon(size_t radio_id, size_t sample_window) {
  ssize_t sync_index = -1;
  long long rx_time = 0;
  assert(sample_window <= config_->samps_per_frame());
  const size_t num_rx_buffs = config_->cl_sdr_ch();
  std::vector<std::vector<std::complex<int16_t>>> syncbuffmem(
      num_rx_buffs, std::vector<std::complex<int16_t>>(
                        sample_window, std::complex<int16_t>(0, 0)));

  std::vector<void*> syncrxbuffs;
  for (size_t ch = 0; ch < num_rx_buffs; ch++) {
    syncrxbuffs.push_back(syncbuffmem.at(ch).data());
  }

  while (config_->running() && (sync_index < 0)) {
    const int rx_status = client_radio_set_->radioRx(
        radio_id, syncrxbuffs.data(), sample_window, rx_time);

    if (rx_status < 0) {
      MLPD_ERROR("clientSyncBeacon [%zu]: BAD SYNC Received (%d/%zu) %lld\n",
                 radio_id, rx_status, sample_window, rx_time);
    } else {
      const size_t new_samples = static_cast<size_t>(rx_status);
      if (new_samples == sample_window) {
        MLPD_TRACE(
            "clientSyncBeacon - Samples %zu - Window %zu - Check Beacon %ld\n",
            new_samples, sample_window);

        sync_index = syncSearch(syncbuffmem.at(kSyncDetectChannel).data(),
                                sample_window);
      } else {
        MLPD_ERROR(
            "clientSyncBeacon [%zu]: BAD SYNC - Rx samples not requested size "
            "(%zu/%zu) %lld\n",
            radio_id, new_samples, sample_window, rx_time);
      }
    }
  }  // end while sync_index < 0
  return sync_index;
}

/// This function blocks untill all the discard_samples are received for a given radio
void Receiver::clientAdjustRx(size_t radio_id, size_t discard_samples) {
  const size_t num_rx_buffs = config_->cl_sdr_ch();
  long long rx_time = 0;

  //This can be fixed and combined with other scratch memory
  std::vector<std::vector<std::complex<int16_t>>> temp_mem(
      num_rx_buffs, std::vector<std::complex<int16_t>>(discard_samples));

  std::vector<void*> trash_memory;
  for (size_t ch = 0; ch < num_rx_buffs; ch++) {
    trash_memory.push_back(temp_mem.at(ch).data());
  }

  while (config_->running() && (discard_samples > 0)) {
    const int rx_status = client_radio_set_->radioRx(
        radio_id, trash_memory.data(), discard_samples, rx_time);

    if (rx_status < 0) {
      MLPD_ERROR(
          "clientAdjustRx [%zu]: BAD Rx Adjust Status Received (%d/%zu) %lld\n",
          radio_id, rx_status, discard_samples, rx_time);
    } else {
      size_t new_samples = static_cast<size_t>(rx_status);
      if (new_samples <= discard_samples) {
        discard_samples -= new_samples;
        MLPD_TRACE("clientAdjustRx [%zu]: Discarded Samples (%zu/%zu)\n",
                   radio_id, new_samples, discard_samples);
      } else {
        MLPD_ERROR(
            "clientAdjustRx [%zu]: BAD radioRx more samples then requested "
            "(%zu/%zu) %lld\n",
            radio_id, new_samples, discard_samples, rx_time);
      }
    }
  }  // request_samples > 0
}
