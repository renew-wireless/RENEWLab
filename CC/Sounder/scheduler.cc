/*
 Copyright (c) 2018-2022, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 Record received frames from massive-mimo base station in HDF5 format
---------------------------------------------------------------------
*/
#include "include/scheduler.h"

#include "include/logger.h"
#include "include/macros.h"
#include "include/signalHandler.hpp"
#include "include/utils.h"

namespace Sounder {
// dequeue bulk size, used to reduce the overhead of dequeue in main thread
const int Scheduler::KDequeueBulkSize = 5;

#if (DEBUG_PRINT)
const int kDsSim = 5;
#endif

Scheduler::Scheduler(Config* in_cfg, unsigned int core_start)
    : cfg_(in_cfg),
      kMainDispatchCore(core_start),
      kSchedulerCore(kMainDispatchCore + 1),
      kRecvCore(kSchedulerCore + in_cfg->recorder_thread_num() +
                in_cfg->reader_thread_num()) {
  size_t bs_rx_thread_num = cfg_->bs_rx_thread_num();
  size_t cl_rx_thread_num = cfg_->cl_rx_thread_num();
  size_t total_rx_thread_num = bs_rx_thread_num + cl_rx_thread_num;
  size_t sdr_per_rx_thread =
      total_rx_thread_num > 0 ? cfg_->getNumRecordedSdrs() / total_rx_thread_num
                              : 1;
  rx_thread_buff_size_ =
      kSampleBufferFrameNum * cfg_->slot_per_frame() * sdr_per_rx_thread;

  message_queue_ = moodycamel::ConcurrentQueue<Event_data>(
      rx_thread_buff_size_ * kQueueSize);
  MLPD_TRACE(
      "Scheduler construction: rx threads: %zu, recorder threads: %zu, reader "
      "threads: %zu, "
      "chunk size: %zu\n",
      total_rx_thread_num, cfg_->recorder_thread_num(),
      cfg_->reader_thread_num(), rx_thread_buff_size_);

  if (total_rx_thread_num > 0) {
    // initialize rx buffers
    rx_buffer_ = new SampleBuffer[total_rx_thread_num];
    size_t intsize = sizeof(std::atomic_int);
    size_t arraysize = (rx_thread_buff_size_ + intsize - 1) / intsize;
    size_t packetLength = sizeof(Packet) + cfg_->getPacketDataLength();
    for (size_t i = 0; i < total_rx_thread_num; i++) {
      rx_buffer_[i].buffer.resize(rx_thread_buff_size_ * packetLength);
      rx_buffer_[i].pkt_buf_inuse = new std::atomic_int[arraysize];
      std::fill_n(rx_buffer_[i].pkt_buf_inuse, arraysize, 0);
    }
  }

  if (cfg_->ul_slot_per_frame() > 0) {
    // initialize rx buffers
    cl_tx_buffer_ = new SampleBuffer[cfg_->num_cl_sdrs()];
    this->cl_tx_thread_buff_size_ =
        kSampleBufferFrameNum * cfg_->slot_per_frame();
    size_t packetLength = sizeof(Packet) + cfg_->getPacketDataLength();
    for (size_t i = 0; i < cfg_->num_cl_sdrs(); i++) {
      cl_tx_buffer_[i].buffer.resize(cl_tx_thread_buff_size_ * packetLength);
      cl_tx_buffer_[i].pkt_buf_inuse =
          new std::atomic_int[kSampleBufferFrameNum];
      std::fill_n(cl_tx_buffer_[i].pkt_buf_inuse, kSampleBufferFrameNum, 0);
      cl_tx_queue_.push_back(new moodycamel::ConcurrentQueue<Event_data>(
          cl_tx_thread_buff_size_ * kQueueSize));
      cl_tx_ptoks_ptr_.push_back(
          new moodycamel::ProducerToken(*cl_tx_queue_.back()));
    }
  }

  if (cfg_->dl_slot_per_frame() > 0) {
    // initialize rx buffers
    bs_tx_buffer_ = new SampleBuffer[cfg_->num_bs_sdrs_all()];
    this->bs_tx_thread_buff_size_ =
        kSampleBufferFrameNum * cfg_->slot_per_frame();
    size_t intsize = sizeof(std::atomic_int);
    size_t arraysize = (bs_tx_thread_buff_size_ + intsize - 1) / intsize;
    size_t packetLength = sizeof(Packet) + cfg_->getPacketDataLength();
    for (size_t i = 0; i < cfg_->num_bs_sdrs_all(); i++) {
      bs_tx_buffer_[i].buffer.resize(bs_tx_thread_buff_size_ * packetLength);
      bs_tx_buffer_[i].pkt_buf_inuse = new std::atomic_int[arraysize];
      std::fill_n(bs_tx_buffer_[i].pkt_buf_inuse, arraysize, 0);
      tx_queue_.push_back(new moodycamel::ConcurrentQueue<Event_data>(
          bs_tx_thread_buff_size_ * kQueueSize));
      tx_ptoks_ptr_.push_back(new moodycamel::ProducerToken(*tx_queue_.back()));
    }
  }

  // Receiver object will be used for both BS and clients
  try {
    receiver_.reset(new Receiver(cfg_, &message_queue_, tx_queue_,
                                 tx_ptoks_ptr_, cl_tx_queue_,
                                 cl_tx_ptoks_ptr_));
  } catch (ReceiverException& re) {
    std::cout << re.what() << '\n';
    gc();
    throw ReceiverException("Radios Not Found. Will attempt a retry...");
  }
}

void Scheduler::gc(void) {
  MLPD_TRACE("Garbage collect\n");
  this->receiver_.reset();
  if (this->cfg_->bs_rx_thread_num() > 0) {
    for (size_t i = 0; i < this->cfg_->bs_rx_thread_num(); i++) {
      delete[] this->rx_buffer_[i].pkt_buf_inuse;
    }
    delete[] this->rx_buffer_;
  }
}

Scheduler::~Scheduler() { this->gc(); }

void Scheduler::do_it() {
  size_t recorder_threads = this->cfg_->recorder_thread_num();
  size_t total_antennas = cfg_->getNumRecordedSdrs() * cfg_->bs_sdr_ch();
  size_t total_rx_thread_num =
      cfg_->bs_rx_thread_num() + cfg_->cl_rx_thread_num();
  size_t thread_antennas = 0;
  std::vector<pthread_t> recv_threads;

  if (this->cfg_->core_alloc() == true) {
    if (Utils::PinToCore(kMainDispatchCore) != 0) {
      std::string err_str =
          std::string("Pinning main recorder thread to core ") +
          std::to_string(kMainDispatchCore) + std::string(" failed");
      throw std::runtime_error(err_str);
    } else {
      MLPD_INFO("Successfully pinned main scheduler thread to core %d\n",
                kMainDispatchCore);
    }
  }

  if (this->cfg_->client_present() == true) {
    auto client_threads = this->receiver_->startClientThreads(
        this->rx_buffer_, this->cl_tx_buffer_,
        kRecvCore + cfg_->bs_rx_thread_num());
  }

  if (cfg_->reader_thread_num() > 0) {
    int reader_thread_index = 0;
    this->readers_.resize(2);
    if (cfg_->dl_slot_per_frame() > 0 && cfg_->bs_present()) {
      int thread_core = -1;
      if (this->cfg_->core_alloc() == true) {
        thread_core = kSchedulerCore + recorder_threads + reader_thread_index;
        reader_thread_index++;
      }
      Sounder::Hdf5Reader* bs_hdf5_reader = new Sounder::Hdf5Reader(
          this->cfg_, this->message_queue_, bs_tx_buffer_, 0, thread_core,
          this->bs_tx_thread_buff_size_ * kQueueSize, true);
      bs_hdf5_reader->Start();
      this->readers_.at(0) = bs_hdf5_reader;
    }
    if (cfg_->ul_slot_per_frame() > 0 && cfg_->client_present()) {
      int thread_core = -1;
      if (this->cfg_->core_alloc() == true) {
        thread_core = kSchedulerCore + recorder_threads + reader_thread_index;
        reader_thread_index++;
      }
      Sounder::Hdf5Reader* cl_hdf5_reader = new Sounder::Hdf5Reader(
          this->cfg_, this->message_queue_, cl_tx_buffer_, 1, thread_core,
          this->cl_tx_thread_buff_size_ * kQueueSize, true);
      cl_hdf5_reader->Start();
      this->readers_.at(1) = cl_hdf5_reader;
    }
  }

  if (total_rx_thread_num > 0) {
    thread_antennas = (total_antennas / recorder_threads);
    // If antennas are left, distribute them over the threads. This may assign antennas that don't
    // exist to the threads at the end. This isn't a concern.
    if ((total_antennas % recorder_threads) != 0) {
      thread_antennas = (thread_antennas + 1);
    }

    for (unsigned int i = 0u; i < recorder_threads; i++) {
      int thread_core = -1;
      if (this->cfg_->core_alloc() == true) {
        thread_core = kSchedulerCore + i;
      }

      MLPD_INFO(
          "Creating recorder thread: %u, with antennas %zu:%zu "
          "total %zu\n",
          i, (i * thread_antennas), ((i + 1) * thread_antennas) - 1,
          thread_antennas);
      Sounder::RecorderThread* new_recorder = new Sounder::RecorderThread(
          this->cfg_, i, thread_core, (this->rx_thread_buff_size_ * kQueueSize),
          (i * thread_antennas), thread_antennas, true);
      new_recorder->Start();
      this->recorders_.push_back(new_recorder);
    }
    if (cfg_->bs_rx_thread_num() > 0) {
      // create socket buffer and socket threads
      recv_threads = this->receiver_->startRecvThreads(
          this->rx_buffer_, cfg_->bs_rx_thread_num(), this->bs_tx_buffer_,
          kRecvCore);
    }
  } else
    this->receiver_->go();  // only beamsweeping

  moodycamel::ConsumerToken ctok(this->message_queue_);

  Event_data events_list[KDequeueBulkSize];
  int ret = 0;

  /* TODO : we can probably remove the dispatch function and pass directly to the recievers */
  while ((this->cfg_->running() == true) &&
         (SignalHandler::gotExitSignal() == false)) {
    // get a bulk of events from the receivers
    ret = this->message_queue_.try_dequeue_bulk(ctok, events_list,
                                                KDequeueBulkSize);
    // handle each event
    for (int bulk_count = 0; bulk_count < ret; bulk_count++) {
      Event_data& event = events_list[bulk_count];

      // if kEventRxSymbol, dispatch to proper worker
      if (event.event_type == kEventRxSymbol) {
        if (cfg_->internal_measurement() == false &&
            event.slot_id == 0) {  // Beacon event, schedule read
          Event_data do_read_task;
          do_read_task.event_type = kTaskRead;
          do_read_task.ant_id = event.ant_id;
          do_read_task.frame_id = event.frame_id;
          do_read_task.node_type = event.node_type;
          do_read_task.buff_size = event.buff_size;
          if (this->readers_.at(event.node_type)->DispatchWork(do_read_task) ==
              false) {
            MLPD_ERROR("Record task enqueue failed\n");
            throw std::runtime_error("Record task enqueue failed");
          }
        } else {
          // Pass the work off to the applicable worker
          // Worker must free the buffer, future work would involve making this cleaner
          size_t thread_index = event.ant_id / thread_antennas;
          Event_data do_record_task;
          do_record_task.event_type = kTaskRecord;
          do_record_task.offset = event.offset;
          do_record_task.buffer = this->rx_buffer_;
          do_record_task.buff_size = this->rx_thread_buff_size_;
          if (this->recorders_.at(thread_index)->DispatchWork(do_record_task) ==
              false) {
            MLPD_ERROR("Record task enqueue failed\n");
            throw std::runtime_error("Record task enqueue failed");
          }
        }
      } else if (event.event_type == kTaskRead) {
        size_t ant_id = event.ant_id;
        Event_data do_tx_task;
        do_tx_task.event_type = kEventTxSymbol;
        do_tx_task.offset = event.offset;
        do_tx_task.ant_id = ant_id;
        do_tx_task.frame_id = event.frame_id;
        do_tx_task.node_type = event.node_type;
        if (event.node_type == kClient) {
          if (cl_tx_queue_.at(ant_id)->try_enqueue(*cl_tx_ptoks_ptr_.at(ant_id),
                                                   do_tx_task) == 0) {
            MLPD_WARN(
                "Queue limit has reached! try to increase queue "
                "size.\n");
            if (cl_tx_queue_.at(ant_id)->enqueue(*cl_tx_ptoks_ptr_.at(ant_id),
                                                 do_tx_task) == 0) {
              MLPD_ERROR("Record task enqueue failed\n");
              throw std::runtime_error("Record task enqueue failed");
            }
          }
        } else {  // Push BS Tx event
          if (tx_queue_.at(ant_id)->try_enqueue(*tx_ptoks_ptr_.at(ant_id),
                                                do_tx_task) == 0) {
            MLPD_WARN(
                "Queue limit has reached! try to increase queue "
                "size.\n");
            if (tx_queue_.at(ant_id)->enqueue(*tx_ptoks_ptr_.at(ant_id),
                                              do_tx_task) == 0) {
              MLPD_ERROR("Record task enqueue failed\n");
              throw std::runtime_error("Record task enqueue failed");
            }
          }
        }
      }
    }
  }
  this->cfg_->running(false);
  this->receiver_->completeRecvThreads(recv_threads);
  this->receiver_.reset();

  /* Force the recorders to process all of the data they have left and exit cleanly
         * Send a stop to all the recorders to allow the finalization to be done in parrallel */
  for (auto recorder : this->recorders_) {
    recorder->Stop();
  }
  for (auto recorder : this->recorders_) {
    delete recorder;
  }
  this->recorders_.clear();
}

int Scheduler::getRecordedFrameNum() { return this->max_frame_number_; }

extern "C" {
Scheduler* Scheduler_new(Config* in_cfg) {
  Scheduler* rec = new Scheduler(in_cfg);
  return rec;
}

void Scheduler_start(Scheduler* rec) { rec->do_it(); }
int Scheduler_getRecordedFrameNum(Scheduler* rec) {
  return rec->getRecordedFrameNum();
}
const char* Scheduler_getTraceFileName(Scheduler* rec) {
  return rec->getTraceFileName().c_str();
}
}
};  // end namespace Sounder
