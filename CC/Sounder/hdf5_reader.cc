/*
 Copyright (c) 2018-2022, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

---------------------------------------------------------------------
 Event based message queue thread class for the recorder worker
---------------------------------------------------------------------
*/

#include "include/hdf5_reader.h"

#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"

namespace Sounder {

Hdf5Reader::Hdf5Reader(Config* in_cfg,
                       moodycamel::ConcurrentQueue<Event_data>& msg_queue,
                       SampleBuffer* tx_buffer, size_t thread_id, int core,
                       size_t queue_size, bool wait_signal)
    : msg_queue_(msg_queue),
      event_queue_(queue_size),
      producer_token_(event_queue_),
      tx_buffer_(tx_buffer),
      config_(in_cfg),
      thread_(),
      id_(thread_id),
      core_alloc_(core),
      wait_signal_(wait_signal) {
  packet_data_length_ = in_cfg->getPacketDataLength();
  running_ = false;
}

Hdf5Reader::~Hdf5Reader() { Finalize(); }

//Launching thread in seperate function to guarantee that the object is fully constructed
//before calling member function
void Hdf5Reader::Start(void) {
  MLPD_INFO("Launching reader task thread with id: %zu and core %d\n",
            this->id_, this->core_alloc_);
  {
    std::lock_guard<std::mutex> thread_lock(this->sync_);
    this->thread_ = std::thread(&Hdf5Reader::DoReading, this);
    this->running_ = true;
  }
  this->condition_.notify_all();
}

/* Cleanly allows the thread to exit */
void Hdf5Reader::Stop(void) {
  Event_data event;
  event.event_type = kThreadTermination;
  this->DispatchWork(event);
}

void Hdf5Reader::Finalize(void) {
  //Wait for thread to cleanly finish the messages in the queue
  if (this->thread_.joinable() == true) {
    MLPD_TRACE("Joining Reader Thread on CPU %d \n", sched_getcpu());
    this->Stop();
    this->thread_.join();
  }
}

/* TODO:  handle producer token better */
//Returns true for success, false otherwise
bool Hdf5Reader::DispatchWork(Event_data event) {
  bool ret = true;
  if (this->event_queue_.try_enqueue(this->producer_token_, event) == 0) {
    MLPD_WARN("Queue limit has reached! try to increase queue size.\n");
    if (this->event_queue_.enqueue(this->producer_token_, event) == 0) {
      MLPD_ERROR("Record task enqueue failed\n");
      throw std::runtime_error("Read task enqueue failed");
      ret = false;
    }
  }

  if (this->wait_signal_ == true) {
    if (ret == true) {
      std::lock_guard<std::mutex> thread_lock(this->sync_);
    }
    this->condition_.notify_all();
  }
  return ret;
}

Event_data Hdf5Reader::ReadFrame(Event_data event, int* offset) {
  size_t data_frame_num = event.node_type == kBS ? config_->dl_data_frame_num()
                                                 : config_->ul_data_frame_num();
  size_t frame_offset = event.frame_id % data_frame_num;
  size_t num_tx_slots = event.node_type == kBS ? config_->dl_slot_per_frame()
                                               : config_->ul_slot_per_frame();
  std::vector<std::vector<size_t>> tx_slots = (event.node_type == kBS)
                                                  ? config_->cl_dl_slots()
                                                  : config_->cl_ul_slots();
  size_t num_ch =
      event.node_type == kBS ? config_->bs_sdr_ch() : config_->cl_sdr_ch();
  size_t radio_id = event.ant_id;
  size_t sched_id = event.node_type == kBS
                        ? 0  // TODO: consider multicell for kBS
                        : radio_id;
  size_t packet_length = sizeof(Packet) + this->packet_data_length_;
  size_t tx_buf_size = tx_buffer_[radio_id].buffer.size() / packet_length;
  int first_offset = offset[radio_id];
  if (event.frame_id > 0 && frame_offset == 0) {
    std::fseek(fp.at(radio_id), 0, SEEK_SET);
  }
  for (size_t s = 0; s < num_tx_slots; s++) {
    for (size_t ch = 0; ch < num_ch; ch++) {
      char* cur_ptr_buffer = (char*)tx_buffer_[radio_id].buffer.data() +
                             (offset[radio_id] * packet_length);
      offset[radio_id] = (offset[radio_id] + 1) % tx_buf_size;
      Packet* pkt = (Packet*)cur_ptr_buffer;
      size_t read_num = std::fread(pkt->data, 2 * sizeof(int16_t),
                                   config_->samps_per_slot(), fp.at(radio_id));
      if (read_num != config_->samps_per_slot()) {
        MLPD_WARN("BAD File Read For Frame %d: %zu/%zu\n", event.frame_id,
                  read_num, config_->samps_per_slot());
      }
      pkt->frame_id = event.frame_id;
      pkt->slot_id = tx_slots.at(sched_id).at(s);
      pkt->ant_id = radio_id * num_ch + ch;
    }
  }
  Event_data read_complete;
  read_complete.event_type = kTaskRead;
  read_complete.ant_id = radio_id;
  read_complete.offset = first_offset;
  read_complete.frame_id = event.frame_id;
  read_complete.node_type = event.node_type;
  return read_complete;
}

void Hdf5Reader::DoReading(void) {
  //Sync the start
  {
    std::unique_lock<std::mutex> thread_wait(this->sync_);
    this->condition_.wait(thread_wait, [this] { return this->running_; });
  }

  if (this->core_alloc_ >= 0) {
    MLPD_INFO("Pinning reading thread %zu to core %d\n", this->id_,
              this->core_alloc_);
    pthread_t this_thread = this->thread_.native_handle();
    if (pin_thread_to_core(this->core_alloc_, this_thread) != 0) {
      MLPD_ERROR("Pin reading thread %zu to core %d failed\n", this->id_,
                 this->core_alloc_);
      throw std::runtime_error("Pin recording thread to core failed");
    }
  }

  moodycamel::ConsumerToken ctok(this->event_queue_);
  moodycamel::ProducerToken local_ptok(this->msg_queue_);

  size_t radio_num =
      this->id_ ? config_->num_cl_sdrs() : config_->num_bs_sdrs_all();
  std::vector<std::string> td_data_files = this->id_
                                               ? config_->ul_tx_td_data_files()
                                               : config_->dl_tx_td_data_files();

  fp.resize(radio_num, nullptr);
  for (size_t tid = 0; tid < radio_num; tid++) {
    std::printf("Opening time-domain data for radio %zu to %s\n", tid,
                td_data_files.at(tid).c_str());
    fp.at(tid) = std::fopen(td_data_files.at(tid).c_str(), "rb");
  }

  int* offset = new int[radio_num];
  for (size_t i = 0; i < radio_num; i++) offset[i] = 0;
  size_t packet_length = sizeof(Packet) + this->packet_data_length_;
  size_t txFrameDelta = config_->getTxFrameDelta();
  // Initially read a few frames and push to tx buffers
  for (size_t i = 0; i < txFrameDelta; i++) {
    for (size_t u = 0; u < radio_num; u++) {
      Event_data event;
      event.frame_id = i;
      event.node_type = this->id_ ? kClient : kBS;
      event.ant_id = u;
      event.buff_size = tx_buffer_[u].buffer.size() / packet_length;
      Event_data read_complete = this->ReadFrame(event, offset);
      if (msg_queue_.enqueue(local_ptok, read_complete) == false) {
        MLPD_ERROR("Read complete message enqueue failed\n");
        throw std::runtime_error("Read complete message enqueue failed");
      }
    }
  }

  Event_data event;
  bool ret = false;
  while (this->running_ == true) {
    ret = this->event_queue_.try_dequeue(ctok, event);

    if (ret == false) /* Queue empty */
    {
      if (this->wait_signal_ == true) {
        std::unique_lock<std::mutex> thread_wait(this->sync_);
        /* Wait until a new message exists, should eliminate the CPU polling */
        this->condition_.wait(thread_wait, [this, &ctok, &event] {
          return this->event_queue_.try_dequeue(ctok, event);
        });
        /* return from here with a valid event to process */
        ret = true;
      }
    }

    if (ret == true) {
      if (event.event_type == kThreadTermination) {
        this->running_ = false;
      } else {
        if (event.event_type == kTaskRead) {
          // read info
          Event_data read_complete = this->ReadFrame(event, offset);
          if (msg_queue_.enqueue(local_ptok, read_complete) == false) {
            MLPD_ERROR("Read complete message enqueue failed\n");
            throw std::runtime_error("Read complete message enqueue failed");
          }
        }
      }
    }
  }
}

};  //End namespace Sounder
