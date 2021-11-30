/*
 Copyright (c) 2018-2021 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
----------------------------------------------------------------------
 Record received frames from massive-mimo base station in HDF5 format
---------------------------------------------------------------------
*/
#ifndef SOUNDER_SCHEDULER_H_
#define SOUNDER_SCHEDULER_H_

#include "hdf5_reader.h"
#include "receiver.h"
#include "recorder_thread.h"

namespace Sounder {

class Scheduler {
public:
    Scheduler(Config* in_cfg, unsigned int core_start = 0u);
    ~Scheduler();

    void do_it();
    int getRecordedFrameNum();
    std::string getTraceFileName() { return this->cfg_->trace_file(); }

private:
    void gc(void);

    // dequeue bulk size, used to reduce the overhead of dequeue in main thread
    static const int KDequeueBulkSize;

    Config* cfg_;
    std::unique_ptr<Receiver> receiver_;
    SampleBuffer* rx_buffer_;
    size_t rx_thread_buff_size_;
    SampleBuffer* bs_tx_buffer_;
    size_t bs_tx_thread_buff_size_;
    SampleBuffer* cl_tx_buffer_;
    size_t cl_tx_thread_buff_size_;

    //RecorderWorker worker_;
    std::vector<Sounder::RecorderThread*> recorders_;
    std::vector<Sounder::Hdf5Reader*> readers_;
    size_t max_frame_number_;

    moodycamel::ConcurrentQueue<Event_data> message_queue_;
    moodycamel::ConcurrentQueue<Event_data> tx_queue_;
    std::vector<moodycamel::ProducerToken*> tx_ptoks_ptr_;
    std::vector<moodycamel::ConcurrentQueue<Event_data>> cl_tx_queue_;
    std::vector<moodycamel::ProducerToken> cl_tx_ptoks_ptr_;

    /* Core assignment start variables */
    const unsigned int kMainDispatchCore;
    const unsigned int kSchedulerCore;
    const unsigned int kRecvCore;
}; /* class Scheduler */
}; /* Namespace sounder */
#endif /* SOUNDER_SCHEDULER_H_ */
