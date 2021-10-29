/*
 Copyright (c) 2018-2021 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
----------------------------------------------------------------------
 Record received frames from massive-mimo base station in HDF5 format
---------------------------------------------------------------------
*/
#ifndef SOUDER_SCHEDULER_H_
#define SOUDER_SCHEDULER_H_

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

    // buffer length of each rx thread
    static const int kSampleBufferFrameNum;
    // dequeue bulk size, used to reduce the overhead of dequeue in main thread
    static const int KDequeueBulkSize;

    Config* cfg_;
    std::unique_ptr<Receiver> receiver_;
    SampleBuffer* rx_buffer_;
    size_t rx_thread_buff_size_;

    //RecorderWorker worker_;
    std::vector<Sounder::RecorderThread*> recorders_;
    size_t max_frame_number_;

    moodycamel::ConcurrentQueue<Event_data> message_queue_;

    /* Core assignment start variables */
    const unsigned int kMainDispatchCore;
    const unsigned int kSchedulerCore;
    const unsigned int kRecvCore;
}; /* class Scheduler */
}; /* Namespace sounder */
#endif /* SOUDER_SCHEDULER_H_ */
