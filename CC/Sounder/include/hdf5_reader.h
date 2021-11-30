/*
 Copyright (c) 2018-2020
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
----------------------------------------------------------------------
Event based message queue thread class for the recorder worker
---------------------------------------------------------------------
*/
#ifndef SOUNDER_HDF5_READER_H_
#define SOUNDER_HDF5_READER_H_

#include "config.h"
#include "macros.h"
#include "receiver.h"
#include <condition_variable>
#include <mutex>
#include <vector>

namespace Sounder {
class Hdf5Reader {
public:
    //    enum RecordEventType { kThreadTermination, kTaskRecord };
    //
    //    struct RecordEvent_data {
    //        RecordEventType event_type;
    //        int data;
    //        SampleBuffer* rx_buffer;
    //        size_t rx_buff_size;
    //    };

    Hdf5Reader(Config* in_cfg,
        moodycamel::ConcurrentQueue<Event_data>& msg_queue,
        SampleBuffer* tx_buffer, size_t thread_id, int core, size_t queue_size,
        bool wait_signal = true);
    ~Hdf5Reader();

    void Start(void);
    void Stop(void);
    bool DispatchWork(Event_data event);
    Event_data ReadFrame(Event_data event, int& offset);

private:
    /*Main threading loop */
    void DoReading(void);
    void Finalize();

    //1 - Producer (dispatcher), 1 - Consumer
    moodycamel::ConcurrentQueue<Event_data>& msg_queue_;
    moodycamel::ConcurrentQueue<Event_data> event_queue_;
    moodycamel::ProducerToken producer_token_;
    SampleBuffer* tx_buffer_;
    Config* config_;
    std::thread thread_;

    size_t id_;
    size_t packet_data_length_;

    /* >= 0 to assign a core to the thread
         * <0   to disable thread core assignment */
    int core_alloc_;

    /* Synchronization for startup and sleeping */
    /* Setting wait signal to false will disable the thread waiting on new message
         * may cause excessive CPU load for infrequent messages.  
         * However, when the message processing time ~= queue posting time the mutex could 
         * become unnecessary work
         */
    bool wait_signal_;
    std::mutex sync_;
    std::condition_variable condition_;
    bool running_;
    std::vector<FILE*> fp;
};
};

#endif /* SOUNDER_HDF5_READER_H_ */
