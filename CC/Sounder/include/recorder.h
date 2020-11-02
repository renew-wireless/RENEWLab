/*
 Copyright (c) 2018-2019 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Peiyao Zhao 
            Rahman Doost-Mohamamdy, doost@rice.edu
 
----------------------------------------------------------------------
 Record received frames from massive-mimo base station in HDF5 format
---------------------------------------------------------------------
*/
#ifndef DATARECORDER_HEADER
#define DATARECORDER_HEADER

#include "H5Cpp.h"
#include "receiver.h"

class Recorder {
public:
    Recorder(Config* in_cfg);
    ~Recorder();

    void do_it();
    int getRecordedFrameNum();
    std::string getTraceFileName() { return this->cfg_->trace_file(); }

private:
    struct EventHandlerContext {
        Recorder* obj_ptr;
        size_t id;
    };
    herr_t record(int tid, int offset);
    void taskThread(EventHandlerContext* context);
    herr_t initHDF5(const std::string&);
    void openHDF5();
    void closeHDF5();
    void finishHDF5();
    static void* taskThread_launch(void* in_context);
    void gc(void);

    // buffer length of each rx thread
    static const int kSampleBufferFrameNum;
    // dequeue bulk size, used to reduce the overhead of dequeue in main thread
    static const int KDequeueBulkSize;
    // pilot dataset size increment
    static const int kConfigPilotExtentStep;
    // data dataset size increment
    static const int kConfigDataExtentStep;

    Config* cfg_;
    std::unique_ptr<Receiver> receiver_;
    SampleBuffer* rx_buffer_;
    size_t rx_thread_buff_size_;

    H5std_string hdf5_name_;

    H5::H5File* file_;
    // Group* group;
    H5::DSetCreatPropList pilot_prop_;
    H5::DSetCreatPropList data_prop_;

    H5::DataSet* pilot_dataset_;
    H5::DataSet* data_dataset_;

    size_t frame_number_pilot_;
    size_t frame_number_data_;
#if DEBUG_PRINT
    hsize_t cdims_pilot[5];
    hsize_t cdims_data[5];
#endif

    size_t max_frame_number_;
    moodycamel::ConcurrentQueue<Event_data> task_queue_;
    moodycamel::ConcurrentQueue<Event_data> message_queue_;
    // std::vector<std::unique_ptr<moodycamel::ProducerToken>> task_ptok; //[TASK_THREAD_NUM];
};
#endif
