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

using namespace H5;
class Recorder {
public:
    Recorder(Config* cfg);
    ~Recorder();

    void do_it();
    int getRecordedFrameNum();
    std::string getTraceFileName() { return cfg->trace_file; }

private:
    struct EventHandlerContext {
        Recorder* obj_ptr;
        int id;
    };
    herr_t record(int tid, int offset);
    void taskThread(EventHandlerContext* context);
    herr_t initHDF5(const std::string&);
    void openHDF5();
    void closeHDF5();
    void finishHDF5();
    static void* taskThread_launch(void* in_context);

    // buffer length of each rx thread
    static const int SAMPLE_BUFFER_FRAME_NUM;
    // buffer length of recording part
    static const int TASK_BUFFER_FRAME_NUM;
    // dequeue bulk size, used to reduce the overhead of dequeue in main thread
    static const int dequeue_bulk_size;
    // pilot dataset size increment
    static const int config_pilot_extent_step;
    // data dataset size increment
    static const int config_data_extent_step;

    Config* cfg;
    std::unique_ptr<Receiver> receiver_;
    SampleBuffer* rx_buffer_;
    size_t rx_thread_buff_size_;

    H5std_string hdf5name;
    H5std_string hdf5group;

    H5File* file;
    // Group* group;
    DSetCreatPropList pilot_prop;
    DSetCreatPropList data_prop;

    DataSet* pilot_dataset;
    DataSet* data_dataset;

    size_t frame_number_pilot;
    size_t frame_number_data;
#if DEBUG_PRINT
    hsize_t cdims_pilot[5];
    hsize_t cdims_data[5];
#endif

    size_t maxFrameNumber;
    moodycamel::ConcurrentQueue<Event_data> task_queue_;
    moodycamel::ConcurrentQueue<Event_data> message_queue_;
    // std::vector<std::unique_ptr<moodycamel::ProducerToken>> task_ptok; //[TASK_THREAD_NUM];
};
#endif
