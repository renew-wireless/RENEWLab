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
#include "hdf5.h"
#include "receiver.h"
#include "signalHandler.hpp"
#include <algorithm>
#include <complex>
#include <ctime>
#include <fcntl.h>
#include <math.h>
#include <memory>
#include <pthread.h>
#include <queue>
#include <signal.h>
#include <sstream>
#include <string>
#include <system_error>
#include <unistd.h>

using std::cout;
using std::endl;
using namespace H5;
class Recorder {
public:
    // buffer length of each rx thread
    static const int SAMPLE_BUFFER_FRAME_NUM = 80;
    // buffer length of recording part
    static const int TASK_BUFFER_FRAME_NUM = 60;
    // dequeue bulk size, used to reduce the overhead of dequeue in main thread
    static const int dequeue_bulk_size = 5;

    Recorder(Config* cfg);
    ~Recorder();

    void start();
    void stop();
    herr_t initHDF5(const std::string &);
    void openHDF5();
    void closeHDF5();
    static void* taskThread(void* context);
    herr_t record(int tid, int offset);

    struct EventHandlerContext {
        Recorder* obj_ptr;
        int id;
    };

private:
    Config* cfg;
    bool core_alloc;
    std::unique_ptr<Receiver> receiver_;
    SampleBuffer* rx_buffer_;

    H5std_string hdf5name;
    H5std_string hdf5group;

    H5File* file;
    Group* group;
    DSetCreatPropList pilot_prop;
    DSetCreatPropList data_prop;

    DataSpace* pilot_filespace;
    DataSpace* data_filespace;

    DataSet* pilot_dataset;
    DataSet* data_dataset;

    int ndims;
    int cndims;

    hsize_t dims_pilot[5];
    hsize_t dims_data[5];
    hsize_t cdims_pilot[5];
    hsize_t cdims_data[5];

    int config_pilot_extent_step = 400;
    int config_data_extent_step = 400;
    bool config_dump_data;
    int maxFrameNumber;
    int rx_thread_num;
    int task_thread_num;
    moodycamel::ConcurrentQueue<Event_data> task_queue_;
    moodycamel::ConcurrentQueue<Event_data> message_queue_;
    pthread_t* task_threads; //[TASK_THREAD_NUM];

    EventHandlerContext* context; //[TASK_THREAD_NUM];

    std::vector<std::unique_ptr<moodycamel::ProducerToken>> task_ptok; //[TASK_THREAD_NUM];
};
#endif
