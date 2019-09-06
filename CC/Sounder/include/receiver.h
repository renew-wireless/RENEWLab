/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Peiyao Zhao: pdszpy19930218@163.com 
            Rahman Doost-Mohamamdy: doost@rice.edu
 
----------------------------------------------------------
 Handles received samples from massive-mimo base station 
----------------------------------------------------------
*/

#ifndef DATARECEIVER_HEADER
#define DATARECEIVER_HEADER

#include "concurrentqueue.h"
#include "sdr-lib.h"
#include <algorithm>
#include <arpa/inet.h>
#include <cassert>
#include <chrono>
#include <ctime>
#include <iostream>
#include <netinet/in.h>
#include <numeric>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>

typedef unsigned short ushort;

struct complex_float {
    float real;
    float imag;
};

struct Event_data {
    int event_type;
    int data;
};

struct SampleBuffer {
    std::vector<char> buffer;
    std::vector<int> buffer_status;
};

//std::atomic_int thread_count(0);
//std::mutex d_mutex;
//std::condition_variable cond;

class Receiver {
public:
    // use for create pthread
    struct ReceiverContext {
        Receiver* ptr;
        int tid;
    };

    struct dev_profile {
        int tid;
        int nsamps;
        int txSyms;
        int rxSyms;
        int txStartSym;
        unsigned txFrameDelta;
        double rate;
        SoapySDR::Device* device;
        SoapySDR::Stream* rxs;
        SoapySDR::Stream* txs;
        std::string data_file;
        int core;
        Receiver* ptr;
    };

public:
    Receiver(int n_rx_threads, Config* config, moodycamel::ConcurrentQueue<Event_data>* in_queue);
    ~Receiver();

    std::vector<pthread_t> startRecvThreads(char** in_buffer, int** in_buffer_status, int in_buffer_frame_num, int in_buffer_length, int in_core_id = 0);
    void completeRecvThreads(const std::vector<pthread_t>& recv_thread);
    std::vector<pthread_t> startClientThreads();
    void go();
    static void* loopRecv(void* in_context);
    static void* clientTxRx(void* context);

private:
    Config* config_;
    struct sockaddr_in servaddr_; /* server address */
    int* socket_;

    RadioConfig* radioconfig_;

    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

    char** buffer_;
    int** buffer_status_;
    int buffer_length_;
    int buffer_frame_num_;

    int thread_num_;
    // pointer of message_queue_
    moodycamel::ConcurrentQueue<Event_data>* message_queue_;
    int core_id_;
};

#endif
