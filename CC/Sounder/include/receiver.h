/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
----------------------------------------------------------
 Handles received samples from massive-mimo base station 
----------------------------------------------------------
*/

#ifndef DATARECEIVER_HEADER
#define DATARECEIVER_HEADER

#include "BaseRadioSet.h"
#include "ClientRadioSet.h"
#include "concurrentqueue.h"
#include <algorithm>
#include <arpa/inet.h>
#include <cassert>
#include <chrono>
#include <ctime>
#include <exception>
#include <iostream>
#include <netinet/in.h>
#include <numeric>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>

class ReceiverException : public std::exception {
    virtual const char* what() const throw()
    {
        return "Receiver could not be setup correctly!";
    }
};

enum ReceiverEventType { kEventRxSymbol = 0 };

struct Event_data {
    ReceiverEventType event_type;
    int data;
    int ant_id;
};

struct Package {
    uint32_t frame_id;
    uint32_t symbol_id;
    uint32_t cell_id;
    uint32_t ant_id;
    short data[];
    Package(int f, int s, int c, int a)
        : frame_id(f)
        , symbol_id(s)
        , cell_id(c)
        , ant_id(a)
    {
    }
};

// each thread has a SampleBuffer
struct SampleBuffer {
    std::vector<char> buffer;
    std::atomic_int* pkg_buf_inuse;
};

class Receiver {
public:
    // use for create pthread
    struct ReceiverContext {
        Receiver* ptr;
        SampleBuffer* buffer;
        int core_id;
        int tid;
    };

    struct dev_profile {
        int tid;
        Receiver* ptr;
    };

public:
    Receiver(int n_rx_threads, Config* config,
        moodycamel::ConcurrentQueue<Event_data>* in_queue);
    ~Receiver();

    std::vector<pthread_t> startRecvThreads(
        SampleBuffer* rx_buffer, unsigned in_core_id = 0);
    void completeRecvThreads(const std::vector<pthread_t>& recv_thread);
    std::vector<pthread_t> startClientThreads();
    void go();
    static void* loopRecv_launch(void* in_context);
    void loopRecv(int tid, int core_id, SampleBuffer* rx_buffer);
    static void* clientTxRx_launch(void* in_context);
    void clientTxRx(int tid);
    void clientSyncTxRx(int tid);

private:
    Config* config_;
    ClientRadioSet* clientRadioSet_;
    BaseRadioSet* base_radio_set_;

    int thread_num_;
    // pointer of message_queue_
    moodycamel::ConcurrentQueue<Event_data>* message_queue_;
};

#endif
