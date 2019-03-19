/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Peiyao Zhao: pdszpy19930218@163.com 
            Rahman Doost-Mohamamdy: doost@rice.edu
 
----------------------------------------------------------
 Handles received samples from massive-mimo base station 
----------------------------------------------------------
*/

#include "include/receiver.h"

Receiver::Receiver(int N_THREAD, Config *cfg)
{

    config_ = cfg;
    radioconfig_ = new RadioConfig(config_);

    thread_num_ = N_THREAD;
    /* initialize random seed: */
    srand (time(NULL));
    context = new ReceiverContext[thread_num_];

}

Receiver::Receiver(int N_THREAD, Config *config, moodycamel::ConcurrentQueue<Event_data> * in_queue):
Receiver(N_THREAD, config)
{
    message_queue_ = in_queue;
}

Receiver::~Receiver()
{
    delete[] socket_;
    delete[] context;
    radioconfig_->radioStop();
    delete radioconfig_;
    delete config_;
}

std::vector<pthread_t> Receiver::startRecv(void** in_buffer, int** in_buffer_status, int in_buffer_frame_num, int in_buffer_length, int in_core_id)
{
    // check length
    buffer_frame_num_ = in_buffer_frame_num;
    assert(in_buffer_length == config_->getPackageLength() * buffer_frame_num_); // should be integre
    buffer_length_ = in_buffer_length;
    buffer_ = in_buffer;  // for save data
    buffer_status_ = in_buffer_status; // for save status

    core_id_ = in_core_id;
    radioconfig_->radioStart();


    printf("start Recv thread\n");
    // new thread
     
    std::vector<pthread_t> created_threads;
    for(int i = 0; i < thread_num_; i++)
    {
        pthread_t recv_thread_;
        // record the thread id 
        context[i].ptr = this;
        context[i].tid = i;
        // start socket thread
        if(pthread_create( &recv_thread_, NULL, Receiver::loopRecv, (void *)(&context[i])) != 0)
        {
            perror("socket recv thread create failed");
            exit(0);
        }
        created_threads.push_back(recv_thread_);
    }
    
    return created_threads;
}

void* Receiver::loopRecv(void *in_context)
{
    // get the pointer of class & tid
    Receiver* obj_ptr = ((ReceiverContext *)in_context)->ptr;
    Config *cfg = obj_ptr->config_;
    int tid = ((ReceiverContext *)in_context)->tid;
    printf("package receiver thread %d start\n", tid);
    // get pointer of message queue
    moodycamel::ConcurrentQueue<Event_data> *message_queue_ = obj_ptr->message_queue_;
    int core_id = obj_ptr->core_id_;
    // if ENABLE_CPU_ATTACH is enabled, attach threads to specific cores
#ifdef ENABLE_CPU_ATTACH
    printf("pinning thread %d to core %d\n", tid, core_id + tid);
    if(pin_to_core(core_id + tid) != 0)
    {
        printf("pin thread %d to core %d failed\n", tid, core_id + tid);
        exit(0);
    }
#endif
    // use token to speed up
    moodycamel::ProducerToken local_ptok(*message_queue_);

    void* buffer = obj_ptr->buffer_[tid];
    int* buffer_status = obj_ptr->buffer_status_[tid];
    int buffer_length = obj_ptr->buffer_length_;
    int buffer_frame_num = obj_ptr->buffer_frame_num_;

    void* cur_ptr_buffer = buffer;
    int* cur_ptr_buffer_status = buffer_status;

    int nradio_per_thread = cfg->nBsSdrs[0]/obj_ptr->thread_num_;
    int rem_thread_nradio = cfg->nBsSdrs[0]%obj_ptr->thread_num_;
    int nradio_cur_thread = nradio_per_thread;
    if (tid < rem_thread_nradio) nradio_cur_thread += 1;
    printf("receiver thread %d has %d radios\n", tid, nradio_cur_thread);
    RadioConfig *radio = obj_ptr->radioconfig_;

    // to handle second channel at each radio
    // this is assuming buffer_frame_num is at least 2 
    void* cur_ptr_buffer2;
    int* cur_ptr_buffer_status2;
    void* buffer2 = obj_ptr->buffer_[tid] + cfg->getPackageLength(); 
    int* buffer_status2 = obj_ptr->buffer_status_[tid] + 1;
    if (cfg->bsSdrCh == 2)
    {
        cur_ptr_buffer2 = buffer2;
        cur_ptr_buffer_status2 = buffer_status2;
    }
    else
        cur_ptr_buffer2 = malloc(cfg->getPackageLength()); 
    int offset = 0;
    int package_num = 0;
    long long frameTime;
    auto begin = std::chrono::system_clock::now();

    int maxQueueLength = 0;
    int ret = 0;
    while(true)
    {
        // if buffer is full, exit
        if(cur_ptr_buffer_status[0] == 1)
        {
            printf("thread %d buffer full\n", tid);
            exit(0);
        }
        int ant_id, frame_id, symbol_id, cell_id;
        // receive data
        for (int it = 0 ; it < nradio_cur_thread; it++) 
        {
            int rid = (tid < rem_thread_nradio) ? tid * (nradio_per_thread + 1) + it : tid * (nradio_per_thread) + rem_thread_nradio + it ;
            void * samp1 = cur_ptr_buffer + 4*sizeof(int);
            void * samp2 = cur_ptr_buffer2 + 4*sizeof(int);
            void *samp[2] = {samp1, samp2};
            radio->radioRx(rid, samp, frameTime);

            frame_id = (int)(frameTime>>32);
            symbol_id = (int)((frameTime>>16)&0xFFFF);
            ant_id = rid * cfg->bsSdrCh;
            *((int *)cur_ptr_buffer) = frame_id;
            *((int *)cur_ptr_buffer + 1) = symbol_id;
            *((int *)cur_ptr_buffer + 2) = 0; //cell_id 
            *((int *)cur_ptr_buffer + 3) = ant_id;
            if (cfg->bsSdrCh == 2)
            {
                *((int *)cur_ptr_buffer2) = frame_id;
                *((int *)cur_ptr_buffer2 + 1) = symbol_id;
                *((int *)cur_ptr_buffer2 + 2) = 0; //cell_id 
                *((int *)cur_ptr_buffer2 + 3) = ant_id + 1;
            }
#if DEBUG_PRINT
            printf("receive thread %d, frame_id %d, symbol_id %d, cell_id %d, ant_id %d\n", tid, frame_id, symbol_id, cell_id, ant_id);
            printf("receive samples: %d %d %d %d %d %d %d %d ...\n",*((short *)cur_ptr_buffer+9), 
							   *((short *)cur_ptr_buffer+10),
                                                           *((short *)cur_ptr_buffer+11),
                                                           *((short *)cur_ptr_buffer+12),
                                                           *((short *)cur_ptr_buffer+13),
                                                           *((short *)cur_ptr_buffer+14),
                                                           *((short *)cur_ptr_buffer+15),
                                                           *((short *)cur_ptr_buffer+16)); 
#endif        
            // get the position in buffer
            offset = cur_ptr_buffer_status - buffer_status;
            // move ptr & set status to full
            cur_ptr_buffer_status[0] = 1; // has data, after it is read it should be set to 0
            cur_ptr_buffer_status = buffer_status + (cur_ptr_buffer_status - buffer_status + cfg->bsSdrCh) % buffer_frame_num;
            cur_ptr_buffer = buffer + ((char*)cur_ptr_buffer - (char*)buffer + cfg->getPackageLength() * cfg->bsSdrCh) % buffer_length;
            // push EVENT_RX_SYMBOL event into the queue
            Event_data package_message;
            package_message.event_type = EVENT_RX_SYMBOL;
            // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
            package_message.data = offset + tid * buffer_frame_num; // when we multi-thread radio read, this probably would not be correct 
            if ( !message_queue_->enqueue(local_ptok, package_message ) ) {
                printf("socket message enqueue failed\n");
                exit(0);
            }
            if (cfg->bsSdrCh == 2)
            {
                offset = cur_ptr_buffer_status2 - buffer_status; // offset is absolute 
                cur_ptr_buffer_status2[0] = 1; // has data, after doing fft, it is set to 0
                cur_ptr_buffer_status2 = buffer_status2 + (cur_ptr_buffer_status2 - buffer_status2 + cfg->bsSdrCh) % buffer_frame_num;
                cur_ptr_buffer2 = buffer2 + ((char*)cur_ptr_buffer2 - (char*)buffer2 + cfg->getPackageLength() * cfg->bsSdrCh) % buffer_length;
                // push EVENT_RX_SYMBOL event into the queue
                Event_data package_message2;
                package_message2.event_type = EVENT_RX_SYMBOL;
                // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
                package_message2.data = offset + tid * buffer_frame_num;
                if ( !message_queue_->enqueue(local_ptok, package_message2 ) ) {
                    printf("socket message enqueue failed\n");
                    exit(0);
                }
            }
            //printf("enqueue offset %d\n", offset);
            int cur_queue_len = message_queue_->size_approx();
            maxQueueLength = maxQueueLength > cur_queue_len ? maxQueueLength : cur_queue_len;

        }
        package_num++;
        // print some information
        if(package_num >= 1e4)
        {
            auto end = std::chrono::system_clock::now();
            double byte_len = sizeof(ushort) * cfg->sampsPerSymbol * 2 * 1e5;
            std::chrono::duration<double> diff = end - begin;
            // print network throughput & maximum message queue length during this period
            printf("thread %d receive %f bytes in %f secs, throughput %f MB/s, max Message Queue Length %d\n", tid, byte_len, diff.count(), byte_len / diff.count() / 1024 / 1024, maxQueueLength);
            maxQueueLength = 0;
            begin = std::chrono::system_clock::now();
            package_num = 0;
            //radio->readSensors();
        }
    }
}

