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

Receiver::Receiver(int n_rx_threads, Config* config, moodycamel::ConcurrentQueue<Event_data>* in_queue)
{
    this->config_ = config;
    radioconfig_ = new RadioConfig(this->config_);

    thread_num_ = n_rx_threads;

    /* initialize random seed: */
    srand(time(NULL));

    if (thread_num_ > 0)
        context = new ReceiverContext[thread_num_];
    message_queue_ = in_queue;
    radioconfig_->radioConfigure();
}

Receiver::~Receiver()
{
    radioconfig_->radioStop();
    delete radioconfig_;
}

std::vector<pthread_t> Receiver::startClientThreads()
{
    std::vector<pthread_t> client_threads;
    if (config_->clPresent) {
        double frameTime = config_->sampsPerSymbol * config_->clFrames[0].size() * 1e3 / config_->rate; // miliseconds
        unsigned frameTimeDelta = (unsigned)(std::ceil(TIME_DELTA / frameTime));
        std::cout << "Frame time delta " << frameTimeDelta << std::endl;

        for (unsigned int i = 0; i < config_->nClSdrs; i++) {
            pthread_t cl_thread_;
            // record the thread id
            dev_profile* profile = new dev_profile;
            profile->tid = i;
            profile->rate = config_->rate;
            profile->nsamps = config_->sampsPerSymbol;
            profile->txSyms = config_->clULSymbols[i].size();
            profile->rxSyms = config_->clDLSymbols[i].size();
            profile->txStartSym = config_->clULSymbols[i].empty() ? 0 : config_->clULSymbols[i][0];
            profile->txFrameDelta = frameTimeDelta;
            profile->device = radioconfig_->devs[i];
            profile->rxs = radioconfig_->rxss[i];
            profile->txs = radioconfig_->txss[i];
            profile->core = i + 1 + config_->rx_thread_num + config_->task_thread_num;
            profile->ptr = this;
            // start socket thread
            if (pthread_create(&cl_thread_, NULL, Receiver::clientTxRx, (void*)(profile)) != 0) {
                perror("socket client thread create failed");
                exit(0);
            }
            client_threads.push_back(cl_thread_);
        }
    }

    return client_threads;
}

std::vector<pthread_t> Receiver::startRecvThreads(char** in_buffer, int** in_buffer_status, int in_buffer_frame_num, int in_buffer_length, int in_core_id)
{
    buffer_frame_num_ = in_buffer_frame_num;
    assert(in_buffer_length == config_->getPackageLength() * buffer_frame_num_);
    assert(in_buffer_frame_num != 0);
    assert(in_buffer_length != 0);
    buffer_length_ = in_buffer_length;
    buffer_ = in_buffer;
    buffer_status_ = in_buffer_status;

    core_id_ = in_core_id;

    std::vector<pthread_t> created_threads;
    for (int i = 0; i < thread_num_; i++) {
        pthread_t recv_thread_;
        // record the thread id
        context[i].ptr = this;
        context[i].tid = i;
        // start socket thread
        if (pthread_create(&recv_thread_, NULL, Receiver::loopRecv, (void*)(&context[i])) != 0) {
            perror("socket recv thread create failed");
            exit(0);
        }
        created_threads.push_back(recv_thread_);
    }

    sleep(1);
    pthread_cond_broadcast(&cond);
    //sleep(.01);
    go();
    return created_threads;
}

void Receiver::go()
{
    radioconfig_->radioStart(); // hardware trigger
}

void* Receiver::loopRecv(void* in_context)
{
    ReceiverContext* context = (ReceiverContext*)in_context;
    Receiver* obj_ptr = context->ptr;
    Config* cfg = obj_ptr->config_;
    int tid = context->tid;
    moodycamel::ConcurrentQueue<Event_data>* message_queue_ = obj_ptr->message_queue_;

    if (cfg->core_alloc) {
        int core_id = obj_ptr->core_id_;
        printf("pinning thread %d to core %d\n", tid, core_id + tid);
        if (pin_to_core(core_id + tid) != 0) {
            printf("pin thread %d to core %d failed\n", tid, core_id + tid);
            exit(0);
        }
    }

    // Use mutex to sychronize data receiving across threads
    pthread_mutex_lock(&obj_ptr->mutex);
    printf("Recv Thread %d: waiting for release\n", tid);

    pthread_cond_wait(&obj_ptr->cond, &obj_ptr->mutex);
    pthread_mutex_unlock(&obj_ptr->mutex); // unlocking for all other threads

    // use token to speed up
    moodycamel::ProducerToken local_ptok(*message_queue_);

    char* buffer1 = obj_ptr->buffer_[tid];
    int* buffer_status1 = obj_ptr->buffer_status_[tid];
    int buffer_length = obj_ptr->buffer_length_;
    int buffer_frame_num = obj_ptr->buffer_frame_num_;

    int buffer1_index = 0;
    int status1_index = 0;

    int nradio_per_thread = cfg->nBsSdrs[0] / obj_ptr->thread_num_;
    int rem_thread_nradio = cfg->nBsSdrs[0] % obj_ptr->thread_num_;
    int nradio_cur_thread = nradio_per_thread;
    if (tid < rem_thread_nradio)
        nradio_cur_thread += 1;
    printf("receiver thread %d has %d radios\n", tid, nradio_cur_thread);
    RadioConfig* radio = obj_ptr->radioconfig_;

    // to handle second channel at each radio
    // this is assuming buffer_frame_num is at least 2
    char* buffer2;
    int* buffer_status2 = obj_ptr->buffer_status_[tid] + 1;
    int buffer2_index = 0;
    int status2_index = 0;
    const int bsSdrCh = cfg->bsSdrCh;
    if (bsSdrCh == 1)
        buffer2 = obj_ptr->buffer_[tid] + cfg->getPackageLength();
    else
        buffer2 = new char[cfg->getPackageLength()];

    int offset = 0;
    long long frameTime;
    while (cfg->running) {
        // if buffer is full, exit
        if (buffer_status1[status1_index] == 1) {
            printf("thread %d buffer full\n", tid);
            exit(0);
        }
        int ant_id, frame_id, symbol_id; // cell_id;
        // receive data
        for (int it = 0; it < nradio_cur_thread; it++) {
            int rid = (tid < rem_thread_nradio) ? tid * (nradio_per_thread + 1) + it : tid * (nradio_per_thread) + rem_thread_nradio + it;
            void* samp1 = buffer1 + buffer1_index + 4 * sizeof(int);
            void* samp2 = buffer2 + buffer2_index + 4 * sizeof(int);
            void* samp[2] = { samp1, samp2 };
            int* int_sample1 = (int*)(buffer1 + buffer1_index);
            int* int_sample2 = (int*)(buffer2 + buffer2_index);
            if (radio->radioRx(rid, samp, frameTime) < 0) {
                cfg->running = false;
                break;
            }

            frame_id = (int)(frameTime >> 32);
            symbol_id = (int)((frameTime >> 16) & 0xFFFF);
            ant_id = rid * bsSdrCh;
            int_sample1[0] = frame_id;
            int_sample1[1] = symbol_id;
            int_sample1[2] = 0; //cell_id
            int_sample1[3] = ant_id;
            if (bsSdrCh == 2) {
                int_sample2[0] = frame_id;
                int_sample2[1] = symbol_id;
                int_sample2[2] = 0; //cell_id
                int_sample2[3] = ant_id + 1;
            }
#if DEBUG_PRINT
            short* short_sample1 = (short*)(buffer1 + buffer1_index);
            printf("receive thread %d, frame_id %d, symbol_id %d, cell_id %d, ant_id %d\n", tid, frame_id, symbol_id, cell_id, ant_id);
            printf("receive samples: %d %d %d %d %d %d %d %d ...\n",
                short_sample1[9], short_sample1[10], short_sample1[11], short_sample1[12],
                short_sample1[13], short_sample1[14], short_sample1[15], short_sample1[16]);
#endif
            // get the position in buffer
            offset = status1_index;
            // move ptr & set status to full
            buffer_status1[status1_index] = 1; // has data, after it is read it should be set to 0
            status1_index = (status1_index + bsSdrCh) % buffer_frame_num;
            buffer1_index = (buffer1_index + cfg->getPackageLength() * bsSdrCh) % buffer_length;
            // push EVENT_RX_SYMBOL event into the queue
            Event_data package_message;
            package_message.event_type = EVENT_RX_SYMBOL;
            // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit)
            package_message.data = offset + tid * buffer_frame_num;
            if (!message_queue_->enqueue(local_ptok, package_message)) {
                printf("socket message enqueue failed\n");
                exit(0);
            }
            if (bsSdrCh == 2) {
                offset = status2_index; // offset is absolute
                buffer_status2[status2_index] = 1; // has data, after doing fft, it is set to 0
                status2_index = (status2_index + bsSdrCh) % buffer_frame_num;
                buffer2_index = (buffer2_index + cfg->getPackageLength() * bsSdrCh) % buffer_length;
                // push EVENT_RX_SYMBOL event into the queue
                Event_data package_message2;
                package_message2.event_type = EVENT_RX_SYMBOL;
                // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit)
                package_message2.data = offset + tid * buffer_frame_num;
                if (!message_queue_->enqueue(local_ptok, package_message2)) {
                    printf("socket message enqueue failed\n");
                    exit(0);
                }
            }
        }
    }
    if (bsSdrCh != 1)
        delete[] buffer2;
    return 0;
}

void* Receiver::clientTxRx(void* context)
{
    dev_profile* profile = (dev_profile*)context;
    SoapySDR::Device* device = profile->device;
    SoapySDR::Stream* rxStream = profile->rxs;
    SoapySDR::Stream* txStream = profile->txs;
    int tid = profile->tid;
    int txSyms = profile->txSyms;
    int rxSyms = profile->rxSyms;
    int txStartSym = profile->txStartSym;
    unsigned txFrameDelta = profile->txFrameDelta;
    int NUM_SAMPS = profile->nsamps;
    Receiver* obj_ptr = profile->ptr;
    Config* cfg = obj_ptr->config_;

    if (cfg->core_alloc) {
        printf("pinning client thread %d to core %d\n", tid, profile->core);
        if (pin_to_core(profile->core) != 0) {
            printf("pin client thread %d to core %d failed\n", tid, profile->core);
            exit(0);
        }
    }

    //while(!d_mutex.try_lock()){}
    //thread_count++;
    //std::cout << "Thread " << tid << ", txSyms " << txSyms << ", rxSyms " << rxSyms << ", txStartSym " << txStartSym << ", rate " << profile->rate << ", txFrameDelta " << txFrameDelta << ", nsamps " << NUM_SAMPS << std::endl;
    //d_mutex.unlock();

    std::vector<std::complex<float>> buffs(NUM_SAMPS, 0);
    std::vector<void*> rxbuff(2);
    rxbuff[0] = buffs.data();
    rxbuff[1] = buffs.data();

    std::vector<void*> txbuff(2);
    if (txSyms > 0) {
        txbuff[0] = cfg->txdata[tid].data();
        txbuff[1] = cfg->txdata[tid].data();
        std::cout << txSyms << " uplink symbols will be sent per frame..." << std::endl;
    }

    int all_trigs = 0;
    struct timespec tv, tv2;
    clock_gettime(CLOCK_MONOTONIC, &tv);

    while (cfg->running) {
        clock_gettime(CLOCK_MONOTONIC, &tv2);
        double diff = (tv2.tv_sec * 1e9 + tv2.tv_nsec - tv.tv_sec * 1e9 - tv.tv_nsec) / 1e9;
        if (diff > 2) {
            int total_trigs = device->readRegister("IRIS30", 92);
            std::cout << "new triggers: " << total_trigs - all_trigs << ", total: " << total_trigs << std::endl;
            all_trigs = total_trigs;
            tv = tv2;
        }
        // receiver loop
        long long rxTime(0);
        long long txTime(0);
        long long firstRxTime(0);
        bool receiveErrors = false;
        for (int i = 0; i < rxSyms; i++) {
            int flags(0);
            int r = device->readStream(rxStream, rxbuff.data(), NUM_SAMPS, flags, rxTime, 1000000);
            if (r == NUM_SAMPS) {
                if (i == 0)
                    firstRxTime = rxTime;
            } else {
                std::cerr << "waiting for receive frames... " << std::endl;
                receiveErrors = true;
                break;
            }
        }
        if (receiveErrors)
            continue; // just go to the next frame

        // transmit loop
        int flags = SOAPY_SDR_HAS_TIME;
        txTime = firstRxTime & 0xFFFFFFFF00000000;
        txTime += ((long long)txFrameDelta << 32);
        txTime += ((long long)txStartSym << 16);
        //printf("rxTime %llx, txTime %llx \n", firstRxTime, txTime);
        if (!cfg->running)
            flags |= SOAPY_SDR_END_BURST; //end burst on last iter
        for (int i = 0; i < txSyms; i++) {
            //if (i == txSyms - 1)
            flags |= SOAPY_SDR_END_BURST;
            int r = device->writeStream(txStream, txbuff.data(), NUM_SAMPS, flags, txTime, 1000000);
            if (r == NUM_SAMPS) {
                txTime += 0x10000;
            } else {
                std::cerr << "unexpected writeStream error " << SoapySDR::errToStr(r) << std::endl;
                //goto cleanup;
            }
        }
    }
    return 0;
}
