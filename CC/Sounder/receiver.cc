/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
----------------------------------------------------------
 Handles received samples from massive-mimo base station 
----------------------------------------------------------
*/

#include "include/receiver.h"
#include "include/ClientRadioSet.h"
#include "include/comms-lib.h"
#include "include/macros.h"
#include "include/utils.h"
#include <atomic>
#include <random>
#include <unistd.h>

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

Receiver::Receiver(int n_rx_threads, Config* config, moodycamel::ConcurrentQueue<Event_data>* in_queue)
    : config_(config)
    //, clientRadioSet_(config->clPresent ? new ClientRadioSet(config) : NULL)
    //, baseRadioSet_(config->bsPresent ? new BaseRadioSet(config) : NULL)
    , thread_num_(n_rx_threads)
    , message_queue_(in_queue)
{
    /* initialize random seed: */
    srand(time(NULL));
    clientRadioSet_ = config_->clPresent ? new ClientRadioSet(config_) : NULL;
    baseRadioSet_ = config_->bsPresent ? new BaseRadioSet(config_) : NULL;
    bool except = false;
    if (baseRadioSet_ != NULL && baseRadioSet_->getRadioNotFound()) {
        delete baseRadioSet_;
        except = true;
    }
    if (clientRadioSet_ != NULL && clientRadioSet_->getRadioNotFound()) {
        delete clientRadioSet_;
        except = true;
    }
    if (except)
        throw ReceiverException();
}

Receiver::~Receiver()
{
    if (baseRadioSet_ != NULL) {
        baseRadioSet_->radioStop();
        delete baseRadioSet_;
    }
    if (clientRadioSet_ != NULL) {
        clientRadioSet_->radioStop();
        delete clientRadioSet_;
    }
}

std::vector<pthread_t> Receiver::startClientThreads()
{
    std::vector<pthread_t> client_threads;
    if (config_->clPresent) {
        client_threads.resize(config_->nClSdrs);
        for (unsigned int i = 0; i < config_->nClSdrs; i++) {
            pthread_t cl_thread_;
            // record the thread id
            dev_profile* profile = new dev_profile;
            profile->tid = i;
            profile->ptr = this;
            // start socket thread
            if (pthread_create(&cl_thread_, NULL, Receiver::clientTxRx_launch, profile) != 0) {
                perror("socket client thread create failed");
                exit(0);
            }
            client_threads[i] = cl_thread_;
        }
    }
    return client_threads;
}

std::vector<pthread_t> Receiver::startRecvThreads(SampleBuffer* rx_buffer, unsigned in_core_id)
{
    assert(rx_buffer[0].buffer.size() != 0);

    std::vector<pthread_t> created_threads;
    created_threads.resize(thread_num_);
    for (int i = 0; i < thread_num_; i++) {
        // record the thread id
        ReceiverContext* context = new ReceiverContext;
        context->ptr = this;
        context->core_id = in_core_id;
        context->tid = i;
        context->buffer = rx_buffer;
        // start socket thread
        if (pthread_create(&created_threads[i], NULL, Receiver::loopRecv_launch, context) != 0) {
            perror("socket recv thread create failed");
            exit(0);
        }
    }

    sleep(1);
    pthread_cond_broadcast(&cond);
    go();
    return created_threads;
}

void Receiver::completeRecvThreads(const std::vector<pthread_t>& recv_thread)
{
    for (std::vector<pthread_t>::const_iterator it = recv_thread.begin();
         it != recv_thread.end(); ++it)
        pthread_join(*it, NULL);
}

void Receiver::go()
{
    if (baseRadioSet_ != NULL)
        baseRadioSet_->radioStart(); // hardware trigger
}

void* Receiver::loopRecv_launch(void* in_context)
{
    ReceiverContext* context = (ReceiverContext*)in_context;
    auto me = context->ptr;
    auto tid = context->tid;
    auto core_id = context->core_id;
    auto buffer = context->buffer;
    delete context;
    me->loopRecv(tid, core_id, buffer);
    return 0;
}

void Receiver::loopRecv(int tid, int core_id, SampleBuffer* rx_buffer)
{
    if (config_->core_alloc) {
        printf("pinning thread %d to core %d\n", tid, core_id + tid);
        if (pin_to_core(core_id + tid) != 0) {
            printf("pin thread %d to core %d failed\n", tid, core_id + tid);
            exit(0);
        }
    }

    // Use mutex to sychronize data receiving across threads
    if (config_->nClSdrs > 0 && config_->nBsSdrs[0] > 0) {
        pthread_mutex_lock(&mutex);
        printf("Recv Thread %d: waiting for release\n", tid);

        pthread_cond_wait(&cond, &mutex);
        pthread_mutex_unlock(&mutex); // unlocking for all other threads
    }

    // use token to speed up
    moodycamel::ProducerToken local_ptok(*message_queue_);

    const int bsSdrCh = config_->bsChannel.length();
    size_t packageLength = sizeof(Package) + config_->getPackageDataLength();
    int buffer_chunk_size = rx_buffer[0].buffer.size() / packageLength;

    // handle two channels at each radio
    // this is assuming buffer_chunk_size is at least 2
    std::atomic_int* pkg_buf_inuse = rx_buffer[tid].pkg_buf_inuse;
    char* buffer = rx_buffer[tid].buffer.data();
    int num_radios = config_->nBsSdrs[0];
    int radio_start = tid * num_radios / thread_num_;
    int radio_end = (tid + 1) * num_radios / thread_num_;

    printf("receiver thread %d has %d radios\n", tid, radio_end - radio_start);

    int cursor = 0;
    while (config_->running) {
        // receive data
        for (int it = radio_start; it < radio_end; it++) {
            Package* pkg[bsSdrCh];
            void* samp[bsSdrCh];

            // Set buffer status(es) to full; fail if full already
            for (auto ch = 0; ch < bsSdrCh; ++ch) {
                int bit = 1 << (cursor + ch) % sizeof(std::atomic_int);
                int offs = (cursor + ch) / sizeof(std::atomic_int);
                int old = std::atomic_fetch_or(&pkg_buf_inuse[offs], bit); // now full
                // if buffer was full, exit
                if ((old & bit) != 0) {
                    printf("thread %d buffer full\n", tid);
                    exit(0);
                }
                // Reserved until marked empty by consumer.
            }

            // Receive data into buffers
            size_t packageLength = sizeof(Package) + config_->getPackageDataLength();
            for (auto ch = 0; ch < bsSdrCh; ++ch) {
                pkg[ch] = (Package*)(buffer + (cursor + ch) * packageLength);
                samp[ch] = pkg[ch]->data;
            }
            long long frameTime;
            assert(baseRadioSet_ != NULL);
            if (baseRadioSet_->radioRx(it, samp, frameTime) < 0) {
                config_->running = false;
                break;
            }

            int frame_id = (int)(frameTime >> 32);
            int symbol_id = (int)((frameTime >> 16) & 0xFFFF);
            int ant_id = it * bsSdrCh;
#if DEBUG_PRINT
            for (auto ch = 0; ch < bsSdrCh; ++ch) {
                printf("receive thread %d, frame %d, symbol %d, cell %d, ant %d samples: %d %d %d %d %d %d %d %d ...\n",
                    tid, frame_id, symbol_id, 0, ant_id + ch,
                    pkg[ch]->data[1], pkg[ch]->data[2], pkg[ch]->data[3], pkg[ch]->data[4],
                    pkg[ch]->data[5], pkg[ch]->data[6], pkg[ch]->data[7], pkg[ch]->data[8]);
            }
#endif
            for (auto ch = 0; ch < bsSdrCh; ++ch) {
                new (pkg[ch]) Package(frame_id, symbol_id, 0, ant_id + ch);
                // push EVENT_RX_SYMBOL event into the queue
                Event_data package_message;
                package_message.event_type = EVENT_RX_SYMBOL;
                // data records the position of this packet in the buffer & tid of this socket
                // (so that task thread could know which buffer it should visit)
                package_message.data = cursor + tid * buffer_chunk_size;
                if (!message_queue_->enqueue(local_ptok, package_message)) {
                    printf("socket message enqueue failed\n");
                    exit(0);
                }
                cursor++;
                cursor %= buffer_chunk_size;
            }
        }
    }
}

void* Receiver::clientTxRx_launch(void* in_context)
{
    dev_profile* context = (dev_profile*)in_context;
    Receiver* receiver = context->ptr;
    int tid = context->tid;
    delete context;
    if (receiver->config_->hw_framer)
        receiver->clientTxRx(tid);
    else
        receiver->clientSyncTxRx(tid);
    return 0;
}

void Receiver::clientTxRx(int tid)
{
    int txSyms = config_->clULSymbols[tid].size();
    int rxSyms = config_->clDLSymbols[tid].size();
    int txStartSym = config_->clULSymbols[tid].empty() ? 0 : config_->clULSymbols[tid][0];

    double frameTime = config_->sampsPerSymbol * config_->clFrames[0].size() * 1e3 / config_->rate; // miliseconds
    unsigned txFrameDelta = (unsigned)(std::ceil(TIME_DELTA / frameTime));
    int NUM_SAMPS = config_->sampsPerSymbol;

    if (config_->core_alloc) {
        int core = tid + 1 + config_->rx_thread_num + config_->task_thread_num;
        printf("pinning client thread %d to core %d\n", tid, core);
        if (pin_to_core(core) != 0) {
            printf("pin client thread %d to core %d failed\n", tid, core);
            exit(0);
        }
    }

    //while(!d_mutex.try_lock()){}
    //thread_count++;
    //std::cout << "Thread " << tid << ", txSyms " << txSyms << ", rxSyms " << rxSyms << ", txStartSym " << txStartSym << ", rate " << config_->rate << ", txFrameDelta " << txFrameDelta << ", nsamps " << NUM_SAMPS << std::endl;
    //d_mutex.unlock();

    std::vector<std::complex<float>> buffs(NUM_SAMPS, 0);
    std::vector<void*> rxbuff(2);
    rxbuff[0] = buffs.data();
    rxbuff[1] = buffs.data();

    std::vector<void*> txbuff(2);
    if (txSyms > 0) {
        size_t txIndex = tid * config_->clSdrCh;
        txbuff[0] = config_->txdata[txIndex].data();
        if (config_->clSdrCh == 2)
            txbuff[1] = config_->txdata[txIndex + 1].data();
        std::cout << txSyms << " uplink symbols will be sent per frame..." << std::endl;
    }

    int all_trigs = 0;
    struct timespec tv, tv2;
    clock_gettime(CLOCK_MONOTONIC, &tv);

    assert(clientRadioSet_ != NULL);
    while (config_->running) {
        clock_gettime(CLOCK_MONOTONIC, &tv2);
        double diff = ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec)) / 1e9;
        if (config_->frame_mode != "free_running" and diff > 2) {
            int total_trigs = clientRadioSet_->triggers(tid);
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
            int r = clientRadioSet_->radioRx(tid, rxbuff.data(), NUM_SAMPS, rxTime);
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
        txTime = firstRxTime & 0xFFFFFFFF00000000;
        txTime += ((long long)txFrameDelta << 32);
        txTime += ((long long)txStartSym << 16);
        //printf("rxTime %llx, txTime %llx \n", firstRxTime, txTime);
        for (int i = 0; i < txSyms; i++) {
            int r = clientRadioSet_->radioTx(tid, txbuff.data(), NUM_SAMPS, 1, txTime);
            if (r == NUM_SAMPS)
                txTime += 0x10000;
        }
    }
}

void Receiver::clientSyncTxRx(int tid)
{
    size_t frameTimeLen = config_->sampsPerSymbol * config_->clFrames[0].size();
    size_t txFrameDelta = 10 * frameTimeLen;

    int NUM_SAMPS = config_->sampsPerSymbol;
    int SYNC_NUM_SAMPS = config_->sampsPerSymbol * config_->symbolsPerFrame;

    if (config_->core_alloc) {
        int core = tid + 1 + config_->rx_thread_num + config_->task_thread_num;
        printf("pinning client thread %d to core %d\n", tid, core);
        if (pin_to_core(core) != 0) {
            printf("pin client thread %d to core %d failed\n", tid, core);
            exit(0);
        }
    }

    std::vector<std::complex<float>> syncbuff0(SYNC_NUM_SAMPS, 0);
    std::vector<std::complex<float>> syncbuff1(SYNC_NUM_SAMPS, 0);
    std::vector<void*> syncrxbuff(2);
    syncrxbuff[0] = syncbuff0.data();

    std::vector<std::complex<float>> buff0(NUM_SAMPS, 0);
    std::vector<std::complex<float>> buff1(NUM_SAMPS, 0);
    std::vector<void*> rxbuff(2);
    rxbuff[0] = buff0.data();

    std::vector<void*> pilotbuffA(2);
    std::vector<void*> pilotbuffB(2);
    pilotbuffA[0] = config_->pilot_cf32.data();
    if (config_->clSdrCh == 2) {
        pilotbuffA[1] = std::vector<std::complex<float>>(NUM_SAMPS, 0).data();
        pilotbuffB[1] = config_->pilot_cf32.data();
        pilotbuffB[0] = std::vector<std::complex<float>>(NUM_SAMPS, 0).data();
        syncrxbuff[1] = syncbuff1.data();
        rxbuff[1] = buff1.data();
    }

    std::vector<void*> txbuff(2);
    size_t txSyms = config_->clULSymbols[tid].size();
    if (txSyms > 0) {
        size_t txIndex = tid * config_->clSdrCh;
        txbuff[0] = config_->txdata[txIndex].data();
        if (config_->clSdrCh == 2)
            txbuff[1] = config_->txdata[txIndex + 1].data();
        std::cout << txSyms << " uplink symbols will be sent per frame..." << std::endl;
    }

    long long rxTime(0);
    long long txTime(0);
    int sync_index(-1);
    int rx_offset = 0;
    // Keep reading one frame worth of data until a beacon is found
    while (config_->running && sync_index < 0) {
        int r = clientRadioSet_->radioRx(tid, syncrxbuff.data(), SYNC_NUM_SAMPS, rxTime);

        if (r != SYNC_NUM_SAMPS) {
            std::cerr << "BAD SYNC Receive(" << r << "/" << SYNC_NUM_SAMPS << ") at Time " << rxTime << std::endl;
            continue;
        }
        sync_index = CommsLib::find_beacon_avx(syncbuff0, config_->gold_cf32);
        if (sync_index < 0)
            continue;
        std::cout << "Beacon detected at Time " << rxTime << ", sync_index: " << sync_index << std::endl;
        rx_offset = sync_index - config_->beaconSize - config_->prefix;
    }

    // Read rx_offset to align with the begining of a frame
    clientRadioSet_->radioRx(tid, syncrxbuff.data(), rx_offset, rxTime);

    // Main client read/write loop.
    size_t recv_count = 0;
    while (config_->running) {
        for (int sf = 0; sf < config_->symbolsPerFrame; sf++) {
            int r = clientRadioSet_->radioRx(tid, rxbuff.data(), NUM_SAMPS, rxTime);
            if (r != NUM_SAMPS) {
                std::cerr << "BAD Receive(" << r << "/" << NUM_SAMPS << ") at Time " << rxTime << ", recv count " << recv_count << std::endl;
            }
            if (r < 0) {
                config_->running = false;
                return;
            }
            recv_count++;
            // schedule all TX subframes
            if (sf == 0) {
                txTime = rxTime + txFrameDelta + config_->clPilotSymbols[tid][0] * NUM_SAMPS - config_->txAdvance;
                r = clientRadioSet_->radioTx(tid, pilotbuffA.data(), NUM_SAMPS, 2, txTime);
                if (r < NUM_SAMPS)
                    std::cout << "BAD Write: " << r << "/" << NUM_SAMPS << std::endl;
                if (config_->clSdrCh == 2) {
                    txTime = rxTime + txFrameDelta + config_->clPilotSymbols[tid][1] * NUM_SAMPS - config_->txAdvance;
                    //Time += NUM_SAMPS;
                    r = clientRadioSet_->radioTx(tid, pilotbuffB.data(), NUM_SAMPS, 2, txTime);
                    if (r < NUM_SAMPS)
                        std::cout << "BAD Write: " << r << "/" << NUM_SAMPS << std::endl;
                }
                if (config_->ulDataSymPresent) {
                    for (size_t s = 0; s < txSyms; s++) {
                        txTime = rxTime + txFrameDelta + config_->clULSymbols[tid][s] * NUM_SAMPS - config_->txAdvance;
                        r = clientRadioSet_->radioTx(tid, txbuff.data(), NUM_SAMPS, 2, txTime);
                        if (r < NUM_SAMPS)
                            std::cout << "BAD Write: " << r << "/" << NUM_SAMPS << std::endl;
                    }
                }
            }
            // perhaps resync every few thousands frames
        }
    }
}
