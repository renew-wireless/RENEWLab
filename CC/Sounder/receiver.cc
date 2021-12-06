/*
 Copyright (c) 2018-2021, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
----------------------------------------------------------
 Handles received samples from massive-mimo base station 
----------------------------------------------------------
*/

#include "include/receiver.h"
#include "include/ClientRadioSet.h"
#include "include/comms-lib.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"

#include <SoapySDR/Time.hpp>
#include <atomic>
#include <random>
#include <unistd.h>

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

Receiver::Receiver(Config* config,
    moodycamel::ConcurrentQueue<Event_data>* in_queue,
    std::vector<moodycamel::ConcurrentQueue<Event_data>*> tx_queue,
    std::vector<moodycamel::ProducerToken*> tx_ptoks,
    std::vector<moodycamel::ConcurrentQueue<Event_data>*> cl_tx_queue,
    std::vector<moodycamel::ProducerToken*> cl_tx_ptoks)
    : config_(config)
    , message_queue_(in_queue)
    , tx_queue_(tx_queue)
    , tx_ptoks_(tx_ptoks)
    , cl_tx_queue_(cl_tx_queue)
    , cl_tx_ptoks_(cl_tx_ptoks)
{
    /* initialize random seed: */
    srand(time(NULL));

    MLPD_TRACE("Receiver Construction - CL present: %d, BS Present: %d\n",
        config_->client_present(), config_->bs_present());

    try {
        this->clientRadioSet_
            = config_->client_present() ? new ClientRadioSet(config_) : nullptr;
        this->base_radio_set_
            = config_->bs_present() ? new BaseRadioSet(config_) : nullptr;
    } catch (std::exception& e) {
        throw ReceiverException("Invalid Radio Setup");
    }

    MLPD_TRACE("Receiver Construction -- number radios %zu\n",
        config_->num_bs_sdrs_all());

    if (((this->base_radio_set_ != nullptr)
            && (this->base_radio_set_->getRadioNotFound()))
        || ((this->clientRadioSet_ != nullptr)
               && (this->clientRadioSet_->getRadioNotFound()))) {
        if (this->base_radio_set_ != nullptr) {
            MLPD_WARN("Invalid Base Radio Setup: %d\n",
                this->base_radio_set_ == nullptr);
            this->base_radio_set_->radioStop();
            delete this->base_radio_set_;
        }
        if (this->clientRadioSet_ != nullptr) {
            MLPD_WARN("Invalid Client Radio Setup: %d\n",
                this->clientRadioSet_ == nullptr);
            this->clientRadioSet_->radioStop();
            delete this->clientRadioSet_;
        }
        throw ReceiverException("Invalid Radio Setup");
    }
    this->initBuffers();
    MLPD_TRACE("Construction complete\n");
}

Receiver::~Receiver()
{
    MLPD_TRACE("Radio Set cleanup, Base: %d, Client: %d\n",
        this->base_radio_set_ == nullptr, this->clientRadioSet_ == nullptr);
    if (this->base_radio_set_ != nullptr) {
        this->base_radio_set_->radioStop();
        delete this->base_radio_set_;
    }
    if (this->clientRadioSet_ != nullptr) {
        this->clientRadioSet_->radioStop();
        delete this->clientRadioSet_;
    }

    for (auto memory : zeros) {
        MLPD_SYMBOL("Process %d -- Client Sync Tx Rx Freed memory at %p\n", tid,
            memory);
        free(memory);
    }
    zeros.clear();
}

void Receiver::initBuffers()
{
    size_t frameTimeLen = config_->samps_per_frame();
    txFrameDelta
        = std::ceil(TIME_DELTA / (1e3 * config_->getFrameDurationSec()));
    txTimeDelta = txFrameDelta * frameTimeLen;

    zeros.resize(2);
    pilotbuffA.resize(2);
    pilotbuffB.resize(2);
    for (auto& memory : zeros) {
        memory = calloc(config_->samps_per_slot(), sizeof(int16_t) * 2);
        if (memory == NULL) {
            throw std::runtime_error("Error allocating memory");
        }
    }
    pilotbuffA.at(0) = config_->pilot_cf32().data();
    if (config_->cl_sdr_ch() == 2) {
        pilotbuffA.at(1) = zeros.at(0);
        pilotbuffB.at(1) = config_->pilot_ci16().data();
        pilotbuffB.at(0) = zeros.at(1);
    }
}

std::vector<pthread_t> Receiver::startClientThreads(
    SampleBuffer* rx_buffer, SampleBuffer* tx_buffer, unsigned in_core_id)
{
    cl_tx_buffer_ = tx_buffer;
    std::vector<pthread_t> client_threads;
    if (config_->client_present() == true) {
        client_threads.resize(config_->num_cl_sdrs());
        for (unsigned int i = 0; i < config_->num_cl_sdrs(); i++) {
            pthread_t cl_thread_;
            // record the thread id
            ReceiverContext* context = new ReceiverContext;
            context->ptr = this;
            context->core_id = in_core_id;
            context->tid = i;
            context->buffer = rx_buffer;
            // start socket thread
            if (pthread_create(
                    &cl_thread_, NULL, Receiver::clientTxRx_launch, context)
                != 0) {
                MLPD_ERROR("Socket client thread create failed in start client "
                           "threads");
                throw std::runtime_error("Socket client thread create failed "
                                         "in start client threads");
            }
            client_threads[i] = cl_thread_;
        }
    }
    return client_threads;
}

std::vector<pthread_t> Receiver::startRecvThreads(SampleBuffer* rx_buffer,
    size_t n_rx_threads, SampleBuffer* tx_buffer, unsigned in_core_id)
{
    assert(rx_buffer[0].buffer.size() != 0);
    thread_num_ = n_rx_threads;
    bs_tx_buffer_ = tx_buffer;
    std::vector<pthread_t> created_threads;
    created_threads.resize(this->thread_num_);
    for (size_t i = 0; i < this->thread_num_; i++) {
        // record the thread id
        ReceiverContext* context = new ReceiverContext;
        context->ptr = this;
        context->core_id = in_core_id;
        context->tid = i;
        context->buffer = rx_buffer;
        // start socket thread
        if (pthread_create(&created_threads.at(i), NULL,
                Receiver::loopRecv_launch, context)
            != 0) {
            MLPD_ERROR("Socket recv thread create failed");
            throw std::runtime_error("Socket recv thread create failed");
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
         it != recv_thread.end(); ++it) {
        pthread_join(*it, NULL);
    }
}

void Receiver::go()
{
    if (this->base_radio_set_ != NULL) {
        this->base_radio_set_->radioStart(); // hardware trigger
    }
}

int Receiver::baseTxData(int radio_id, int cell, long long base_time)
{
    int num_samps = config_->samps_per_slot();
    size_t packetLength = sizeof(Packet) + config_->getPacketDataLength();
    size_t tx_buffer_size
        = bs_tx_buffer_[radio_id].buffer.size() / packetLength;
    int flagsTxData;
    std::vector<void*> dl_txbuff(2);
    Event_data event;
    if (tx_queue_.at(radio_id)->try_dequeue_from_producer(
            *tx_ptoks_.at(radio_id), event)
        == true) {
        assert(event.event_type == kEventTxSymbol);
        assert(event.ant_id == radio_id);
        size_t cur_offset = event.offset;
        for (size_t s = 0; s < config_->dl_slot_per_frame(); s++) {
            for (size_t ch = 0; ch < config_->bs_sdr_ch(); ++ch) {
                char* cur_ptr_buffer = bs_tx_buffer_[radio_id].buffer.data()
                    + (cur_offset * packetLength);
                Packet* pkt = reinterpret_cast<Packet*>(cur_ptr_buffer);
                assert(pkt->slot_id == config_->dl_slots().at(cell).at(s));
                assert(pkt->ant_id == config_->bs_sdr_ch() * radio_id + ch);
                dl_txbuff.at(ch) = pkt->data;
                cur_offset = (cur_offset + 1) % tx_buffer_size;
            }
            long long txTime = 0;
            if (kUseUHD == true || config_->bs_hw_framer() == false) {
                txTime = base_time + txTimeDelta
                    + config_->dl_slots().at(radio_id).at(s) * num_samps
                    - config_->tx_advance();
            } else {
                size_t frame_id = (size_t)(base_time >> 32);
                txTime = (frame_id + txFrameDelta) << 32
                    | (config_->dl_slots().at(cell).at(s) << 16);
            }
            if (kUseUHD == true && s < (config_->dl_slot_per_frame() - 1))
                flagsTxData = kStreamContinuous; // HAS_TIME
            else
                flagsTxData = kStreamEndBurst; // HAS_TIME & END_BURST, fixme
            int r = this->base_radio_set_->radioTx(
                radio_id, cell, dl_txbuff.data(), flagsTxData, txTime);
            if (r < num_samps) {
                MLPD_WARN("BAD Write: %d/%d\n", r, num_samps);
            }
        }
        bs_tx_buffer_[radio_id]
            .pkt_buf_inuse[event.frame_id % kSampleBufferFrameNum]
            = 0;
        return 0;
    }
    return -1;
}

void Receiver::notifyPacket(NodeType node_type, int frame_id, int slot_id,
    int ant_id, int buff_size, int offset)
{
    moodycamel::ProducerToken local_ptok(*message_queue_);
    Event_data new_frame;
    new_frame.event_type = kEventRxSymbol;
    new_frame.frame_id = frame_id;
    new_frame.slot_id = slot_id;
    new_frame.ant_id = ant_id;
    new_frame.node_type = node_type;
    new_frame.buff_size = buff_size;
    new_frame.offset = offset;
    if (message_queue_->enqueue(local_ptok, new_frame) == false) {
        MLPD_ERROR("New frame message enqueue failed\n");
        throw std::runtime_error("New frame message enqueue failed");
    }
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
    if (config_->core_alloc() == true) {
        MLPD_INFO("Pinning rx thread %d to core %d\n", tid, core_id + tid);
        if (pin_to_core(core_id + tid) != 0) {
            MLPD_ERROR(
                "Pin rx thread %d to core %d failed\n", tid, core_id + tid);
            throw std::runtime_error("Pin rx thread to core failed");
        }
    }

    // Use mutex to sychronize data receiving across threads
    if (config_->internal_measurement()
        || ((config_->num_cl_sdrs() > 0) && (config_->num_bs_sdrs_all() > 0))) {
        pthread_mutex_lock(&mutex);
        MLPD_INFO("Recv Thread %d: waiting for release\n", tid);
        pthread_cond_wait(&cond, &mutex);
        pthread_mutex_unlock(&mutex); // unlocking for all other threads
    }

    // use token to speed up
    moodycamel::ProducerToken local_ptok(*message_queue_);

    const size_t num_channels = config_->bs_channel().length();
    size_t packetLength = sizeof(Packet) + config_->getPacketDataLength();
    int buffer_chunk_size = rx_buffer[0].buffer.size() / packetLength;
    int bs_tx_buff_size = kSampleBufferFrameNum * config_->slot_per_frame();

    // handle two channels at each radio
    // this is assuming buffer_chunk_size is at least 2
    std::atomic_int* pkt_buf_inuse = rx_buffer[tid].pkt_buf_inuse;
    char* buffer = rx_buffer[tid].buffer.data();

    size_t num_radios = config_->num_bs_sdrs_all(); //config_->n_bs_sdrs()[0]
    std::vector<size_t> radio_ids_in_thread;
    if (config_->internal_measurement() && config_->ref_node_enable()) {
        if (tid == 0)
            radio_ids_in_thread.push_back(config_->cal_ref_sdr_id());
        else
            // FIXME: Does this work in multi-cell case?
            for (size_t it = 0; it < config_->num_bs_sdrs_all(); it++)
                if (it != config_->cal_ref_sdr_id())
                    radio_ids_in_thread.push_back(it);
    } else {
        size_t radio_start = (tid * num_radios) / thread_num_;
        size_t radio_end = ((tid + 1) * num_radios) / thread_num_;
        for (size_t it = radio_start; it < radio_end; it++)
            radio_ids_in_thread.push_back(it);
    }
    MLPD_INFO(
        "Receiver thread %d has %zu radios\n", tid, radio_ids_in_thread.size());
    MLPD_TRACE(
        " -- %d - radio start: %zu, end: %zu, total radios %zu, thread: %d\n",
        tid, radio_ids_in_thread.front(), radio_ids_in_thread.back(),
        num_radios, thread_num_);

    // prepare BS beacon in host buffer
    std::vector<void*> beaconbuff(2);
    void* zeroes_memory
        = calloc(config_->samps_per_slot(), sizeof(int16_t) * 2);

    if (zeroes_memory == NULL) {
        throw std::runtime_error("Memory allocation error");
    }

    MLPD_SYMBOL(
        "Process %d -- Loop Rx Allocated memory at: %p, approx size: %lu\n",
        tid, zeroes_memory, (sizeof(int16_t) * 2) * config_->samps_per_slot());
    beaconbuff.at(0u) = config_->beacon_ci16().data();
    beaconbuff.at(1u) = zeroes_memory;

    long long rxTimeBs(0);
    long long txTimeBs(0);

    // read rx_offset to align the FPGA time of the BS
    // by performing dummy readStream()
    std::vector<std::complex<int16_t>> samp_buffer0(
        config_->samps_per_frame(), 0);
    std::vector<std::complex<int16_t>> samp_buffer1(
        config_->samps_per_frame(), 0);
    std::vector<void*> samp_buffer(2);
    samp_buffer[0] = samp_buffer0.data();
    if (num_channels == 2)
        samp_buffer[1] = samp_buffer1.data();

    int cell = 0;
    // for UHD device, the first pilot should not have an END_BURST flag
    if (kUseUHD == true) {
        // For multi-USRP BS perform dummy radioRx to avoid initial late packets
        int bs_sync_ret = -1;
        MLPD_INFO("Sync BS host and FPGA timestamp for thread %d\n", tid);
        for (auto& it : radio_ids_in_thread) {
            // Find cell this USRP belongs to..,
            for (size_t i = 0; i <= config_->num_cells(); i++) {
                if (it < config_->n_bs_sdrs_agg().at(i)) {
                    cell = i - 1;
                    break;
                }
            }
            size_t radio_id = it - config_->n_bs_sdrs_agg().at(cell);
            bs_sync_ret = -1;
            while (bs_sync_ret < 0) {
                bs_sync_ret = this->base_radio_set_->radioRx(radio_id, cell,
                    samp_buffer.data(), config_->samps_per_slot(), rxTimeBs);
            }
        }
    }

    int cursor = 0;
    size_t frame_id = 0;
    size_t slot_id = 0;
    size_t ant_id = 0;
    cell = 0;
    MLPD_INFO("Start BS main recv loop in thread %d\n", tid);
    while (config_->running() == true) {

        // Global updates of frame and slot IDs for USRPs
        if (kUseUHD == true || config_->bs_hw_framer() == false) {
            if (slot_id == config_->slot_per_frame()) {
                slot_id = 0;
                frame_id++;
            }
        }

        // Receive data
        for (auto& it : radio_ids_in_thread) {
            Packet* pkt[num_channels];
            void* samp[num_channels];

            // Find cell this board belongs to...
            for (size_t i = 0; i <= config_->num_cells(); i++) {
                if (it < config_->n_bs_sdrs_agg().at(i)) {
                    cell = i - 1;
                    break;
                }
            }

            size_t radio_id = it - config_->n_bs_sdrs_agg().at(cell);

            size_t num_packets = config_->internal_measurement()
                    && radio_id == config_->cal_ref_sdr_id()
                    && config_->ref_node_enable()
                ? 1
                : num_channels; // receive only on one channel at the ref antenna

            // Set buffer status(es) to full; fail if full already
            for (size_t ch = 0; ch < num_packets; ++ch) {
                int bit = 1 << (cursor + ch) % sizeof(std::atomic_int);
                int offs = (cursor + ch) / sizeof(std::atomic_int);
                int old = std::atomic_fetch_or(
                    &pkt_buf_inuse[offs], bit); // now full
                // if buffer was full, exit
                if ((old & bit) != 0) {
                    MLPD_ERROR("thread %d buffer full\n", tid);
                    throw std::runtime_error("Thread %d buffer full\n");
                }
                // Reserved until marked empty by consumer
            }

            // Receive data into buffers
            for (size_t ch = 0; ch < num_packets; ++ch) {
                pkt[ch] = (Packet*)(buffer + (cursor + ch) * packetLength);
                samp[ch] = pkt[ch]->data;
            }
            if (num_packets != num_channels)
                samp[num_channels - 1] = std::vector<char>(packetLength).data();

            assert(this->base_radio_set_ != NULL);
            ant_id = radio_id * num_channels;

            if (kUseUHD == true || config_->bs_hw_framer() == false) {
                int rx_len = config_->samps_per_slot();
                int r;

                // only write received pilot or data into samp
                // otherwise use samp_buffer as a dummy buffer
                if (config_->isPilot(frame_id, slot_id)
                    || config_->isUlData(frame_id, slot_id))
                    r = this->base_radio_set_->radioRx(
                        radio_id, cell, samp, rxTimeBs);
                else
                    r = this->base_radio_set_->radioRx(
                        radio_id, cell, samp_buffer.data(), rxTimeBs);

                if (r < 0) {
                    config_->running(false);
                    break;
                }
                if (r != rx_len) {
                    std::cerr << "BAD Receive(" << r << "/" << rx_len
                              << ") at Time " << rxTimeBs << ", frame count "
                              << frame_id << std::endl;
                }

                // schedule all TX slot
                if (slot_id == 0) {
                    // Schedule next beacon in BEACON_INTERVAL frames
                    // FIXME?? From EACH cell or only one cell?
                    txTimeBs = rxTimeBs
                        + config_->samps_per_frame() * BEACON_INTERVAL;
                    int r_tx = this->base_radio_set_->radioTx(radio_id, cell,
                        beaconbuff.data(), kStreamEndBurst, txTimeBs);
                    if (r_tx != (int)config_->samps_per_slot())
                        std::cerr << "BAD Transmit(" << r_tx << "/"
                                  << config_->samps_per_slot() << ") at Time "
                                  << txTimeBs << ", frame count " << frame_id
                                  << std::endl;

                    // schedule downlink slots
                    if (config_->dl_data_slot_present() == true) {
                        while (-1 != baseTxData(radio_id, cell, rxTimeBs))
                            ;
                        this->notifyPacket(kBS, frame_id + this->txFrameDelta,
                            0, radio_id,
                            bs_tx_buff_size); // Notify new frame
                    } // end if config_->dul_data_slot_present()
                }
                if (!config_->isPilot(frame_id, slot_id)
                    && !config_->isUlData(frame_id, slot_id))
                    continue;
            } else {
                long long frameTime;
                if (this->base_radio_set_->radioRx(
                        radio_id, cell, samp, frameTime)
                    < 0) {
                    config_->running(false);
                    break;
                }

                frame_id = (size_t)(frameTime >> 32);
                slot_id = (size_t)((frameTime >> 16) & 0xFFFF);

                if (config_->internal_measurement()
                    && config_->ref_node_enable()) {
                    if (radio_id == config_->cal_ref_sdr_id()) {
                        ant_id = slot_id < radio_id * num_channels
                            ? slot_id
                            : slot_id - num_channels;
                        slot_id = 0; // downlink reciprocal pilot
                    } else {
                        if (radio_id >= config_->cal_ref_sdr_id())
                            ant_id -= num_channels;
                        slot_id = 1; // uplink reciprocal pilot
                    }
                } else if (config_->internal_measurement()
                    && !config_->ref_node_enable()) {
                    // Mapping (compress schedule to eliminate Gs)
                    size_t adv
                        = int(slot_id / (config_->guard_mult() * num_channels));
                    slot_id = slot_id - ((config_->guard_mult() - 1) * 2 * adv);
                }
                if (config_->getClientId(frame_id, slot_id)
                    == 0) { // first received pilot
                    if (config_->dl_data_slot_present() == true) {
                        while (-1 != baseTxData(radio_id, cell, frameTime))
                            ;
                        this->notifyPacket(kBS, frame_id + this->txFrameDelta,
                            0, radio_id,
                            bs_tx_buff_size); // Notify new frame
                    }
                }
            }

#if DEBUG_PRINT
            for (size_t ch = 0; ch < num_packets; ++ch) {
                printf("receive thread %d, frame %zu, slot %zu, cell %zu, ant "
                       "%zu samples: %d %d %d %d %d %d %d %d ...\n",
                    tid, frame_id, slot_id, cell, ant_id + ch, pkt[ch]->data[1],
                    pkt[ch]->data[2], pkt[ch]->data[3], pkt[ch]->data[4],
                    pkt[ch]->data[5], pkt[ch]->data[6], pkt[ch]->data[7],
                    pkt[ch]->data[8]);
            }
#endif

            for (size_t ch = 0; ch < num_packets; ++ch) {
                // new (pkt[ch]) Packet(frame_id, slot_id, 0, ant_id + ch);
                new (pkt[ch]) Packet(frame_id, slot_id, cell, ant_id + ch);
                // push kEventRxSymbol event into the queue
                this->notifyPacket(kBS, frame_id, slot_id, ant_id + ch,
                    buffer_chunk_size, cursor + tid * buffer_chunk_size);
                cursor++;
                cursor %= buffer_chunk_size;
            }
        }

        // for UHD device update slot_id on host
        if (kUseUHD == true || config_->bs_hw_framer() == false) {
            slot_id++;
        }
    }
    MLPD_SYMBOL(
        "Process %d -- Loop Rx Freed memory at: %p\n", tid, zeroes_memory);
    free(zeroes_memory);
}

void* Receiver::clientTxRx_launch(void* in_context)
{
    ReceiverContext* context = (ReceiverContext*)in_context;
    auto me = context->ptr;
    auto tid = context->tid;
    auto core_id = context->core_id;
    auto buffer = context->buffer;
    delete context;
    if (me->config_->hw_framer())
        me->clientTxRx(tid);
    else
        me->clientSyncTxRx(tid, core_id, buffer);
    return 0;
}

void Receiver::clientTxRx(int tid)
{
    size_t tx_slots = config_->cl_ul_slots().at(tid).size();
    size_t rxSyms = config_->cl_dl_slots().at(tid).size();
    int txStartSym = config_->cl_ul_slots().at(tid).empty()
        ? 0
        : config_->cl_ul_slots().at(tid).at(0);
    int NUM_SAMPS = config_->samps_per_slot();

    if (config_->core_alloc() == true) {
        int core = tid + 1 + config_->bs_rx_thread_num()
            + config_->recorder_thread_num();
        MLPD_INFO("Pinning client TxRx thread %d to core %d\n", tid, core);
        if (pin_to_core(core) != 0) {
            MLPD_ERROR(
                "Pin client TxRx thread %d to core %d failed in client txrx\n",
                tid, core);
            throw std::runtime_error(
                "Pin client TxRx thread to core failed in client txr");
        }
    }

    std::vector<std::complex<int16_t>> buffs(NUM_SAMPS, 0);
    std::vector<void*> rxbuff(2);
    rxbuff[0] = buffs.data();
    rxbuff[1] = buffs.data();

    std::vector<void*> ul_txbuff(2);
    for (size_t ch = 0; ch < config_->cl_sdr_ch(); ch++) {
        ul_txbuff.at(ch)
            = std::calloc(config_->samps_per_slot(), sizeof(int16_t) * 2);
    }
    size_t slot_byte_size = config_->samps_per_slot() * sizeof(int16_t) * 2;
    if (tx_slots > 0) {
        size_t txIndex = tid * config_->cl_sdr_ch();
        for (size_t ch = 0; ch < config_->cl_sdr_ch(); ch++) {
            std::memcpy(ul_txbuff.at(ch),
                config_->txdata_time_dom().at(txIndex + ch).data(),
                slot_byte_size);
        }
        MLPD_INFO("%zu uplink slots will be sent per frame...\n", tx_slots);
    }

    int all_trigs = 0;
    struct timespec tv, tv2;
    clock_gettime(CLOCK_MONOTONIC, &tv);

    assert(clientRadioSet_ != NULL);
    while (config_->running() == true) {
        clock_gettime(CLOCK_MONOTONIC, &tv2);
        double diff
            = ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec))
            / 1e9;
        if ((config_->frame_mode() != "free_running") && (diff > 2)) {
            int total_trigs = clientRadioSet_->triggers(tid);
            std::cout << "new triggers: " << total_trigs - all_trigs
                      << ", total: " << total_trigs << std::endl;
            all_trigs = total_trigs;
            tv = tv2;
        }
        // receiver loop
        long long rxTime(0);
        long long txTime(0);
        long long firstRxTime(0);
        bool receiveErrors = false;
        for (size_t i = 0; i < rxSyms; i++) {
            int r = clientRadioSet_->radioRx(
                tid, rxbuff.data(), NUM_SAMPS, rxTime);
            if (r == NUM_SAMPS) {
                if (i == 0)
                    firstRxTime = rxTime;
            } else {
                std::cerr << "waiting for receive frames... " << std::endl;
                receiveErrors = true;
                break;
            }
        }
        if (receiveErrors == false) {
            // transmit loop
            txTime = firstRxTime & 0xFFFFFFFF00000000;
            txTime += ((long long)this->txFrameDelta << 32);
            txTime += ((long long)txStartSym << 16);
            //printf("rxTime %llx, txTime %llx \n", firstRxTime, txTime);
            for (size_t i = 0; i < tx_slots; i++) {
                int r = clientRadioSet_->radioTx(
                    tid, ul_txbuff.data(), NUM_SAMPS, 1, txTime);
                if (r == NUM_SAMPS) {
                    txTime += 0x10000;
                }
            }
        } // end receiveErrors == false)
    } // end while config_->running() == true)
    for (size_t ch = 0; ch < config_->cl_sdr_ch(); ch++) {
        std::free(ul_txbuff.at(ch));
    }
}

void Receiver::txPilots(size_t user_id, long long base_time)
{
    // for UHD device, the first pilot should not have an END_BURST flag
    int flags = (((kUseUHD == true) && (config_->cl_sdr_ch() == 2)))
        ? kStreamContinuous
        : kStreamEndBurst;
    int num_samps = config_->samps_per_slot();
    long long txTime = base_time
        + config_->cl_pilot_slots().at(user_id).at(0) * num_samps
        - config_->tx_advance();

    int r = clientRadioSet_->radioTx(
        user_id, pilotbuffA.data(), num_samps, flags, txTime);
    if (r < num_samps) {
        MLPD_WARN("BAD Write: %d/%d\n", r, num_samps);
    }
    if (config_->cl_sdr_ch() == 2) {
        txTime = base_time
            + config_->cl_pilot_slots().at(user_id).at(1) * num_samps
            - config_->tx_advance();

        r = clientRadioSet_->radioTx(
            user_id, pilotbuffB.data(), num_samps, kStreamEndBurst, txTime);
        if (r < num_samps) {
            MLPD_WARN("BAD Write: %d/%d\n", r, num_samps);
        }
    }
}

int Receiver::clientTxData(int tid, int frame_id, long long base_time)
{
    size_t tx_slots = config_->cl_ul_slots().at(tid).size();
    int num_samps = config_->samps_per_slot();
    size_t packetLength = sizeof(Packet) + config_->getPacketDataLength();
    size_t tx_buffer_size = cl_tx_buffer_[tid].buffer.size() / packetLength;
    int flagsTxUlData;
    std::vector<void*> ul_txbuff(2);
    Event_data event;
    if (cl_tx_queue_.at(tid)->try_dequeue_from_producer(
            *cl_tx_ptoks_.at(tid), event)
        == true) {
        assert(event.event_type == kEventTxSymbol);
        assert(event.ant_id == tid);
        size_t cur_offset = event.offset;
        long long txFrameTime = base_time
            + (event.frame_id - frame_id) * config_->samps_per_frame();
        txPilots(tid, txFrameTime); // assuming pilot is always sent before data

        for (size_t s = 0; s < tx_slots; s++) {
            for (size_t ch = 0; ch < config_->cl_sdr_ch(); ++ch) {
                char* cur_ptr_buffer = cl_tx_buffer_[tid].buffer.data()
                    + (cur_offset * packetLength);
                Packet* pkt = reinterpret_cast<Packet*>(cur_ptr_buffer);
                assert(pkt->slot_id == config_->cl_ul_slots().at(tid).at(s));
                assert(pkt->ant_id == config_->cl_sdr_ch() * tid + ch);
                ul_txbuff.at(ch) = pkt->data;
                cur_offset = (cur_offset + 1) % tx_buffer_size;
            }
            long long txTime = txFrameTime
                + config_->cl_ul_slots().at(tid).at(s) * num_samps
                - config_->tx_advance();
            if (kUseUHD && s < (tx_slots - 1))
                flagsTxUlData = kStreamContinuous; // HAS_TIME
            else
                flagsTxUlData = kStreamEndBurst; // HAS_TIME & END_BURST, fixme
            int r = clientRadioSet_->radioTx(
                tid, ul_txbuff.data(), num_samps, flagsTxUlData, txTime);
            if (r < num_samps) {
                MLPD_WARN("BAD Write: %d/%d\n", r, num_samps);
            }
        }
        cl_tx_buffer_[tid].pkt_buf_inuse[event.frame_id % kSampleBufferFrameNum]
            = 0;
        return 0;
    }
    return -1;
}

int Receiver::syncSearch(
    std::vector<std::complex<int16_t>> sync_buff, size_t sync_num_samps)
{
    int sync_index(-1);
    // convert data to complex float for sync detection
    std::vector<std::complex<float>> sync_buff_float(sync_num_samps, 0);
    for (size_t i = 0; i < sync_num_samps; i++) {
        sync_buff_float[i] = (std::complex<float>(
            sync_buff[i].real() / 32768.0, sync_buff[i].imag() / 32768.0));
    }
    //TODO syncbuff0 is sloppy here since we recevied into syncrxbuff.data(), r bytes.
#if defined(__x86_64__)
    sync_index
        = CommsLib::find_beacon_avx(sync_buff_float, config_->gold_cf32());
#else
    sync_index = CommsLib::find_beacon(sync_buff_float);
#endif
    return sync_index;
}

void Receiver::clientSyncTxRx(int tid, int core_id, SampleBuffer* rx_buffer)
{
    if (config_->core_alloc() == true) {
        int core = tid + core_id;

        MLPD_INFO("Pinning client synctxrx thread %d to core %d\n", tid, core);
        if (pin_to_core(core) != 0) {
            MLPD_ERROR(
                "Pin client synctxrx thread %d to core %d failed\n", tid, core);
            throw std::runtime_error(
                "Failed to Pin client synctxrx thread to core");
        }
    }

    MLPD_INFO("Scheduling TX: %zu Frames (%lf ms) in the future!\n",
        this->txFrameDelta, 1e3 * config_->getFrameDurationSec());

    int SYNC_NUM_SAMPS = config_->samps_per_frame();
    std::vector<std::complex<int16_t>> syncbuff0(SYNC_NUM_SAMPS, 0);
    std::vector<std::complex<int16_t>> syncbuff1(SYNC_NUM_SAMPS, 0);
    std::vector<void*> syncrxbuff(2);
    syncrxbuff.at(0) = syncbuff0.data();
    if (config_->cl_sdr_ch() == 2) {
        syncrxbuff.at(1) = syncbuff1.data();
    }

    int NUM_SAMPS = config_->samps_per_slot();
    std::vector<std::complex<int16_t>> buff0(SYNC_NUM_SAMPS, 0);
    std::vector<std::complex<int16_t>> buff1(SYNC_NUM_SAMPS, 0);
    std::vector<void*> rxbuff(2);
    rxbuff.at(0) = buff0.data();
    if (config_->cl_sdr_ch() == 2) {
        rxbuff.at(1) = buff1.data();
    }

    size_t packetLength = sizeof(Packet) + config_->getPacketDataLength();
    size_t ant_id = tid * config_->cl_sdr_ch();
    int buffer_chunk_size = 0;
    // use token to speed up
    moodycamel::ProducerToken local_ptok(*message_queue_);
    char* buffer = nullptr;
    std::atomic_int* pkt_buf_inuse = nullptr;
    if (config_->dl_slot_per_frame() > 0) {

        buffer_chunk_size = rx_buffer[0].buffer.size() / packetLength;
        // handle two channels at each radio
        // this is assuming buffer_chunk_size is at least 2
        pkt_buf_inuse
            = rx_buffer[tid + config_->bs_rx_thread_num()].pkt_buf_inuse;
        buffer = rx_buffer[tid + config_->bs_rx_thread_num()].buffer.data();
    }

    // tx_buffer info
    size_t tx_buffer_size = 0;
    if (config_->ul_data_slot_present() == true) {
        tx_buffer_size = cl_tx_buffer_[tid].buffer.size() / packetLength;
    }

    long long rxTime(0);
    int sync_index(-1);
    int rx_offset = 0;

    // For USRP clients skip UHD_INIT_TIME_SEC to avoid late packets
    if (kUseUHD == true) {
        int cl_sync_ret = -1;
        sleep(UHD_INIT_TIME_SEC);
        while (cl_sync_ret < 0) {
            cl_sync_ret = clientRadioSet_->radioRx(
                tid, syncrxbuff.data(), SYNC_NUM_SAMPS, rxTime);
        }
    }

    // Keep reading one frame worth of data until a beacon is found
    // Perform initial beacon detection once every BEACON_INTERVAL frames
    while ((config_->running() == true) && (sync_index < 0)) {
        for (int find_beacon_retry = 0; find_beacon_retry < BEACON_INTERVAL;
             find_beacon_retry++) {
            int r = clientRadioSet_->radioRx(
                tid, syncrxbuff.data(), SYNC_NUM_SAMPS, rxTime);
            if (r != SYNC_NUM_SAMPS) {
                MLPD_WARN("BAD SYNC Receive( %d / %d ) at Time %lld\n", r,
                    SYNC_NUM_SAMPS, rxTime);
            }
        }
        sync_index = this->syncSearch(syncbuff0, SYNC_NUM_SAMPS);
    }
    if (sync_index >= 0) {
        MLPD_INFO("Beacon detected at Time %lld, sync_index: %d\n", rxTime,
            sync_index);
        rx_offset = sync_index - config_->beacon_size() - config_->prefix();
    }

    // Read rx_offset to align with the begining of a frame
    assert((rx_offset >= 0) && (rx_offset <= SYNC_NUM_SAMPS));
    if (config_->running() == true) {
        MLPD_INFO("Start main client txrx loop... tid=%d\n", tid);
        if (rx_offset > 0) {
            int rx_data;
            rx_data = clientRadioSet_->radioRx(
                tid, syncrxbuff.data(), rx_offset, rxTime);
            if (rx_data != rx_offset) {
                MLPD_WARN("Rx data: %d : %d failed to align sync read\n",
                    rx_data, rx_offset);
            }
        }
    }

    // Main client read/write loop.
    size_t frame_id = 0;
    size_t slot_id = 0;
    size_t buffer_offset = 0;
    bool resync = false;
    bool resync_enable = (config_->frame_mode() == "continuous_resync");
    size_t resync_retry_cnt(0);
    size_t resync_retry_max(100);
    size_t resync_success(0);
    int r = 0;
    rx_offset = 0;

    while (config_->running() == true) {
        if (config_->max_frame() > 0 && frame_id >= config_->max_frame()) {
            config_->running(false);
            break;
        }

        r = clientRadioSet_->radioRx(
            tid, syncrxbuff.data(), NUM_SAMPS + rx_offset, rxTime);
        if (r < 0) {
            config_->running(false);
            break;
        }
        if (config_->ul_data_slot_present() == true) {
            this->notifyPacket(kClient, frame_id + this->txFrameDelta, 0, tid,
                tx_buffer_size); // Notify new frame
        }

        // resync every X=1000 frames:
        // TODO: X should be a function of sample rate and max CFO
        if (((frame_id / 1000) > 0) && ((frame_id % 1000) == 0)) {
            resync = resync_enable;
            MLPD_TRACE("Enable resyncing at frame %zu\n", frame_id);
        }
        rx_offset = 0;
        if (resync == true) {
            //Need to bound the beacon detection to the last 'r not the size of the memory (vector)
            //TODO: Use SIMD for faster conversion
            // convert data to complex float for sync detection
            sync_index = this->syncSearch(syncbuff0, NUM_SAMPS);
            if (sync_index >= 0) {
                rx_offset
                    = sync_index - config_->beacon_size() - config_->prefix();
                rxTime += rx_offset;
                resync = false;
                resync_retry_cnt = 0;
                resync_success++;
                MLPD_INFO("Re-syncing with offset: %d, after %zu "
                          "tries, index: %d, tid %d\n",
                    rx_offset, resync_retry_cnt + 1, sync_index, tid);
            } else {
                resync_retry_cnt++;
            }
        }
        if ((resync == true) && (resync_retry_cnt > resync_retry_max)) {

            MLPD_WARN("Exceeded resync retry limit (%zu) for client "
                      "%d reached after %zu resync successes at "
                      "frame: %zu.  Stopping!\n",
                resync_retry_max, tid, resync_success, frame_id);
            resync = false;
            resync_retry_cnt = 0;
            config_->running(false);
            break;
        }
        // schedule all TX slot
        // config_->tx_advance() needs calibration based on SDR model and sampling rate
        if (config_->ul_data_slot_present() == true) {
            while (-1 != this->clientTxData(tid, frame_id, rxTime))
                ;
        } else {
            this->txPilots(tid, rxTime + txTimeDelta);
        } // end if config_->ul_data_slot_present()

        slot_id++;

        for (; slot_id < config_->slot_per_frame(); slot_id++) {
            if (config_->isDlData(frame_id, slot_id)) {
                // Set buffer status(es) to full; fail if full already
                for (size_t ch = 0; ch < config_->cl_sdr_ch(); ++ch) {
                    int old = std::atomic_fetch_or(
                        &pkt_buf_inuse[buffer_offset], 1); // now full
                    // if buffer was full, exit
                    if (old != 0) {
                        MLPD_ERROR("thread %d buffer full\n", tid);
                        throw std::runtime_error("Thread %d buffer full\n");
                    }
                    // Reserved until marked empty by consumer
                }

                // Receive data into buffers
                Packet* pkt[config_->cl_sdr_ch()];
                void* samp[config_->cl_sdr_ch()];
                for (size_t ch = 0; ch < config_->cl_sdr_ch(); ++ch) {
                    pkt[ch] = (Packet*)(buffer
                        + (buffer_offset + ch) * packetLength);
                    samp[ch] = pkt[ch]->data;
                }

                if ((r = this->clientRadioSet_->radioRx(
                         tid, samp, NUM_SAMPS, rxTime))
                    < 0) {
                    config_->running(false);
                    break;
                }
                for (size_t ch = 0; ch < config_->cl_sdr_ch(); ++ch) {
                    new (pkt[ch]) Packet(frame_id, slot_id, 0, ant_id + ch);
                    // push kEventRxSymbol event into the queue
                    this->notifyPacket(kClient, frame_id, slot_id, ant_id + ch,
                        buffer_chunk_size,
                        buffer_offset + tid * buffer_chunk_size);
                    buffer_offset++;
                    buffer_offset %= buffer_chunk_size;
                }
            } else {
                r = this->clientRadioSet_->radioRx(
                    tid, rxbuff.data(), NUM_SAMPS, rxTime);
                if (r < 0) {
                    config_->running(false);
                    break;
                }
            }
            if (r != NUM_SAMPS) {
                MLPD_WARN("BAD Receive(%d/%d) at Time %lld, frame count %zu\n",
                    r, NUM_SAMPS, rxTime, frame_id);
            }
        } // end for
        frame_id++;
        slot_id = 0;
    } // end while
}
