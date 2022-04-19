/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
            Doug Moore: dougm@rice.edu
	    Oscar Bejarano: ob4@rice.edu
 
---------------------------------------------------------------------
 Initializes and Configures Radios in the massive-MIMO base station 
---------------------------------------------------------------------
*/

#include "include/BaseRadioSetUHD.h"
#include "include/RadioUHD.h"
#include "include/comms-lib.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"

#include "nlohmann/json.hpp"
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>

#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <complex>

using json = nlohmann::json;

BaseRadioSetUHD::BaseRadioSetUHD(Config* cfg)
    : _cfg(cfg)
{
    std::vector<size_t> num_bs_antenntas(_cfg->num_cells());
    radioNotFound = false;
    std::vector<std::string> radio_serial_not_found;

    // need to be further modified
    for (size_t c = 0; c < _cfg->num_cells(); c++) {
        size_t num_radios = _cfg->n_bs_sdrs()[c];
        num_bs_antenntas[c] = num_radios * _cfg->bs_channel().length();
        MLPD_TRACE("Setting up radio: %zu, cells: %zu\n", num_radios,
            _cfg->num_cells());

        std::atomic_ulong thread_count = ATOMIC_VAR_INIT(num_radios);

        MLPD_TRACE("Init base radios: %zu\n", num_radios);
        for (size_t i = 0; i < num_radios; i++) {
            BaseRadioContext* context = new BaseRadioContext;
            context->brs = this;
            context->thread_count = &thread_count;
            context->tid = i;
            context->cell = c;
#ifdef THREADED_INIT
            pthread_t init_thread_;

            if (pthread_create(
                    &init_thread_, NULL, BaseRadioSetUHD::init_launch, context)
                != 0) {
                delete context;
                throw std::runtime_error(
                    "BaseRadioSet - init thread create failed");
            }
#else
            init(context);
#endif
        }

        // Wait for init
        while (thread_count.load() > 0) {
        }
        // commented out for now, will add this when future doing Multi USRP
        // Strip out broken radios.
        // elimated this part
//        for (size_t i = 0; i < num_radios; i++) {
//            if (bsRadios == NULL) {
//                radioNotFound = true;
//                radio_serial_not_found.push_back(
//                    _cfg->bs_sdr_ids().at(c).at(i));
//                while (num_radios != 0
//                    && bsRadios.at(c).at(num_radios - 1) == NULL) {
//                    --num_radios;
//                    bsRadios.at(c).pop_back();
//                }
//                if (i < num_radios) {
//                    bsRadios.at(c).at(i) = bsRadios.at(c).at(--num_radios);
//                    bsRadios.at(c).pop_back();
//                }
//            }
//        }
//        bsRadios.at(c).shrink_to_fit();
        _cfg->n_bs_sdrs().at(c) = num_radios;
        if (radioNotFound == true) {
            break;
        }

        // Perform DC Offset & IQ Imbalance Calibration
        if (_cfg->imbalance_cal_en() == true) {
            if (_cfg->bs_channel().find('A') != std::string::npos)
                dciqCalibrationProcUHD(0);
            if (_cfg->bs_channel().find('B') != std::string::npos)
                dciqCalibrationProcUHD(1);
        }

        thread_count.store(num_radios);
        for (size_t i = 0; i < num_radios; i++) {
            BaseRadioContext* context = new BaseRadioContext;
            context->brs = this;
            context->thread_count = &thread_count;
            context->tid = i;
            context->cell = c;
#ifdef THREADED_INIT
            pthread_t configure_thread_;
            if (pthread_create(&configure_thread_, NULL,
                    BaseRadioSetUHD::configure_launch, context)
                != 0) {
                delete context;
                throw std::runtime_error(
                    "BaseRadioSet - configure thread create failed");
            }
#else
            configure(context);
#endif
        }

        while (thread_count.load() > 0) {
        }

        auto channels = Utils::strToChannels(_cfg->bs_channel());

        for (size_t i = 0; i <bsRadios->dev->get_num_mboards(); i++) {
            auto dev = bsRadios->dev;
            std::cout << _cfg->bs_sdr_ids().at(c).at(i) << ": Front end "<< dev->get_usrp_rx_info()["frontend"] << std::endl;
            for (auto ch : channels) {
                if (ch < dev->get_rx_num_channels()) {
                    printf("RX Channel %zu\n", ch);
                    printf("Actual RX sample rate: %fMSps...\n",
                        (dev->get_rx_rate(ch) / 1e6));
                    printf("Actual RX frequency: %fGHz...\n",
                        (dev->get_rx_freq(ch) / 1e9));
                    printf("Actual RX gain: %f...\n",
                        (dev->get_rx_gain(ch)));
                    printf("Actual RX bandwidth: %fM...\n",
                        (dev->get_rx_bandwidth(ch) / 1e6));
                    printf("Actual RX antenna: %s...\n",
                        (dev->get_rx_antenna(ch).c_str()));
                }
            }

            for (auto ch : channels) {
                if (ch < dev->get_tx_num_channels()) {
                    printf("TX Channel %zu\n", ch);
                    printf("Actual TX sample rate: %fMSps...\n",
                        (dev->get_tx_rate(ch) / 1e6));
                    printf("Actual TX frequency: %fGHz...\n",
                        (dev->get_tx_freq(ch) / 1e9));
                    printf("Actual TX gain: %f...\n",
                        (dev->get_tx_gain(ch)));
                    printf("Actual TX bandwidth: %fM...\n",
                        (dev->get_tx_bandwidth(ch) / 1e6));
                    printf("Actual TX antenna: %s...\n",
                        (dev->get_tx_antenna(ch).c_str()));
                }
            }
            std::cout << std::endl;
        }
    }

    if (radioNotFound == true) {
        for (auto st = radio_serial_not_found.begin();
             st != radio_serial_not_found.end(); st++)
            std::cout << "\033[1;31m" << *st << "\033[0m" << std::endl;
        std::cout << "\033[1;31mERROR: the above base station serials were not "
                     "discovered in the network!\033[0m"
                  << std::endl;
    } else {
        if (_cfg->sample_cal_en() == true) {
//            bool adjust = false;
            int cal_cnt = 0;
            int offset_diff = 1;

            while (offset_diff > 0) {
                if (++cal_cnt > 10) {
                    std::cout << "10 attemps of sample offset calibration, "
                                 "stopping..."
                              << std::endl;
                    break;
                }
                offset_diff =
                        syncTimeOffsetUHD(false, true);  // run 1: find offsets and adjust
            }
            usleep(100000);
            offset_diff = syncTimeOffsetUHD(false, false);  // run 2: verify
            if (offset_diff > 1)
                std::cout << "Failed ";
            else
                std::cout << "Successful ";
            std::cout << "sample offset calibration!" << std::endl;
        }

        nlohmann::json tddConf;
        tddConf["tdd_enabled"] = true;
        tddConf["frame_mode"] = "free_running";
        tddConf["max_frame"] = _cfg->max_frame();
        tddConf["symbol_size"] = _cfg->samps_per_slot();

        // write TDD schedule and beacons to FPFA buffers only for Iris
        for (size_t c = 0; c < _cfg->num_cells(); c++) {
            bsRadios->dev->set_time_source("external", 0);
            bsRadios->dev->set_clock_source("external", 0);
            uhd::time_spec_t time = uhd::time_spec_t::from_ticks(0, 1e9);
            bsRadios->dev->set_time_next_pps(time);

            // Wait for pps sync pulse
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Activate Rx and Tx streamers
            bsRadios->activateRecv();
            bsRadios->activateXmit();
        }
        MLPD_INFO("%s done!\n", __func__);
    }
}

BaseRadioSetUHD::~BaseRadioSetUHD(void)
{
    delete bsRadios;
}

void* BaseRadioSetUHD::init_launch(void* in_context)
{
    BaseRadioContext* context = reinterpret_cast<BaseRadioContext*>(in_context);
    context->brs->init(context);
    return 0;
}

void BaseRadioSetUHD::init(BaseRadioContext* context)
{
    int i = context->tid;
    int c = context->cell;
    std::atomic_ulong* thread_count = context->thread_count;
    delete context;

    MLPD_TRACE("Deleting context for tid: %d\n", i);

    auto channels = Utils::strToChannels(_cfg->bs_channel());
    std::map< std::string, std::string > args;

    args["driver"] = "uhd";
    args["addr"] = _cfg->bs_sdr_ids().at(c).at(i);
    std::cout << "Init bsRadios: " << args["addr"] << std::endl;
//    }
    args["timeout"] = "1000000";
    try {
        bsRadios = nullptr;
        bsRadios = new RadioUHD(args, SOAPY_SDR_CS16, channels);
    } catch (std::runtime_error& err) {
        std::cerr << "Ignoring uhd device "
                  << _cfg->bs_sdr_ids().at(c).at(i) << std::endl;
//        }
        if (bsRadios != nullptr) {
            MLPD_TRACE("Deleting radio ptr due to exception\n");
            delete bsRadios;
            bsRadios = nullptr;
        }
    }
    MLPD_TRACE("BaseRadioSet: Init complete\n");
    assert(thread_count->load() != 0);
    thread_count->store(thread_count->load() - 1);
}

void* BaseRadioSetUHD::configure_launch(void* in_context)
{
    BaseRadioContext* context = reinterpret_cast<BaseRadioContext*>(in_context);
    context->brs->configure(context);
    return 0;
}

void BaseRadioSetUHD::configure(BaseRadioContext* context)
{
    int i = context->tid;
    int c = context->cell;
    std::atomic_ulong* thread_count = context->thread_count;
    delete context;

    //load channels
    auto channels = Utils::strToChannels(_cfg->bs_channel());
    RadioUHD* bsRadio = bsRadios;
    uhd::usrp::multi_usrp::sptr dev = bsRadio->dev;
    for (auto ch : channels) {
        double rxgain = _cfg->rx_gain().at(ch);
        double txgain = _cfg->tx_gain().at(ch);
        bsRadios->dev_init(_cfg, ch, rxgain, txgain);
    }
    assert(thread_count->load() != 0);
    thread_count->store(thread_count->load() - 1);
}

uhd::usrp::multi_usrp::sptr BaseRadioSetUHD::baseRadio(size_t cellId)
{
//    if (cellId < hubs.size())
//        return (hubs.at(cellId));
//    if (cellId < bsRadios.size() && bsRadios.at(cellId).size() > 0)
//        return bsRadios.at(cellId).at(0)->dev;
//    return NULL;

    // update for UHD multi USRP
    return NULL;
//    return bsRadios->dev;
}

void BaseRadioSetUHD::sync_delays(size_t cellIdx){
    /*
     * Compute Sync Delays
     */
    uhd::usrp::multi_usrp::sptr base = baseRadio(cellIdx);
}

void BaseRadioSetUHD::radioTriggerUHD(void)
{
//    for (size_t c = 0; c < _cfg->num_cells(); c++) {
//        SoapySDR::Device* base = baseRadio(c);
//        if (base != NULL)
//            base->writeSetting("TRIGGER_GEN", "");
//    }
}

void BaseRadioSetUHD::radioStart()
{
//    if (!kUseUHD)
//        radioTrigger();
}

void BaseRadioSetUHD::readSensors()
{
    uhd::usrp::multi_usrp::sptr dev = bsRadios->dev;
    for (size_t i = 0; i < dev->get_num_mboards(); i++){
        std::cout <<  dev->get_mboard_sensor_names(i).at(0) << std::endl;
    }
}

void BaseRadioSetUHD::radioStop(void)
{
    std::string tddConfStr = "{\"tdd_enabled\":false}";
//    for (size_t c = 0; c < _cfg->num_cells(); c++) {
//        for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
//            if (!kUseUHD) {
//                SoapySDR::Device* dev = bsRadios.at(c).at(i)->dev;
//                dev->writeSetting("TDD_CONFIG", tddConfStr);
//                dev->writeSetting("TDD_MODE", "false");
//            }
    bsRadios->reset_DATA_clk_domain();
//        }
//    }
}

void BaseRadioSetUHD::radioTx(const void* const* buffs)
{
    long long frameTime(0);
//    for (size_t c = 0; c < _cfg->num_cells(); c++) {
//        for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
    std::cout<<"running tx 1" << std::endl;
    bsRadios->xmit(buffs, _cfg->samps_per_slot(), 0, frameTime);
//        }
//    }
}

int BaseRadioSetUHD::radioTx(size_t radio_id, size_t cell_id,
    const void* const* buffs, int flags, long long& frameTime)
{
    int w;
    // for UHD device xmit from host using frameTimeNs
//    if (!kUseUHD) {
//        w = bsRadios.at(cell_id).at(radio_id)->xmit(
//            buffs, _cfg->samps_per_symbol(), flags, frameTime);
//    } else {
    long long frameTimeNs = SoapySDR::ticksToTimeNs(frameTime, _cfg->rate());
    w = bsRadios->xmit(buffs, _cfg->samps_per_slot(), flags, frameTimeNs);
//    }
#if DEBUG_RADIO
    size_t chanMask;
    long timeoutUs(0);
    auto* dev = bsRadios->dev;
    auto* txs = bsRadios->txs;
//    int s = dev->readStreamStatus(txs, chanMask, flags, frameTime, timeoutUs);
//    std::cout << "cell " << cell_id << " radio " << radio_id << " tx returned "
//              << w << " and status " << s << std::endl;
#endif
//    std::cout<<"running tx 2" << std::endl;

    return w;
}

void BaseRadioSetUHD::radioRx(void* const* buffs)
{
    long long frameTime(0);
    void *const *buff;

//    for (size_t c = 0; c < _cfg->num_cells(); c++) {
    for (size_t i = 0; i < bsRadios->dev->get_num_mboards(); i++) {
        buff = buffs + (i * 2);
    }
    bsRadios->recv(buff, _cfg->samps_per_slot(), frameTime);
//        }
//    }
}

int BaseRadioSetUHD::radioRx(
    size_t radio_id, size_t cell_id, void* const* buffs, long long& frameTime)
{
    return this->radioRx(
        radio_id, cell_id, buffs, _cfg->samps_per_slot(), frameTime);
}

int BaseRadioSetUHD::radioRx(size_t radio_id, size_t cell_id, void* const* buffs,
    int numSamps, long long& frameTime)
{
    int ret = 0;

//    std::cout<< "radio id is "<<radio_id<<std::endl;
    if (radio_id < bsRadios->dev->get_num_mboards()) {
        long long frameTimeNs = 0;
        ret = bsRadios->recv(buffs, numSamps, frameTimeNs);
//        std::cout<<"bsRadios->dev->get_num_mboards() " << bsRadios->dev->get_num_mboards() << std::endl;

// for UHD device recv using ticks
        frameTime = SoapySDR::timeNsToTicks(frameTimeNs, _cfg->rate());
#if DEBUG_RADIO
        if (ret != numSamps)
            std::cout << "recv returned " << ret << " from radio " << radio_id
                      << ", in cell " << cell_id << ". Expected: " << numSamps
                      << std::endl;
        else
            std::cout << "radio " << radio_id << " in cell " << cell_id
                      << ". Received " << ret << " at " << frameTime
                      << std::endl;
#endif
    } else {
        MLPD_WARN("Invalid radio id: %zu in cell %zu\n", radio_id, cell_id);
        ret = 0;
    }
    return ret;
}