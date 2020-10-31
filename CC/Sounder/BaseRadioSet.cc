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

#include "include/BaseRadioSet.h"
#include "include/Radio.h"
#include "include/comms-lib.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"

#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>

BaseRadioSet::BaseRadioSet(Config* cfg)
    : _cfg(cfg)
{
    std::vector<int> nBsAntennas(_cfg->num_cells());
    bsRadios.resize(_cfg->num_cells());
    radioNotFound = false;
    std::vector<std::string> radioSerialNotFound;

    for (size_t c = 0; c < _cfg->num_cells(); c++) {
        int radioNum = _cfg->n_bs_sdrs()[c];
        nBsAntennas[c] = radioNum * _cfg->bs_channel().length();
        MLPD_TRACE("Setting up radio: %d, cells: %d\n", radioNum,
            (int)_cfg->num_cells());
        if ((kUseUHD == false) && (_cfg->hub_ids().empty() == false)) {
            SoapySDR::Kwargs args;
            args["driver"] = "remote";
            args["timeout"] = "1000000";
            args["serial"] = _cfg->hub_ids().at(c);
            try {
                hubs.push_back(SoapySDR::Device::make(args));
            } catch (const std::exception& e) {
                MLPD_ERROR(
                    "Caught exception in call to SDR make: %s\n", e.what());
            } catch (...) {
                MLPD_WARN("Unknown exception");
            }
        }
        bsRadios[c].resize(radioNum);
        std::atomic_int threadCount = ATOMIC_VAR_INIT(radioNum);

        MLPD_TRACE("Init base radios: %d\n", radioNum);
        for (int i = 0; i < radioNum; i++) {
            BaseRadioContext* context = new BaseRadioContext;
            context->brs = this;
            context->threadCount = &threadCount;
            context->tid = i;
            context->cell = c;
#ifdef THREADED_INIT
            pthread_t init_thread_;

            if (pthread_create(
                    &init_thread_, NULL, BaseRadioSet::init_launch, context)
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
        while (threadCount > 0)
            ;

        // Strip out broken radios.
        for (int i = 0; i < radioNum; i++) {
            if (bsRadios[c][i] == NULL) {
                radioNotFound = true;
                radioSerialNotFound.push_back(_cfg->bs_sdr_ids().at(c).at(i));
                while (radioNum != 0 && bsRadios[c][radioNum - 1] == NULL) {
                    --radioNum;
                    bsRadios[c].pop_back();
                }
                if (i < radioNum) {
                    bsRadios[c][i] = bsRadios[c][--radioNum];
                    bsRadios[c].pop_back();
                }
            }
        }
        bsRadios[c].shrink_to_fit();
        _cfg->n_bs_sdrs().at(c) = radioNum;
        if (radioNotFound == true) {
            break;
        }

        // Perform DC Offset & IQ Imbalance Calibration
        if (_cfg->imbalance_cal_en() == true) {
            if (_cfg->bs_channel().find('A') != std::string::npos)
                dciqCalibrationProc(0);
            if (_cfg->bs_channel().find('B') != std::string::npos)
                dciqCalibrationProc(1);
        }

        threadCount = radioNum;
        for (int i = 0; i < radioNum; i++) {
            BaseRadioContext* context = new BaseRadioContext;
            context->brs = this;
            context->threadCount = &threadCount;
            context->tid = i;
            context->cell = c;
#ifdef THREADED_INIT
            pthread_t configure_thread_;
            if (pthread_create(&configure_thread_, NULL,
                    BaseRadioSet::configure_launch, context)
                != 0) {
                delete context;
                throw std::runtime_error(
                    "BaseRadioSet - configure thread create failed");
            }
#else
            configure(context);
#endif
        }

        while (threadCount > 0)
            ;
        // Measure Sync Delays now!
        if (kUseUHD == false)
            sync_delays(c);
    }

    if (radioNotFound == true) {
        for (auto st = radioSerialNotFound.begin();
             st != radioSerialNotFound.end(); st++)
            std::cout << "\033[1;31m" << *st << "\033[0m" << std::endl;
        std::cout << "\033[1;31mERROR: the above base station serials were not "
                     "discovered in the network!\033[0m"
                  << std::endl;
    } else {
        if (_cfg->sample_cal_en() == true) {
            bool adjust = false;
            int cal_cnt = 0;
            while (!adjust) {
                if (++cal_cnt > 10) {
                    std::cout << "10 attemps of sample offset calibration, "
                                 "stopping..."
                              << std::endl;
                    break;
                }
                adjust = true;
                collectCSI(adjust); // run 1: find offsets and adjust
            }
            collectCSI(adjust); // run 2: verify adjustments
            usleep(100000);
            std::cout << "sample offset calibration done!" << std::endl;
        }

        json tddConf;
        tddConf["tdd_enabled"] = true;
        tddConf["frame_mode"] = "free_running";
        tddConf["max_frame"] = _cfg->max_frame();
        tddConf["symbol_size"] = _cfg->samps_per_symbol();

        // write TDD schedule and beacons to FPFA buffers only for Iris
        for (size_t c = 0; c < _cfg->num_cells(); c++) {
            if (!kUseUHD) {
                if (_cfg->reciprocal_calib()) {
                    for (size_t i = 0; i < bsRadios[c].size(); i++) {
                        tddConf["frames"] = json::array();
                        tddConf["frames"].push_back(_cfg->calib_frames()[c][i]);
                        std::cout << "Cell " << c << ", SDR " << i
                                  << " calibration schedule : "
                                  << _cfg->calib_frames()[c][i] << std::endl;
                        std::string tddConfStr = tddConf.dump();
                        SoapySDR::Device* dev = bsRadios[c][i]->dev;
                        dev->writeSetting("TDD_CONFIG", tddConfStr);
                    }
                } else {
                    tddConf["frames"] = json::array();
                    tddConf["frames"].push_back(_cfg->frames()[c]);
                    std::cout << "Cell " << c
                              << " FPGA schedule: " << _cfg->frames()[c]
                              << std::endl;
                    tddConf["beacon_start"] = _cfg->prefix();
                    tddConf["beacon_stop"]
                        = _cfg->prefix() + _cfg->beacon_size();
                    std::string tddConfStr = tddConf.dump();
                    for (size_t i = 0; i < bsRadios[c].size(); i++) {
                        SoapySDR::Device* dev = bsRadios[c][i]->dev;
                        dev->writeSetting("TDD_CONFIG", tddConfStr);
                    }

                    // write beacons to FPGA buffers
                    size_t ndx = 0;
                    for (size_t i = 0; i < bsRadios[c].size(); i++) {
                        SoapySDR::Device* dev = bsRadios[c][i]->dev;
                        dev->writeRegisters("BEACON_RAM", 0, _cfg->beacon());
                        for (char const& ch : _cfg->bs_channel()) {
                            bool isBeaconAntenna = !_cfg->beam_sweep()
                                && ndx == _cfg->beacon_ant();
                            std::vector<unsigned> beacon_weights(
                                nBsAntennas[c], isBeaconAntenna ? 1 : 0);
                            std::string tx_ram_wgt = "BEACON_RAM_WGT_";
                            if (_cfg->beam_sweep()) {
                                for (int j = 0; j < nBsAntennas[c]; j++)
                                    beacon_weights[j]
                                        = CommsLib::hadamard2(ndx, j);
                            }
                            dev->writeRegisters(
                                tx_ram_wgt + ch, 0, beacon_weights);
                            ++ndx;
                        }
                        dev->writeSetting(
                            "BEACON_START", std::to_string(bsRadios[c].size()));
                    }
                }
                for (size_t i = 0; i < bsRadios[c].size(); i++) {
                    SoapySDR::Device* dev = bsRadios[c][i]->dev;
                    dev->writeSetting("TX_SW_DELAY",
                        "30"); // experimentally good value for dev front-end
                    dev->writeSetting("TDD_MODE", "true");
                }
            }

            if (!kUseUHD) {
                for (size_t i = 0; i < bsRadios[c].size(); i++) {
                    SoapySDR::Device* dev = bsRadios[c][i]->dev;
                    bsRadios[c][i]->activateRecv();
                    bsRadios[c][i]->activateXmit();
                    dev->setHardwareTime(0, "TRIGGER");
                }
            } else {
                // Set freq and time source for multiple USRPs
                for (size_t i = 0; i < bsRadios[c].size(); i++) {
                    SoapySDR::Device* dev = bsRadios[c][i]->dev;
                    dev->setClockSource("external");
                    dev->setTimeSource("external");
                    dev->setHardwareTime(0, "PPS");
                }
                // Wait for pps sync pulse
                std::this_thread::sleep_for(std::chrono::seconds(2));
                // Activate Rx and Tx streamers
                for (size_t i = 0; i < bsRadios[c].size(); i++) {
                    bsRadios[c][i]->activateRecv();
                    bsRadios[c][i]->activateXmit();
                }
            }
        }
        MLPD_INFO("%s done!\n", __func__);
    }
}

BaseRadioSet::~BaseRadioSet(void)
{
    if (!_cfg->hub_ids().empty()) {
        for (unsigned int i = 0; i < hubs.size(); i++)
            SoapySDR::Device::unmake(hubs[i]);
    }
    for (unsigned int c = 0; c < _cfg->num_cells(); c++)
        for (size_t i = 0; i < _cfg->n_bs_sdrs().at(c); i++)
            delete bsRadios[c][i];
}

void* BaseRadioSet::init_launch(void* in_context)
{
    BaseRadioContext* context = reinterpret_cast<BaseRadioContext*>(in_context);
    context->brs->init(context);
    return 0;
}

void BaseRadioSet::init(BaseRadioContext* context)
{
    int i = context->tid;
    int c = context->cell;
    std::atomic_int* threadCount = context->threadCount;
    delete context;

    MLPD_TRACE("Deleting context for tid: %d\n", i);

    auto channels = Utils::strToChannels(_cfg->bs_channel());
    SoapySDR::Kwargs args;
    if (kUseUHD == false) {
        args["driver"] = "iris";
        args["serial"] = _cfg->bs_sdr_ids().at(c).at(i);
    } else {
        args["driver"] = "uhd";
        args["addr"] = _cfg->bs_sdr_ids().at(c).at(i);
        std::cout << "Init bsRadios: " << args["addr"] << std::endl;
    }
    args["timeout"] = "1000000";
    try {
        bsRadios[c][i] = nullptr;
        bsRadios[c][i]
            = new Radio(args, SOAPY_SDR_CS16, channels, _cfg->rate());
    } catch (std::runtime_error& err) {
        if (kUseUHD == false) {
            std::cerr << "Ignoring iris " << _cfg->bs_sdr_ids().at(c).at(i)
                      << std::endl;
        } else {
            std::cerr << "Ignoring uhd device "
                      << _cfg->bs_sdr_ids().at(c).at(i) << std::endl;
        }
        if (bsRadios[c][i] != nullptr) {
            MLPD_TRACE("Deleting radio ptr due to exception\n");
            delete bsRadios[c][i];
            bsRadios[c][i] = nullptr;
        }
    }
    MLPD_TRACE("BaseRadioSet: Init complete\n");
    (*threadCount)--;
}

void* BaseRadioSet::configure_launch(void* in_context)
{
    BaseRadioContext* context = reinterpret_cast<BaseRadioContext*>(in_context);
    context->brs->configure(context);
    return 0;
}

void BaseRadioSet::configure(BaseRadioContext* context)
{
    int i = context->tid;
    int c = context->cell;
    std::atomic_int* threadCount = context->threadCount;
    delete context;

    //load channels
    auto channels = Utils::strToChannels(_cfg->bs_channel());
    Radio* bsRadio = bsRadios[c][i];
    SoapySDR::Device* dev = bsRadio->dev;
    SoapySDR::Kwargs info = dev->getHardwareInfo();
    for (auto ch : channels) {
        double rxgain = _cfg->rx_gain().at(ch);
        double txgain = _cfg->tx_gain().at(ch);
        bsRadios[c][i]->dev_init(_cfg, ch, rxgain, txgain);
    }

    (*threadCount)--;
}

SoapySDR::Device* BaseRadioSet::baseRadio(size_t cellId)
{
    if (cellId < hubs.size())
        return (hubs[cellId]);
    if (cellId < bsRadios.size() && bsRadios[cellId].size() > 0)
        return bsRadios[cellId][0]->dev;
    return NULL;
}

void BaseRadioSet::sync_delays(int cellIdx)
{
    /*
     * Compute Sync Delays
     */
    SoapySDR::Device* base = baseRadio(cellIdx);
    if (base != NULL)
        base->writeSetting("SYNC_DELAYS", "");
}

void BaseRadioSet::radioTrigger(void)
{
    for (size_t c = 0; c < _cfg->num_cells(); c++) {
        SoapySDR::Device* base = baseRadio(c);
        if (base != NULL)
            base->writeSetting("TRIGGER_GEN", "");
    }
}

void BaseRadioSet::radioStart()
{
    if (!kUseUHD)
        radioTrigger();
}

void BaseRadioSet::readSensors()
{
    for (size_t c = 0; c < _cfg->num_cells(); c++) {
        for (size_t i = 0; i < bsRadios[c].size(); i++) {
            SoapySDR::Device* dev = bsRadios[c][i]->dev;
            std::cout << "TEMPs on Iris " << i << std::endl;
            std::cout << "ZYNQ_TEMP: " << dev->readSensor("ZYNQ_TEMP")
                      << std::endl;
            std::cout << "LMS7_TEMP  : " << dev->readSensor("LMS7_TEMP")
                      << std::endl;
            std::cout << "FE_TEMP  : " << dev->readSensor("FE_TEMP")
                      << std::endl;
            std::cout << "TX0 TEMP  : "
                      << dev->readSensor(SOAPY_SDR_TX, 0, "TEMP") << std::endl;
            std::cout << "TX1 TEMP  : "
                      << dev->readSensor(SOAPY_SDR_TX, 1, "TEMP") << std::endl;
            std::cout << "RX0 TEMP  : "
                      << dev->readSensor(SOAPY_SDR_RX, 0, "TEMP") << std::endl;
            std::cout << "RX1 TEMP  : "
                      << dev->readSensor(SOAPY_SDR_RX, 1, "TEMP") << std::endl;
            std::cout << std::endl;
        }
    }
}

void BaseRadioSet::radioStop(void)
{
    std::string tddConfStr = "{\"tdd_enabled\":false}";
    for (size_t c = 0; c < _cfg->num_cells(); c++) {
        for (size_t i = 0; i < bsRadios[c].size(); i++) {
            if (!kUseUHD) {
                SoapySDR::Device* dev = bsRadios[c][i]->dev;
                dev->writeSetting("TDD_CONFIG", tddConfStr);
                dev->writeSetting("TDD_MODE", "false");
            }
            bsRadios[c][i]->reset_DATA_clk_domain();
        }
    }
}

void BaseRadioSet::radioTx(const void* const* buffs)
{
    long long frameTime(0);
    for (size_t c = 0; c < _cfg->num_cells(); c++) {
        for (size_t i = 0; i < bsRadios[c].size(); i++) {
            bsRadios[c][i]->xmit(buffs, _cfg->samps_per_symbol(), 0, frameTime);
        }
    }
}

int BaseRadioSet::radioTx(size_t radio_id, size_t cell_id,
    const void* const* buffs, int flags, long long& frameTime)
{
    int w;
    // for UHD device xmit from host using frameTimeNs
    if (!kUseUHD) {
        w = bsRadios[cell_id][radio_id]->xmit(
            buffs, _cfg->samps_per_symbol(), flags, frameTime);
    } else {
        long long frameTimeNs
            = SoapySDR::ticksToTimeNs(frameTime, _cfg->rate());
        w = bsRadios[cell_id][radio_id]->xmit(
            buffs, _cfg->samps_per_symbol(), flags, frameTimeNs);
    }
#if DEBUG_RADIO
    size_t chanMask;
    long timeoutUs(0);
    auto* dev = bsRadios[cell_id][radio_id]->dev;
    auto* txs = bsRadios[cell_id][radio_id]->txs;
    int s = dev->readStreamStatus(txs, chanMask, flags, frameTime, timeoutUs);
    std::cout << "cell " << cell_id << " radio " << radio_id << " tx returned "
              << w << " and status " << s << std::endl;
#endif
    return w;
}

void BaseRadioSet::radioRx(void* const* buffs)
{
    long long frameTime(0);
    for (size_t c = 0; c < _cfg->num_cells(); c++) {
        for (size_t i = 0; i < bsRadios[c].size(); i++) {
            void* const* buff = buffs + (i * 2);
            bsRadios[c][i]->recv(buff, _cfg->samps_per_symbol(), frameTime);
        }
    }
}

int BaseRadioSet::radioRx(
    size_t radio_id, size_t cell_id, void* const* buffs, long long& frameTime)
{
    return this->radioRx(
        radio_id, cell_id, buffs, _cfg->samps_per_symbol(), frameTime);
}

int BaseRadioSet::radioRx(size_t radio_id, size_t cell_id, void* const* buffs,
    int numSamps, long long& frameTime)
{
    int ret = 0;

    if (radio_id < bsRadios[cell_id].size()) {
        long long frameTimeNs = 0;
        ret = bsRadios[cell_id][radio_id]->recv(buffs, numSamps, frameTimeNs);
        // for UHD device recv using ticks
        if (kUseUHD == false)
            frameTime = frameTimeNs;
        else
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
