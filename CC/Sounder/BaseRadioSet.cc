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
#include "include/macros.h"
#include "include/utils.h"
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>

BaseRadioSet::BaseRadioSet(Config* cfg)
    : _cfg(cfg)
{
    std::vector<int> nBsAntennas(_cfg->nCells);
    bsRadios.resize(_cfg->nCells);
    radioNotFound = false;
    std::vector<std::string> radioSerialNotFound;

    for (size_t c = 0; c < _cfg->nCells; c++) {
        int radioNum = _cfg->nBsSdrs[c];
        nBsAntennas[c] = radioNum * _cfg->bsChannel.length();

        if (!kUseUHD && !_cfg->hub_ids.empty()) {
            SoapySDR::Kwargs args;
            args["driver"] = "remote";
            args["timeout"] = "1000000";
            args["serial"] = _cfg->hub_ids.at(c);
            hubs.push_back(SoapySDR::Device::make(args));
        }

        bsRadios[c].resize(radioNum);
        std::atomic_int threadCount = ATOMIC_VAR_INIT(radioNum);
        for (int i = 0; i < radioNum; i++) {
            BaseRadioContext* context = new BaseRadioContext;
            context->brs = this;
            context->threadCount = &threadCount;
            context->tid = i;
            context->cell = c;
#ifdef THREADED_INIT
            pthread_t init_thread_;
            if (pthread_create(&init_thread_, NULL, BaseRadioSet::init_launch, context) != 0) {
                perror("init thread create failed");
                exit(0);
            }
#else
            init(context);
#endif
        }

        while (threadCount > 0)
            ;

        // Strip out broken radios.
        for (int i = 0; i < radioNum; i++) {
            if (bsRadios[c][i] == NULL) {
                radioNotFound = true;
                radioSerialNotFound.push_back(_cfg->bs_sdr_ids[c][i]);
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
        _cfg->nBsSdrs[c] = radioNum;
        if (radioNotFound)
            break;

        // Perform DC Offset & IQ Imbalance Calibration
        if (_cfg->imbalanceCalEn) {
            if (_cfg->bsChannel.find('A') != std::string::npos)
                dciqCalibrationProc(0);
            if (_cfg->bsChannel.find('B') != std::string::npos)
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
            if (pthread_create(&configure_thread_, NULL, BaseRadioSet::configure_launch, context) != 0) {
                perror("configure thread create failed");
                exit(0);
            }
#else
            configure(context);
#endif
        }

        while (threadCount > 0)
            ;
        // Measure Sync Delays now!
        if (!kUseUHD)
            sync_delays(c);
    }

    if (radioNotFound) {
        for (auto st = radioSerialNotFound.begin(); st != radioSerialNotFound.end(); st++)
            std::cout << "\033[1;31m" << *st << "\033[0m" << std::endl;
        std::cout << "\033[1;31mERROR: the above base station serials were not discovered in the network!\033[0m" << std::endl;
    } else {
        if (_cfg->sampleCalEn) {
            bool adjust = false;
            int cal_cnt = 0;
            while (!adjust) {
                if (++cal_cnt > 10) {
                    std::cout << "10 attemps of sample offset calibration, stopping..." << std::endl;
                    break;
                }
                adjust = true;
                collectCSI(adjust); // run 1: find offsets and adjust
            }
            collectCSI(adjust); // run 2: verify adjustments
            usleep(100000);
            std::cout << "sample offset calibration done!" << std::endl;
        }

        std::vector<std::string> _tddSched;
        _tddSched.resize(_cfg->frames.size());
        // change symbol letter to Soapy equivalent
        for (unsigned int f = 0; f < _cfg->frames.size(); f++) {
            _tddSched[f] = _cfg->frames[f];
            for (size_t s = 0; s < _cfg->frames[f].size(); s++) {
                char c = _cfg->frames[f].at(s);
                if (c == 'P')
                    _tddSched[f].replace(s, 1, "R"); // uplink pilots
                else if (c == 'U')
                    _tddSched[f].replace(s, 1, "R"); // uplink data
                else if (c == 'D')
                    _tddSched[f].replace(s, 1, "T"); // downlink data
            }
            std::cout << _tddSched[f] << std::endl;
        }

#ifdef JSON
        json tddConf;
        tddConf["tdd_enabled"] = true;
        tddConf["frame_mode"] = "free_running";
        tddConf["max_frame"] = _cfg->max_frame;
        tddConf["frames"] = _tddSched;
        tddConf["symbol_size"] = _cfg->sampsPerSymbol;
        tddConf["beacon_start"] = _cfg->prefix;
        tddConf["beacon_stop"] = _cfg->prefix + _cfg->beaconSize;
        std::string tddConfStr = tddConf.dump();
#else
        std::string tddConfStr = "{\"tdd_enabled\":true,\"frame_mode\":\"free_running\"";
        tddConfStr += ",\"symbol_size\":" + std::to_string(_cfg->sampsPerSymbol);
        tddConfStr += ",\"beacon_start\":" + std::to_string(_cfg->prefix);
        tddConfStr += ",\"beacon_stop\":" + std::to_string(_cfg->prefix + _cfg->beaconSize);
        tddConfStr += ",\"frames\":[";
        for (size_t f = 0; f < _cfg->frames.size(); f++)
            tddConfStr += (f == _cfg->frames.size() - 1) ? "\"" + _tddSched[f] + "\"" : "\"" + _tddSched[f] + "\",";
        tddConfStr += "]}";
        std::cout << tddConfStr << std::endl;
#endif
        // write TDD schedule and beacons to FPFA buffers only for Iris
        if (!kUseUHD) {
            for (size_t i = 0; i < bsRadios[0].size(); i++) {
                SoapySDR::Device* dev = bsRadios[0][i]->dev;
                dev->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
                dev->writeSetting("TDD_MODE", "true");
                dev->writeSetting("TDD_CONFIG", tddConfStr);
            }

            // write beacons to FPGA buffers
            size_t ndx = 0;
            for (size_t i = 0; i < bsRadios[0].size(); i++) {
                SoapySDR::Device* dev = bsRadios[0][i]->dev;
                dev->writeRegisters("BEACON_RAM", 0, _cfg->beacon);
                for (char const& c : _cfg->bsChannel) {
                    bool isBeaconAntenna = !_cfg->beamsweep && ndx == _cfg->beacon_ant;
                    std::vector<unsigned> beacon_weights(nBsAntennas[0], isBeaconAntenna ? 1 : 0);
                    std::string tx_ram_wgt = "BEACON_RAM_WGT_";
                    if (_cfg->beamsweep) {
                        for (int j = 0; j < nBsAntennas[0]; j++)
                            beacon_weights[j] = CommsLib::hadamard2(ndx, j);
                    }
                    dev->writeRegisters(tx_ram_wgt + c, 0, beacon_weights);
                    ++ndx;
                }
                dev->writeSetting("BEACON_START", std::to_string(bsRadios[0].size()));
            }
        }

        for (size_t i = 0; i < bsRadios[0].size(); i++) {
            std::cout << "bsRadios i: " << i << std::endl;
            SoapySDR::Device* dev = bsRadios[0][i]->dev;
            if (!kUseUHD) {
                bsRadios[0][i]->activateRecv();
                bsRadios[0][i]->activateXmit();
                dev->setHardwareTime(0, "TRIGGER");
            } else {
                dev->setHardwareTime(0.0); // "CMD"
                bsRadios[0][i]->activateRecv();
                bsRadios[0][i]->activateXmit();
            }
        }
        std::cout << __func__ << " done!" << std::endl;
    }
}

BaseRadioSet::~BaseRadioSet(void)
{
    if (!_cfg->hub_ids.empty()) {
        for (unsigned int i = 0; i < hubs.size(); i++)
            SoapySDR::Device::unmake(hubs[i]);
    }
    for (unsigned int c = 0; c < _cfg->nCells; c++)
        for (size_t i = 0; i < _cfg->nBsSdrs[c]; i++)
            delete bsRadios[c][i];
}

void* BaseRadioSet::init_launch(void* in_context)
{
    BaseRadioContext* context = (BaseRadioContext*)in_context;
    BaseRadioSet* brs = context->brs;
    brs->init(context);
    return 0;
}

void BaseRadioSet::init(BaseRadioContext* context)
{
    int i = context->tid;
    int c = context->cell;
    std::atomic_int* threadCount = context->threadCount;
    delete context;

    auto channels = Utils::strToChannels(_cfg->bsChannel);
    SoapySDR::Kwargs args;
    if (!kUseUHD) {
        args["driver"] = "iris";
        args["serial"] = _cfg->bs_sdr_ids[c][i];
    } else {
        args["driver"] = "uhd";
        args["addr"] = _cfg->bs_sdr_ids[c][i];
    }
    args["timeout"] = "1000000";
    try {
        bsRadios[c][i] = new Radio(args, SOAPY_SDR_CS16, channels, _cfg->rate);
        std::cout << "Init bsRadios: " << args["addr"] << std::endl;
    } catch (std::runtime_error) {
        if (!kUseUHD)
            std::cerr << "Ignoring iris " << _cfg->bs_sdr_ids[c][i] << std::endl;
        else
            std::cerr << "Ignoring uhd device " << _cfg->bs_sdr_ids[c][i] << std::endl;
        bsRadios[c][i] = NULL;
    }

    (*threadCount)--;
}

void* BaseRadioSet::configure_launch(void* in_context)
{
    BaseRadioContext* context = (BaseRadioContext*)in_context;
    BaseRadioSet* brs = context->brs;
    brs->configure(context);
    return 0;
}

void BaseRadioSet::configure(BaseRadioContext* context)
{
    int i = context->tid;
    int c = context->cell;
    std::atomic_int* threadCount = context->threadCount;
    delete context;

    //load channels
    auto channels = Utils::strToChannels(_cfg->bsChannel);
    Radio* bsRadio = bsRadios[c][i];
    SoapySDR::Device* dev = bsRadio->dev;
    SoapySDR::Kwargs info = dev->getHardwareInfo();
    for (auto ch : channels) {
        double rxgain = _cfg->rxgain[ch];
        double txgain = _cfg->txgain[ch];
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
    for (size_t c = 0; c < _cfg->nCells; c++) {
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
    for (size_t i = 0; i < bsRadios[0].size(); i++) {
        SoapySDR::Device* dev = bsRadios[0][i]->dev;
        std::cout << "TEMPs on Iris " << i << std::endl;
        std::cout << "ZYNQ_TEMP: " << dev->readSensor("ZYNQ_TEMP") << std::endl;
        std::cout << "LMS7_TEMP  : " << dev->readSensor("LMS7_TEMP") << std::endl;
        std::cout << "FE_TEMP  : " << dev->readSensor("FE_TEMP") << std::endl;
        std::cout << "TX0 TEMP  : " << dev->readSensor(SOAPY_SDR_TX, 0, "TEMP") << std::endl;
        std::cout << "TX1 TEMP  : " << dev->readSensor(SOAPY_SDR_TX, 1, "TEMP") << std::endl;
        std::cout << "RX0 TEMP  : " << dev->readSensor(SOAPY_SDR_RX, 0, "TEMP") << std::endl;
        std::cout << "RX1 TEMP  : " << dev->readSensor(SOAPY_SDR_RX, 1, "TEMP") << std::endl;
        std::cout << std::endl;
    }
}

void BaseRadioSet::radioStop(void)
{
    std::string tddConfStr = "{\"tdd_enabled\":false}";
    for (size_t i = 0; i < bsRadios[0].size(); i++) {
        if (!kUseUHD) {
            SoapySDR::Device* dev = bsRadios[0][i]->dev;
            dev->writeSetting("TDD_CONFIG", tddConfStr);
            dev->writeSetting("TDD_MODE", "false");
        }
        bsRadios[0][i]->reset_DATA_clk_domain();
    }
}

void BaseRadioSet::radioTx(const void* const* buffs)
{
    long long frameTime(0);
    for (size_t i = 0; i < bsRadios[0].size(); i++) {
        bsRadios[0][i]->xmit(buffs, _cfg->sampsPerSymbol, 0, frameTime);
    }
}

int BaseRadioSet::radioTx(size_t radio_id, const void* const* buffs, int flags, long long& frameTime)
{
    int w;
    // for UHD device xmit from host using frameTimeNs
    if (!kUseUHD) {
        w = bsRadios[0][radio_id]->xmit(buffs, _cfg->sampsPerSymbol, flags, frameTime);
    } else {
        long long frameTimeNs = SoapySDR::ticksToTimeNs(frameTime, _cfg->rate);
        w = bsRadios[0][radio_id]->xmit(buffs, _cfg->sampsPerSymbol, flags, frameTimeNs);
    }
#if DEBUG_RADIO
    size_t chanMask;
    long timeoutUs(0);
    SoapySDR::Device* dev = bsRadios[0][radio_id]->dev;
    int s = dev->readStreamStatus(bsRadio->txs, chanMask, flags, frameTime, timeoutUs);
    int s = dev->readStreamStatus(SOAPY_SDR_TX, chanMask, flags, frameTime, timeoutUs);
    std::cout << "radio " << radio_id << " tx returned " << w << " and status " << s << std::endl;
#endif
    return w;
}

void BaseRadioSet::radioRx(void* const* buffs)
{
    long long frameTime(0);
    for (size_t i = 0; i < bsRadios[0].size(); i++) {
        void* const* buff = buffs + (i * 2);
        bsRadios[0][i]->recv(buff, _cfg->sampsPerSymbol, frameTime);
    }
}

int BaseRadioSet::radioRx(size_t radio_id, void* const* buffs, long long& frameTime)
{
    if (radio_id < bsRadios[0].size()) {
        long long frameTimeNs = 0;
        int ret = bsRadios[0][radio_id]->recv(buffs, _cfg->sampsPerSymbol, frameTimeNs);
        // for UHD device recv using ticks
        if (!kUseUHD)
            frameTime = frameTimeNs; //SoapySDR::timeNsToTicks(frameTimeNs, _rate);
        else
            frameTime = SoapySDR::timeNsToTicks(frameTimeNs, _cfg->rate);
#if DEBUG_RADIO
        if (ret != _cfg->sampsPerSymbol)
            std::cout << "recv returned " << ret << " from radio " << radio_id << ", Expected "
                      << _cfg->sampsPerSymbol << std::endl;
        else
            std::cout << "radio " << radio_id << "received " << ret << " at " << frameTime << std::endl;
#endif
        return ret;
    }
    std::cout << "invalid radio id " << radio_id << std::endl;
    return 0;
}

int BaseRadioSet::radioRx(size_t radio_id, void* const* buffs, int numSamps, long long& frameTime)
{
    if (radio_id < bsRadios[0].size()) {
        long long frameTimeNs = 0;
        int ret = bsRadios[0][radio_id]->recv(buffs, numSamps, frameTimeNs);
        frameTime = SoapySDR::timeNsToTicks(frameTimeNs, _cfg->rate);
        return ret;
    }
    std::cout << "invalide radio id " << radio_id << std::endl;
    return 0;
}
