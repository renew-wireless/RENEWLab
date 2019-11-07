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

#include "include/sdr-lib.h"
#include "include/comms-lib.h"
#include "include/macros.h"
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>

static void
reset_DATA_clk_domain(SoapySDR::Device* dev)
{
    dev->writeSetting("RESET_DATA_LOGIC", "");
}

static void
dev_init(SoapySDR::Device* dev, Config* _cfg, int ch, double rxgain, double txgain,
    const std::string& feType)
{
    // these params are sufficient to set befor DC offset and IQ imbalance calibration
    dev->setAntenna(SOAPY_SDR_RX, ch, "TRX");
    dev->setBandwidth(SOAPY_SDR_RX, ch, _cfg->bwFilter);
    dev->setBandwidth(SOAPY_SDR_TX, ch, _cfg->bwFilter);

    dev->setFrequency(SOAPY_SDR_RX, ch, "RF", _cfg->radioRfFreq);
    dev->setFrequency(SOAPY_SDR_RX, ch, "BB", _cfg->nco);
    dev->setFrequency(SOAPY_SDR_TX, ch, "RF", _cfg->radioRfFreq);
    dev->setFrequency(SOAPY_SDR_TX, ch, "BB", _cfg->nco);

    // lime
    dev->setGain(SOAPY_SDR_RX, ch, "LNA", rxgain);
    dev->setGain(SOAPY_SDR_RX, ch, "TIA", 0); //[0,12]
    dev->setGain(SOAPY_SDR_RX, ch, "PGA", 0); //[-12,19]
    dev->setGain(SOAPY_SDR_TX, ch, "IAMP", 0); //[0,12]
    dev->setGain(SOAPY_SDR_TX, ch, "PAD", txgain);

    // Set Front-end gains based on type
    // TODO: replace these with one gain setting
    if (feType.find("CBRS") != std::string::npos) {
        // receive gains
        if (_cfg->freq > 3e9) { // CBRS HI Band
            dev->setGain(SOAPY_SDR_RX, ch, "ATTN", 0); //[-18,0]
            dev->setGain(SOAPY_SDR_RX, ch, "LNA2", 14); //[0,14]
        } else { // CBRS LO Band
            dev->setGain(SOAPY_SDR_RX, ch, "ATTN", -12); //[-18,0]
            dev->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
        }

        // transmit gains
        if (_cfg->freq > 3e9) { // CBRS HI Band
            dev->setGain(SOAPY_SDR_TX, ch, "ATTN", -6); //[-18,0] by 3
            // Setting PA2 can cause saturation or even damage!! DO NOT USE IF NOT SURE!!!
            dev->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //[0|14]   can bypass
        } else if (_cfg->freq > 2e9) { // CBRS LO Band
            dev->setGain(SOAPY_SDR_TX, ch, "ATTN", -6); //[-18,0] by 3
            dev->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //[0|17]   can bypass.
        }
    }
    if (feType.find("UHF") != std::string::npos) {
        // receive gains
        dev->setGain(SOAPY_SDR_RX, ch, "ATTN1", -6); //[-18,0]
        dev->setGain(SOAPY_SDR_RX, ch, "ATTN2", -12); //[-18,0]

        // transmit gains
        dev->setGain(SOAPY_SDR_TX, ch, "ATTN", 0); //[-18,0] by 3
    }

    dev->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
}

ClientRadioSet::ClientRadioSet(Config* cfg)
    : _cfg(cfg)
{
    //load channels
    std::vector<size_t> channels;
    if (_cfg->bsChannel == "A")
        channels = { 0 };
    else if (cfg->bsChannel == "B")
        channels = { 1 };
    else
        channels = { 0, 1 };
    radios.resize(_cfg->nClSdrs);
    for (size_t i = 0; i < _cfg->nClSdrs; i++) {
        SoapySDR::Kwargs args;
        args["timeout"] = "1000000";
        args["serial"] = _cfg->cl_sdr_ids.at(i);
        radios[i] = new Radio(args, SOAPY_SDR_CF32, channels, _cfg->rate);
        auto dev = radios[i]->dev;
        SoapySDR::Kwargs info = dev->getHardwareInfo();

        for (auto ch : { 0, 1 }) //channels)
        {
            double rxgain = _cfg->clRxgain_vec[ch][i]; //[0,30]
            double txgain = _cfg->clTxgain_vec[ch][i]; //[0,52]
            dev_init(dev, _cfg, ch, rxgain, txgain, info["frontend"]);
        }

        initAGC(dev);
    }

    //beaconSize + 82 (BS FE delay) + 68 (path delay) + 17 (correlator delay) + 82 (Client FE Delay)
    int ueTrigOffset = _cfg->beaconSize + 249;
    int sf_start = ueTrigOffset / _cfg->sampsPerSymbol;
    int sp_start = ueTrigOffset % _cfg->sampsPerSymbol;

    std::vector<std::string> tddSched;
    tddSched.resize(_cfg->nClSdrs);
    for (size_t i = 0; i < _cfg->nClSdrs; i++) {
        tddSched[i] = _cfg->clFrames[i];
        for (size_t s = 0; s < _cfg->clFrames[i].size(); s++) {
            char c = _cfg->clFrames[i].at(s);
            if (c == 'B')
                tddSched[i].replace(s, 1, "G");
            else if (c == 'U')
                tddSched[i].replace(s, 1, "T");
            else if (c == 'D')
                tddSched[i].replace(s, 1, "R");
        }
        std::cout << "Client " << i << " schedule: " << tddSched[i] << std::endl;
    }

    for (size_t i = 0; i < _cfg->nClSdrs; i++) {
        auto dev = radios[i]->dev;
        std::string corrConfString = "{\"corr_enabled\":true,\"corr_threshold\":" + std::to_string(1) + "}";
        dev->writeSetting("CORR_CONFIG", corrConfString);
        dev->writeRegisters("CORR_COE", 0, _cfg->coeffs);

#ifdef JSON
        json tddConf;
        tddConf["tdd_enabled"] = true;
        tddConf["frame_mode"] = _cfg->frame_mode;
        int max_frame_ = (int)(2.0 / ((_cfg->sampsPerSymbol * _cfg->symbolsPerFrame) / _cfg->rate));
        tddConf["max_frame"] = max_frame_;
        //std::cout << "max_frames for client " << i << " is " << max_frame_ << std::endl;
        if (_cfg->clSdrCh == 2)
            tddConf["dual_pilot"] = true;
        tddConf["frames"] = json::array();
        tddConf["frames"].push_back(tddSched[i]);
        tddConf["symbol_size"] = _cfg->sampsPerSymbol;
        std::string tddConfStr = tddConf.dump();
#else
        std::string tddConfStr = "{\"tdd_enabled\":true,\"frame_mode\":" + _cfg->frame_mode + ",";
        tddConfStr += "\"symbol_size\":" + std::to_string(_cfg->sampsPerSymbol);
        if (_cfg->clSdrCh == 2)
            tddConfStr += "\"dual_pilot\":true,";
        tddConfStr += ",\"frames\":[\"" + tddSched[i] + "\"]}";
        std::cout << tddConfStr << std::endl;
#endif
        dev->writeSetting("TDD_CONFIG", tddConfStr);

        dev->setHardwareTime(SoapySDR::ticksToTimeNs((sf_start << 16) | sp_start, _cfg->rate), "TRIGGER");
        dev->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
        dev->writeSetting("TDD_MODE", "true");
        // write beacons to FPGA buffers
        if (_cfg->bsChannel != "B")
            dev->writeRegisters("TX_RAM_A", 0, _cfg->pilot);
        if (_cfg->bsChannel == "B")
            dev->writeRegisters("TX_RAM_B", 0, _cfg->pilot);
        if (_cfg->clSdrCh == 2)
            dev->writeRegisters("TX_RAM_B", 0, _cfg->pilot);

        radios[i]->activateRecv();
        radios[i]->activateXmit();

        dev->writeSetting("CORR_START", (_cfg->bsChannel == "B") ? "B" : "A");
    }
    std::cout << __func__ << " done!" << std::endl;
}

ClientRadioSet::~ClientRadioSet(void)
{
    for (size_t i = 0; i < _cfg->nClSdrs; i++)
        delete radios[i];
}

void ClientRadioSet::radioStop(void)
{
    std::string corrConfStr = "{\"corr_enabled\":false}";
    std::string tddConfStr = "{\"tdd_enabled\":false}";
    for (size_t i = 0; i < _cfg->nClSdrs; i++) {
        auto dev = radios[i]->dev;
        dev->writeSetting("CORR_CONFIG", corrConfStr);
        const auto timeStamp = SoapySDR::timeNsToTicks(dev->getHardwareTime(""), _cfg->rate);
        std::cout << "device " << i << ": Frame=" << (timeStamp >> 32)
                  << ", Symbol=" << ((timeStamp && 0xFFFFFFFF) >> 16) << std::endl;
        dev->writeSetting("TDD_CONFIG", tddConfStr);
        dev->writeSetting("TDD_MODE", "false");
        reset_DATA_clk_domain(dev);
    }
}

Radio* ClientRadioSet::getRadio(int i)
{
    return (radios[i]);
}

void ClientRadioSet::initAGC(SoapySDR::Device* dev)
{
    /*
     * Initialize AGC parameters
     */
#ifdef JSON
    json agcConf;
    agcConf["agc_enabled"] = _cfg->clAgcEn;
    agcConf["agc_gain_init"] = _cfg->clAgcGainInit;
    std::string agcConfStr = agcConf.dump();
#else
    std::string agcConfStr = "{\"agc_enabled\":" + _cfg->clAgcEn ? "true" : "false";
    agcConfStr += ",\"agc_gain_init\":" + std::to_string(_cfg->clAgcGainInit);
    agcConfStr += "}";
#endif
    dev->writeSetting("AGC_CONFIG", agcConfStr);
}

BaseRadioSet::BaseRadioSet(Config* cfg)
    : _cfg(cfg)
{
    std::vector<int> nBsAntennas(_cfg->nCells);
    bsRadios.resize(_cfg->nCells);

    for (size_t c = 0; c < _cfg->nCells; c++) {
        int radioNum = _cfg->nBsSdrs[c];
        nBsAntennas[c] = radioNum * _cfg->bsChannel.length();

        std::cout << radioNum << " radios in cell " << c << std::endl;
        if (!_cfg->hub_ids.empty()) {
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
            pthread_t init_thread_;
            if (pthread_create(&init_thread_, NULL, BaseRadioSet::configure_launch, context) != 0) {
                perror("init thread create failed");
                exit(0);
            }
#else
            configure(context);
#endif
        }

        while (threadCount > 0)
            ;
        // Measure Sync Delays now!
        sync_delays(c);
    }

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
    for (size_t i = 0; i < bsRadios[0].size(); i++) {
        SoapySDR::Device* dev = bsRadios[0][i]->dev;
        dev->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
        dev->writeSetting("TDD_MODE", "true");
        dev->writeSetting("TDD_CONFIG", tddConfStr);
    }

    // write beacons to FPGA buffers
    std::vector<unsigned> zeros(_cfg->sampsPerSymbol, 0);
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

    for (size_t i = 0; i < bsRadios[0].size(); i++) {
        SoapySDR::Device* dev = bsRadios[0][i]->dev;
        bsRadios[0][i]->activateRecv();
        bsRadios[0][i]->activateXmit();
        dev->setHardwareTime(0, "TRIGGER");
    }
    std::cout << __func__ << " done!" << std::endl;
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

    std::vector<size_t> channels;
    if (_cfg->bsChannel == "A")
        channels = { 0 };
    else if (_cfg->bsChannel == "B")
        channels = { 1 };
    else
        channels = { 0, 1 };

    SoapySDR::Kwargs args;
    args["driver"] = "iris";
    args["timeout"] = "1000000";
    args["serial"] = _cfg->bs_sdr_ids[c][i];
    bsRadios[c][i] = new Radio(args, SOAPY_SDR_CS16, channels, _cfg->rate);
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
    std::vector<size_t> channels;
    if (_cfg->bsChannel == "A")
        channels = { 0 };
    else if (_cfg->bsChannel == "B")
        channels = { 1 };
    else
        channels = { 0, 1 };

    Radio* bsRadio = bsRadios[c][i];
    SoapySDR::Device* dev = bsRadio->dev;
    SoapySDR::Kwargs info = dev->getHardwareInfo();
    for (auto ch : channels) {
        double rxgain = _cfg->rxgain[ch];
        double txgain = _cfg->txgain[ch];
        dev_init(dev, _cfg, ch, rxgain, txgain, info["frontend"]);
    }

    (*threadCount)--;
}

SoapySDR::Device* BaseRadioSet::baseRadio(int cellId)
{
    return (hubs.empty() ? bsRadios[cellId][0]->dev : hubs[cellId]);
}

void BaseRadioSet::radioTrigger(void)
{
    for (size_t c = 0; c < _cfg->nCells; c++)
        baseRadio(c)->writeSetting("TRIGGER_GEN", "");
}

void BaseRadioSet::radioStart()
{
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
        SoapySDR::Device* dev = bsRadios[0][i]->dev;
        dev->writeSetting("TDD_CONFIG", tddConfStr);
        dev->writeSetting("TDD_MODE", "false");
        reset_DATA_clk_domain(dev);
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
    int w = bsRadios[0][radio_id]->xmit(buffs, _cfg->sampsPerSymbol, flags, frameTime);
#if DEBUG_RADIO
    size_t chanMask;
    long timeoutUs(0);
    int s = dev->readStreamStatus(bsRadio->txs, chanMask, flags, frameTime, timeoutUs);
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
        frameTime = frameTimeNs; //SoapySDR::timeNsToTicks(frameTimeNs, _rate);
#if DEBUG_RADIO
        if (ret != _cfg->sampsPerSymbol)
            std::cout << "recv returned " << ret << " from radio " << radio_id << ", Expected "
                      << _cfg->sampsPerSymbol << std::endl;
        else
            std::cout << "radio " << radio_id << "received " << ret << std::endl;
#endif
        return ret;
    }
    std::cout << "invalid radio id " << radio_id << std::endl;
    return 0;
}

void Radio::drain_buffers(std::vector<void*> buffs, int symSamp)
{
    /*
     *  "Drain" rx buffers during initialization
     *  Input:
     *      buffs   - Vector to which we will write received IQ samples
     *      symSamp - Number of samples
     *
     *  Output:
     *      None
     */
    long long frameTime = 0;
    int flags = 0, r = 0, i = 0;
    while (r != -1) {
        r = dev->readStream(rxs, buffs.data(), symSamp, flags, frameTime, 0);
        i++;
    }
    //std::cout << "Number of reads needed to drain: " << i << std::endl;
}

void BaseRadioSet::sync_delays(int cellIdx)
{
    /*
     * Compute Sync Delays
     */
    baseRadio(cellIdx)->writeSetting("SYNC_DELAYS", "");
}

Radio::Radio(const SoapySDR::Kwargs& args, const char soapyFmt[],
    const std::vector<size_t>& channels, double rate)
{
    dev = SoapySDR::Device::make(args);
    for (auto ch : channels) {
        dev->setSampleRate(SOAPY_SDR_RX, ch, rate);
        dev->setSampleRate(SOAPY_SDR_TX, ch, rate);
    }
    rxs = dev->setupStream(SOAPY_SDR_RX, soapyFmt, channels);
    txs = dev->setupStream(SOAPY_SDR_TX, soapyFmt, channels);
    reset_DATA_clk_domain(dev);
}

Radio::~Radio(void)
{
    deactivateRecv();
    deactivateXmit();
    dev->closeStream(rxs);
    dev->closeStream(txs);
    SoapySDR::Device::unmake(dev);
}

int Radio::recv(void* const* buffs, int samples, long long& frameTime)
{
    int flags(0);
    return dev->readStream(rxs, buffs, samples, flags, frameTime, 1000000);
}

int Radio::activateRecv(const long long rxTime, const size_t numSamps)
{
    return dev->activateStream(rxs, 0, rxTime, numSamps);
}

void Radio::deactivateRecv(void)
{
    dev->deactivateStream(rxs);
}

int Radio::xmit(const void* const* buffs, int samples, int flags, long long& frameTime)
{
    int soapyFlags[] = {
        0,
        SOAPY_SDR_HAS_TIME,
        SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
        SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST
    };
    int flag_args = soapyFlags[flags];
    int r = dev->writeStream(txs, buffs, samples, flag_args, frameTime, 1000000);
    if (r != samples)
        std::cerr << "unexpected writeStream error " << SoapySDR::errToStr(r) << std::endl;
    return (r);
}

void Radio::activateXmit(void)
{
    dev->activateStream(txs);
}

void Radio::deactivateXmit(void)
{
    dev->deactivateStream(txs);
}

int Radio::getTriggers(void) const
{
    return std::stoi(dev->readSetting("TRIGGER_COUNT"));
}
