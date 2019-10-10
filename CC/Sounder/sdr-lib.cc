/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
 
---------------------------------------------------------------------
 Initializes and Configures Radios in the massive-MIMO base station 
---------------------------------------------------------------------
*/

#include "include/sdr-lib.h"
#include "include/comms-lib.h"
#include "include/macros.h"
#include "include/matplotlibcpp.h"

#include <fstream>
#include <iostream>

namespace plt = matplotlibcpp;

static void
reset_DATA_clk_domain(SoapySDR::Device* dev)
{
    // What does 29 mean?
    dev->writeRegister("IRIS30", RF_RST_REG, (1 << 29) | 1);
    dev->writeRegister("IRIS30", RF_RST_REG, (1 << 29));
    dev->writeRegister("IRIS30", RF_RST_REG, 0);
}

static void
dev_init(SoapySDR::Device* dev, Config* _cfg, int ch, double rxgain, double txgain)
{
    dev->setBandwidth(SOAPY_SDR_RX, ch, (1 + 2 * _cfg->bbf_ratio) * _cfg->rate);
    dev->setBandwidth(SOAPY_SDR_TX, ch, (1 + 2 * _cfg->bbf_ratio) * _cfg->rate);

    dev->setSampleRate(SOAPY_SDR_RX, ch, _cfg->rate);
    dev->setSampleRate(SOAPY_SDR_TX, ch, _cfg->rate);

    dev->setFrequency(SOAPY_SDR_RX, ch, "RF", _cfg->freq - _cfg->bbf_ratio * _cfg->rate);
    dev->setFrequency(SOAPY_SDR_RX, ch, "BB", _cfg->bbf_ratio * _cfg->rate);
    dev->setFrequency(SOAPY_SDR_TX, ch, "RF", _cfg->freq - _cfg->bbf_ratio * _cfg->rate);
    dev->setFrequency(SOAPY_SDR_TX, ch, "BB", _cfg->bbf_ratio * _cfg->rate);

    // lime
    dev->setGain(SOAPY_SDR_RX, ch, "LNA", rxgain);
    dev->setGain(SOAPY_SDR_RX, ch, "TIA", 0); //[0,12]
    dev->setGain(SOAPY_SDR_RX, ch, "PGA", 0); //[-12,19]
    dev->setGain(SOAPY_SDR_TX, ch, "IAMP", 0); //[0,12]
    dev->setGain(SOAPY_SDR_TX, ch, "PAD", txgain);
}

RadioConfig::RadioConfig(Config* cfg)
    : _cfg(cfg)
{
    if (_cfg->bsPresent) {
        nBsAntennas.resize(_cfg->nCells);
        bsRadios.resize(_cfg->nCells);

        for (unsigned int c = 0; c < _cfg->nCells; c++) {
            int radioNum = _cfg->nBsSdrs[c];
            nBsAntennas[c] = radioNum * _cfg->bsChannel.length();
            std::cout << radioNum << " radios in cell " << c << std::endl;
            //isUE = _cfg->isUE;
            if (!_cfg->hub_ids.empty()) {
                SoapySDR::Kwargs args;
                args["driver"] = "remote";
                args["timeout"] = "1000000";
                args["serial"] = _cfg->hub_ids.at(c);
                hubs.push_back(SoapySDR::Device::make(args));
            }

            bsRadios[c].resize(radioNum);
            remainingJobs = radioNum;
            for (int i = 0; i < radioNum; i++) {
                //args["serial"] = _cfg->bs_sdr_ids[c][i];
                //args["timeout"] = "1000000";
                //bsSdrs[c].push_back(SoapySDR::Device::make(args));
                RadioConfigContext* context = new RadioConfigContext;
                context->ptr = this;
                context->tid = i;
                context->cell = c;
#ifdef THREADED_INIT
                pthread_t init_thread_;
                if (pthread_create(&init_thread_, NULL, RadioConfig::initBSRadio_launch, context) != 0) {
                    perror("init thread create failed");
                    exit(0);
                }
#else
                initBSRadio(context);
#endif
            }

            while (remainingJobs > 0)
                ;
            // Measure Sync Delays now!
            sync_delays(0);
        }
    }
    if (_cfg->clPresent) {
        //load channels
        std::vector<size_t> channels;
        if (_cfg->bsChannel == "A")
            channels = { 0 };
        else if (cfg->bsChannel == "B")
            channels = { 1 };
        else
            channels = { 0, 1 };
        radios.resize(_cfg->nClSdrs);
        for (size_t i = 0; i < radios.size(); i++) {
            auto dev = SoapySDR::Device::make("serial=" + _cfg->cl_sdr_ids.at(i) + ",timeout=10000000");
            radios[i].dev = dev;
            SoapySDR::Kwargs info = dev->getHardwareInfo();

            for (auto ch : { 0, 1 }) //channels)
            {
                double rxgain = _cfg->clRxgain_vec[ch][i]; //[0,30]
                double txgain = _cfg->clTxgain_vec[ch][i]; //[0,52]
                dev_init(dev, _cfg, ch, rxgain, txgain);

                if (info["frontend"].find("CBRS") != std::string::npos) {
                    // receive gains
                    dev->setGain(SOAPY_SDR_RX, ch, "LNA1", 30); //[0,33]
                    if (cfg->freq >= 3e9) {
                        dev->setGain(SOAPY_SDR_RX, ch, "ATTN", 0); //[-18,0]
                        dev->setGain(SOAPY_SDR_RX, ch, "LNA2", 14); //[0,14]
                    } else if (_cfg->freq > 2e9) {
                        dev->setGain(SOAPY_SDR_RX, ch, "ATTN", -12); //[-18,0]
                        dev->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
                    }

                    // transmit gains
                    dev->setGain(SOAPY_SDR_TX, ch, "ATTN", -6); //{-18,-12,-6,0}
                    if (info["frontend"].find("CBRSc") != std::string::npos) {
                        // on revC front-end, it is safe to turn on PA2
                        if (cfg->freq >= 3e9)
                            dev->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //CBRS HI, [0|14]
                        else if (_cfg->freq > 2e9)
                            dev->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //CBRS LO, [0|17]
                    } else
                        dev->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //[0|17]
                }
            }

            for (auto ch : channels) {
                //dev->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
                //dev->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
                dev->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
            }

            reset_DATA_clk_domain(dev);
            radios[i].rxs = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, channels);
            radios[i].txs = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, channels);

            RadioConfig::initAGC(dev);
        }
    }
    std::cout << "radio init done!" << std::endl;
}

void* RadioConfig::initBSRadio_launch(void* in_context)
{
    RadioConfigContext* context = (RadioConfigContext*)in_context;
    RadioConfig* rc = context->ptr;
    rc->initBSRadio(context);
    return 0;
}

void RadioConfig::initBSRadio(RadioConfigContext* context)
{
    int i = context->tid;
    int c = context->cell;
    delete context;

    //load channels
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
    args["serial"] = _cfg->bs_sdr_ids[0][i];
    struct Radio* bsRadio = &bsRadios[c][i];
    SoapySDR::Device* dev = bsRadio->dev = (SoapySDR::Device::make(args));
    //use the TRX antenna port for both tx and rx
    for (auto ch : channels)
        dev->setAntenna(SOAPY_SDR_RX, ch, "TRX");

    SoapySDR::Kwargs info = dev->getHardwareInfo();
    for (auto ch : { 0, 1 }) //channels)
    {
        double rxgain = _cfg->rxgain[ch]; //[0,30]
        double txgain = _cfg->txgain[ch]; //[0,30]
        dev_init(dev, _cfg, ch, rxgain, txgain);

        if (info["frontend"].find("CBRS") != std::string::npos) {
            // receive gains
            dev->setGain(SOAPY_SDR_RX, ch, "LNA1", 33); //[0,33]
            if (_cfg->freq > 3e9) {
                dev->setGain(SOAPY_SDR_RX, ch, "ATTN", 0); //[-18,0]
                dev->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //LO[0,17]
            } else {
                dev->setGain(SOAPY_SDR_RX, ch, "ATTN", -12); //[-18,0]
                dev->setGain(SOAPY_SDR_RX, ch, "LNA2", 14); //HI[0,14]
            }

            // transmit gains
            if (_cfg->freq > 3e9) { // CBRS HI
                dev->setGain(SOAPY_SDR_TX, ch, "ATTN", -6); //[-18,0] by 3
                dev->setGain(SOAPY_SDR_TX, ch, "PA1", 15); //[0|13.7] no bypass
                dev->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //[0|14]   can bypass
                dev->setGain(SOAPY_SDR_TX, ch, "PA3", 30); //[0|31]   no bypass
            } else if (_cfg->freq > 2e9) { // CBRS LO
                dev->setGain(SOAPY_SDR_TX, ch, "ATTN", -6); //[-18,0] by 3
                dev->setGain(SOAPY_SDR_TX, ch, "PA1", 14); //[0|14] no bypass
                dev->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //[0|17]   can bypass.
                // Can cause saturation or PA damage!! DO NOT USE IF NOT SURE!!!
                dev->setGain(SOAPY_SDR_TX, ch, "PA3", 30); //[0|31.5]   no bypass
            }
        }
        if (info["frontend"].find("UHF") != std::string::npos) {
            // receive gains
            dev->setGain(SOAPY_SDR_RX, ch, "ATTN1", -6); //[-18,0]
            dev->setGain(SOAPY_SDR_RX, ch, "ATTN2", -12); //[-18,0]
            //dev->setGain(SOAPY_SDR_RX, ch, "LNA1", 30); //[0,33]
            //dev->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]

            // transmit gains
            dev->setGain(SOAPY_SDR_TX, ch, "ATTN", 0); //[-18,0] by 3
        }
    }

    for (auto ch : channels) {
        //dev->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
        //dev->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
        dev->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    }

    SoapySDR::Kwargs sargs;
    reset_DATA_clk_domain(dev);
    bsRadio->rxs = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
    bsRadio->txs = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs);
    remainingJobs--;
}

void RadioConfig::radioConfigure()
{
    int flags = 0;
    if (_cfg->bsPresent) {

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
        for (unsigned int f = 0; f < _cfg->frames.size(); f++) {
            _tddSched[f] = _cfg->frames[f];
            for (size_t s = 0; s < _cfg->frames[f].size(); s++) {
                char c = _cfg->frames[f].at(s);
                if (c == 'B')
                    _tddSched[f].replace(s, 1, "P");
                else if (c == 'P')
                    _tddSched[f].replace(s, 1, "R");
                else if (c == 'U')
                    _tddSched[f].replace(s, 1, "R");
                else if (c == 'D')
                    _tddSched[f].replace(s, 1, "T");
            }
            std::cout << _tddSched[f] << std::endl;
        }

#ifdef JSON
        json conf;
        conf["tdd_enabled"] = true;
        conf["frame_mode"] = "free_running";
        conf["max_frame"] = _cfg->max_frame;
        conf["frames"] = _tddSched;
        conf["symbol_size"] = _cfg->sampsPerSymbol;
        std::string confString = conf.dump();
#else
        std::string confString = "{\"tdd_enabled\":true,\"frame_mode\":\"free_running\",";
        confString += "\"symbol_size\":" + std::to_string(_cfg->sampsPerSymbol);
        confString += ",\"frames\":[";
        for (int f = 0; f < _cfg->frames.size(); f++)
            confString += (f == _cfg->frames.size() - 1) ? "\"" + _tddSched[f] + "\"" : "\"" + _tddSched[f] + "\",";
        confString += "]}";
        std::cout << confString << std::endl;
#endif
        for (size_t i = 0; i < bsRadios[0].size(); i++) {
            SoapySDR::Device* dev = bsRadios[0][i].dev;
            dev->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
            dev->writeSetting("TDD_MODE", "true");
            dev->writeSetting("TDD_CONFIG", confString);
        }

        // write beacons to FPGA buffers
        if (!_cfg->beamsweep or nBsAntennas[0] == 1) {
            std::vector<unsigned> zeros(_cfg->sampsPerSymbol, 0);
            size_t ndx = 0;
            for (size_t i = 0; i < bsRadios[0].size(); i++) {
                SoapySDR::Device* dev = bsRadios[0][i].dev;
                for (char const& c : _cfg->bsChannel) {
                    std::string tx_ram = "TX_RAM_";
                    std::vector<unsigned>& msg = ndx == _cfg->beacon_ant ? _cfg->beacon : zeros;
                    dev->writeRegisters(tx_ram + c, 0, msg);
                    ++ndx;
                }
                dev->writeRegister("RFCORE", 156, 0);
            }
        } else { // beamsweep
            std::vector<unsigned> beacon_weights(nBsAntennas[0]);
            size_t ndx = 0;
            for (size_t i = 0; i < bsRadios[0].size(); i++) {
                SoapySDR::Device* dev = bsRadios[0][i].dev;
                for (char const& c : _cfg->bsChannel) {
                    std::string tx_ram = "TX_RAM_";
                    dev->writeRegisters(tx_ram + c, 0, _cfg->beacon);
                    std::string tx_ram_wgt = "TX_RAM_WGT_";
                    for (int j = 0; j < nBsAntennas[0]; j++)
                        beacon_weights[j] = CommsLib::hadamard2(ndx, j);
                    dev->writeRegisters(tx_ram_wgt + c, 0, beacon_weights);
                    ++ndx;
                }
                dev->writeRegister("RFCORE", 156, bsRadios[0].size());
                dev->writeRegister("RFCORE", 160, 1);
            }
        }

        for (size_t i = 0; i < bsRadios[0].size(); i++) {
            SoapySDR::Device* dev = bsRadios[0][i].dev;
            dev->activateStream(bsRadios[0][i].rxs, flags, 0);
            dev->activateStream(bsRadios[0][i].txs);
            dev->setHardwareTime(0, "TRIGGER");
        }
    }

    if (_cfg->clPresent) {
        int ueTrigOffset = 505; //_cfg->prefix + 256 + _cfg->postfix + 17 + _cfg->prefix;
        int sf_start = ueTrigOffset / _cfg->sampsPerSymbol;
        int sp_start = ueTrigOffset % _cfg->sampsPerSymbol;

        std::vector<std::string> tddSched;
        tddSched.resize(radios.size());
        for (size_t i = 0; i < radios.size(); i++) {
            tddSched[i] = _cfg->clFrames[i];
            for (size_t s = 0; s < _cfg->clFrames[i].size(); s++) {
                char c = _cfg->clFrames[i].at(s);
                if (c == 'B')
                    tddSched[i].replace(s, 1, "G");
                else if (c == 'P')
                    tddSched[i].replace(s, 1, "P");
                else if (c == 'U')
                    tddSched[i].replace(s, 1, "T");
                else if (c == 'D')
                    tddSched[i].replace(s, 1, "R");
            }
            std::cout << "Client " << i << " schedule: " << tddSched[i] << std::endl;
        }

        for (size_t i = 0; i < radios.size(); i++) {
            auto dev = radios[i].dev;
            dev->writeRegister("IRIS30", CORR_CONF, 0x1);
            for (int k = 0; k < 128; k++)
                dev->writeRegister("ARGCOE", k * 4, 0);
            usleep(100000);

            dev->writeRegister("IRIS30", 64, 1); // reset faros_corr
            dev->writeRegister("IRIS30", 64, 0); // unreset faros_corr
            dev->writeRegister("IRIS30", 92, 1); // threshold is left-shifted by this many bits

            for (int k = 0; k < 128; k++)
                dev->writeRegister("ARGCOE", k * 4, _cfg->coeffs[k]);

#ifdef JSON
            json conf;
            conf["tdd_enabled"] = true;
            conf["frame_mode"] = _cfg->frame_mode;
            int max_frame_ = (int)(2.0 / ((_cfg->sampsPerSymbol * _cfg->symbolsPerFrame) / _cfg->rate));
            conf["max_frame"] = max_frame_;
            //std::cout << "max_frames for client " << i << " is " << max_frame_ << std::endl;

            conf["frames"] = json::array();
            conf["frames"].push_back(tddSched[i]);
            conf["symbol_size"] = _cfg->sampsPerSymbol;
            std::string confString = conf.dump();
#else
            std::string confString = "{\"tdd_enabled\":true,\"frame_mode\":" + _cfg->frame_mode + ",";
            confString += "\"symbol_size\":" + std::to_string(_cfg->sampsPerSymbol);
            confString += ",\"frames\":[\"" + tddSched[i] + "\"]}";
            std::cout << confString << std::endl;
#endif
            dev->writeSetting("TDD_CONFIG", confString);

            dev->setHardwareTime(SoapySDR::ticksToTimeNs((sf_start << 16) | sp_start, _cfg->rate), "TRIGGER");
            dev->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
            dev->writeSetting("TDD_MODE", "true");
            // write beacons to FPGA buffers
            if (_cfg->bsChannel != "B")
                dev->writeRegisters("TX_RAM_A", 0, _cfg->pilot);
            if (_cfg->bsChannel == "B")
                dev->writeRegisters("TX_RAM_B", 0, _cfg->pilot);
            if (_cfg->clSdrCh == 2)
                dev->writeRegisters("TX_RAM_B", 2048, _cfg->pilot);

            dev->activateStream(radios[i].rxs);
            dev->activateStream(radios[i].txs);

            if (_cfg->bsChannel != "B") // A or AB
                dev->writeRegister("IRIS30", CORR_CONF, 0x11);
            else
                dev->writeRegister("IRIS30", CORR_CONF, 0x31);
        }
    }
    std::cout << "Done with frame configuration!" << std::endl;
}

SoapySDR::Device* RadioConfig::baseRadio(int cellId)
{
    return (hubs.empty() ? bsRadios[cellId][0].dev : hubs[cellId]);
}

void RadioConfig::radioTrigger(void)
{
    baseRadio(0)->writeSetting("TRIGGER_GEN", "");
}

void RadioConfig::radioStart()
{
    if (_cfg->bsPresent)
        radioTrigger();
}

void RadioConfig::readSensors()
{
    if (_cfg->bsPresent) {
        for (size_t i = 0; i < bsRadios[0].size(); i++) {
            SoapySDR::Device* dev = bsRadios[0][i].dev;
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
}

void RadioConfig::radioStop()
{
    if (_cfg->bsPresent) {
        for (size_t i = 0; i < bsRadios[0].size(); i++) {
            SoapySDR::Device* dev = bsRadios[0][i].dev;
            // write schedule
            for (unsigned int j = 0; j < _cfg->frames.size(); j++) {
                for (int k = 0; k < _cfg->symbolsPerFrame; k++) // symnum <= 256
                {
                    dev->writeRegister("RFCORE", SCH_ADDR_REG, j * 256 + k);
                    dev->writeRegister("RFCORE", SCH_MODE_REG, 0);
                }
            }
            dev->writeSetting("TDD_MODE", "false");
            reset_DATA_clk_domain(dev);
        }
    }
    if (_cfg->clPresent) {
        for (size_t i = 0; i < radios.size(); i++) {
            auto dev = radios[i].dev;
            dev->writeRegister("IRIS30", CORR_CONF, 0);
            std::cout << "device " << i << " T=" << std::hex << SoapySDR::timeNsToTicks(dev->getHardwareTime(""), _cfg->rate) << std::dec << std::endl;
            for (int i = 0; i < _cfg->symbolsPerFrame; i++) {
                dev->writeRegister("RFCORE", SCH_ADDR_REG, i);
                dev->writeRegister("RFCORE", SCH_MODE_REG, 0);
            }
            dev->writeSetting("TDD_MODE", "false");
            reset_DATA_clk_domain(dev);
            //SoapySDR::Device::unmake(dev);
        }
    }
}

void RadioConfig::radioTx(const void* const* buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (size_t i = 0; i < bsRadios[0].size(); i++) {
        struct Radio* bsRadio = &bsRadios[0][i];
        SoapySDR::Device* dev = bsRadio->dev;
        dev->writeStream(bsRadio->txs, buffs, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
    }
}

int RadioConfig::radioTx(size_t r /*radio id*/, const void* const* buffs, int flags, long long& frameTime)
{
    if (flags == 1)
        flags = SOAPY_SDR_HAS_TIME;
    else if (flags == 2)
        flags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
    //long long frameTime(0);

    struct Radio* bsRadio = &bsRadios[0][r];
    SoapySDR::Device* dev = bsRadio->dev;
    int w = dev->writeStream(bsRadios[0][r].txs, buffs, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
#if DEBUG_RADIO
    size_t chanMask;
    long timeoutUs(0);
    int s = dev->readStreamStatus(bsRadio->txs, chanMask, flags, frameTime, timeoutUs);
    std::cout << "radio " << r << " tx returned " << w << " and status " << s << std::endl;
#endif
    return w;
}

void RadioConfig::radioRx(void* const* buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (size_t i = 0; i < bsRadios[0].size(); i++) {
        void* const* buff = buffs + (i * 2);
        struct Radio* bsRadio = &bsRadios[0][i];
        SoapySDR::Device* dev = bsRadio->dev;
        dev->readStream(bsRadio->rxs, buff, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
    }
}

int RadioConfig::radioRx(size_t r /*radio id*/, void* const* buffs, long long& frameTime)
{
    int flags = 0;
    if (r < bsRadios[0].size()) {
        long long frameTimeNs = 0;
        struct Radio* bsRadio = &bsRadios[0][r];
        SoapySDR::Device* dev = bsRadio->dev;

        int ret = dev->readStream(bsRadio->rxs, buffs, _cfg->sampsPerSymbol, flags, frameTimeNs, 1000000);
        frameTime = frameTimeNs; //SoapySDR::timeNsToTicks(frameTimeNs, _rate);
#if DEBUG_RADIO
        if (ret != _cfg->sampsPerSymbol)
            std::cout << "readStream returned " << ret << " from radio " << r << ", Expected " << _cfg->sampsPerSymbol << std::endl;
        else
            std::cout << "radio " << r << "received " << ret << std::endl;
#endif
        return ret;
    }
    std::cout << "invalid radio id " << r << std::endl;
    return 0;
}

void RadioConfig::collectCSI(bool& adjust)
{
    int R = bsRadios[0].size();
    if (R < 2) {
        std::cout << "No need to sample calibrate with one Iris! skipping ..." << std::endl;
        return;
    }
    std::vector<std::vector<double>> pilot;
    //std::vector<std::complex<float>> pilot_cf32;
    std::vector<std::complex<int16_t>> pilot_cint16;
    int type = CommsLib::LTS_SEQ;
    int seqLen = 160; // Sequence length
    pilot = CommsLib::getSequence(seqLen, type);
    // double array to complex 32-bit float vector
    double max_abs = 0;
    for (int i = 0; i < seqLen; i++) {
        std::complex<double> samp(pilot[0][i], pilot[1][i]);
        max_abs = max_abs > std::abs(samp) ? max_abs : std::abs(samp);
    }

    pilot_cint16.resize(seqLen);
    for (int i = 0; i < seqLen; i++) {
        auto re = 0.25 * pilot[0][i] / max_abs * 32767;
        auto im = 0.25 * pilot[1][i] / max_abs * 32767;
        pilot_cint16[i] = std::complex<int16_t>((int16_t)re, (int16_t)im);
    }

    // Prepend/Append vectors with prefix/postfix number of null samples
    std::vector<std::complex<int16_t>> prefix_vec(_cfg->prefix, 0);
    std::vector<std::complex<int16_t>> postfix_vec(_cfg->sampsPerSymbol - _cfg->prefix - seqLen, 0);
    pilot_cint16.insert(pilot_cint16.begin(), prefix_vec.begin(), prefix_vec.end());
    pilot_cint16.insert(pilot_cint16.end(), postfix_vec.begin(), postfix_vec.end());
    // Transmitting from only one chain, create a null vector for chainB
    std::vector<std::complex<int16_t>> dummy_cint16(pilot_cint16.size(), 0);

    int ch = (_cfg->bsChannel == "B") ? 1 : 0;
    std::vector<void*> txbuff(2);
    if (ch == 0) {
        txbuff[0] = pilot_cint16.data(); //pilot_cf32.data();
        txbuff[1] = dummy_cint16.data(); //zero_vec.data();
    } else {
        txbuff[0] = dummy_cint16.data(); //zero_vec.data();
        txbuff[1] = pilot_cint16.data(); //pilot_cf32.data();
    }

    //std::vector<std::vector<std::complex<float>>> buff;
    std::vector<std::vector<std::complex<int16_t>>> buff;
    //int ant = _cfg->bsChannel.length();
    //int M = bsRadios[0].size() * ant;
    buff.resize(R * R);
    for (int i = 0; i < R; i++) {
        for (int j = 0; j < R; j++) {
            buff[i * R + j].resize(_cfg->sampsPerSymbol);
        }
    }

    std::vector<std::complex<int16_t>> dummyBuff0(_cfg->sampsPerSymbol);
    std::vector<std::complex<int16_t>> dummyBuff1(_cfg->sampsPerSymbol);
    std::vector<void*> dummybuffs(2);
    dummybuffs[0] = dummyBuff0.data();
    dummybuffs[1] = dummyBuff1.data();

    for (int i = 0; i < R; i++) {
        struct Radio* bsRadio = &bsRadios[0][i];
        SoapySDR::Device* dev = bsRadio->dev;
        RadioConfig::drain_buffers(dev, bsRadio->rxs, dummybuffs, _cfg->sampsPerSymbol);
    }

    for (int i = 0; i < R; i++) {
        struct Radio* bsRadio = &bsRadios[0][i];
        SoapySDR::Device* dev = bsRadio->dev;
        dev->setGain(SOAPY_SDR_TX, ch, "PAD", _cfg->calTxGain[ch]);
        dev->writeSetting("TDD_CONFIG", "{\"tdd_enabled\":false}");
        dev->writeSetting("TDD_MODE", "false");
        dev->activateStream(bsRadio->txs);
    }

    long long txTime(0);
    long long rxTime(0);
    for (int i = 0; i < R; i++) {
        // All write, or prepare to receive.
        for (int j = 0; j < R; j++) {
            struct Radio* bsRadio = &bsRadios[0][j];
            SoapySDR::Device* dev = bsRadio->dev;
            int flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
            if (j == i) {
                int ret = dev->writeStream(bsRadio->txs, txbuff.data(),
                    _cfg->sampsPerSymbol, flags, txTime, 1000000);
                if (ret < 0)
                    std::cout << "bad write" << std::endl;
            } else {
                int ret = dev->activateStream(bsRadio->rxs, flags, rxTime,
                    _cfg->sampsPerSymbol);
                if (ret < 0)
                    std::cout << "bad activate at node " << j << std::endl;
            }
        }

        radioTrigger();

        // All but one receive.
        int flags = 0;
        for (int j = 0; j < R; j++) {
            struct Radio* bsRadio = &bsRadios[0][j];
            SoapySDR::Device* dev = bsRadio->dev;
            if (j == i)
                continue;
            std::vector<void*> rxbuff(2);
            rxbuff[0] = buff[(i * R + j)].data();
            //rxbuff[1] = ant == 2 ? buff[(i*M+j)*ant+1].data() : dummyBuff.data();
            rxbuff[1] = dummyBuff0.data();
            int ret = dev->readStream(bsRadio->rxs, rxbuff.data(),
                _cfg->sampsPerSymbol, flags, rxTime, 1000000);
            if (ret < 0)
                std::cout << "bad read at node " << j << std::endl;
        }
    }

    int ref_ant = 0;
    int ref_offset = ref_ant == 0 ? 1 : 0;
    std::vector<int> offset(R);

    bool good_csi = true;
    for (int i = 0; i < R; i++) {
        std::vector<std::complex<double>> rx(_cfg->sampsPerSymbol);
        int k = ((i == ref_ant) ? ref_offset : ref_ant) * R + i;
        std::transform(buff[k].begin(), buff[k].end(), rx.begin(),
            [](std::complex<int16_t> cf) {
                return std::complex<double>(cf.real() / 32768.0, cf.imag() / 32768.0);
            });
        int peak = CommsLib::findLTS(rx, seqLen);
        offset[i] = peak < 128 ? 0 : peak - 128;
        //std::cout << i << " " << offset[i] << std::endl;
        if (offset[i] == 0)
            good_csi = false;

#if DEBUG_PLOT
        std::vector<double> rx_I(_cfg->sampsPerSymbol);
        std::transform(rx.begin(), rx.end(), rx_I.begin(),
            [](std::complex<double> cf) {
                return cf.real();
            });
        plt::figure_size(1200, 780);
        plt::plot(rx_I);
        plt::xlim(0, _cfg->sampsPerSymbol);
        plt::ylim(-1, 1);
        plt::title("Sample figure");
        plt::legend();
        plt::save(std::to_string(i) + ".png");
#endif
    }

    // adjusting trigger delays based on lts peak index
    adjust &= good_csi;
    if (adjust) {
        for (int i = 0; i < R; i++) {
            struct Radio* bsRadio = &bsRadios[0][i];
            SoapySDR::Device* dev = bsRadio->dev;
            // if offset[i] == 0, then good_csi is false and we never get here???
            int delta = (offset[i] == 0) ? 0 : offset[ref_offset] - offset[i];
            std::cout << "adjusting delay of node " << i << " by " << delta << std::endl;
            while (delta < 0) {
                dev->writeSetting("ADJUST_DELAYS", "-1");
                ++delta;
            }
            while (delta > 0) {
                dev->writeSetting("ADJUST_DELAYS", "1");
                --delta;
            }
        }
    }

    for (int i = 0; i < R; i++) {
        struct Radio* bsRadio = &bsRadios[0][i];
        SoapySDR::Device* dev = bsRadio->dev;
        dev->deactivateStream(bsRadio->txs);
        dev->deactivateStream(bsRadio->rxs);
        dev->setGain(SOAPY_SDR_TX, ch, "PAD", _cfg->txgain[ch]); //[0,30]
        RadioConfig::drain_buffers(dev, bsRadio->rxs, dummybuffs, _cfg->sampsPerSymbol);
    }
}

void RadioConfig::initAGC(SoapySDR::Device* iclSdr)
{
    /*
     * Initialize AGC parameters
     */
    // OBCH
    int en = (int)_cfg->clAgcEn;
    // AGC Core
    // Enable AGC Flag (set to 0 initially)
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_ENABLE_FLAG, 0);
    // Reset AGC Flag
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_RESET_FLAG, 1);
    // Saturation Threshold: 10300 about -6dBm
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_IQ_THRESH, 8000);
    // Number of samples needed to claim sat.
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_NUM_SAMPS_SAT, 3);
    // Threshold at which AGC stops
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_MAX_NUM_SAMPS_AGC, 10);
    // Gain settle takes about 20 samps(value=20)
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_WAIT_COUNT_THRESH, 20);
    // Drop gain at initial saturation detection
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_BIG_JUMP, 30);
    // Drop gain at subsequent sat. detections
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_SMALL_JUMP, 3);
    // RSSI Target for AGC: ideally around 14 (3.6GHz) or 27 (2.5GHz)
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_RSSI_TARGET, 14);
    // Disable
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_TEST_GAIN_SETTINGS, 0);
    // Clear AGC reset flag
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_RESET_FLAG, 0);
    // Enable AGC
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_ENABLE_FLAG, en);
    // Initialize gains to this value
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_GAIN_INIT,
        _cfg->clAgcGainInit);

    // Packet Detect Core
    // RSSI value at which Pkt is detected
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_THRESH, 500);
    // Number of samples needed to detect frame
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NUM_SAMPS, 5);
    // Enable packet detection flag
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_ENABLE, 1);
    // trigger first one if enabled
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NEW_FRAME, en);
    // clear
    iclSdr->writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NEW_FRAME, 0);
}

void RadioConfig::drain_buffers(SoapySDR::Device* ibsSdrs,
    SoapySDR::Stream* istream, std::vector<void*> buffs, int symSamp)
{
    /*
     *  "Drain" rx buffers during initialization
     *  Input:
     *      ibsSdrs - Current Iris board
     *      istream - Current SoapySDR stream
     *      buffs   - Vector to which we will write received IQ samples
     *      symSamp - Number of samples
     *
     *  Output:
     *      None
     */
    long long frameTime = 0;
    int flags = 0, r = 0, i = 0;
    while (r != -1) {
        r = ibsSdrs->readStream(istream, buffs.data(), symSamp, flags, frameTime, 0);
        i++;
    }
    //std::cout << "Number of reads needed to drain: " << i << std::endl;
}

void RadioConfig::sync_delays(int cellIdx)
{
    /*
     * Compute Sync Delays
     */
    baseRadio(cellIdx)->writeSetting("SYNC_DELAYS", "");
}

RadioConfig::~RadioConfig()
{
    if (_cfg->bsPresent) {
        if (!_cfg->hub_ids.empty()) {
            for (unsigned int i = 0; i < hubs.size(); i++)
                SoapySDR::Device::unmake(hubs[i]);
        }
    }
}

Radio::~Radio(void)
{
    dev->deactivateStream(rxs);
    dev->closeStream(rxs);
    dev->deactivateStream(txs);
    dev->closeStream(txs);
    SoapySDR::Device::unmake(dev);
}
