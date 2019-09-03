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
#include "include/matplotlibcpp.h"

#include <fstream>
#include <iostream>

namespace plt = matplotlibcpp;

RadioConfig::RadioConfig(Config* cfg)
    : _cfg(cfg)
{
    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    if (_cfg->bsPresent) {
        nBsSdrs.resize(_cfg->nCells);
        nBsAntennas.resize(_cfg->nCells);
        bsSdrs.resize(_cfg->nCells);
        bsTxStreams.resize(_cfg->nCells);
        bsRxStreams.resize(_cfg->nCells);

        for (unsigned int c = 0; c < _cfg->nCells; c++) {
            //load channels
            std::vector<RadioConfigContext> context;
            std::vector<size_t> channels;
            if (_cfg->bsChannel == "A")
                channels = { 0 };
            else if (_cfg->bsChannel == "B")
                channels = { 1 };
            else if (_cfg->bsSdrCh == 2)
                channels = { 0, 1 };
            else {
                std::cout << "Error! Supported number of channels 1 or 2, setting to 2!" << std::endl;
                _cfg->bsSdrCh = 2;
                channels = { 0, 1 };
            }

            this->nBsSdrs[c] = _cfg->nBsSdrs[c];
            this->nBsAntennas[c] = nBsSdrs[c] * _cfg->bsSdrCh;
            std::cout << this->nBsSdrs[c] << " radios in cell " << c << std::endl;
            //isUE = _cfg->isUE;
            if (!_cfg->hub_ids.empty()) {
                args["driver"] = "remote";
                args["timeout"] = "1000000";
                args["serial"] = _cfg->hub_ids.at(c);
                hubs.push_back(SoapySDR::Device::make(args));
            }

            int radioNum = this->nBsSdrs[c];
            bsSdrs[c].resize(radioNum);
            bsTxStreams[c].resize(radioNum);
            bsRxStreams[c].resize(radioNum);
            context.resize(radioNum);
            remainingJobs = radioNum;
            for (int i = 0; i < radioNum; i++) {
                //args["serial"] = _cfg->bs_sdr_ids[c][i];
                //args["timeout"] = "1000000";
                //bsSdrs[c].push_back(SoapySDR::Device::make(args));
                context[i].ptr = this;
                context[i].tid = i;
                context[i].cell = c;
#ifdef THREADED_INIT
                pthread_t init_thread_;
                if (pthread_create(&init_thread_, NULL, RadioConfig::initBSRadio, (void*)(&context[i])) != 0) {
                    perror("init thread create failed");
                    exit(0);
                }
#else
                RadioConfig::initBSRadio((void*)&context[i]);
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
        if (_cfg->clChannel == "A")
            channels = { 0 };
        else if (cfg->bsChannel == "B") {
            channels = { 1 };
            std::cout << "selecting channel " << cfg->bsChannel << "(" << std::to_string(channels[0]) << ")" << std::endl;
        } else if (_cfg->clSdrCh == 2)
            channels = { 0, 1 };
        else {
            std::cout << "Error! Supported number of channels 1 or 2, setting to 2!" << std::endl;
            _cfg->bsSdrCh = 2;
            channels = { 0, 1 };
        }
        nClSdrs = _cfg->nClSdrs;
        for (int i = 0; i < nClSdrs; i++) {
            auto device = SoapySDR::Device::make("serial=" + _cfg->cl_sdr_ids.at(i) + ",timeout=10000000");
            if (device == nullptr) {
                std::cerr << "No device!" << std::endl;
            }
            devs.push_back(device);
            SoapySDR::Kwargs info = device->getHardwareInfo();

            for (auto ch : { 0, 1 }) //channels)
            {
                device->setBandwidth(SOAPY_SDR_RX, ch, (1 + 2 * _cfg->bbf_ratio) * cfg->rate);
                device->setBandwidth(SOAPY_SDR_TX, ch, (1 + 2 * _cfg->bbf_ratio) * cfg->rate);

                device->setSampleRate(SOAPY_SDR_RX, ch, _cfg->rate);
                device->setSampleRate(SOAPY_SDR_TX, ch, _cfg->rate);

                //device->setFrequency(SOAPY_SDR_RX, ch, freq);
                //device->setFrequency(SOAPY_SDR_TX, ch, freq);
                device->setFrequency(SOAPY_SDR_RX, ch, "RF", _cfg->freq - _cfg->bbf_ratio * _cfg->rate);
                device->setFrequency(SOAPY_SDR_RX, ch, "BB", _cfg->bbf_ratio * _cfg->rate);
                device->setFrequency(SOAPY_SDR_TX, ch, "RF", _cfg->freq - _cfg->bbf_ratio * _cfg->rate);
                device->setFrequency(SOAPY_SDR_TX, ch, "BB", _cfg->bbf_ratio * _cfg->rate);

                // receive gains

                if (info["frontend"].find("CBRS") != std::string::npos) {
                    device->setGain(SOAPY_SDR_RX, ch, "ATTN", -12); //[-18,0]
                    device->setGain(SOAPY_SDR_RX, ch, "LNA1", 30); //[0,33]
                    if (cfg->freq < 3e9 && cfg->freq > 2e9)
                        device->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
                    else if (cfg->freq >= 3e9)
                        device->setGain(SOAPY_SDR_RX, ch, "LNA2", 14); //[0,14]
                }

                device->setGain(SOAPY_SDR_RX, ch, "LNA", ch != 0 ? _cfg->clRxgainB_vec[i] : _cfg->clRxgainA_vec[i]); //[0,30]
                device->setGain(SOAPY_SDR_RX, ch, "TIA", 0); //[0,12]
                device->setGain(SOAPY_SDR_RX, ch, "PGA", 0); //[-12,19]

                // transmit gains

                if (info["frontend"].find("CBRS") != std::string::npos) {
                    device->setGain(SOAPY_SDR_TX, ch, "ATTN", -6); //{-18,-12,-6,0}
                    if (info["frontend"].find("CBRSc") != std::string::npos) // on revC front-end, it is safe to turn on PA2
                    {
                        if (cfg->freq < 3e9 && cfg->freq > 2e9)
                            device->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //CBRS LO, [0|17]
                        else if (cfg->freq >= 3e9)
                            device->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //CBRS HI, [0|14]
                    } else
                        device->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //[0|17]
                }
                device->setGain(SOAPY_SDR_TX, ch, "IAMP", 12); //[-12,12]
                device->setGain(SOAPY_SDR_TX, ch, "PAD", ch != 0 ? _cfg->clTxgainB_vec[i] : _cfg->clTxgainA_vec[i]); //[0,52]
            }

            for (auto ch : channels) {
                //device->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
                //device->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
                device->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
            }

            device->writeRegister("IRIS30", RF_RST_REG, (1 << 29) | 1);
            device->writeRegister("IRIS30", RF_RST_REG, (1 << 29));
            device->writeRegister("IRIS30", RF_RST_REG, 0);

            auto rxStream = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, channels);
            auto txStream = device->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, channels);
            rxss.push_back(rxStream);
            txss.push_back(txStream);
        }
    }
    std::cout << "radio init done!" << std::endl;
}

void* RadioConfig::initBSRadio(void* in_context)
{
    RadioConfig* rc = ((RadioConfigContext*)in_context)->ptr;
    int i = ((RadioConfigContext*)in_context)->tid;
    int c = ((RadioConfigContext*)in_context)->cell;
    Config* cfg = rc->_cfg;

    //load channels
    std::vector<size_t> channels;
    if (cfg->bsChannel == "A")
        channels = { 0 };
    else if (cfg->bsChannel == "B")
        channels = { 1 };
    else if (cfg->bsSdrCh == 2)
        channels = { 0, 1 };
    else {
        std::cout << "Error! Supported number of channels 1 or 2, setting to 2!" << std::endl;
        cfg->bsSdrCh = 2;
        channels = { 0, 1 };
    }

    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    args["driver"] = "iris";
    args["timeout"] = "1000000";
    args["serial"] = cfg->bs_sdr_ids[0][i];
    rc->bsSdrs[c][i] = (SoapySDR::Device::make(args));
    //use the TRX antenna port for both tx and rx
    for (auto ch : channels)
        rc->bsSdrs[c][i]->setAntenna(SOAPY_SDR_RX, ch, "TRX");

    SoapySDR::Kwargs info = rc->bsSdrs[c][i]->getHardwareInfo();
    for (auto ch : { 0, 1 }) //channels)
    {
        rc->bsSdrs[c][i]->setBandwidth(SOAPY_SDR_RX, ch, (1 + 2 * cfg->bbf_ratio) * cfg->rate);
        rc->bsSdrs[c][i]->setBandwidth(SOAPY_SDR_TX, ch, (1 + 2 * cfg->bbf_ratio) * cfg->rate);

        rc->bsSdrs[c][i]->setSampleRate(SOAPY_SDR_RX, ch, cfg->rate);
        rc->bsSdrs[c][i]->setSampleRate(SOAPY_SDR_TX, ch, cfg->rate);

        rc->bsSdrs[c][i]->setFrequency(SOAPY_SDR_RX, ch, "RF", cfg->freq - cfg->bbf_ratio * cfg->rate);
        rc->bsSdrs[c][i]->setFrequency(SOAPY_SDR_RX, ch, "BB", cfg->bbf_ratio * cfg->rate);
        rc->bsSdrs[c][i]->setFrequency(SOAPY_SDR_TX, ch, "RF", cfg->freq - cfg->bbf_ratio * cfg->rate);
        rc->bsSdrs[c][i]->setFrequency(SOAPY_SDR_TX, ch, "BB", cfg->bbf_ratio * cfg->rate);

        // receive gains

        // front-end
        if (info["frontend"].find("CBRS") != std::string::npos) {
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -12); //[-18,0]
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "LNA1", 33); //[0,33]
            if (cfg->freq > 3e9)
                rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 14); //HI[0,14]
            else
                rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //LO[0,17]
        }
        if (info["frontend"].find("UHF") != std::string::npos) {
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "ATTN1", -6); //[-18,0]
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "ATTN2", -12); //[-18,0]
            //bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "LNA1", 30); //[0,33]
            //bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
        }

        // lime
        rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "LNA", ch != 0 ? cfg->rxgainB : cfg->rxgainA); //[0,30]
        rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "TIA", 0); //[0,12]
        rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "PGA", 0); //[-12,19]

        // transmit gains

        // front-end
        if (info["frontend"].find("CBRS") != std::string::npos && cfg->freq > 3e9) // CBRS HI
        {
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "ATTN", 0); //[-18,0] by 3
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PA1", 15); //[0|13.7] no bypass
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //[0|14]   can bypass
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PA3", 30); //[0|31]   no bypass
        }
        if (info["frontend"].find("CBRS") != std::string::npos && cfg->freq < 3e9 && cfg->freq > 2e9) // CBRS LO
        {
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "ATTN", -6); //[-18,0] by 3
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PA1", 14); //[0|14] no bypass
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //[0|17]   can bypass. Can cause saturation or PA damage!! DO NOT USE IF NOT SURE!!!
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PA3", 30); //[0|31.5]   no bypass
        }
        if (info["frontend"].find("UHF") != std::string::npos) {
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "ATTN", 0); //[-18,0] by 3
        }

        // lime
        rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "IAMP", 12); //[0,12]
        rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PAD", ch != 0 ? cfg->txgainB : cfg->txgainA); //[0,30]
    }

    for (auto ch : channels) {
        //bsSdrs[c][i]->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
        //bsSdrs[c][i]->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
        rc->bsSdrs[c][i]->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    }

    // resets the DATA_clk domain logic.
    rc->bsSdrs[c][i]->writeRegister("IRIS30", 48, (1 << 29) | 0x1);
    rc->bsSdrs[c][i]->writeRegister("IRIS30", 48, (1 << 29));
    rc->bsSdrs[c][i]->writeRegister("IRIS30", 48, 0);
    rc->bsRxStreams[c][i] = rc->bsSdrs[c][i]->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
    rc->bsTxStreams[c][i] = rc->bsSdrs[c][i]->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs);
    rc->remainingJobs--;
    return (NULL);
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
        for (int i = 0; i < this->nBsSdrs[0]; i++) {
            bsSdrs[0][i]->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
            bsSdrs[0][i]->writeSetting("TDD_MODE", "true");
            bsSdrs[0][i]->writeSetting("TDD_CONFIG", confString);
            // write beacons to FPGA buffers
            {
                if (!_cfg->beamsweep or nBsAntennas[0] == 1) {
                    if (i * _cfg->bsSdrCh == _cfg->beacon_ant && _cfg->bsChannel == "A")
                        bsSdrs[0][i]->writeRegisters("TX_RAM_A", 0, _cfg->beacon);
                    else if ((i * _cfg->bsSdrCh == _cfg->beacon_ant && _cfg->bsChannel == "B")
                        || (_cfg->bsSdrCh == 2 and i * 2u + 1 == _cfg->beacon_ant))
                        bsSdrs[0][i]->writeRegisters("TX_RAM_B", 0, _cfg->beacon);
                    else {
                        std::vector<unsigned> zeros(_cfg->sampsPerSymbol, 0);
                        bsSdrs[0][i]->writeRegisters("TX_RAM_A", 0, zeros);
                        bsSdrs[0][i]->writeRegisters("TX_RAM_B", 0, zeros);
                    }
                    bsSdrs[0][i]->writeRegister("RFCORE", 156, 0);
                } else // beamsweep
                {
                    std::vector<unsigned> beacon_weights(nBsAntennas[0]);
                    int hadamardLog = 8 * sizeof(nBsAntennas[0]) - __builtin_clz(nBsAntennas[0] - 1);
                    int hadamardSize = 1 << hadamardLog;
                    std::vector<std::vector<double>> hadamard_weights = CommsLib::getSequence(hadamardSize, CommsLib::HADAMARD);
                    if (_cfg->bsChannel != "B")
                        bsSdrs[0][i]->writeRegisters("TX_RAM_A", 0, _cfg->beacon);
                    if (_cfg->bsChannel != "A")
                        bsSdrs[0][i]->writeRegisters("TX_RAM_B", 0, _cfg->beacon);
                    //int residue = hadamardSize - nBsAntennas[0];
                    for (int j = 0; j < nBsAntennas[0]; j++)
                        beacon_weights[j] = (unsigned)hadamard_weights[i * _cfg->bsSdrCh][j];
                    if (_cfg->bsChannel != "B")
                        bsSdrs[0][i]->writeRegisters("TX_RAM_WGT_A", 0, beacon_weights);
                    if (_cfg->bsChannel == "B")
                        bsSdrs[0][i]->writeRegisters("TX_RAM_WGT_B", 0, beacon_weights);
                    if (_cfg->bsSdrCh == 2) {
                        for (int j = 0; j < nBsAntennas[0]; j++)
                            beacon_weights[j] = (unsigned)hadamard_weights[i * _cfg->bsSdrCh + 1][j];
                        bsSdrs[0][i]->writeRegisters("TX_RAM_WGT_B", 0, beacon_weights);
                    }
                    bsSdrs[0][i]->writeRegister("RFCORE", 156, nBsSdrs[0]);
                    bsSdrs[0][i]->writeRegister("RFCORE", 160, 1);
                }
            }
            bsSdrs[0][i]->activateStream(this->bsRxStreams[0][i], flags, 0);
            bsSdrs[0][i]->activateStream(this->bsTxStreams[0][i]);
            bsSdrs[0][i]->setHardwareTime(0, "TRIGGER");
        }
    }

    if (_cfg->clPresent) {
        int ueTrigOffset = 505; //_cfg->prefix + 256 + _cfg->postfix + 17 + _cfg->prefix;
        int sf_start = ueTrigOffset / _cfg->sampsPerSymbol;
        int sp_start = ueTrigOffset % _cfg->sampsPerSymbol;

        std::vector<std::string> tddSched;
        tddSched.resize(nClSdrs);
        for (int i = 0; i < nClSdrs; i++) {
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

        for (int i = 0; i < nClSdrs; i++) {
            auto device = devs[i];
            device->writeRegister("IRIS30", CORR_CONF, 0x1);
            for (int k = 0; k < 128; k++)
                device->writeRegister("ARGCOE", k * 4, 0);
            usleep(100000);

            device->writeRegister("IRIS30", 64, 1); // reset faros_corr
            device->writeRegister("IRIS30", 64, 0); // unreset faros_corr
            device->writeRegister("IRIS30", 92, 1); // threshold is left-shifted by this many bits

            for (int k = 0; k < 128; k++)
                device->writeRegister("ARGCOE", k * 4, _cfg->coeffs[k]);

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
            device->writeSetting("TDD_CONFIG", confString);

            device->setHardwareTime(SoapySDR::ticksToTimeNs((sf_start << 16) | sp_start, _cfg->rate), "TRIGGER");
            device->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
            device->writeSetting("TDD_MODE", "true");
            // write beacons to FPGA buffers
            if (_cfg->bsChannel != "B")
                device->writeRegisters("TX_RAM_A", 0, _cfg->pilot);
            if (_cfg->bsChannel == "B")
                device->writeRegisters("TX_RAM_B", 0, _cfg->pilot);
            if (_cfg->clSdrCh == 2)
                device->writeRegisters("TX_RAM_B", 2048, _cfg->pilot);

            device->activateStream(rxss[i]);
            device->activateStream(txss[i]);

            if (_cfg->bsChannel != "B") // A or AB
                device->writeRegister("IRIS30", CORR_CONF, 0x11);
            else
                device->writeRegister("IRIS30", CORR_CONF, 0x31);
        }
    }
    std::cout << "Done with frame configuration!" << std::endl;
}

void RadioConfig::radioStart()
{
    if (_cfg->bsPresent) {
        if (hubs.empty()) {
            bsSdrs[0][0]->writeSetting("TRIGGER_GEN", "");
        } else {
            hubs[0]->writeSetting("TRIGGER_GEN", "");
        }
    }
}

void RadioConfig::readSensors()
{
    if (_cfg->bsPresent) {
        for (int i = 0; i < nBsSdrs[0]; i++) {
            std::cout << "TEMPs on Iris " << i << std::endl;
            std::cout << "ZYNQ_TEMP: " << bsSdrs[0][i]->readSensor("ZYNQ_TEMP") << std::endl;
            std::cout << "LMS7_TEMP  : " << bsSdrs[0][i]->readSensor("LMS7_TEMP") << std::endl;
            std::cout << "FE_TEMP  : " << bsSdrs[0][i]->readSensor("FE_TEMP") << std::endl;
            std::cout << "TX0 TEMP  : " << bsSdrs[0][i]->readSensor(SOAPY_SDR_TX, 0, "TEMP") << std::endl;
            std::cout << "TX1 TEMP  : " << bsSdrs[0][i]->readSensor(SOAPY_SDR_TX, 1, "TEMP") << std::endl;
            std::cout << "RX0 TEMP  : " << bsSdrs[0][i]->readSensor(SOAPY_SDR_RX, 0, "TEMP") << std::endl;
            std::cout << "RX1 TEMP  : " << bsSdrs[0][i]->readSensor(SOAPY_SDR_RX, 1, "TEMP") << std::endl;
            std::cout << std::endl;
        }
    }
}

void RadioConfig::radioStop()
{
    if (_cfg->bsPresent) {
        for (int i = 0; i < nBsSdrs[0]; i++) {
            // write schedule
            for (unsigned int j = 0; j < _cfg->frames.size(); j++) {
                for (int k = 0; k < _cfg->symbolsPerFrame; k++) // symnum <= 256
                {
                    bsSdrs[0][i]->writeRegister("RFCORE", SCH_ADDR_REG, j * 256 + k);
                    bsSdrs[0][i]->writeRegister("RFCORE", SCH_MODE_REG, 0);
                }
            }
            bsSdrs[0][i]->writeSetting("TDD_MODE", "false");
            bsSdrs[0][i]->writeRegister("IRIS30", 48, (1 << 29) | 0x1);
            bsSdrs[0][i]->writeRegister("IRIS30", 48, (1 << 29));
            bsSdrs[0][i]->writeRegister("IRIS30", 48, 0);
        }
    }
    if (_cfg->clPresent) {
        for (int i = 0; i < nClSdrs; i++) {
            auto device = devs[i];
            device->writeRegister("IRIS30", CORR_CONF, 0);
            std::cout << "device " << i << " T=" << std::hex << SoapySDR::timeNsToTicks(device->getHardwareTime(""), _cfg->rate) << std::dec << std::endl;
            for (int i = 0; i < _cfg->symbolsPerFrame; i++) {
                device->writeRegister("RFCORE", SCH_ADDR_REG, i);
                device->writeRegister("RFCORE", SCH_MODE_REG, 0);
            }
            device->writeSetting("TDD_MODE", "false");
            device->writeRegister("IRIS30", RF_RST_REG, (1 << 29) | 1);
            device->writeRegister("IRIS30", RF_RST_REG, (1 << 29));
            device->writeRegister("IRIS30", RF_RST_REG, 0);
            //SoapySDR::Device::unmake(device);
        }
    }
}

void RadioConfig::radioTx(const void* const* buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (int i = 0; i < nBsSdrs[0]; i++) {
        bsSdrs[0][i]->writeStream(this->bsTxStreams[0][i], buffs, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
    }
}

int RadioConfig::radioTx(int r /*radio id*/, const void* const* buffs, int flags, long long& frameTime)
{
    if (flags == 1)
        flags = SOAPY_SDR_HAS_TIME;
    else if (flags == 2)
        flags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
    //long long frameTime(0);
    int w = bsSdrs[0][r]->writeStream(this->bsTxStreams[0][r], buffs, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
#if DEBUG_RADIO
    size_t chanMask;
    long timeoutUs(0);
    int s = bsSdrs[0][r]->readStreamStatus(this->bsTxStreams[0][r], chanMask, flags, frameTime, timeoutUs);
    std::cout << "radio " << r << " tx returned " << w << " and status " << s << std::endl;
#endif
    return w;
}

void RadioConfig::radioRx(void* const* buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (int i = 0; i < nBsSdrs[0]; i++) {
        void* const* buff = buffs + (i * 2);
        bsSdrs[0][i]->readStream(this->bsRxStreams[0][i], buff, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
    }
}

int RadioConfig::radioRx(int r /*radio id*/, void* const* buffs, long long& frameTime)
{
    int flags = 0;
    if (r < nBsSdrs[0]) {
        long long frameTimeNs = 0;
        int ret = bsSdrs[0][r]->readStream(this->bsRxStreams[0][r], buffs, _cfg->sampsPerSymbol, flags, frameTimeNs, 1000000);
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
    int R = nBsSdrs[0];
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

    for (int i = 0; i < seqLen; i++) {
        pilot[0][i] = 0.25 * pilot[0][i] / max_abs;
        pilot[1][i] = 0.25 * pilot[1][i] / max_abs;
        pilot_cint16.push_back(std::complex<int16_t>((int16_t)(pilot[0][i] * 32767), (int16_t)(pilot[1][i] * 32767)));
    }

    // Prepend/Append vectors with prefix/postfix number of null samples
    std::vector<std::complex<int16_t>> prefix_vec(_cfg->prefix, 0);
    std::vector<std::complex<int16_t>> postfix_vec(_cfg->sampsPerSymbol - _cfg->prefix - seqLen, 0);
    pilot_cint16.insert(pilot_cint16.begin(), prefix_vec.begin(), prefix_vec.end());
    pilot_cint16.insert(pilot_cint16.end(), postfix_vec.begin(), postfix_vec.end());
    // Transmitting from only one chain, create a null vector for chainB
    std::vector<std::complex<int16_t>> dummy_cint16(pilot_cint16.size(), 0);

    std::vector<void*> txbuff0(2);
    txbuff0[0] = pilot_cint16.data(); //pilot_cf32.data();
    txbuff0[1] = dummy_cint16.data(); //zero_vec.data();

    std::vector<void*> txbuff1(2);
    txbuff1[0] = dummy_cint16.data(); //zero_vec.data();
    txbuff1[1] = pilot_cint16.data(); //pilot_cf32.data();

    //std::vector<std::vector<std::complex<float>>> buff;
    std::vector<std::vector<std::complex<int16_t>>> buff;
    //int ant = _cfg->bsSdrCh;
    //int M = nBsSdrs[0] * ant;
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

    for (int i = 0; i < R; i++)
        RadioConfig::drain_buffers(bsSdrs[0][i], this->bsRxStreams[0][i], dummybuffs, _cfg->sampsPerSymbol);

    int ch = (_cfg->bsChannel == "B") ? 1 : 0;
    for (int i = 0; i < R; i++) {
        bsSdrs[0][i]->setGain(SOAPY_SDR_TX, ch, "PAD", ch != 0 ? _cfg->calTxGainB : _cfg->calTxGainA);
        bsSdrs[0][i]->writeSetting("TDD_CONFIG", "{\"tdd_enabled\":false}");
        bsSdrs[0][i]->writeSetting("TDD_MODE", "false");
        bsSdrs[0][i]->activateStream(this->bsTxStreams[0][i]);
    }

    long long txTime(0);
    long long rxTime(0);
    for (int i = 0; i < R; i++) {

        auto ref_sdr = bsSdrs[0][i];
        int tx_flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
        int ret = ref_sdr->writeStream(this->bsTxStreams[0][i], ch != 0 ? txbuff1.data() : txbuff0.data(), _cfg->sampsPerSymbol, tx_flags, txTime, 1000000);
        if (ret < 0)
            std::cout << "bad write\n";
        for (int j = 0; j < R; j++) {
            if (j == i)
                continue;
            int rx_flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
            ret = bsSdrs[0][j]->activateStream(this->bsRxStreams[0][j], rx_flags, rxTime, _cfg->sampsPerSymbol);
            if (ret < 0)
                std::cout << "bad activate at node " << j << std::endl;
        }

        if (hubs.empty())
            bsSdrs[0][0]->writeSetting("TRIGGER_GEN", "");
        else
            hubs[0]->writeSetting("TRIGGER_GEN", "");

        int flags = 0;
        for (int j = 0; j < R; j++) {
            if (j == i)
                continue;
            std::vector<void*> rxbuff(2);
            rxbuff[0] = buff[(i * R + j)].data();
            //rxbuff[1] = ant == 2 ? buff[(i*M+j)*ant+1].data() : dummyBuff.data();
            rxbuff[1] = dummyBuff0.data();
            ret = bsSdrs[0][j]->readStream(this->bsRxStreams[0][j], rxbuff.data(), _cfg->sampsPerSymbol, flags, rxTime, 1000000);
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
        if (i == ref_ant) {
            std::transform(buff[ref_offset * R + i].begin(), buff[ref_offset * R + i].end(), rx.begin(), [](std::complex<int16_t> cf) { return std::complex<double>(cf.real() / 32768.0, cf.imag() / 32768.0); });
        } else {
            std::transform(buff[ref_ant * R + i].begin(), buff[ref_ant * R + i].end(), rx.begin(), [](std::complex<int16_t> cf) { return std::complex<double>(cf.real() / 32768.0, cf.imag() / 32768.0); });
        }

        int peak = CommsLib::findLTS(rx, seqLen);
        offset[i] = peak < 128 ? 0 : peak - 128;
        //std::cout << i << " " << offset[i] << std::endl;
        if (offset[i] == 0)
            good_csi = false;

#if DEBUG_PLOT
        std::vector<double> rx_I(_cfg->sampsPerSymbol);
        std::transform(rx.begin(), rx.end(), rx_I.begin(), [](std::complex<double> cf) { return cf.real(); });

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
    if (adjust) {
        if (good_csi) {
            for (int i = 0; i < R; i++) {
                int delta = (offset[i] == 0) ? 0 : offset[ref_offset] - offset[i];
                std::cout << "adjusting delay of node " << i << " by " << delta << std::endl;
                int iter = delta < 0 ? -delta : delta;
                for (int j = 0; j < iter; j++) {
                    if (delta < 0)
                        bsSdrs[0][i]->writeSetting("ADJUST_DELAYS", "-1");
                    else
                        bsSdrs[0][i]->writeSetting("ADJUST_DELAYS", "1");
                }
            }
        } else
            adjust = false;
    }

    for (int i = 0; i < R; i++) {
        bsSdrs[0][i]->deactivateStream(this->bsTxStreams[0][i]);
        bsSdrs[0][i]->deactivateStream(this->bsRxStreams[0][i]);
        bsSdrs[0][i]->setGain(SOAPY_SDR_TX, ch, "PAD", ch != 0 ? _cfg->txgainB : _cfg->txgainA); //[0,30]
    }

    for (int i = 0; i < R; i++)
        RadioConfig::drain_buffers(bsSdrs[0][i], this->bsRxStreams[0][i], dummybuffs, _cfg->sampsPerSymbol);
}

void RadioConfig::drain_buffers(SoapySDR::Device* ibsSdrs, SoapySDR::Stream* istream, std::vector<void*> buffs, int symSamp)
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
    if (!hubs.empty()) {
        hubs[cellIdx]->writeSetting("SYNC_DELAYS", "");
    } else {
        bsSdrs[cellIdx][0]->writeSetting("SYNC_DELAYS", "");
    }
}

RadioConfig::~RadioConfig()
{
    if (_cfg->bsPresent) {
        if (!_cfg->hub_ids.empty()) {
            for (unsigned int i = 0; i < hubs.size(); i++)
                SoapySDR::Device::unmake(hubs[i]);
        }
        for (int i = 0; i < nBsSdrs[0]; i++) {
            bsSdrs[0][i]->deactivateStream(this->bsRxStreams[0][i]);
            bsSdrs[0][i]->deactivateStream(this->bsTxStreams[0][i]);
            bsSdrs[0][i]->closeStream(this->bsRxStreams[0][i]);
            bsSdrs[0][i]->closeStream(this->bsTxStreams[0][i]);
        }
        for (int i = 0; i < nBsSdrs[0]; i++) {
            SoapySDR::Device::unmake(bsSdrs[0][i]);
        }
    }
    if (_cfg->clPresent) {
        for (int i = 0; i < nClSdrs; i++) {
            auto device = devs[i];
            device->deactivateStream(rxss[i]);
            device->deactivateStream(txss[i]);

            device->closeStream(rxss[i]);
            device->closeStream(txss[i]);
            SoapySDR::Device::unmake(device);
        }
    }
}
