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

#include <iostream>
#include <fstream>

RadioConfig::RadioConfig(Config *cfg):
    _cfg(cfg)
{
    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    if (_cfg->bsPresent)
    {
        nBsSdrs.resize(_cfg->nCells);
        nBsAntennas.resize(_cfg->nCells);
        bsSdrs.resize(_cfg->nCells);
        bsTxStreams.resize(_cfg->nCells);
        bsRxStreams.resize(_cfg->nCells);

        for (int c = 0; c < _cfg->nCells; c++)
        {
            //load channels
            std::vector<size_t> channels;
            if (_cfg->bsChannel == "A") channels = {0};
            else if (_cfg->bsChannel == "B") channels = {1};
            else if (_cfg->bsSdrCh == 2) channels = {0, 1};
            else
            {
                std::cout << "Error! Supported number of channels 1 or 2, setting to 2!" << std::endl;
                _cfg->bsSdrCh = 2;
                channels = {0, 1};
            }

            this->nBsSdrs[c] = _cfg->nBsSdrs[c];
            this->nBsAntennas[c] = nBsSdrs[c] * _cfg->bsSdrCh;
            std::cout << this->nBsSdrs[c] << " radios in cell " << c << std::endl;
            //isUE = _cfg->isUE;
            if (_cfg->hub_ids.size() > 0)
            {
                args["serial"] = _cfg->hub_ids.at(c);
                hubs.push_back(SoapySDR::Device::make(args)); 
            }

            int radioNum = this->nBsSdrs[c];
            bsSdrs[c].resize(radioNum);
            bsTxStreams[c].resize(radioNum);
            bsRxStreams[c].resize(radioNum);
            context = new RadioConfigContext[radioNum];
            remainingJobs = radioNum;
            for (int i = 0; i < radioNum; i++)
            {
                //args["serial"] = _cfg->bs_sdr_ids[c][i];
                //args["timeout"] = "1000000";
                //bsSdrs[c].push_back(SoapySDR::Device::make(args));
                context[i].ptr = this;
                context[i].tid = i;
                context[i].cell = c;
#ifdef THREADED_INIT
                pthread_t init_thread_;
                if(pthread_create( &init_thread_, NULL, RadioConfig::initBSRadio, (void *)(&context[i])) != 0)
                {
                    perror("init thread create failed");
                    exit(0);
                }
#else
                RadioConfig::initBSRadio((void *)&context[i]);
#endif
            }

            while(remainingJobs>0);

        }
    }
    if (_cfg->clPresent)
    {
        //load channels
        std::vector<size_t> channels;
        if (_cfg->clChannel == "A") channels = {0};
        else if (_cfg->clChannel == "B") channels = {1};
        else if (_cfg->clSdrCh == 2) channels = {0, 1};
        else
        {
            std::cout << "Error! Supported number of channels 1 or 2, setting to 2!" << std::endl;
            _cfg->bsSdrCh = 2;
            channels = {0, 1};
        }
        nClSdrs = _cfg->nClSdrs;
        for(int i = 0; i < nClSdrs; i++)
        {
            auto device = SoapySDR::Device::make("serial="+_cfg->cl_sdr_ids.at(i)+",timeout=10000000");
            if (device == nullptr)
            {
                std::cerr << "No device!" << std::endl;
            }
            devs.push_back(device);
            SoapySDR::Kwargs info = device->getHardwareInfo();

            for (auto ch : channels)
            {
                device->setSampleRate(SOAPY_SDR_RX, ch, _cfg->rate);
                device->setSampleRate(SOAPY_SDR_TX, ch, _cfg->rate);

                //device->setFrequency(SOAPY_SDR_RX, ch, freq);  
                //device->setFrequency(SOAPY_SDR_TX, ch, freq); 
                device->setFrequency(SOAPY_SDR_RX, ch, "RF", _cfg->freq-.75*_cfg->rate);
                device->setFrequency(SOAPY_SDR_RX, ch, "BB", .75*_cfg->rate);
                device->setFrequency(SOAPY_SDR_TX, ch, "RF", _cfg->freq-.75*_cfg->rate);
                device->setFrequency(SOAPY_SDR_TX, ch, "BB", .75*_cfg->rate);
 
                // receive gains

                if (info["frontend"].find("CBRS") != std::string::npos)
                {
                    device->setGain(SOAPY_SDR_RX, ch, "ATTN", -12); //[-18,0]  
                    device->setGain(SOAPY_SDR_RX, ch, "LNA1", 30); //[0,33]
                    if ( cfg->freq < 3e9 && cfg->freq > 2e9) 
                        device->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
                    else if (cfg->freq >= 3e9) 
                        device->setGain(SOAPY_SDR_RX, ch, "LNA2", 14); //[0,14]
                }

		device->setGain(SOAPY_SDR_RX, ch, "LNA", ch ? _cfg->clRxgainB_vec[i] : _cfg->clRxgainA_vec[i]);  //[0,30]
                device->setGain(SOAPY_SDR_RX, ch, "TIA", 0);  //[0,12]
                device->setGain(SOAPY_SDR_RX, ch, "PGA", 0);  //[-12,19]

                // transmit gains

                if (info["frontend"].find("CBRS") != std::string::npos)
                {
                    device->setGain(SOAPY_SDR_TX, ch, "ATTN", -6);           //{-18,-12,-6,0}
                    if (info["frontend"].find("CBRSc") != std::string::npos) // on revC front-end, it is safe to turn on PA2
                        if ( cfg->freq < 3e9 && cfg->freq > 2e9) 
                            device->setGain(SOAPY_SDR_TX, ch, "PA2", 0);     //CBRS LO, [0|17]
                        else if (cfg->freq >= 3e9) 
                            device->setGain(SOAPY_SDR_TX, ch, "PA2", 0);     //CBRS HI, [0|14]
                    else
                        device->setGain(SOAPY_SDR_TX, ch, "PA2", 0);         //[0|17]
                }
                device->setGain(SOAPY_SDR_TX, ch, "IAMP", 12);                //[-12,12] 
		device->setGain(SOAPY_SDR_TX, ch, "PAD", ch ? _cfg->clTxgainB_vec[i] : _cfg->clTxgainA_vec[i]);       //[0,52]
            }

            for (auto ch : channels)
            {
                //device->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
                //device->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
                device->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
            }

            device->writeRegister("IRIS30", RF_RST_REG, (1<<29) | 1);
            device->writeRegister("IRIS30", RF_RST_REG, (1<<29));
            device->writeRegister("IRIS30", RF_RST_REG, 0);

            auto rxStream = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, channels);
            auto txStream = device->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, channels);
            rxss.push_back(rxStream);
            txss.push_back(txStream);
        }
    }
    std::cout << "radio init done!" << std::endl;
}

void *RadioConfig::initBSRadio(void *in_context)
{
    RadioConfig* rc = ((RadioConfigContext *)in_context)->ptr;
    int i = ((RadioConfigContext *)in_context)->tid;
    int c = ((RadioConfigContext *)in_context)->cell;
    Config *cfg = rc->_cfg;

    //load channels
    std::vector<size_t> channels;
    if (cfg->bsChannel == "A") channels = {0};
    else if (cfg->bsChannel == "B") channels = {1};
    else if (cfg->bsSdrCh == 2) channels = {0, 1};
    else
    {
        std::cout << "Error! Supported number of channels 1 or 2, setting to 2!" << std::endl;
        cfg->bsSdrCh = 2;
        channels = {0, 1};
    }

    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    args["serial"] = cfg->bs_sdr_ids[0][i];
    args["timeout"] = "1000000";
    rc->bsSdrs[c][i] = (SoapySDR::Device::make(args));
    //use the TRX antenna port for both tx and rx
    for (auto ch : channels) rc->bsSdrs[c][i]->setAntenna(SOAPY_SDR_RX, ch, "TRX");

    SoapySDR::Kwargs info = rc->bsSdrs[c][i]->getHardwareInfo();
    for (auto ch : channels)
    {
        //bsSdrs[c][i]->setBandwidth(SOAPY_SDR_RX, ch, 30e6);
        //bsSdrs[c][i]->setBandwidth(SOAPY_SDR_TX, ch, 30e6);

        rc->bsSdrs[c][i]->setSampleRate(SOAPY_SDR_RX, ch, cfg->rate);
        rc->bsSdrs[c][i]->setSampleRate(SOAPY_SDR_TX, ch, cfg->rate);

        rc->bsSdrs[c][i]->setFrequency(SOAPY_SDR_RX, ch, "RF", cfg->freq-.75*cfg->rate);
        rc->bsSdrs[c][i]->setFrequency(SOAPY_SDR_RX, ch, "BB", .75*cfg->rate);
        rc->bsSdrs[c][i]->setFrequency(SOAPY_SDR_TX, ch, "RF", cfg->freq-.75*cfg->rate);
        rc->bsSdrs[c][i]->setFrequency(SOAPY_SDR_TX, ch, "BB", .75*cfg->rate);


        // receive gains

        // front-end 
        if (info["frontend"].find("CBRS") != std::string::npos)
        {
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -12); //[-18,0]  
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "LNA1", 33); //[0,33]
            if (cfg->freq > 3e9)
                rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 14); //HI[0,14]
            else 
                rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //LO[0,17]
        }
        if (info["frontend"].find("UHF") != std::string::npos)
        {
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "ATTN1", -6); //[-18,0]  
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "ATTN2", -12); //[-18,0]  
            //bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "LNA1", 30); //[0,33]
            //bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
        }

        // lime  
        rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "LNA", ch ? cfg->rxgainB : cfg->rxgainA);  //[0,30]
        rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "TIA", 0);  //[0,12]
        rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "PGA", 0);  //[-12,19]


        // transmit gains

        // front-end 
        if (info["frontend"].find("CBRS") != std::string::npos && cfg->freq > 3e9) // CBRS HI
        {
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "ATTN", 0);  //[-18,0] by 3
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PA1", 15);  //[0|13.7] no bypass
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PA2", 0);   //[0|14]   can bypass
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PA3", 30);  //[0|31]   no bypass
        }
        if (info["frontend"].find("CBRS") != std::string::npos && cfg->freq < 3e9 && cfg->freq > 2e9) // CBRS LO
        {
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "ATTN", -6);  //[-18,0] by 3
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PA1", 14);  //[0|14] no bypass
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PA2", 0);   //[0|17]   can bypass. Can cause saturation or PA damage!! DO NOT USE IF NOT SURE!!!
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PA3", 30);  //[0|31.5]   no bypass
        }
        if (info["frontend"].find("UHF") != std::string::npos)
        {
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "ATTN", 0);  //[-18,0] by 3
        }

        // lime 
        rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "IAMP", 12);     //[0,12] 
        rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PAD", ch ? cfg->txgainB : cfg->txgainA);  //[0,30]

    }

    for (auto ch : channels)
    {
        //bsSdrs[c][i]->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
        //bsSdrs[c][i]->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
        rc->bsSdrs[c][i]->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    }

    // resets the DATA_clk domain logic. 
    rc->bsSdrs[c][i]->writeRegister("IRIS30", 48, (1<<29) | 0x1);
    rc->bsSdrs[c][i]->writeRegister("IRIS30", 48, (1<<29));
    rc->bsSdrs[c][i]->writeRegister("IRIS30", 48, 0);
    rc->bsRxStreams[c][i] = rc->bsSdrs[c][i]->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
    rc->bsTxStreams[c][i] = rc->bsSdrs[c][i]->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs);
    rc->remainingJobs--;

}

void RadioConfig::radioStart()
{
    int flags = 0;
    if (_cfg->bsPresent)
    {

        std::vector<std::string> _tddSched;
        _tddSched.resize(_cfg->framePeriod);
        for (int f = 0; f < _cfg->framePeriod; f++)
        {
            _tddSched[f] = _cfg->frames[f];
            for (size_t s =0; s < _cfg->frames[f].size(); s++)
            {
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
        confString +="\"symbol_size\":"+std::to_string(_cfg->sampsPerSymbol);
        confString +=",\"frames\":[";
        for (int f = 0; f < _cfg->framePeriod; f++)
            confString += (f == _cfg->framePeriod - 1) ? "\""+_tddSched[f]+"\"" : "\""+_tddSched[f]+"\",";
        confString +="]}";
        std::cout << confString << std::endl;
#endif
        for (int i = 0; i < this->nBsSdrs[0]; i++)
        {
            bsSdrs[0][i]->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
            bsSdrs[0][i]->writeSetting("TDD_MODE", "true");
            bsSdrs[0][i]->writeSetting("TDD_CONFIG", confString);
            // write beacons to FPGA buffers
            {
                if (!_cfg->beamsweep or nBsAntennas[0] == 1)
                {
                    if (i*_cfg->bsSdrCh == _cfg->beacon_ant && _cfg->bsChannel == "A")
                        bsSdrs[0][i]->writeRegisters("TX_RAM_A", 0, _cfg->beacon);
                    else if ((i*_cfg->bsSdrCh == _cfg->beacon_ant && _cfg->bsChannel == "B")
                              || (_cfg->bsSdrCh == 2 and i*2+1 == _cfg->beacon_ant))
                        bsSdrs[0][i]->writeRegisters("TX_RAM_B", 0, _cfg->beacon);
                    else 
                    {
                        std::vector<unsigned> zeros(_cfg->sampsPerSymbol,0);
                        bsSdrs[0][i]->writeRegisters("TX_RAM_A", 0, zeros);
                        bsSdrs[0][i]->writeRegisters("TX_RAM_B", 0, zeros);
                    }
                    bsSdrs[0][i]->writeRegister("RFCORE", 156, 0);
                } 
                else // beamsweep
                {
                    std::vector<unsigned> beacon_weights(nBsAntennas[0]);
                    int hadamardSize =  int(pow(2,ceil(log2(nBsAntennas[0]))));
                    std::vector<std::vector<double>> hadamard_weights = CommsLib::getSequence(hadamardSize, CommsLib::HADAMARD);
                    if (_cfg->bsChannel != "B")
                        bsSdrs[0][i]->writeRegisters("TX_RAM_A", 0, _cfg->beacon);
                    if (_cfg->bsChannel != "A")
                        bsSdrs[0][i]->writeRegisters("TX_RAM_B", 0, _cfg->beacon);
                    int residue = int(pow(2,ceil(log2(nBsAntennas[0]))))-nBsAntennas[0];
                    for (int j = 0; j < nBsAntennas[0]; j++) beacon_weights[j] = (unsigned)hadamard_weights[i*_cfg->bsSdrCh][j];
                    if (_cfg->bsChannel != "B")
                        bsSdrs[0][i]->writeRegisters("TX_RAM_WGT_A", 0, beacon_weights);
                    if (_cfg->bsChannel == "B")
                        bsSdrs[0][i]->writeRegisters("TX_RAM_WGT_B", 0, beacon_weights);
                    if (_cfg->bsSdrCh == 2)
                    {
                        for (int j = 0; j < nBsAntennas[0]; j++) beacon_weights[j] = (unsigned)hadamard_weights[i*_cfg->bsSdrCh+1][j];
                        bsSdrs[0][i]->writeRegisters("TX_RAM_WGT_B", 0, beacon_weights);
                    }
                    bsSdrs[0][i]->writeRegister("RFCORE", 156, nBsSdrs[0]);
                    bsSdrs[0][i]->writeRegister("RFCORE", 160, 1);
                }
            }
            bsSdrs[0][i]->activateStream(this->bsRxStreams[0][i], flags, 0);
            bsSdrs[0][i]->activateStream(this->bsTxStreams[0][i]);
        }
    }

    if (_cfg->clPresent)
    {
        int ueTrigOffset = _cfg->prefix + 256 + _cfg->postfix + 17 + _cfg->prefix;
        int sf_start = ueTrigOffset/_cfg->sampsPerSymbol;
        int sp_start = ueTrigOffset%_cfg->sampsPerSymbol;

        std::vector<std::string> tddSched;
        tddSched.resize(nClSdrs);
        for (int i = 0; i < nClSdrs; i++)
        {
            tddSched[i] = _cfg->clFrames[i];
            for (size_t s = 0; s < _cfg->clFrames[i].size(); s++)
            {
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
 
        for(int i = 0; i < nClSdrs; i++)
        {
            auto device = devs[i];
            device->writeRegister("IRIS30", CORR_CONF, 0x1);
            for (int k = 0; k < 128; k++)
                device->writeRegister("ARGCOE", k*4, 0);
            usleep(100000);
#ifdef NEWCORR
            device->writeRegister("IRIS30", 64, 1); // reset faros_corr
            device->writeRegister("IRIS30", 64, 0); // unreset faros_corr
            device->writeRegister("IRIS30", 92, 1); // threshold is left-shifted by this many bits
#else
            device->writeRegister("ARGCOR", CORR_THRESHOLD, 128);
            device->writeRegister("ARGCOR", CORR_RST, 1);
            device->writeRegister("ARGCOR", CORR_RST, 0);
#endif
            for (int k = 0; k < 128; k++)
                device->writeRegister("ARGCOE", k*4, _cfg->coeffs[k]);

#ifdef JSON
            json conf;
            conf["tdd_enabled"] = true;
#ifdef NEWCORR
            conf["frame_mode"] = _cfg->frame_mode;
            int max_frame_ = (int)(2.0 / ((_cfg->sampsPerSymbol * _cfg->symbolsPerFrame) / _cfg->rate));
            conf["max_frame"] = max_frame_; 
            //std::cout << "max_frames for client " << i << " is " << max_frame_ << std::endl;
#else
            conf["trigger_out"] = true; 
            conf["wait_trigger"] = true; 
#endif
            conf["frames"] = json::array();
            conf["frames"].push_back(tddSched[i]);
            conf["symbol_size"] = _cfg->sampsPerSymbol; 
            std::string confString = conf.dump();
#else
            std::string confString ="{\"tdd_enabled\":true,\"frame_mode\":"+_cfg->frame_mode+",";
            confString +="\"symbol_size\":"+std::to_string(_cfg->sampsPerSymbol);
            confString +=",\"frames\":[\""+tddSched[i]+"\"]}";
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

    // "Drain" rx buffers before sounding
    int cellIdx = 0;
    int symSamp = _cfg->pilot.size();
    std::vector<uint32_t> buffA(symSamp);
    std::vector<uint32_t> buffB(symSamp);
    std::vector<void *> buffs(2);
    buffs[0] = buffA.data();
    buffs[1] = buffB.data();
    for (int j = _cfg->bsSdrCh; j < nBsAntennas[cellIdx]; j+=_cfg->bsSdrCh)
    {
        std::cout << "SOUNDER: Draining Rx Chain: " << j << std::endl;
        RadioConfig::drain_buffers(bsSdrs[cellIdx][j/_cfg->bsSdrCh], this->bsRxStreams[cellIdx][j/_cfg->bsSdrCh], buffs, symSamp);
    }
    std::cout << "SOUNDER: Done Draining Rx Chains" << std::endl;


    if (_cfg->bsPresent)
    {
        if (hubs.size() == 0)
        {
            std::cout << "triggering first Iris ..." << std::endl;
            //bsSdrs[0][0]->writeSetting("SYNC_DELAYS", "");   // Already doing this before sample offset cal
            bsSdrs[0][0]->writeSetting("TRIGGER_GEN", "");
        }
        else
        {
            std::cout << "triggering Hub ..." << std::endl;
            //hubs[0]->writeSetting("SYNC_DELAYS", "");       // Already doing this before sample offset cal
            hubs[0]->writeSetting("TRIGGER_GEN", "");
        }
    }
    std::cout << "radio start done!" << std::endl;
}

void RadioConfig::readSensors()
{
    if (_cfg->bsPresent)
    {
        for (int i = 0; i < nBsSdrs[0]; i++)
        {
            std::cout << "TEMPs on Iris " << i << std::endl;
            std::cout << "ZYNQ_TEMP: " <<  bsSdrs[0][i]->readSensor("ZYNQ_TEMP") << std::endl;
            std::cout << "LMS7_TEMP  : " <<bsSdrs[0][i]->readSensor("LMS7_TEMP") << std::endl;
            std::cout << "FE_TEMP  : " <<  bsSdrs[0][i]->readSensor("FE_TEMP") << std::endl;
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
    if (_cfg->bsPresent)
    {
        for (int i = 0; i < nBsSdrs[0]; i++)
        {
            // write schedule
            for (int j = 0; j < _cfg->frames.size(); j++) 
            {
                for(int k = 0; k < _cfg->symbolsPerFrame; k++) // symnum <= 256
                {
            	    bsSdrs[0][i]->writeRegister("RFCORE", SCH_ADDR_REG, j*256+k);
            	    bsSdrs[0][i]->writeRegister("RFCORE", SCH_MODE_REG, 0);
                }
            }
            bsSdrs[0][i]->writeSetting("TDD_MODE", "false");
            bsSdrs[0][i]->writeRegister("IRIS30", 48, (1<<29)| 0x1);
            bsSdrs[0][i]->writeRegister("IRIS30", 48, (1<<29));
            bsSdrs[0][i]->writeRegister("IRIS30", 48, 0);
        }
    }
    if (_cfg->clPresent)
    {
        for(int i = 0; i < nClSdrs; i++)
        {
            auto device = devs[i];
            device->writeRegister("IRIS30", CORR_CONF, 0);
            std::cout << "device " << i << " T=" << std::hex << SoapySDR::timeNsToTicks(device->getHardwareTime(""), _cfg->rate) << std::dec << std::endl;
            for (int i = 0; i < _cfg->symbolsPerFrame; i++)
            {
                device->writeRegister("RFCORE", SCH_ADDR_REG, i);
                device->writeRegister("RFCORE", SCH_MODE_REG, 0);
            }
            device->writeSetting("TDD_MODE", "false");
            device->writeRegister("IRIS30", RF_RST_REG, (1<<29) | 1);
            device->writeRegister("IRIS30", RF_RST_REG, (1<<29));
            device->writeRegister("IRIS30", RF_RST_REG, 0);
            //SoapySDR::Device::unmake(device);
        }
    }
}

void RadioConfig::radioTx(void ** buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (int i = 0; i < nBsSdrs[0]; i++)
    {
        bsSdrs[0][i]->writeStream(this->bsTxStreams[0][i], buffs, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
    }
}

int RadioConfig::radioTx(int r /*radio id*/, void ** buffs, int flags, long long & frameTime)
{
    if (flags == 1) flags = SOAPY_SDR_HAS_TIME;
    else if (flags == 2) flags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
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

void RadioConfig::radioRx(void ** buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (int i = 0; i < nBsSdrs[0]; i++)
    {
        void **buff = buffs + (i * 2);
        bsSdrs[0][i]->readStream(this->bsRxStreams[0][i], buff, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
    }
}

int RadioConfig::radioRx(int r /*radio id*/, void ** buffs, long long & frameTime)
{
    int flags = 0;
    if (r < nBsSdrs[0])
    {
        long long frameTimeNs = 0;
        int ret = bsSdrs[0][r]->readStream(this->bsRxStreams[0][r], buffs, _cfg->sampsPerSymbol, flags, frameTimeNs, 1000000);
        frameTime = frameTimeNs; //SoapySDR::timeNsToTicks(frameTimeNs, _rate);
#if DEBUG_RADIO
        if (ret != _cfg->sampsPerSymbol)
            std::cout << "readStream returned " << ret << " from radio " << r << ", Expected " << _cfg->sampsPerSymbol <<std::endl;
        else
            std::cout << "radio " << r << "received " << ret << std::endl;
#endif
        return ret;
    }
    std::cout << "invalid radio id " << r << std::endl;
    return 0;
}

int RadioConfig::sampleOffsetCal()
{
    /*
     * Calibrate sample offset observed between boards in Base Station
     * Procedure: (i)   Transmit pilot from Board0 (first in list of BS boards)
     *		  (ii)  Find correlation peak index at all antennas in BS
     *		  (iii) Calculate index offset with respect to reference board Board1
     *		  (iv)  Adjust trigger delay (increase or decrease accordingly)
     */

    /************
     *   Init   *
     ************/
    // Only one cell considered at the moment
    int cellIdx = 0;
    int debug = 0;


    /***********************
     *   Generate pilots   *
     ***********************/
    std::vector<std::vector<double>> pilot;
    std::vector<std::complex<int16_t>> pilot_cint16;
    // Pilot consists of 802.11-based LTF sequences (or LTS)
    int type = CommsLib::LTS_SEQ;
    int N = 160;  // Sequence length
    pilot = CommsLib::getSequence(N, type);
    // double array to complex 16-bit int vector
    for (int i=0; i<static_cast<int>(pilot[0].size()); i++) {
        pilot_cint16.push_back(std::complex<int16_t>((int16_t)(pilot[0][i]*32768),(int16_t)(pilot[1][i]*32768)));
    }

    // Prepend/Append vectors with prefix/postfix number of null samples
    std::vector<std::complex<int16_t>> prefix_vec(_cfg->prefix, 0);
    std::vector<std::complex<int16_t>> postfix_vec(_cfg->postfix, 0);
    pilot_cint16.insert(pilot_cint16.begin(), prefix_vec.begin(), prefix_vec.end());
    pilot_cint16.insert(pilot_cint16.end(), postfix_vec.begin(), postfix_vec.end());
    // Transmitting from only one chain, create a null vector for chainB
    std::vector<std::complex<int16_t>> pilot_dummy(pilot_cint16.size(), 0);
    pilot_uint32 = Utils::cint16_to_uint32(pilot_cint16, false, "IQ");
    dummy_uint32 = Utils::cint16_to_uint32(pilot_dummy, false, "IQ");
    int symSamp = pilot_cint16.size();
    std::cout << "Number of TX pilot samples: " << pilot[0].size() << std::endl;
    std::cout << "Number of TX total samples: " << symSamp << std::endl;


    /***********************************************
     *             Compute Sync Delays             *
     ***********************************************/
    // NOTE: MOVED TO RECEIVER.CPP
    //if (!hubs.empty()) {
    //    hubs[cellIdx]->writeSetting("SYNC_DELAYS", "");
    //} else {
    //    bsSdrs[cellIdx][0]->writeSetting("SYNC_DELAYS", "");
    //}


    /***********************************************
     *   Write pilots to RAM and activate stream   *
     ***********************************************/
    int flags = 0;
    for (int i = 0; i < nBsSdrs[cellIdx]; i++)
    {
        bsSdrs[cellIdx][i]->writeRegisters("TX_RAM_A", 0, pilot_uint32);
        if (_cfg->bsSdrCh == 2) {
            // Doesn't matter: for cal, chainB not used
            bsSdrs[cellIdx][i]->writeRegisters("TX_RAM_B", 2048, dummy_uint32);
        }
        bsSdrs[cellIdx][i]->activateStream(this->bsRxStreams[cellIdx][i], flags, 0);
    }


    /***********************
     *   Create schedule   *
     ***********************/
    // For now, we assume reference board is part of the BS and is triggered from hub
    int refBoardId = 0;
    // write config for reference radio node
    std::vector<std::string> sched = {"PG"};
    //std::string sched = "PG";
    std::cout << "Samp Offset Cal Ref node schedule: " << sched[0] << std::endl;
#ifdef JSON
    json conf;
    conf["tdd_enabled"] = true;
    conf["trigger_out"] = false;
    conf["frames"] = sched;
    conf["symbol_size"] = symSamp;
    conf["max_frame"] = 1;
    std::string confString = conf.dump();
#else
    std::string confString ="{\"tdd_enabled\":true,\"trigger_out\":false,";
    confString +="\"symbol_size\":"+std::to_string(symSamp);
    confString +=",\"frames\":[\"";
    confString += sched;
    confString +="\"]}";
    std::cout << confString << std::endl;
#endif
    bsSdrs[cellIdx][refBoardId]->writeSetting("TDD_CONFIG", confString);
    bsSdrs[cellIdx][refBoardId]->writeSetting("TDD_MODE", "true");
    bsSdrs[cellIdx][refBoardId]->writeSetting("TX_SW_DELAY", "30");
    // write config for array radios (all boards in Base Station except Board0==refBoard)
    for (int i = 0; i < nBsSdrs[cellIdx] - 1; i++)
    {
        sched.clear();
        sched = {"RG"};
        std::cout << "Samp Offset Cal node " << i << " schedule: " << sched[0] << std::endl;
#ifdef JSON
        conf["tdd_enabled"] = true;
        conf["trigger_out"] = false;
        conf["frames"] = sched;
        conf["symbol_size"] = symSamp;
        conf["max_frame"] = 1;
        confString = conf.dump();
#else
        confString ="{\"tdd_enabled\":true,\"trigger_out\":false,";
        confString +="\"symbol_size\":"+std::to_string(symSamp);
        confString +=",\"frames\":[\"";
        confString += sched;
        confString +="\"]}";
        std::cout << confString << std::endl;
#endif
        bsSdrs[cellIdx][i+1]->writeSetting("TDD_CONFIG", confString);
        bsSdrs[cellIdx][i+1]->writeSetting("TDD_MODE", "true");
        bsSdrs[cellIdx][i+1]->writeSetting("TX_SW_DELAY", "30");
    }


    /*********************************
     *   Begin Calibration Process   *
     *********************************/
    // Multiple iterations
    int numCalTx = 100;
    int numVerTx = debug?200:0;
    // Cal-Passing Threshold
    double pass_thresh = 0.6 * numCalTx;
    // Read buffers
    //std::vector<std::complex<float>> buffA(symSamp);
    //std::vector<std::complex<float>> buffB(symSamp);
    std::vector<uint32_t> buffA(symSamp);
    std::vector<uint32_t> buffB(symSamp);

    std::vector<void *> buffs(2);
    buffs[0] = buffA.data();
    buffs[1] = buffB.data();

    // "Drain" rx buffers
    for (int j = _cfg->bsSdrCh; j < nBsAntennas[cellIdx]; j+=_cfg->bsSdrCh)
    {
	std::cout << "SAMP_CAL: Draining Rx Chain: " << j << std::endl;
        RadioConfig::drain_buffers(bsSdrs[cellIdx][j/_cfg->bsSdrCh], this->bsRxStreams[cellIdx][j/_cfg->bsSdrCh], buffs, symSamp);
    }
    std::cout << "SAMP_CAL: Done Draining Rx Chains" << std::endl;

    // Aggregate over iterations (for calibration and for verification)
    std::vector<std::vector<int>> calCorrIdx(nBsSdrs[cellIdx] - 1, std::vector<int>(numCalTx, 0));
    std::vector<std::vector<int>> verCorrIdx(nBsSdrs[cellIdx] - 1, std::vector<int>(numVerTx, 0));
    std::vector<int> max_freq(nBsSdrs[cellIdx] - 1, -999);
    std::vector<int> most_freq(nBsSdrs[cellIdx] - 1, -999);
    std::vector<int> samp_offset(nBsSdrs[cellIdx] - 1, -999);
    std::vector<int> max_freq_ver(nBsSdrs[cellIdx] - 1, -999);
    std::vector<int> most_freq_ver(nBsSdrs[cellIdx] - 1, -999);
    std::vector<int> samp_offset_ver(nBsSdrs[cellIdx] - 1, -999);

    for (int i = 0; i < numCalTx + numVerTx; i++)
    {
        // Generate trigger. Use hub if present, otherwise use
        if (!hubs.empty()) {
            hubs[cellIdx]->writeSetting("TRIGGER_GEN", ""); // assume a single cell and single hub
        } else {
            bsSdrs[cellIdx][0]->writeSetting("TRIGGER_GEN", ""); // assume a single cell and single hub
        }

        // Read streams
        long long frameTime = 0;
        std::vector<std::vector<std::complex<double>>> waveRxA(nBsSdrs[cellIdx]-1, std::vector<std::complex<double>>(symSamp, 0));
        std::vector<std::vector<std::complex<double>>> waveRxB(nBsSdrs[cellIdx]-1, std::vector<std::complex<double>>(symSamp, 0));

        // Ignore first board (TX)
        for (int j = _cfg->bsSdrCh; j < nBsAntennas[cellIdx]; j+=_cfg->bsSdrCh)
        {
            // Read stream data type: SOAPY_SDR_CS16
            int r = bsSdrs[cellIdx][j/_cfg->bsSdrCh]->readStream(this->bsRxStreams[cellIdx][j/_cfg->bsSdrCh], buffs.data(), symSamp, flags, frameTime, 1000000);
            if(r == -1){
		std::cout << "ReadStream: " << r << "  Num BS ant: " << nBsAntennas[cellIdx] << std::endl;
	    }
            // cast vectors
            //std::vector<std::complex<double>> buffA_d(buffA.begin(), buffA.end());
            //std::vector<std::complex<double>> buffB_d(buffB.begin(), buffB.end());
            std::vector<std::complex<double>> buffA_d = Utils::uint32tocdouble(buffA, "IQ");
            std::vector<std::complex<double>> buffB_d = Utils::uint32tocdouble(buffB, "IQ");
            waveRxA[j/_cfg->bsSdrCh - _cfg->bsSdrCh] = buffA_d;
            waveRxB[j/_cfg->bsSdrCh - _cfg->bsSdrCh] = buffB_d;
        }

        // Find correlation index at each board
        int peak=0;
        for (int j = 0; j < nBsSdrs[cellIdx] - 1; j++)
        {
            // Across all base station boards
            for (int k = 0; k < static_cast<int>(_cfg->bsSdrCh); k++)
            {
                // Across all RX channels in base station board j
                int seqLen = 160;
                if (k==0) peak = CommsLib::findLTS(waveRxA[j], seqLen);
                if (k==1) peak = CommsLib::findLTS(waveRxB[j], seqLen);

                if (peak == -1){
                    // If no LTS found, use value from second channel
                    continue;
                }
                if (i < numCalTx) {
                    calCorrIdx[j][i] = peak;
                } else {
                    verCorrIdx[j][i-numCalTx] = peak;
                }
            }
        }

        // Calibrate: If we are done collecting datapoints for cal
        if(i == numCalTx-1){
            // Find most common corr. index for each BS board (ignore tx board)
            int cal_ref_idx = 0;
            for (int j = 0; j < nBsSdrs[cellIdx] - 1; j++) {
                std::map<int,int> m;
                typedef std::vector<int>::const_iterator iter;
                for (iter vi = calCorrIdx[j].begin(); vi != calCorrIdx[j].end(); vi++) {
                    int count = m[*vi]++;
                    if (count > max_freq[j]) {
                        max_freq[j] = count;
                        most_freq[j] = *vi;
                    }
                }
                if (max_freq[j] < pass_thresh || most_freq[j] == 0) {
                    std::cout << "Sample Offset Calibration FAILED at board " << j << " MostFreq: " << most_freq[j] << " Counts: " << max_freq[j] << " ... RE-RUN!" << std::endl;
                    return -1;
                }
                // record sample offset (from a reference board)
                samp_offset[j] = most_freq[cal_ref_idx] - most_freq[j];
                for (int k = 0; k < abs(samp_offset[j]); k++) {
                    if (samp_offset[j] > 0) {
			//std::cout << "Board[" << j << "]: INCREASE" << std::endl;
                        bsSdrs[cellIdx][j+1]->writeRegister("IRIS30", FPGA_IRIS30_TRIGGERS, FPGA_IRIS30_INCR_TIME);
			usleep(5000);
                    } else if (samp_offset[j] < 0) {
			//std::cout << "Board[" << j << "]: DECREASE" << std::endl;
                        bsSdrs[cellIdx][j+1]->writeRegister("IRIS30", FPGA_IRIS30_TRIGGERS, FPGA_IRIS30_DECR_TIME);
			usleep(5000);
                    }
                }
            }
        } // end calibration


        // Verification
        if((i == numCalTx + numVerTx - 1) && debug){
            // Find most common corr. index for each BS board (ignore tx board)
            int cal_ref_idx = 0;
            for (int j = 0; j < nBsSdrs[cellIdx] - 1; j++) {
                std::map<int,int> m;
                typedef std::vector<int>::const_iterator iter;
		// Change takes time to take effect, only consider second half of verification vals
                for (iter vi = verCorrIdx[j].begin()+(numVerTx/2); vi != verCorrIdx[j].end(); vi++) {
                    int count = m[*vi]++;
                    if (count > max_freq_ver[j]) {
                        max_freq_ver[j] = count;
                        most_freq_ver[j] = *vi;
                    }
                }
                // record sample offset (from a reference board)
                samp_offset_ver[j] = most_freq_ver[cal_ref_idx] - most_freq_ver[j];

                // debug print
                std::cout << "Board[" << j << "] - Cal Offsets: " << samp_offset[j] << " Ver Offsets: " << samp_offset_ver[j] << " Most Freq[0]: " << most_freq[0] << " MostFreq[j]: " << most_freq[j] <<  " MostFreqVer[0]: " << most_freq_ver[0] << " MostFreqVer[j]: "<< most_freq_ver[j] << std::endl;
            }
        } // end verification
    } // end numCalTx + numVerTx for loop

    if(debug){
        std::ofstream myfile;
        myfile.open("./cal_ver.txt");
        myfile << "CALIBRATION VALUES:";
        myfile << std::endl;
        for (size_t idx = 0; idx<calCorrIdx[0].size(); idx++){
            for (size_t idx2 = 0; idx2<calCorrIdx.size(); idx2++) {
                myfile << calCorrIdx[idx2][idx];
                myfile << ",  ";
            }
            myfile << std::endl;
        }
        myfile << "VERIFICATION VALUES:";
        myfile << std::endl;
        for (size_t idx = 0; idx<verCorrIdx[0].size(); idx++){
            for (size_t idx2 = 0; idx2<verCorrIdx.size(); idx2++) {
                myfile << verCorrIdx[idx2][idx];
                myfile << ",  ";
            }
            myfile << std::endl;
        }
        myfile.close();
    }
    return 0;
}

void RadioConfig::drain_buffers(SoapySDR::Device * ibsSdrs, SoapySDR::Stream * istream, std::vector<void *> buffs, int symSamp) {
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
    long long frameTime=0;
    int flags=0, r=0, i=0;
    while (r != -1){
        r = ibsSdrs->readStream(istream, buffs.data(), symSamp, flags, frameTime, 1000000);
        i++;
    }
    std::cout << "Number of reads needed to drain: " << i << std::endl;
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
    if (_cfg->bsPresent)
    {
        for (int i = 0; i < nBsSdrs[0]; i++)
        {
            bsSdrs[0][i]->deactivateStream(this->bsRxStreams[0][i]);
            bsSdrs[0][i]->deactivateStream(this->bsTxStreams[0][i]);
            bsSdrs[0][i]->closeStream(this->bsRxStreams[0][i]);
            bsSdrs[0][i]->closeStream(this->bsTxStreams[0][i]);
            SoapySDR::Device::unmake(bsSdrs[0][i]);
        }
    }
    if (_cfg->clPresent)
    {
        for(int i = 0; i < nClSdrs; i++)
        {
            auto device = devs[i];
            device->deactivateStream(rxss[i]);
            device->deactivateStream(txss[i]);

            device->closeStream(rxss[i]);
            device->closeStream(txss[i]);
            SoapySDR::Device::unmake(device);
        }
    }
}

