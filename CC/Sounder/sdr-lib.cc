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
                    device->setGain(SOAPY_SDR_RX, ch, "ATTN", 0); //[-18,0]  
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
                device->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);                //[-12,12] 
		device->setGain(SOAPY_SDR_TX, ch, "PAD", ch ? _cfg->clTxgainB_vec[i] : _cfg->clTxgainA_vec[i]);       //[0,52]
            }

            for (auto ch : channels)
            {
                //device->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
                //device->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
                device->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
            }

            if (_cfg->clSdrCh == 1)
            {
                device->writeSetting(SOAPY_SDR_RX, 1, "ENABLE_CHANNEL", "false");
                device->writeSetting(SOAPY_SDR_TX, 1, "ENABLE_CHANNEL", "false");
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
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_RX, ch, "ATTN", 0); //[-18,0]  
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
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "ATTN", 0);  //[-18,0] by 3
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PA1", 14);  //[0|14] no bypass
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PA2", 0);   //[0|17]   can bypass. Can cause saturation or PA damage!! DO NOT USE IF NOT SURE!!!
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PA3", 30);  //[0|31.5]   no bypass
        }
        if (info["frontend"].find("UHF") != std::string::npos)
        {
            rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "ATTN", 0);  //[-18,0] by 3
        }

        // lime 
        rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);     //[0,12] 
        rc->bsSdrs[c][i]->setGain(SOAPY_SDR_TX, ch, "PAD", ch ? cfg->txgainB : cfg->txgainA);  //[0,30]

    }

    for (auto ch : channels)
    {
        //bsSdrs[c][i]->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
        //bsSdrs[c][i]->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
        rc->bsSdrs[c][i]->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    }

    if (cfg->bsSdrCh == 1)
    {
        // we setup SPI TDD mode to bypass the internal LDO issue in revision D and prior
        if (cfg->freq > 3e9 and cfg->bs_sdr_ids[c][i].find("RF3E") == std::string::npos)
        {
            std::cout << "setting up SPI_TDD" << std::endl;
            std::vector<unsigned> txActive, rxActive;
            unsigned ch = rc->bsSdrs[c][i]->readRegister("LMS7IC", 0x0020);
            rc->bsSdrs[c][i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 1);
            //unsigned regRfeA = bsSdrs[c][i]->readRegister("LMS7IC", 0x010C);
            //unsigned regRfeALo = bsSdrs[c][i]->readRegister("LMS7IC", 0x010D);
            unsigned regRbbA = rc->bsSdrs[c][i]->readRegister("LMS7IC", 0x0115);
            //unsigned regTrfA = bsSdrs[c][i]->readRegister("LMS7IC", 0x0100);
            unsigned regTbbA = rc->bsSdrs[c][i]->readRegister("LMS7IC", 0x0105);

            // disable TX
            txActive = {
                //0xa10C0000 | 0xfe, //RFE in power down
                //0xa10D0000 | 0x0, //RFE SISO and disables
                0xa1150000 | 0xe, //RBB in power down
                //0xa1000000 | regTrfA //TRF stays the same
                0xa1050000 | regTbbA //TBB stays the same
            };
            rc->bsSdrs[c][i]->writeRegisters("LMS7_PROG_SPI", 16, txActive); //trig1 offset
            // disable RX
            rxActive = {
                //0xa10C0000 | regRfeA, //RFE stays the same
                //0xa10D0000 | regRfeALo, //RFE stays the same
                0xa1150000 | regRbbA, //RBB stays the same
                //0xa1000000 | 0xe //TRF in power down + SISO
                0xa1050000 | 0x1e //TBB in power down
            };
            rc->bsSdrs[c][i]->writeRegisters("LMS7_PROG_SPI", 32, rxActive); //trig2 offset

            //bsSdrs[i]->writeSetting("SPI_TDD_MODE", "SISO"); // a FPGA hack that bypasses the LDO issue
        }
        rc->bsSdrs[c][i]->writeSetting(SOAPY_SDR_RX, 1, "ENABLE_CHANNEL", "false");
        rc->bsSdrs[c][i]->writeSetting(SOAPY_SDR_TX, 1, "ENABLE_CHANNEL", "false");
    } 
    else if (cfg->bsSdrCh == 2)
    {
        // we setup SPI TDD mode to bypass the internal LDO issue in revision D and prior
        if (cfg->freq > 3e9 and cfg->bs_sdr_ids[c][i].find("RF3E") == std::string::npos)
        {
            std::vector<unsigned> txActive, rxActive;
            unsigned ch = rc->bsSdrs[c][i]->readRegister("LMS7IC", 0x0020);
            rc->bsSdrs[c][i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 1);
            //unsigned regRfeA = bsSdrs[c][i]->readRegister("LMS7IC", 0x010C);
            //unsigned regRfeALo = bsSdrs[c][i]->readRegister("LMS7IC", 0x010D);
            unsigned regRbbA = rc->bsSdrs[c][i]->readRegister("LMS7IC", 0x0115);
            //unsigned regTrfA = bsSdrs[c][i]->readRegister("LMS7IC", 0x0100);
            unsigned regTbbA = rc->bsSdrs[c][i]->readRegister("LMS7IC", 0x0105);

            ch = rc->bsSdrs[c][i]->readRegister("LMS7IC", 0x0020);
            rc->bsSdrs[c][i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 2);
            //unsigned regRfeB = bsSdrs[c][i]->readRegister("LMS7IC", 0x010C);
            //unsigned regRbbB = bsSdrs[c][i]->readRegister("LMS7IC", 0x0115);
            //unsigned regTrfB = bsSdrs[c][i]->readRegister("LMS7IC", 0x0100);
            //unsigned regTbbB = bsSdrs[c][i]->readRegister("LMS7IC", 0x0105);

            txActive = {
                //0xe10C0000 | 0xfe, //RFE in power down
                //0xe10D0000 | 0x0, //RFE SISO and disables
                0xe1150000 | 0xe, //RBB in power down
                //0xe1000000 | regTrfA, //TRF stays the same
                0xe1050000 | regTbbA}; //TBB stays the same

            rxActive = {
                //0xe10C0000 | regRfeA, //RFE stays the same
                //0xe10D0000 | regRfeALo, //RFE stays the same
                0xe1150000 | regRbbA, //RBB stays the same
                //0xe1000000 | 0xe, //TRF in power down + SISO
                0xe1050000 | 0x1e}; //TBB in power down

            rc->bsSdrs[c][i]->writeRegisters("LMS7_PROG_SPI", 16, txActive); //trig1 offset
            rc->bsSdrs[c][i]->writeRegisters("LMS7_PROG_SPI", 32, rxActive); //trig2 offset
            //bsSdrs[i]->writeSetting("SPI_TDD_MODE", "MIMO");
        }
    }
    //The following must be done by the driver at initialization
    //bsSdrs[i]->writeRegister("RFCORE", 120, 0); // reset the tdd mode in the FPGA
    // resets the DATA_clk domain logic. 
    rc->bsSdrs[c][i]->writeRegister("IRIS30", 48, (1<<29) | 0x1);
    rc->bsSdrs[c][i]->writeRegister("IRIS30", 48, (1<<29));
    rc->bsSdrs[c][i]->writeRegister("IRIS30", 48, 0);
    rc->bsRxStreams[c][i] = rc->bsSdrs[c][i]->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
    rc->bsTxStreams[c][i] = rc->bsSdrs[c][i]->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs);
    rc->remainingJobs--;
    return 0;    
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

    if (_cfg->bsPresent)
    {
        if (hubs.size() == 0)
        {
            std::cout << "triggering first Iris ..." << std::endl;
            bsSdrs[0][0]->writeSetting("SYNC_DELAYS", "");
            bsSdrs[0][0]->writeSetting("TRIGGER_GEN", "");
        }
        else
        {
            std::cout << "triggering Hub ..." << std::endl;
            hubs[0]->writeSetting("SYNC_DELAYS", "");
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
            std::cout << "device " << i << " T=" << std::hex << SoapySDR::timeNsToTicks(device->getHardwareTime(""), _cfg->rate) << std::endl;
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

