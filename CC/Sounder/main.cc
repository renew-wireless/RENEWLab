/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
 
---------------------------------------------------------------------
 main function
 - initializes all Clients
 - Brings up Recorder and the BaseStation
---------------------------------------------------------------------
*/

#include "include/recorder.h"
#include "include/utils.h"
#include "include/comms-lib.h"

static sig_atomic_t loopDone = false;
std::atomic_int thread_count(0);
std::mutex d_mutex;
std::condition_variable cond;

typedef struct
{
    int id;
    int nsamps;
    int txSyms;
    int rxSyms;
    int txStartSym;
    unsigned txFrameDelta;
    double rate;
    SoapySDR::Device * device;
    SoapySDR::Stream * rxs;
    SoapySDR::Stream * txs;
    std::string data_file;
} dev_profile;

void txrx(void * context, void* data)
{
    dev_profile *profile = (dev_profile *)context;
    SoapySDR::Device * device = profile->device;
    SoapySDR::Stream * rxStream = profile->rxs;
    SoapySDR::Stream * txStream = profile->txs;
    int id = profile->id;
    int txSyms = profile->txSyms; 
    int rxSyms = profile->rxSyms; 
    int txStartSym = profile->txStartSym;
    double rate = profile->rate;
    unsigned txFrameDelta = profile->txFrameDelta; 
    int NUM_SAMPS = profile->nsamps;

    while(!d_mutex.try_lock()){}
    thread_count++;
    std::cout << "Thread " << id << ", txSyms " << txSyms << ", rxSyms " << rxSyms << ", txStartSym " << txStartSym << ", rate " << rate << ", txFrameDelta " << txFrameDelta << ", nsamps " << NUM_SAMPS << std::endl;
    d_mutex.unlock();
 
    std::vector<std::complex<float>> buffs(NUM_SAMPS, 0);
    std::vector<void *> rxbuff(2);
    rxbuff[0] = buffs.data();
    rxbuff[1] = buffs.data();

    std::vector<void *> txbuff(2);
    txbuff[0] = data;
    txbuff[1] = data;

    device->activateStream(rxStream);
    device->activateStream(txStream);

    bool exitLoop = false;
    while (not exitLoop)
    {
        exitLoop = loopDone;

        // receiver loop
        long long rxTime(0);
        long long txTime(0);
        long long firstRxTime (0);
        bool receiveErrors = false;
        for (int i = 0; i < rxSyms; i++)
        {
            int flags(0);
            int r = device->readStream(rxStream, rxbuff.data(), NUM_SAMPS, flags, rxTime, 1000000);
            if (r == NUM_SAMPS)
            {
                if (i == 0) firstRxTime = rxTime;
            }
            else
            {
                std::cerr << "waiting for receive frames... " << std::endl;
                receiveErrors = true;
                break; 
            }
        }
        if (receiveErrors) continue; // just go to the next frame

        // transmit loop
        int flags = SOAPY_SDR_HAS_TIME;
        txTime = firstRxTime & 0xFFFFFFFF00000000;
        txTime += ((long long)txFrameDelta << 32); 
        txTime += ((long long)txStartSym << 16);
        //printf("rxTime %llx, txTime %llx \n", firstRxTime, txTime);
        if (exitLoop) flags |= SOAPY_SDR_END_BURST; //end burst on last iter
        bool transmitErrors = false;
        for (int i = 0; i < txSyms; i++)
        {
            if (i == txSyms - 1)  flags |= SOAPY_SDR_END_BURST;
            int r = device->writeStream(txStream, txbuff.data(), NUM_SAMPS, flags, txTime, 1000000);
            if (r == NUM_SAMPS)
            {
                txTime += 0x10000;
            }
            else
            {
                std::cerr << "unexpected writeStream error " << SoapySDR::errToStr(r) << std::endl;
                transmitErrors = true;
                //goto cleanup;
            }
        }

    }

    device->deactivateStream(rxStream);
    device->deactivateStream(txStream);

    device->closeStream(rxStream);
    device->closeStream(txStream);


    std::lock_guard<std::mutex> lock(d_mutex); 
    device->writeRegister("IRIS30", CORR_CONF, 0);
    std::cout << "device " << id << " T=" << std::hex << SoapySDR::timeNsToTicks(device->getHardwareTime(""), rate) << std::endl;
    for (int i = 0; i < 20; i++)
    {
        device->writeRegister("RFCORE", SCH_ADDR_REG, i);
        device->writeRegister("RFCORE", SCH_MODE_REG, 0);
    }
    device->writeSetting("TDD_MODE", "false");
    device->writeRegister("IRIS30", RF_RST_REG, (1<<29) | 1);
    device->writeRegister("IRIS30", RF_RST_REG, (1<<29));
    device->writeRegister("IRIS30", RF_RST_REG, 0);
    SoapySDR::Device::unmake(device);
    free(context);
    thread_count--;
    cond.notify_one();
}


void sigIntHandler(const int)
{
    loopDone = true;
}

int main(int argc, char const *argv[])
{
    if(argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " CONFIG_FILE_NAME" << std::endl;
        return EXIT_FAILURE;
    }
    else if (strcmp(argv[1], "--help") == 0 or strcmp(argv[1], "-h") == 0)
    {
        std::cerr << "Usage: " << argv[0] << " CONFIG_FILE_NAME" << std::endl;
        return EXIT_SUCCESS;
    }

    Config *config = new Config(argv[1]);
    srand(time(NULL));
    std::vector<std::thread> threads;
    int nClSdrs = config->nClSdrs;
    std::vector<SoapySDR::Device *> devs;
    std::vector<SoapySDR::Stream *> rxss;
    std::vector<SoapySDR::Stream *> txss;

    if (config->clPresent)
    {
        std::vector<std::vector<size_t>> pilotSymbols = Utils::loadSymbols(config->clFrames, 'P');
        std::vector<std::vector<size_t>> ULSymbols = Utils::loadSymbols(config->clFrames, 'U');
        std::vector<std::vector<size_t>> DLSymbols = Utils::loadSymbols(config->clFrames, 'D');

        /**** Signal Generation ****/

        std::vector<std::complex<int16_t>> pre(config->prefix, 0);
        std::vector<std::complex<int16_t>> post(config->postfix, 0);

        std::vector<std::vector<double>> gold_ifft = CommsLib::getSequence(128, CommsLib::GOLD_IFFT);
        std::vector<std::complex<int16_t>> coeffs_ci16 = Utils::double_to_int16(gold_ifft, "IQ"); 
        std::vector<uint32_t> coeffs = Utils::cint16_to_uint32(coeffs_ci16, true, "QI");

        // compose pilot subframe
        std::vector<std::vector<double>> lts = CommsLib::getSequence(160, CommsLib::LTS_SEQ);
        std::vector<std::complex<int16_t>> lts_ci16 = Utils::double_to_int16(lts, "IQ"); 
        int nSamps = config->sampsPerSymbol - config->prefix - config->postfix;
        int rep = nSamps / 160;
        int frac = nSamps % 160;
        std::vector<std::complex<int16_t>> pilot_ci16;
        pilot_ci16.insert(pilot_ci16.begin(), pre.begin(), pre.end());

        for (int i = 0 ; i < rep; i++)
            pilot_ci16.insert(pilot_ci16.end(), lts_ci16.begin(), lts_ci16.end());

        pilot_ci16.insert(pilot_ci16.end(), lts_ci16.begin(), lts_ci16.begin()+frac);
        pilot_ci16.insert(pilot_ci16.end(), post.begin(), post.end());

        std::vector<uint32_t> pilot = Utils::cint16_to_uint32(pilot_ci16, false, "IQ");
#if DEBUG_PRINT
        for (int j = 0; j < pilot.size(); j++)
        {
            std::cout << "Pilot[" << j << "]: \t " << pilot_ci16[j] << std::endl;
        }
#endif

        // compose data subframe
        std::vector<std::vector<std::complex<float>>> txdata;
        if (ULSymbols[0].size() > 0)
        {
            int fftSize = config->fftSize; 
            int cpSize = config->cpSize; 
            int mod_type = config->clDataMod == "64QAM" ? CommsLib::QAM64 : (config->clDataMod == "16QAM" ? CommsLib::QAM16 : CommsLib::QPSK); 
            std::cout << mod_type << std::endl;
            int mod_order = (int)pow(2, mod_type); 
            std::cout << mod_order << std::endl;
            if (fftSize != 64) fftSize = 64;
            if (cpSize != 16) cpSize = 16;
            std::vector<int> data_ind = CommsLib::getDataSc(fftSize);
            std::vector<std::vector<int>> pilot_sc = CommsLib::getPilotSc(fftSize);
            int ofdmSize = fftSize + cpSize;
            int nDataScs = data_ind.size(); 
            int syms = nSamps / ofdmSize;
            std::vector<std::complex<float>> pre1(config->prefix, 0);
            std::vector<std::complex<float>> post1(nSamps % ofdmSize + config->postfix, 0);
            for (int i = 0; i < nClSdrs; i++)
            {
                std::vector<std::complex<float>> data_cf;
		std::vector<std::complex<float>> data_freq_dom;
                data_cf.insert(data_cf.begin(), pre1.begin(), pre1.end()); 
		data_freq_dom.insert(data_freq_dom.begin(), pre1.begin(), pre1.end());
                std::vector<std::vector<int>> dataBits;
                dataBits.resize(syms);
                for (int s = 0; s < syms; s++)
                {
                    for (int c = 0; c < nDataScs; c++) dataBits[s].push_back(rand() % mod_order);
                    std::vector<std::complex<float>> mod_data = CommsLib::modulate(dataBits[s], mod_type);
                    std::cout << "Modulation output: "<< mod_data[0] << " " << mod_data[1] << std::endl;
                    std::vector<std::complex<float>> ofdmSym(fftSize);
                    int sc = 0;
                    for (int c = 0; c < nDataScs; c++)
                    {
                        sc = data_ind[c];
                        ofdmSym[sc] = mod_data[c];
                    }
                    std::cout << "Data symbol: " << ofdmSym[sc-2] << " " << ofdmSym[sc-1] << std::endl;
                    for (int c = 0; c < pilot_sc[0].size(); c++)
                    {
                        sc = pilot_sc[0][c];
                        ofdmSym[sc] = pilot_sc[1][c];
                    }
                    std::cout << "Pilot symbol: " << ofdmSym[pilot_sc[0][0]] << " " << ofdmSym[pilot_sc[0][1]] << std::endl;
                    std::vector<std::complex<float>> txSym = CommsLib::IFFT(ofdmSym, fftSize);
                    txSym.insert(txSym.begin(), txSym.end()-cpSize, txSym.end()); // add CP
                    std::cout << "IFFT output: " << txSym[0] << " " << txSym[64] << std::endl;
                    data_cf.insert(data_cf.end(), txSym.begin(), txSym.end());
		    data_freq_dom.insert(data_freq_dom.end(), ofdmSym.begin(), ofdmSym.end());
                }
                data_cf.insert(data_cf.end(), post1.begin(), post1.end());
                txdata.push_back(data_cf);
		config->txdata_freq_dom.push_back(data_freq_dom);
            } 
#if DEBUG_PRINT
            for (int i = 0; i < txdata.size(); i++)
            {
                for (int j = 0; j < txdata[i].size(); j++){
            	    std::cout << "Values["<< i <<"][" << j << "]: \t " << txdata[i][j] << std::endl;
                }
            }
            for (int i = 0; i < config->txdata_freq_dom.size(); i++)
            {
                for (int j = 0; j < config->txdata_freq_dom[i].size(); j++){
                    std::cout << "FREQ DOMAIN Values["<< i <<"][" << j << "]: \t " << config->txdata_freq_dom[i][j] << std::endl;
                }
            }
#endif
        }
        /**** End Signal Generation ****/

        int ueTrigOffset = config->prefix + 256 + config->postfix + 17 + config->prefix;
        int sf_start = ueTrigOffset/config->sampsPerSymbol;
        int sp_start = ueTrigOffset%config->sampsPerSymbol;

        //load channels
        std::vector<size_t> channels;
        if (config->clSdrCh == 1) channels = {0};
        else if (config->clSdrCh == 2) channels = {0, 1};
        else
        {
            std::cout << "Error! Supported number of channels 1 or 2, setting to 2!" << std::endl;
            config->bsSdrCh = 2;
            channels = {0, 1};
        }

        double frameTime = config->sampsPerSymbol*config->clFrames[0].size()*1e3/config->rate; // miliseconds
        unsigned frameTimeDelta = (unsigned)(std::ceil(TIME_DELTA/frameTime)); 
        std::cout << "Frame time delta " << frameTimeDelta << std::endl;

        std::vector<std::string> tddSched;
        tddSched.resize(nClSdrs);
        for (int i = 0; i < nClSdrs; i++)
        {
            tddSched[i] = config->clFrames[i];
            for (int s = 0; s < config->clFrames[i].size(); s++)
            {
                char c = config->clFrames[i].at(s);
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
 
        for(size_t i = 0; i < nClSdrs; i++)
        {
            auto device = SoapySDR::Device::make("serial="+config->cl_sdr_ids.at(i));
            if (device == nullptr)
            {
                std::cerr << "No device!" << std::endl;
                return EXIT_FAILURE;
            }
            devs.push_back(device);
            SoapySDR::Kwargs info = device->getHardwareInfo();

            for (auto ch : channels)
            {
                device->setSampleRate(SOAPY_SDR_RX, ch, config->rate);
                device->setSampleRate(SOAPY_SDR_TX, ch, config->rate);

                //device->setFrequency(SOAPY_SDR_RX, ch, freq);  
                //device->setFrequency(SOAPY_SDR_TX, ch, freq); 
                device->setFrequency(SOAPY_SDR_RX, ch, "RF", config->freq-.75*config->rate);
                device->setFrequency(SOAPY_SDR_RX, ch, "BB", .75*config->rate);
                device->setFrequency(SOAPY_SDR_TX, ch, "RF", config->freq-.75*config->rate);
                device->setFrequency(SOAPY_SDR_TX, ch, "BB", .75*config->rate);
 
                if (info["frontend"].find("CBRS") != std::string::npos)
                {
                    device->setGain(SOAPY_SDR_RX, ch, "ATTN", 0); //[-18,0]  
                    device->setGain(SOAPY_SDR_RX, ch, "LNA1", 30); //[0,33]
                    device->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
                }

                device->setGain(SOAPY_SDR_RX, ch, "LNA", ch ? config->clRxgainB : config->clRxgainA);  //[0,30]
                device->setGain(SOAPY_SDR_RX, ch, "TIA", 0);  //[0,12]
                device->setGain(SOAPY_SDR_RX, ch, "PGA", 0);  //[-12,19]

                if (info["frontend"].find("CBRS") != std::string::npos)
                {
                    device->setGain(SOAPY_SDR_TX, ch, "ATTN", 0);       //[-18,0] by 3
                    device->setGain(SOAPY_SDR_TX, ch, "PA1", 15);       //[0|15]
                    device->setGain(SOAPY_SDR_TX, ch, "PA3", 30);       //[0|30]
                }
                device->setGain(SOAPY_SDR_TX, ch, "IAMP", 12);          //[0,12] 
                device->setGain(SOAPY_SDR_TX, ch, "PAD", ch ? config->clTxgainB : config->clTxgainA);       //[0,52] 
            }

            for (auto ch : channels)
            {
                //device->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
                //device->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
                device->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
            }

            if (config->clSdrCh == 1)
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

            device->writeRegister("IRIS30", CORR_CONF, 0x1);
            for (int k = 0; k < 128; k++)
                device->writeRegister("ARGCOE", k*4, 0);
            usleep(100000);
            device->writeRegister("ARGCOR", CORR_THRESHOLD, 128);
            device->writeRegister("ARGCOR", CORR_RST, 1);
            device->writeRegister("ARGCOR", CORR_RST, 0);
            for (int k = 0; k < 128; k++)
                device->writeRegister("ARGCOE", k*4, coeffs[k]);

#ifdef JSON
            json conf;
            conf["tdd_enabled"] = true;
            conf["trigger_out"] = true;
            conf["wait_trigger"] = true;
            conf["frames"] = json::array();
            conf["frames"].push_back(tddSched[i]);
            conf["symbol_size"] = config->sampsPerSymbol; 
            std::string confString = conf.dump();
#else
            std::string confString ="{\"tdd_enabled\":true,\"trigger_out\":true,\"wait_trigger\":true,";
            confString +="\"symbol_size\":"+std::to_string(config->sampsPerSymbol);
            confString +=",\"frames\":[\""+tddSched[i]+"\"]}";
            std::cout << confString << std::endl;
#endif
            device->writeSetting("TDD_CONFIG", confString);

            device->setHardwareTime(SoapySDR::ticksToTimeNs((sf_start << 16) | sp_start, config->rate), "TRIGGER");
            device->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
            device->writeSetting("TDD_MODE", "true");
            // write beacons to FPGA buffers
            device->writeRegisters("TX_RAM_A", 0, pilot);
            if (config->clSdrCh == 2)
                device->writeRegisters("TX_RAM_B", 0, pilot); // no dual channel beacon for now
            device->writeRegister("IRIS30", CORR_CONF, 0x11);
            
        }

        std::cout << "Initializing Threads ..." << std::endl;
        unsigned nCores = std::thread::hardware_concurrency();
        threads.resize(nClSdrs);
        for (size_t i = 0; i < nClSdrs; i++)
        {
            dev_profile *profile = (dev_profile *)malloc(sizeof(dev_profile));
            profile->id = i;
            profile->rate = config->rate;
            profile->nsamps = config->sampsPerSymbol;
            profile->txSyms = ULSymbols[i].size();
            profile->rxSyms = DLSymbols[i].size();
            profile->txStartSym = ULSymbols[i].size() > 0 ? ULSymbols[i][0] : 0;
            profile->txFrameDelta = frameTimeDelta;
            profile->device = devs[i];
            profile->rxs = rxss[i];
            profile->txs = txss[i];
            threads[i] = std::thread(txrx, (void*)profile, profile->txSyms > 0 ? txdata[i].data() : NULL);
            if (nCores > i+1+SOCKET_THREAD_NUM+TASK_THREAD_NUM)
            {
                cpu_set_t cpuset;
                CPU_ZERO(&cpuset);
                CPU_SET(i+1+SOCKET_THREAD_NUM+TASK_THREAD_NUM, &cpuset);
                int rc = pthread_setaffinity_np(threads[i].native_handle(),
                                                sizeof(cpu_set_t), &cpuset);
                if (rc != 0) {
                  std::cerr << "Error calling pthread_setaffinity_np: " << rc << "\n";
                }
            }
        }
        
    }

    if (config->bsPresent)
    {
        Recorder dr(config);
        time_t now = time(0);
        tm *ltm = localtime(&now);
        int cell_num = config->nCells; 
        int ant_num = config->getNumAntennas(); 
        int ue_num = nClSdrs; 
        std::string filename = "logs/Argos-"+std::to_string(1900 + ltm->tm_year)+"-"+std::to_string(ltm->tm_mon)+"-"+std::to_string(ltm->tm_mday)+"-"+std::to_string(ltm->tm_hour)+"-"+std::to_string(ltm->tm_min)+"-"+std::to_string(ltm->tm_sec)+"_"+std::to_string(cell_num)+"x"+std::to_string(ant_num)+"x"+std::to_string(ue_num)+".hdf5";    
        if (dr.initHDF5(filename) < 0) return -1;
        dr.openHDF5();
        dr.start();
    }

    if (config->clPresent)
    {
        for (auto& t : threads){
            t.detach();
        }

        usleep(100000);
        signal(SIGINT, sigIntHandler);
        std::cout << "Press Ctrl+C to stop..." << std::endl;
        std::unique_lock<std::mutex> lock(d_mutex);
        cond.wait(lock, [](){ return thread_count == 0; });

        int loopCount = 0;
        std::vector<uint32_t> nSync(nClSdrs);
        std::vector<uint32_t> tm(nClSdrs);
        //while(true)
        //{
        //    sleep(1);
        //    loopCount = 0;
        //    for (int i = 0; i < nClSdrs; i++)
        //        tm[i] = SoapySDR::timeNsToTicks(devs[i]->getHardwareTime(""), config->rate) >> 32;
        //    for (int i = 0; i < nClSdrs; i++)
        //    {
        //        std::cout << "device " << i << ": " << (tm[i] - nSync[i]) << " new triggers, " << tm[i] <<" total"  << std::endl;
        //        nSync[i] = tm[i];
        //    }
        //}
    }
    return 0;
}

