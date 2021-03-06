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
    std::vector<size_t> num_bs_antenntas(_cfg->num_cells());
    bsRadios.resize(_cfg->num_cells());
    radioNotFound = false;
    std::vector<std::string> radio_serial_not_found;

    for (size_t c = 0; c < _cfg->num_cells(); c++) {
        size_t num_radios = _cfg->n_bs_sdrs()[c];
        num_bs_antenntas[c] = num_radios * _cfg->bs_channel().length();
        MLPD_TRACE("Setting up radio: %zu, cells: %zu\n", num_radios,
            _cfg->num_cells());
        if ((kUseUHD == false) && (_cfg->hub_ids().empty() == false)) {
            SoapySDR::Kwargs args;
            args["driver"] = "remote";
            args["timeout"] = "1000000";
            args["serial"] = _cfg->hub_ids().at(c);
            hubs.push_back(SoapySDR::Device::make(args));
        }
        bsRadios.at(c).resize(num_radios);
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
        while (thread_count.load() > 0) {
        }

        // Strip out broken radios.
        for (size_t i = 0; i < num_radios; i++) {
            if (bsRadios.at(c).at(i) == NULL) {
                radioNotFound = true;
                radio_serial_not_found.push_back(
                    _cfg->bs_sdr_ids().at(c).at(i));
                while (num_radios != 0
                    && bsRadios.at(c).at(num_radios - 1) == NULL) {
                    --num_radios;
                    bsRadios.at(c).pop_back();
                }
                if (i < num_radios) {
                    bsRadios.at(c).at(i) = bsRadios.at(c).at(--num_radios);
                    bsRadios.at(c).pop_back();
                }
            }
        }
        bsRadios.at(c).shrink_to_fit();
        _cfg->n_bs_sdrs().at(c) = num_radios;
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

        while (thread_count.load() > 0) {
        }

        auto channels = Utils::strToChannels(_cfg->bs_channel());

        for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
            auto dev = bsRadios.at(c).at(i)->dev;
            std::cout << _cfg->bs_sdr_ids().at(c).at(i) << ": Front end "
                      << dev->getHardwareInfo()["frontend"] << std::endl;
            for (auto ch : channels) {
                if (ch < dev->getNumChannels(SOAPY_SDR_RX)) {
                    printf("RX Channel %zu\n", ch);
                    printf("Actual RX sample rate: %fMSps...\n",
                        (dev->getSampleRate(SOAPY_SDR_RX, ch) / 1e6));
                    printf("Actual RX frequency: %fGHz...\n",
                        (dev->getFrequency(SOAPY_SDR_RX, ch) / 1e9));
                    printf("Actual RX gain: %f...\n",
                        (dev->getGain(SOAPY_SDR_RX, ch)));
                    if (!kUseUHD) {
                        printf("Actual RX LNA gain: %f...\n",
                            (dev->getGain(SOAPY_SDR_RX, ch, "LNA")));
                        printf("Actual RX PGA gain: %f...\n",
                            (dev->getGain(SOAPY_SDR_RX, ch, "PGA")));
                        printf("Actual RX TIA gain: %f...\n",
                            (dev->getGain(SOAPY_SDR_RX, ch, "TIA")));
                        if (dev->getHardwareInfo()["frontend"].find("CBRS")
                            != std::string::npos) {
                            printf("Actual RX LNA1 gain: %f...\n",
                                (dev->getGain(SOAPY_SDR_RX, ch, "LNA1")));
                            printf("Actual RX LNA2 gain: %f...\n",
                                (dev->getGain(SOAPY_SDR_RX, ch, "LNA2")));
                        }
                    }
                    printf("Actual RX bandwidth: %fM...\n",
                        (dev->getBandwidth(SOAPY_SDR_RX, ch) / 1e6));
                    printf("Actual RX antenna: %s...\n",
                        (dev->getAntenna(SOAPY_SDR_RX, ch).c_str()));
                }
            }

            for (auto ch : channels) {
                if (ch < dev->getNumChannels(SOAPY_SDR_TX)) {
                    printf("TX Channel %zu\n", ch);
                    printf("Actual TX sample rate: %fMSps...\n",
                        (dev->getSampleRate(SOAPY_SDR_TX, ch) / 1e6));
                    printf("Actual TX frequency: %fGHz...\n",
                        (dev->getFrequency(SOAPY_SDR_TX, ch) / 1e9));
                    printf("Actual TX gain: %f...\n",
                        (dev->getGain(SOAPY_SDR_TX, ch)));
                    if (!kUseUHD) {
                        printf("Actual TX PAD gain: %f...\n",
                            (dev->getGain(SOAPY_SDR_TX, ch, "PAD")));
                        printf("Actual TX IAMP gain: %f...\n",
                            (dev->getGain(SOAPY_SDR_TX, ch, "IAMP")));
                        if (dev->getHardwareInfo()["frontend"].find("CBRS")
                            != std::string::npos) {
                            printf("Actual TX PA1 gain: %f...\n",
                                (dev->getGain(SOAPY_SDR_TX, ch, "PA1")));
                            printf("Actual TX PA2 gain: %f...\n",
                                (dev->getGain(SOAPY_SDR_TX, ch, "PA2")));
                            printf("Actual TX PA3 gain: %f...\n",
                                (dev->getGain(SOAPY_SDR_TX, ch, "PA3")));
                        }
                    }
                    printf("Actual TX bandwidth: %fM...\n",
                        (dev->getBandwidth(SOAPY_SDR_TX, ch) / 1e6));
                    printf("Actual TX antenna: %s...\n",
                        (dev->getAntenna(SOAPY_SDR_TX, ch).c_str()));
                }
            }
            std::cout << std::endl;
        }
        // Measure Sync Delays now!
        if (kUseUHD == false) {
            sync_delays(c);
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
                    for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
                        tddConf["frames"] = json::array();
                        tddConf["frames"].push_back(
                            _cfg->calib_frames().at(c).at(i));
                        std::cout << "Cell " << c << ", SDR " << i
                                  << " calibration schedule : "
                                  << _cfg->calib_frames().at(c).at(i)
                                  << std::endl;
                        std::string tddConfStr = tddConf.dump();
                        SoapySDR::Device* dev = bsRadios.at(c).at(i)->dev;
                        dev->writeSetting("TDD_CONFIG", tddConfStr);
                        // write pilot to FPGA buffers
                        for (char const& c : _cfg->bs_channel()) {
                            std::string tx_ram = "TX_RAM_";
                            dev->writeRegisters(tx_ram + c, 0, _cfg->pilot());
                        }
                    }
                } else {
                    tddConf["frames"] = json::array();
                    size_t frame_size = _cfg->frames().at(c).size();
                    std::string fw_frame = _cfg->frames().at(c);
                    for (size_t s = 0; s < frame_size; s++) {
                        char sym_type = fw_frame.at(s);
                        if (sym_type == 'P')
                            fw_frame.replace(s, 1, "R"); // uplink pilots
                        else if (sym_type == 'N')
                            fw_frame.replace(s, 1, "R"); // uplink data
                        else if (sym_type == 'U')
                            fw_frame.replace(s, 1, "R"); // uplink data
                        else if (sym_type == 'D')
                            fw_frame.replace(s, 1, "T"); // downlink data
                    }
                    tddConf["frames"].push_back(fw_frame);
                    std::cout << "Cell " << c
                              << " FPGA schedule: " << _cfg->frames().at(c)
                              << std::endl;
                    tddConf["beacon_start"] = _cfg->prefix();
                    tddConf["beacon_stop"]
                        = _cfg->prefix() + _cfg->beacon_size();
                    std::string tddConfStr = tddConf.dump();
                    for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
                        SoapySDR::Device* dev = bsRadios.at(c).at(i)->dev;
                        dev->writeSetting("TDD_CONFIG", tddConfStr);
                    }

                    // write beacons to FPGA buffers
                    size_t ndx = 0;
                    for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
                        SoapySDR::Device* dev = bsRadios.at(c).at(i)->dev;
                        dev->writeRegisters("BEACON_RAM", 0, _cfg->beacon());
                        for (char const& ch : _cfg->bs_channel()) {
                            bool isBeaconAntenna = !_cfg->beam_sweep()
                                && ndx == _cfg->beacon_ant();
                            std::vector<unsigned> beacon_weights(
                                num_bs_antenntas[c], isBeaconAntenna ? 1 : 0);
                            std::string tx_ram_wgt = "BEACON_RAM_WGT_";
                            if (_cfg->beam_sweep()) {
                                for (size_t j = 0; j < num_bs_antenntas[c]; j++)
                                    beacon_weights[j]
                                        = CommsLib::hadamard2(ndx, j);
                            }
                            dev->writeRegisters(
                                tx_ram_wgt + ch, 0, beacon_weights);
                            ++ndx;
                        }
                        dev->writeSetting("BEACON_START",
                            std::to_string(bsRadios.at(c).size()));
                    }
                }
                for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
                    SoapySDR::Device* dev = bsRadios.at(c).at(i)->dev;
                    dev->writeSetting("TX_SW_DELAY",
                        "30"); // experimentally good value for dev front-end
                    dev->writeSetting("TDD_MODE", "true");
                }
            }

            if (!kUseUHD) {
                for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
                    SoapySDR::Device* dev = bsRadios.at(c).at(i)->dev;
                    bsRadios.at(c).at(i)->activateRecv();
                    bsRadios.at(c).at(i)->activateXmit();
                    dev->setHardwareTime(0, "TRIGGER");
                }
            } else {
                // Set freq and time source for multiple USRPs
                for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
                    SoapySDR::Device* dev = bsRadios.at(c).at(i)->dev;
                    dev->setClockSource("external");
                    dev->setTimeSource("external");
                    dev->setHardwareTime(0, "PPS");
                }
                // Wait for pps sync pulse
                std::this_thread::sleep_for(std::chrono::seconds(2));
                // Activate Rx and Tx streamers
                for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
                    bsRadios.at(c).at(i)->activateRecv();
                    bsRadios.at(c).at(i)->activateXmit();
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
            SoapySDR::Device::unmake(hubs.at(i));
    }
    for (unsigned int c = 0; c < _cfg->num_cells(); c++)
        for (size_t i = 0; i < _cfg->n_bs_sdrs().at(c); i++)
            delete bsRadios.at(c).at(i);
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
    std::atomic_ulong* thread_count = context->thread_count;
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
        bsRadios.at(c).at(i) = nullptr;
        bsRadios.at(c).at(i)
            = new Radio(args, SOAPY_SDR_CS16, channels, _cfg->rate());
    } catch (std::runtime_error& err) {
        if (kUseUHD == false) {
            std::cerr << "Ignoring iris " << _cfg->bs_sdr_ids().at(c).at(i)
                      << std::endl;
        } else {
            std::cerr << "Ignoring uhd device "
                      << _cfg->bs_sdr_ids().at(c).at(i) << std::endl;
        }
        if (bsRadios.at(c).at(i) != nullptr) {
            MLPD_TRACE("Deleting radio ptr due to exception\n");
            delete bsRadios.at(c).at(i);
            bsRadios.at(c).at(i) = nullptr;
        }
    }
    MLPD_TRACE("BaseRadioSet: Init complete\n");
    assert(thread_count->load() != 0);
    thread_count->store(thread_count->load() - 1);
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
    std::atomic_ulong* thread_count = context->thread_count;
    delete context;

    //load channels
    auto channels = Utils::strToChannels(_cfg->bs_channel());
    Radio* bsRadio = bsRadios.at(c).at(i);
    SoapySDR::Device* dev = bsRadio->dev;
    SoapySDR::Kwargs info = dev->getHardwareInfo();
    for (auto ch : channels) {
        double rxgain = _cfg->rx_gain().at(ch);
        double txgain = _cfg->tx_gain().at(ch);
        bsRadios.at(c).at(i)->dev_init(_cfg, ch, rxgain, txgain);
    }

    assert(thread_count->load() != 0);
    thread_count->store(thread_count->load() - 1);
}

SoapySDR::Device* BaseRadioSet::baseRadio(size_t cellId)
{
    if (cellId < hubs.size())
        return (hubs.at(cellId));
    if (cellId < bsRadios.size() && bsRadios.at(cellId).size() > 0)
        return bsRadios.at(cellId).at(0)->dev;
    return NULL;
}

void BaseRadioSet::sync_delays(size_t cellIdx)
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
        for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
            SoapySDR::Device* dev = bsRadios.at(c).at(i)->dev;
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
        for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
            if (!kUseUHD) {
                SoapySDR::Device* dev = bsRadios.at(c).at(i)->dev;
                dev->writeSetting("TDD_CONFIG", tddConfStr);
                dev->writeSetting("TDD_MODE", "false");
            }
            bsRadios.at(c)[i]->reset_DATA_clk_domain();
        }
    }
}

void BaseRadioSet::radioTx(const void* const* buffs)
{
    long long frameTime(0);
    for (size_t c = 0; c < _cfg->num_cells(); c++) {
        for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
            bsRadios.at(c).at(i)->xmit(
                buffs, _cfg->samps_per_symbol(), 0, frameTime);
        }
    }
}

int BaseRadioSet::radioTx(size_t radio_id, size_t cell_id,
    const void* const* buffs, int flags, long long& frameTime)
{
    int w;
    // for UHD device xmit from host using frameTimeNs
    if (!kUseUHD) {
        w = bsRadios.at(cell_id).at(radio_id)->xmit(
            buffs, _cfg->samps_per_symbol(), flags, frameTime);
    } else {
        long long frameTimeNs
            = SoapySDR::ticksToTimeNs(frameTime, _cfg->rate());
        w = bsRadios.at(cell_id).at(radio_id)->xmit(
            buffs, _cfg->samps_per_symbol(), flags, frameTimeNs);
    }
#if DEBUG_RADIO
    size_t chanMask;
    long timeoutUs(0);
    auto* dev = bsRadios.at(cell_id).at(radio_id)->dev;
    auto* txs = bsRadios.at(cell_id).at(radio_id)->txs;
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
        for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
            void* const* buff = buffs + (i * 2);
            bsRadios.at(c).at(i)->recv(
                buff, _cfg->samps_per_symbol(), frameTime);
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

    if (radio_id < bsRadios.at(cell_id).size()) {
        long long frameTimeNs = 0;
        ret = bsRadios.at(cell_id).at(radio_id)->recv(
            buffs, numSamps, frameTimeNs);
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
