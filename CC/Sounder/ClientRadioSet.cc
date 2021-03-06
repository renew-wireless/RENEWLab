/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
----------------------------------------------------------
 Initializes and Configures Client Radios 
----------------------------------------------------------
*/

#include "include/ClientRadioSet.h"
#include "include/Radio.h"
#include "include/comms-lib.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>

static void initAGC(SoapySDR::Device* dev, Config* cfg);

static void freeRadios(std::vector<Radio*>& radios)
{
    for (size_t i = 0; i < radios.size(); i++)
        delete radios.at(i);
}

ClientRadioSet::ClientRadioSet(Config* cfg)
    : _cfg(cfg)
{
    size_t num_radios = _cfg->num_cl_sdrs();

    //load channels
    auto channels = Utils::strToChannels(_cfg->cl_channel());
    radios.clear();
    radios.resize(num_radios);
    radioNotFound = false;
    std::vector<std::string> radioSerialNotFound;
    std::atomic_ulong thread_count = ATOMIC_VAR_INIT(num_radios);
    for (size_t i = 0; i < num_radios; i++) {
        ClientRadioContext* context = new ClientRadioContext;
        context->crs = this;
        context->thread_count = &thread_count;
        context->tid = i;
#ifdef THREADED_INIT
        pthread_t init_thread_;

        if (pthread_create(
                &init_thread_, NULL, ClientRadioSet::init_launch, context)
            != 0) {
            delete context;
            throw std::runtime_error(
                "ClientRadioSet - init thread create failed");
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
        if (radios.at(i) == nullptr) {
            radioNotFound = true;
            radioSerialNotFound.push_back(_cfg->cl_sdr_ids().at(i));
            while (num_radios != 0 && radios.at(num_radios - 1) == NULL) {
                --num_radios;
                radios.pop_back();
            }
            if (i < num_radios) {
                radios.at(i) = radios.at(--num_radios);
                radios.pop_back();
            }
        }
    }
    radios.shrink_to_fit();

    for (size_t i = 0; i < radios.size(); i++) {
        auto dev = radios.at(i)->dev;
        std::cout << _cfg->cl_sdr_ids().at(i) << ": Front end "
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
    if (radioNotFound == true) {
        for (auto st = radioSerialNotFound.begin();
             st != radioSerialNotFound.end(); st++)
            std::cout << "\033[1;31m" << *st << "\033[0m" << std::endl;
        std::cout << "\033[1;31mERROR: the above client serials were not "
                     "discovered in the network!\033[0m"
                  << std::endl;
    } else {
        //beaconSize + 82 (BS FE delay) + 68 (path delay) + 17 (correlator delay) + 82 (Client FE Delay)
        int clTrigOffset = _cfg->beacon_size() + _cfg->tx_advance();
        int sf_start = clTrigOffset / _cfg->samps_per_symbol();
        int sp_start = clTrigOffset % _cfg->samps_per_symbol();

        for (size_t i = 0; i < radios.size(); i++) {
            auto dev = radios.at(i)->dev;

            // hw_frame is only for Iris
            if (_cfg->hw_framer()) {
                std::string corrConfString
                    = "{\"corr_enabled\":true,\"corr_threshold\":"
                    + std::to_string(1) + "}";
                dev->writeSetting("CORR_CONFIG", corrConfString);
                dev->writeRegisters("CORR_COE", 0, _cfg->coeffs());

                std::string tddSched = _cfg->cl_frames().at(i);
                for (size_t s = 0; s < _cfg->cl_frames().at(i).size(); s++) {
                    char c = _cfg->cl_frames().at(i).at(s);
                    if (c == 'B')
                        tddSched.replace(s, 1, "G");
                    else if (c == 'U')
                        tddSched.replace(s, 1, "T");
                    else if (c == 'D')
                        tddSched.replace(s, 1, "R");
                }
                std::cout << "Client " << i << " schedule: " << tddSched
                          << std::endl;
#ifdef JSON
                json tddConf;
                tddConf["tdd_enabled"] = true;
                tddConf["frame_mode"] = _cfg->frame_mode();
                int max_frame_ = (int)(2.0
                    / ((_cfg->samps_per_symbol() * _cfg->symbols_per_frame())
                          / _cfg->rate()));
                tddConf["max_frame"]
                    = _cfg->frame_mode() == "free_running" ? 0 : max_frame_;
                //std::cout << "max_frames for client " << i << " is " << max_frame_ << std::endl;
                if (_cfg->cl_sdr_ch() == 2)
                    tddConf["dual_pilot"] = true;
                tddConf["frames"] = json::array();
                tddConf["frames"].push_back(tddSched);
                tddConf["symbol_size"] = _cfg->samps_per_symbol();
                std::string tddConfStr = tddConf.dump();
#else
                std::string tddConfStr = "{\"tdd_enabled\":true,\"frame_mode\":"
                    + _cfg->frame_mode() + ",";
                tddConfStr += "\"symbol_size\":"
                    + std::to_string(_cfg->samps_per_symbol());
                if (_cfg->cl_sdr_ch() == 2)
                    tddConfStr += "\"dual_pilot\":true,";
                tddConfStr += ",\"frames\":[\"" + tddSched[i] + "\"]}";
                std::cout << tddConfStr << std::endl;
#endif
                dev->writeSetting("TDD_CONFIG", tddConfStr);

                dev->setHardwareTime(
                    SoapySDR::ticksToTimeNs(
                        (sf_start << 16) | sp_start, _cfg->rate()),
                    "TRIGGER");
                dev->writeSetting("TX_SW_DELAY",
                    "30"); // experimentally good value for dev front-end
                dev->writeSetting("TDD_MODE", "true");
                // write pilot to FPGA buffers
                for (char const& c : _cfg->cl_channel()) {
                    std::string tx_ram = "TX_RAM_";
                    dev->writeRegisters(tx_ram + c, 0, _cfg->pilot());
                }
                radios.at(i)->activateRecv();
                radios.at(i)->activateXmit();
                if (_cfg->frame_mode() == "free_running")
                    dev->writeSetting("TRIGGER_GEN", "");
                else
                    dev->writeSetting(
                        "CORR_START", (_cfg->cl_channel() == "B") ? "B" : "A");
            } else {
                if (!kUseUHD) {
                    dev->setHardwareTime(0, "TRIGGER");
                    radios.at(i)->activateRecv();
                    radios.at(i)->activateXmit();
                    dev->writeSetting("TRIGGER_GEN", "");
                } else {
                    // For USRP clients always use the internal clock
                    dev->setTimeSource("internal");
                    dev->setClockSource("internal");
                    dev->setHardwareTime(0, "UNKNOWN_PPS");
                    radios.at(i)->activateRecv();
                    radios.at(i)->activateXmit();
                }
            }
        }
        MLPD_INFO("%s done!\n", __func__);
    }
}

void* ClientRadioSet::init_launch(void* in_context)
{
    ClientRadioContext* context
        = reinterpret_cast<ClientRadioContext*>(in_context);
    context->crs->init(context);
    return 0;
}

void ClientRadioSet::init(ClientRadioContext* context)
{
    int i = context->tid;
    std::atomic_ulong* thread_count = context->thread_count;
    delete context;

    MLPD_TRACE("Deleting context for tid: %d\n", i);

    bool has_runtime_error(false);
    auto channels = Utils::strToChannels(_cfg->cl_channel());
    MLPD_TRACE("ClientRadioSet setting up radio: %zu : %zu\n", (i + 1),
        _cfg->num_cl_sdrs());
    SoapySDR::Kwargs args;
    args["timeout"] = "1000000";
    if (kUseUHD == false) {
        args["driver"] = "iris";
        args["serial"] = _cfg->cl_sdr_ids().at(i);
    } else {
        args["driver"] = "uhd";
        args["addr"] = _cfg->cl_sdr_ids().at(i);
    }
    try {
        radios.at(i) = nullptr;
        radios.at(i) = new Radio(args, SOAPY_SDR_CF32, channels, _cfg->rate());
    } catch (std::runtime_error& err) {
        has_runtime_error = true;

        if (radios.at(i) != nullptr) {
            MLPD_TRACE("Radio not used due to exception\n");
            delete radios.at(i);
            radios.at(i) = nullptr;
        }
    } catch (...) {
        MLPD_WARN("Unknown exception\n");
        if (radios.at(i) != nullptr) {
            delete radios.at(i);
            radios.at(i) = nullptr;
        }
        throw;
    }
    if (has_runtime_error == false) {
        auto dev = radios.at(i)->dev;
        SoapySDR::Kwargs info = dev->getHardwareInfo();

        for (auto ch : channels) {
            double rxgain = _cfg->cl_rxgain_vec().at(ch).at(
                i); // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
            double txgain = _cfg->cl_txgain_vec().at(ch).at(
                i); // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
            radios.at(i)->dev_init(_cfg, ch, rxgain, txgain);
        }

        // Init AGC only for Iris device
        if (kUseUHD == false) {
            initAGC(dev, _cfg);
        }
    }
    MLPD_TRACE("BaseRadioSet: Init complete\n");
    assert(thread_count->load() != 0);
    thread_count->store(thread_count->load() - 1);
}

ClientRadioSet::~ClientRadioSet(void) { freeRadios(radios); }

void ClientRadioSet::radioStop(void)
{
    if (_cfg->hw_framer()) {
        std::string corrConfStr = "{\"corr_enabled\":false}";
        std::string tddConfStr = "{\"tdd_enabled\":false}";
        for (Radio* stop_radio : radios) {
            auto dev = stop_radio->dev;
            dev->writeSetting("CORR_CONFIG", corrConfStr);
            const auto timeStamp = SoapySDR::timeNsToTicks(
                dev->getHardwareTime(""), _cfg->rate());
            std::cout << "device " << dev->getHardwareKey()
                      << ": Frame=" << (timeStamp >> 32)
                      << ", Symbol=" << ((timeStamp & 0xFFFFFFFF) >> 16)
                      << std::endl;
            dev->writeSetting("TDD_CONFIG", tddConfStr);
            dev->writeSetting("TDD_MODE", "false");
            stop_radio->reset_DATA_clk_domain();
        }
    } else {
        for (Radio* stop_radio : radios) {
            stop_radio->deactivateRecv();
            MLPD_TRACE("Stopping radio...\n");
        }
    }
}

int ClientRadioSet::triggers(int i) { return (radios.at(i)->getTriggers()); }

int ClientRadioSet::radioRx(
    size_t radio_id, void* const* buffs, int numSamps, long long& frameTime)
{
    if (radio_id < radios.size()) {
        int ret(0);
        if (_cfg->hw_framer()) {
            ret = radios.at(radio_id)->recv(buffs, numSamps, frameTime);
        } else {
            long long frameTimeNs(0);
            ret = radios.at(radio_id)->recv(buffs, numSamps, frameTimeNs);
            frameTime = SoapySDR::timeNsToTicks(frameTimeNs, _cfg->rate());
#if DEBUG_RADIO
            if (frameTimeNs < 2e9)
                std::cout << "client " << radio_id << " received " << ret
                          << " at " << frameTimeNs << std::endl;
#endif
        }
        return ret;
    }
    std::cout << "invalid radio id " << radio_id << std::endl;
    return 0;
}

int ClientRadioSet::radioTx(size_t radio_id, const void* const* buffs,
    int numSamps, int flags, long long& frameTime)
{
    if (_cfg->hw_framer()) {
        return radios.at(radio_id)->xmit(buffs, numSamps, flags, frameTime);
    } else {
        long long frameTimeNs
            = SoapySDR::ticksToTimeNs(frameTime, _cfg->rate());
        return radios.at(radio_id)->xmit(buffs, numSamps, flags, frameTimeNs);
    }
}

static void initAGC(SoapySDR::Device* dev, Config* cfg)
{
    /*
     * Initialize AGC parameters
     */
#ifdef JSON
    json agcConf;
    agcConf["agc_enabled"] = cfg->cl_agc_en();
    agcConf["agc_gain_init"] = cfg->cl_agc_gain_init();
    std::string agcConfStr = agcConf.dump();
#else
    std::string agcConfStr
        = "{\"agc_enabled\":" + cfg->cl_agc_en() ? "true" : "false";
    agcConfStr
        += ",\"agc_gain_init\":" + std::to_string(cfg->cl_agc_gain_init());
    agcConfStr += "}";
#endif
    dev->writeSetting("AGC_CONFIG", agcConfStr);
}
