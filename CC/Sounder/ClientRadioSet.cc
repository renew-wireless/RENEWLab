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
#include "include/macros.h"
#include "include/utils.h"
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>

static void initAGC(SoapySDR::Device* dev, Config* cfg);

static void
freeRadios(std::vector<Radio*>& radios)
{
    for (size_t i = 0; i < radios.size(); i++)
        delete radios[i];
}

ClientRadioSet::ClientRadioSet(Config* cfg)
    : _cfg(cfg)
{
    //load channels
    auto channels = Utils::strToChannels(_cfg->clChannel);
    radios.reserve(_cfg->nClSdrs);
    radioNotFound = false;
    std::vector<std::string> radioSerialNotFound;
    for (size_t i = 0; i < _cfg->nClSdrs; i++) {
        SoapySDR::Kwargs args;
        args["timeout"] = "1000000";
        args["serial"] = _cfg->cl_sdr_ids.at(i);
        try {
            radios.push_back(new Radio(args, SOAPY_SDR_CF32, channels, _cfg->rate));
        } catch (std::runtime_error) {
            //std::cerr << "Ignoring serial " << _cfg->cl_sdr_ids.at(i) << std::endl;
            radioSerialNotFound.push_back(_cfg->cl_sdr_ids.at(i));
            radioNotFound = true;
            continue;
        }
        auto dev = radios.back()->dev;
        SoapySDR::Kwargs info = dev->getHardwareInfo();

        for (auto ch : { 0, 1 }) //channels)
        {
            double rxgain = _cfg->clRxgain_vec[ch][i]; // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
            double txgain = _cfg->clTxgain_vec[ch][i]; // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
            radios.back()->dev_init(_cfg, ch, rxgain, txgain);
        }

        initAGC(dev, _cfg);
    }
    radios.shrink_to_fit();

    if (radioNotFound) {
        for (auto st = radioSerialNotFound.begin(); st != radioSerialNotFound.end(); st++)
            std::cout << "\033[1;31m" << *st << "\033[0m" << std::endl;
        std::cout << "\033[1;31mERROR: the above client serials were not discovered in the network!\033[0m" << std::endl;
    } else {
        //beaconSize + 82 (BS FE delay) + 68 (path delay) + 17 (correlator delay) + 82 (Client FE Delay)
        int clTrigOffset = _cfg->beaconSize + _cfg->txAdvance;
        int sf_start = clTrigOffset / _cfg->sampsPerSymbol;
        int sp_start = clTrigOffset % _cfg->sampsPerSymbol;

        for (size_t i = 0; i < radios.size(); i++) {
            auto dev = radios[i]->dev;
            if (_cfg->hw_framer) {
                std::string corrConfString = "{\"corr_enabled\":true,\"corr_threshold\":" + std::to_string(1) + "}";
                dev->writeSetting("CORR_CONFIG", corrConfString);
                dev->writeRegisters("CORR_COE", 0, _cfg->coeffs);

                std::string tddSched = _cfg->clFrames[i];
                for (size_t s = 0; s < _cfg->clFrames[i].size(); s++) {
                    char c = _cfg->clFrames[i].at(s);
                    if (c == 'B')
                        tddSched.replace(s, 1, "G");
                    else if (c == 'U')
                        tddSched.replace(s, 1, "T");
                    else if (c == 'D')
                        tddSched.replace(s, 1, "R");
                }
                std::cout << "Client " << i << " schedule: " << tddSched << std::endl;
#ifdef JSON
                json tddConf;
                tddConf["tdd_enabled"] = true;
                tddConf["frame_mode"] = _cfg->frame_mode;
                int max_frame_ = (int)(2.0 / ((_cfg->sampsPerSymbol * _cfg->symbolsPerFrame) / _cfg->rate));
                tddConf["max_frame"] = _cfg->frame_mode == "free_running" ? 0 : max_frame_;
                //std::cout << "max_frames for client " << i << " is " << max_frame_ << std::endl;
                if (_cfg->clSdrCh == 2)
                    tddConf["dual_pilot"] = true;
                tddConf["frames"] = json::array();
                tddConf["frames"].push_back(tddSched);
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
                // write pilot to FPGA buffers
                for (char const& c : _cfg->clChannel) {
                    std::string tx_ram = "TX_RAM_";
                    dev->writeRegisters(tx_ram + c, 0, _cfg->pilot);
                }
                radios[i]->activateRecv();
                radios[i]->activateXmit();
                if (_cfg->frame_mode == "free_running")
                    dev->writeSetting("TRIGGER_GEN", "");
                else
                    dev->writeSetting("CORR_START", (_cfg->clChannel == "B") ? "B" : "A");
            } else {
                dev->setHardwareTime(0, "TRIGGER");
                radios[i]->activateRecv();
                radios[i]->activateXmit();
                dev->writeSetting("TRIGGER_GEN", "");
            }
        }
        std::cout << __func__ << " done!" << std::endl;
    }
}

ClientRadioSet::~ClientRadioSet(void)
{
    freeRadios(radios);
}

void ClientRadioSet::radioStop(void)
{
    if (_cfg->hw_framer) {
        std::string corrConfStr = "{\"corr_enabled\":false}";
        std::string tddConfStr = "{\"tdd_enabled\":false}";
        for (size_t i = 0; i < _cfg->nClSdrs; i++) {
            auto dev = radios[i]->dev;
            dev->writeSetting("CORR_CONFIG", corrConfStr);
            const auto timeStamp = SoapySDR::timeNsToTicks(dev->getHardwareTime(""), _cfg->rate);
            std::cout << "device " << i << ": Frame=" << (timeStamp >> 32)
                      << ", Symbol=" << ((timeStamp & 0xFFFFFFFF) >> 16) << std::endl;
            dev->writeSetting("TDD_CONFIG", tddConfStr);
            dev->writeSetting("TDD_MODE", "false");
            radios[i]->reset_DATA_clk_domain();
        }
    } else {
        for (size_t i = 0; i < _cfg->nClSdrs; i++)
            radios[i]->deactivateRecv();
    }
}

int ClientRadioSet::triggers(int i)
{
    return (radios[i]->getTriggers());
}

int ClientRadioSet::radioRx(size_t radio_id, void* const* buffs, int numSamps, long long& frameTime)
{
    if (radio_id < radios.size()) {
	int ret(0);
        if (_cfg->hw_framer) {
            ret = radios[radio_id]->recv(buffs, numSamps, frameTime);
	} else {
            long long frameTimeNs(0);
            ret = radios[radio_id]->recv(buffs, numSamps, frameTimeNs);
            frameTime = SoapySDR::timeNsToTicks(frameTimeNs, _cfg->rate);
	}
	return ret;
    }
    std::cout << "invalid radio id " << radio_id << std::endl;
    return 0;
}

int ClientRadioSet::radioTx(size_t radio_id, const void* const* buffs, int numSamps, int flags, long long& frameTime)
{
    if (_cfg->hw_framer) {
        return radios[radio_id]->xmit(buffs, numSamps, flags, frameTime);
    } else { 
        long long frameTimeNs = SoapySDR::ticksToTimeNs(frameTime, _cfg->rate);
        return radios[radio_id]->xmit(buffs, numSamps, flags, frameTimeNs);
    }
}

static void initAGC(SoapySDR::Device* dev, Config* cfg)
{
    /*
     * Initialize AGC parameters
     */
#ifdef JSON
    json agcConf;
    agcConf["agc_enabled"] = cfg->clAgcEn;
    agcConf["agc_gain_init"] = cfg->clAgcGainInit;
    std::string agcConfStr = agcConf.dump();
#else
    std::string agcConfStr = "{\"agc_enabled\":" + cfg->clAgcEn ? "true" : "false";
    agcConfStr += ",\"agc_gain_init\":" + std::to_string(cfg->clAgcGainInit);
    agcConfStr += "}";
#endif
    dev->writeSetting("AGC_CONFIG", agcConfStr);
}
