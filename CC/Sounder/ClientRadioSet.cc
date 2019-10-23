#include "include/ClientRadioSet.h"
#include "include/Radio.h"
#include "include/comms-lib.h"
#include "include/macros.h"
#include "include/sdr-lib.h"
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>

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
    radios.reserve(_cfg->nClSdrs);
    for (size_t i = 0; i < _cfg->nClSdrs; i++) {
        SoapySDR::Kwargs args;
        args["timeout"] = "1000000";
        args["serial"] = _cfg->cl_sdr_ids.at(i);
        try {
            radios.push_back(new Radio(args, SOAPY_SDR_CF32, channels, _cfg->rate));
        } catch (std::runtime_error) {
            std::cerr << "Ignoring serial " << _cfg->cl_sdr_ids.at(i) << std::endl;
            continue;
        }
        auto dev = radios.back()->dev;
        SoapySDR::Kwargs info = dev->getHardwareInfo();

        for (auto ch : { 0, 1 }) //channels)
        {
            double rxgain = _cfg->clRxgain_vec[ch][i]; //[0,30]
            double txgain = _cfg->clTxgain_vec[ch][i]; //[0,52]
            // No client calibration for now!
            radios.back()->dev_init(_cfg, ch, rxgain, txgain, info["frontend"]);
        }

        initAGC(dev);
    }
    radios.shrink_to_fit();

    int ueTrigOffset = 505; //_cfg->prefix + 256 + _cfg->postfix + 17 + _cfg->prefix;
    int sf_start = ueTrigOffset / _cfg->sampsPerSymbol;
    int sp_start = ueTrigOffset % _cfg->sampsPerSymbol;

    for (size_t i = 0; i < radios.size(); i++) {
        auto dev = radios[i]->dev;
        dev->writeRegister("IRIS30", CORR_CONF, 0x1);
        for (int k = 0; k < 128; k++)
            dev->writeRegister("ARGCOE", k * 4, 0);
        usleep(100000);

        dev->writeRegister("IRIS30", 64, 1); // reset faros_corr
        dev->writeRegister("IRIS30", 64, 0); // unreset faros_corr
        dev->writeRegister("IRIS30", 92, 1); // threshold is left-shifted by this many bits

        for (int k = 0; k < 128; k++)
            dev->writeRegister("ARGCOE", k * 4, _cfg->coeffs[k]);

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
        json conf;
        conf["tdd_enabled"] = true;
        conf["frame_mode"] = _cfg->frame_mode;
        int max_frame_ = (int)(2.0 / ((_cfg->sampsPerSymbol * _cfg->symbolsPerFrame) / _cfg->rate));
        conf["max_frame"] = max_frame_;
        if (_cfg->clSdrCh == 2)
            conf["dual_pilot"] = true;
        //std::cout << "max_frames for client " << i << " is " << max_frame_ << std::endl;

        conf["frames"] = json::array();
        conf["frames"].push_back(tddSched[i]);
        conf["symbol_size"] = _cfg->sampsPerSymbol;
        std::string confString = conf.dump();
#else
        std::string confString = "{\"tdd_enabled\":true,\"frame_mode\":" + _cfg->frame_mode + ",";
        confString += "\"symbol_size\":" + std::to_string(_cfg->sampsPerSymbol);
        if (_cfg->clSdrCh == 2)
            confString += "\"dual_pilot\":true,";
        confString += ",\"frames\":[\"" + tddSched[i] + "\"]}";
        std::cout << confString << std::endl;
#endif
        dev->writeSetting("TDD_CONFIG", confString);

        dev->setHardwareTime(SoapySDR::ticksToTimeNs((sf_start << 16) | sp_start, _cfg->rate), "TRIGGER");
        dev->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
        dev->writeSetting("TDD_MODE", "true");
        // write beacons to FPGA buffers
        int val = 0;
        for (char const& c : _cfg->bsChannel) {
            std::string tx_ram = "TX_RAM_";
            dev->writeRegisters(tx_ram + c, val, _cfg->beacon);
            val += 2048;
        }
        radios[i]->activateRecv();
        radios[i]->activateXmit();

        if (_cfg->bsChannel != "B") // A or AB
            dev->writeRegister("IRIS30", CORR_CONF, 0x11);
        else
            dev->writeRegister("IRIS30", CORR_CONF, 0x31);
    }
    std::cout << __func__ << " done!" << std::endl;
}

ClientRadioSet::~ClientRadioSet(void)
{
    for (size_t i = 0; i < radios.size(); i++)
        delete radios[i];
}

void ClientRadioSet::radioStop(void)
{
    for (size_t i = 0; i < radios.size(); i++) {
        auto dev = radios[i]->dev;
        dev->writeRegister("IRIS30", CORR_CONF, 0);
        std::cout << "device " << i << " T=" << std::hex << SoapySDR::timeNsToTicks(dev->getHardwareTime(""), _cfg->rate) << std::dec << std::endl;
        for (int i = 0; i < _cfg->symbolsPerFrame; i++) {
            dev->writeRegister("RFCORE", SCH_ADDR_REG, i);
            dev->writeRegister("RFCORE", SCH_MODE_REG, 0);
        }
        dev->writeSetting("TDD_MODE", "false");
        radios[i]->reset_DATA_clk_domain();
        //SoapySDR::Device::unmake(dev);
    }
}

int ClientRadioSet::triggers(int i)
{
    return (radios[i]->getTriggers());
}

int ClientRadioSet::radioRx(size_t radio_id, void* const* buffs, int numSamps, long long& frameTime)
{
    if (radio_id < radios.size()) {
        long long frameTimeNs = 0;
        int ret = radios[radio_id]->recv(buffs, numSamps, frameTimeNs);
        frameTime = frameTimeNs; //SoapySDR::timeNsToTicks(frameTimeNs, _rate);
        return ret;
    }
    std::cout << "invalid radio id " << radio_id << std::endl;
    return 0;
}

int ClientRadioSet::radioTx(size_t radio_id, const void* const* buffs, int numSamps, int flags, long long& frameTime)
{
    return radios[radio_id]->xmit(buffs, numSamps, flags, frameTime);
}

void ClientRadioSet::initAGC(SoapySDR::Device* iclSdr)
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
