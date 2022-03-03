#include "include/Radio.h"
#include "include/logger.h"
#include "include/macros.h"
#include <SoapySDR/Errors.hpp>
#include <iostream>

#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <complex>

#include <uhd/transport/udp_simple.hpp>
#include <fstream>
#include <sstream>
#include <string.h>

#include "include/comms-lib.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"
#include "nlohmann/json.hpp"
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>



void Radio::dev_init(Config* _cfg, int ch, double rxgain, double txgain)
{
    // these params are sufficient to set before DC offset and IQ imbalance calibration
    if (!kUseUHD) {
//        dev->setAntenna(SOAPY_SDR_RX, ch, "TRX");
//        dev->setBandwidth(SOAPY_SDR_RX, ch, _cfg->bw_filter());
//        dev->setBandwidth(SOAPY_SDR_TX, ch, _cfg->bw_filter());
//        dev->setFrequency(SOAPY_SDR_RX, ch, "BB", _cfg->nco());
//        dev->setFrequency(SOAPY_SDR_TX, ch, "BB", _cfg->nco());

        // update for UHD multi USRP
        dev->set_tx_antenna ("TX/RX", ch);
        dev->set_rx_bandwidth (_cfg->bw_filter(), ch);
        dev->set_tx_bandwidth (_cfg->bw_filter(), ch);
        uhd::tune_request_t tune_request(_cfg->nco(),ch);
        dev->set_rx_freq(tune_request,ch);
        dev->set_tx_freq(tune_request,ch);

    } else {
        MLPD_INFO("Init USRP channel: %d\n", ch);
//        dev->setAntenna(SOAPY_SDR_TX, ch, "TX/RX");
//        dev->setAntenna(SOAPY_SDR_RX, ch, "RX2"); // or "TX/RX"
//        dev->setFrequency(SOAPY_SDR_RX, ch, "BB", 0);
//        dev->setFrequency(SOAPY_SDR_RX, ch, "BB", 0);
//        dev->setFrequency(SOAPY_SDR_TX, ch, "BB", 0);

        // update for UHD multi USRP
        dev->set_tx_antenna ("TX/RX", ch);
        dev->set_rx_antenna ("RX2", ch);
        uhd::tune_request_t tune_request(0);
        dev->set_rx_freq(tune_request,ch);
        dev->set_tx_freq(tune_request,ch);
    }
//    dev->setFrequency(SOAPY_SDR_RX, ch, "RF", _cfg->radio_rf_freq());
//    dev->setFrequency(SOAPY_SDR_TX, ch, "RF", _cfg->radio_rf_freq());

    // update for UHD multi USRP
    uhd::tune_request_t tune_request(_cfg->radio_rf_freq());
    dev->set_rx_freq(tune_request,ch);
    dev->set_tx_freq(tune_request,ch);


    if (kUseUHD == false) {
        // Unified gains for both lime and frontend
        if (_cfg->single_gain()) {
//            dev->setGain(SOAPY_SDR_RX, ch, rxgain); // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
//            dev->setGain(SOAPY_SDR_TX, ch, txgain); // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
//            MLPD_INFO("Tx gain: %lf, Rx gain: %lf\n", dev->getGain(SOAPY_SDR_TX, ch), dev->getGain(SOAPY_SDR_RX, ch));

            // update for UHD multi USRP
            dev->set_rx_gain(rxgain, ch);
            dev->set_tx_gain(rxgain, ch);
            MLPD_INFO("Tx gain: %lf, Rx gain: %lf\n", dev->get_tx_gain(ch), dev->get_rx_gain(ch));

        } else {
//            dev->setGain(SOAPY_SDR_RX, ch, "LNA", std::min(30.0, rxgain)); // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
//            dev->setGain(SOAPY_SDR_RX, ch, "TIA", 0);
//            dev->setGain(SOAPY_SDR_RX, ch, "PGA", 0);
//            dev->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
//            dev->setGain(SOAPY_SDR_RX, ch, "ATTN", _cfg->radio_rf_freq() < 3e9 ? -12 : 0);
//            dev->setGain(SOAPY_SDR_TX, ch, "PAD", std::min(42.0, txgain)); // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
//            dev->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);
//            dev->setGain(SOAPY_SDR_TX, ch, "PA2", 0);
//            dev->setGain(SOAPY_SDR_TX, ch, "ATTN", -6);

            // update for UHD multi USRP
            dev->set_rx_gain(std::min(30.0, rxgain), "LNA", ch);
            dev->set_rx_gain(0, "TIA", ch);
            dev->set_rx_gain(0, "PGA", ch);
            dev->set_rx_gain(17, "LNA2", ch); // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
            dev->set_rx_gain(_cfg->radio_rf_freq() < 3e9 ? -12 : 0, "ATTN", ch);
            dev->set_tx_gain(std::min(42.0, txgain), "PAD", ch); //w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
            dev->set_tx_gain(0, "IAMP", ch);
            dev->set_tx_gain(0, "PA2", ch);
            dev->set_tx_gain(-6, "ATTN", ch);
        }
    } else {
//        dev->setGain(SOAPY_SDR_RX, ch, "PGA0", std::min(31.5, rxgain));
//        dev->setGain(SOAPY_SDR_TX, ch, "PGA0", std::min(31.5, txgain));

        // update for UHD multi USRP
        dev->set_rx_gain(std::min(31.5, rxgain), "PGA0", ch);
        dev->set_tx_gain(std::min(31.5, rxgain), "PGA0", ch);
    }

    // DC Offset for Iris
    if (!kUseUHD)
//        dev->setDCOffsetMode(SOAPY_SDR_RX, ch, true);

        // update for UHD multi USRP
        dev->set_rx_dc_offset(true,ch);
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
//    while (r != -1) {
//        r = dev->readStream(rxs, buffs.data(), symSamp, flags, frameTime, 0);
//        i++;
//    }
//    MLPD_TRACE("Number of reads needed to drain: %d\n", i);


    // update for UHD multi USRP
//    const size_t samps_per_buff = rxs->get_max_num_samps();
    r = rxs->recv(buffs, symSamp, rmd);
//    if (rmd.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT){
//        break;
//    }
    if (rmd.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
        std::cout<<"in drain_buffer"<<std::endl;
        throw std::runtime_error(
                str(boost::format("Receiver error %s") % rmd.strerror()));
    }
//    i = r/samps_per_buff;
    i = r/symSamp;
    MLPD_TRACE("Number of reads needed to drain: %d\n", i);
}

Radio::Radio(const std::map< std::string, std::string >& args, const char uhdFmt[], const std::vector<size_t>& channels, double rate)
{
//    dev = SoapySDR::Device::make(args);
    dev = uhd::usrp::multi_usrp::make(args);
//    uhd::stream_args_t stream_args("fc32");
    if (dev == NULL)
        throw std::invalid_argument("error making UHD:Device\n");
    for (auto ch : channels) {
//        dev->setSampleRate(SOAPY_SDR_RX, ch, rate);
//        dev->setSampleRate(SOAPY_SDR_TX, ch, rate);

        // update for UHD multi USRP
        dev->set_rx_rate(rate, ch);
        dev->set_tx_rate(rate, ch);
    }
//    std::string hostFormat;
//    for(const char ch : uhdFmt)
//    {
//        if (ch == 'C') hostFormat += "c";
//        else if (ch == 'F') hostFormat = "f" + hostFormat;
//        else if (ch == 'S') hostFormat = "s" + hostFormat;
//        else if (std::isdigit(ch)) hostFormat += ch;
////        else throw std::runtime_error("SoapyUHDDevice::setupStream("+uhdFmt+") unknown format");
//    }

    uhd::stream_args_t stream_args("fc32");
//    stream_args.channels = channels;
//    stream_args.args = kwargsToDict(args);
    if (args.count("WIRE") != 0) stream_args.otw_format = args.at("WIRE");

//    rxs = dev->setupStream(SOAPY_SDR_RX, soapyFmt, channels);
//    txs = dev->setupStream(SOAPY_SDR_TX, soapyFmt, channels);

    rxs = dev->get_rx_stream(stream_args);
//    txs = dev->get_tx_stream(stream_args);

//    txs = dev->get_tx_stream(stream_args);

//    uhd::stream_cmd_t stream_cmd_1(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
//    stream_cmd_rx = stream_cmd_1;
////
//    uhd::stream_cmd_t stream_cmd_2(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
//    stream_cmd_tx = stream_cmd_2;
    // initial consideration,
// not necessary, can be calimed in activate or deac
    // rx
//    uhd::stream_cmd_t stream_cmd_1(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
//    stream_cmd_rx = stream_cmd_1;
//    stream_cmd_rx.stream_now = false;
//    rxs->issue_stream_cmd(stream_cmd_rx);

    // tx
//    uhd::stream_cmd_t stream_cmd_2(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
//    stream_cmd_tx = stream_cmd_2;

    if (!kUseUHD)
        reset_DATA_clk_domain();
}

int Radio::activateRecv(
        const long long rxTime, const size_t numSamps, int flags)
{
    std::cout << "activate recv" << std::endl;
    int soapyFlags[]
            = { 0, SOAPY_SDR_HAS_TIME, SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
                SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST };
    int flag_args = soapyFlags[flags];
    // for USRP device start rx stream UHD_INIT_TIME_SEC sec in the future
    if (!kUseUHD) {
        // the following probabaly not going to be called, considered deleting, but decied to rewrite it to use UHD same
        // as the else condition
        uhd::stream_cmd_t stream_cmd_1(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
//        stream_cmd_rx = stream_cmd_1;
        stream_cmd_1.stream_now = true;
        rxs->issue_stream_cmd(stream_cmd_1);
        return 0;
    }
//        dev->activateStream(rxs, flag_args, rxTime, numSamps);
    else {
        uhd::stream_cmd_t::stream_mode_t mode;
        if (numSamps == 0){
            mode = uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS;
        }
        else if ((flags & SOAPY_SDR_END_BURST) != 0) {
            mode = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
        }
        else {mode = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE;}

        uhd::stream_cmd_t cmd(mode);
        cmd.stream_now = (flags & SOAPY_SDR_HAS_TIME) == 0;
        cmd.time_spec = uhd::time_spec_t::from_ticks(rxTime, 1e9);
        cmd.num_samps = numSamps;

        rxs->issue_stream_cmd(cmd);

//        uhd::stream_cmd_t stream_cmd_1(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
////        stream_cmd_rx = stream_cmd_1;
//        stream_cmd_1.stream_now = true;
//        rxs->issue_stream_cmd(stream_cmd_1);
        return 0;
    }
//        dev->activateStream(
//            rxs, SOAPY_SDR_HAS_TIME, UHD_INIT_TIME_SEC * 1e9, 0);
}

void Radio::activateXmit(void) {
    std::cout << "activate xmit" << std::endl;
    // for USRP device start tx stream UHD_INIT_TIME_SEC sec in the future
//    if (!kUseUHD) {
////        dev->activateStream(txs);
//    // update for UHD multi USRP
//        uhd::stream_cmd_t stream_cmd_2(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
////        stream_cmd_tx = stream_cmd_2;
//        stream_cmd_2.stream_now = true;
//        txs->issue_stream_cmd(stream_cmd_2);
//    }
//    else {
////        dev->activateStream(c
////            txs, SOAPY_SDR_HAS_TIME, UHD_INIT_TIME_SEC * 1e9, 0);
//        // update for UHD multi USRP
//        uhd::stream_cmd_t stream_cmd_2(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
////        stream_cmd_tx = stream_cmd_2;
//        stream_cmd_2.stream_now = true;
//        txs->issue_stream_cmd(stream_cmd_2);
//    }
    uhd::stream_args_t stream_args("fc32");
    txs = dev->get_tx_stream(stream_args);

//    _cfg(cfg);
//    auto channels_num = Utils::strToChannels(_cfg->cl_channel());
//    stream_args.channels = channels_num;
    std::cout << "no input yet, from example, seems no issue command is needed" << std::endl;
    std::cout << "init txmd" << std::endl;
    tmd.start_of_burst = true;
    tmd.end_of_burst = false;
    tmd.has_time_spec =true;
    auto delay = 0.1;
    tmd.time_spec = dev->get_time_now() + uhd::time_spec_t(delay);
//    tmd.time_spec = dev->get_time_now();

}

void Radio::deactivateRecv(void) {
//    dev->deactivateStream(rxs);
    uhd::stream_cmd_t stream_cmd_1(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
//    stream_cmd_rx = stream_cmd_1;
    rxs->issue_stream_cmd(stream_cmd_1);
}

void Radio::deactivateXmit(void) {
//    dev->deactivateStream(txs);
//    uhd::stream_cmd_t stream_cmd_2(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
//    stream_cmd_tx = stream_cmd_2;
//    txs->issue_stream_cmd(stream_cmd_2);
    tmd.end_of_burst=true;
    txs->send("", 0,tmd);
    std::cout << "no input yet, from example, seems no issue command is needed" << std::endl;
}

Radio::~Radio(void)
{
//    deactivateRecv();
//    deactivateXmit();
//    dev->closeStream(rxs);
//    dev->closeStream(txs);
//    SoapySDR::Device::unmake(dev);

    // update on UHD multi USRP
    deactivateRecv();
    deactivateXmit();
    // not sure if those deoncstructor is needed, will comment those out first
//    ~uhd::multi_usrp(dev);
//    ~rx_streamer(rxs);
//    ~tx_streamer(txs);
}


int Radio::recv(void* const* buffs, int samples, long long& frameTime)
{
    int flags(0);
//    int r = dev->readStream(rxs, buffs, samples, flags, frameTime, 1000000);
    int r = rxs->recv(buffs, samples, rmd, 1, (flags & SOAPY_SDR_ONE_PACKET) != 0);

    flags = 0;
    if (rmd.has_time_spec) flags |= SOAPY_SDR_HAS_TIME;
    if (rmd.end_of_burst) flags |= SOAPY_SDR_END_BURST;
    if (rmd.more_fragments) flags |= SOAPY_SDR_MORE_FRAGMENTS;
//    timeNs = rmd.time_spec.to_ticks(1e9);

//    if (r < 0) {
//        MLPD_ERROR("Time: %lld, readStream error: %d - %s, flags: %d\n",
//            frameTime, r, SoapySDR::errToStr(r), flags);
//        MLPD_TRACE("Samples: %d, Frame time: %lld\n", samples, frameTime);
//    } else if (r < samples) {
//        MLPD_WARN("Time: %lld, readStream returned less than requested "
//                  "samples: %d : %d, flags: %d\n",
//            frameTime, r, samples, flags);
//    }
//    if (rmd.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT)
//        break;
    if (rmd.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
        throw std::runtime_error(
                str(boost::format("Receiver error %s") % rmd.strerror()));
    }
    return r;
}

int Radio::xmit(
    const void* const* buffs, int samples, int flags, long long& frameTime)
{
//    tmd.start_of_burst = true;
//    tmd.end_of_burst = false;
//    tmd.has_time_spec =true;
//    auto delay = 0.1;
//    tmd.time_spec = dev->get_time_now() + uhd::time_spec_t(delay);
//    int soapyFlags[]
//        = { 0, SOAPY_SDR_HAS_TIME, SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
//              SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST };
//    int flag_args = soapyFlags[flags];
//    int r
//        = dev->writeStream(txs, buffs, samples, flag_args, frameTime, 1000000);
//    std::cout<<"sending_before"<<std::endl;
    tmd.has_time_spec = (flags & SOAPY_SDR_HAS_TIME) != 0;
    tmd.end_of_burst = (flags & SOAPY_SDR_END_BURST) != 0;
    tmd.time_spec = uhd::time_spec_t::from_ticks(frameTime, 1e9);
    int r = txs-> send (buffs, samples, tmd, 1);

//    flags = 0;
//    tmd.start_of_burst = false;
//    tmd.has_time_spec=false;
//    std::cout<<"sending_after"<<std::endl;

    if (r != samples)
        std::cerr << "unexpected writeStream error " << SoapySDR::errToStr(r)
                  << std::endl;
    return (r);
}


int Radio::getTriggers(void) const
{
//    return std::stoi(dev->readSetting("TRIGGER_COUNT"));
    return 0;
}

void Radio::reset_DATA_clk_domain(void)
{
//    dev->writeSetting("RESET_DATA_LOGIC", "");
    return;
}
