#include "include/RadioUHD.h"
#include "include/logger.h"
#include "include/macros.h"
#include <SoapySDR/Errors.hpp>
#include <iostream>

#include <uhd/utils/thread.hpp>
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

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>
#include <uhd/version.hpp>
#include <uhd/device.hpp>
#ifdef UHD_HAS_MSG_HPP
#include <uhd/utils/msg.hpp>
#else
#include <uhd/utils/log_add.hpp>
#endif
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/version.hpp>
#include <cctype>
#include <iostream>



void RadioUHD::dev_init(Config* _cfg, int ch, double rxgain, double txgain)
{
    // these params are sufficient to set before DC offset and IQ imbalance calibration
    std::cout<<"radioUHD.cc being called"<<std::endl;

    dev->set_rx_rate(_cfg->rate(), ch);
    dev->set_tx_rate(_cfg->rate(), ch);

    MLPD_INFO("Init USRP channel: %d\n", ch);
    // update for UHD multi USRP
    dev->set_tx_antenna ("TX/RX", ch);
    dev->set_rx_antenna ("TX/RX", ch);
    uhd::tune_request_t tune_request(0);
    dev->set_rx_freq(tune_request,ch);
    dev->set_tx_freq(tune_request,ch);

    // update for UHD multi USRP
    tune_request = _cfg->radio_rf_freq();
    dev->set_rx_freq(tune_request,ch);
    dev->set_tx_freq(tune_request,ch);

    // update for UHD multi USRP
    dev->set_rx_gain(std::min(31.5, rxgain), "PGA0", ch);
    dev->set_tx_gain(std::min(31.5, rxgain), "PGA0", ch);
}

void RadioUHD::drain_buffers(std::vector<void*> buffs, int symSamp)
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
    while (r != -1) {
        uhd::rx_streamer::buffs_type stream_buffs(buffs.data(), rxs->get_num_channels());
        uhd::rx_metadata_t md;
        r = rxs->recv(stream_buffs, symSamp, md, 0, (flags & SOAPY_SDR_ONE_PACKET) != 0);
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            std::cout<<"in drain_buffer"<<std::endl;
            throw std::runtime_error(
                    str(boost::format("Receiver error %s") % md.strerror()));
        }
        i++;
    }

    MLPD_TRACE("Number of reads needed to drain: %d\n", i);
}

RadioUHD::RadioUHD(const std::map< std::string, std::string >& args, const char uhdFmt[], const std::vector<size_t>& channels)
{
//    dev = SoapySDR::Device::make(args);
    dev = uhd::usrp::multi_usrp::make(args);
//    uhd::stream_args_t stream_args("fc32");
    if (dev == NULL)
        throw std::invalid_argument("error making UHD:Device\n");
//    for (auto ch : channels) {
//        // update for UHD multi USRP
//        dev->set_rx_rate(rate, ch);
//        dev->set_tx_rate(rate, ch);
//    }
    const std::string &format = uhdFmt;
    std::string hostFormat;
    for(const char ch : format)
    {
        if (ch == 'C') hostFormat += "c";
        else if (ch == 'F') hostFormat = "f" + hostFormat;
        else if (ch == 'S') hostFormat = "s" + hostFormat;
        else if (std::isdigit(ch)) hostFormat += ch;
        else throw std::runtime_error("unknown format");
    }

    uhd::stream_args_t stream_args(hostFormat);
    stream_args.channels = channels;
    stream_args.args = args;
    if (args.count("WIRE") != 0) stream_args.otw_format = args.at("WIRE");

    uhd::stream_args_t stream_args1(hostFormat);
    stream_args1.channels = channels;
    stream_args1.args = args;
    if (args.count("WIRE") != 0) stream_args1.otw_format = args.at("WIRE");

    std::cout<< "format is " << hostFormat << std::endl;

    rxs = dev->get_rx_stream(stream_args);
    txs = dev->get_tx_stream(stream_args1);
}

int RadioUHD::activateRecv(
        const long long rxTime, const size_t numSamps, int flags)
{
    std::cout << "activate recv" << std::endl;
//    std::cout<<"flags are: " << flags<<std::endl;
    int soapyFlags[]
            = { 0, SOAPY_SDR_HAS_TIME, SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
                SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST };
    int flag_args = soapyFlags[flags];
    // for USRP device start rx stream UHD_INIT_TIME_SEC sec in the future
    uhd::stream_cmd_t::stream_mode_t mode;
    const size_t numElems = 0 ;
    if (numElems == 0){
        mode = uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS;
    }
    else if ((SOAPY_SDR_HAS_TIME & SOAPY_SDR_END_BURST) != 0) {
        mode = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
    }
    else {mode = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE;}

    uhd::stream_cmd_t cmd(mode);
    cmd.stream_now = (SOAPY_SDR_HAS_TIME & SOAPY_SDR_HAS_TIME) == 0;
    cmd.time_spec = uhd::time_spec_t::from_ticks(UHD_INIT_TIME_SEC * 1e9, 1e9);
    cmd.num_samps = 0;

    rxs->issue_stream_cmd(cmd);
    return 0;
}

void RadioUHD::activateXmit(void) {
    std::cout << "activate xmit" << std::endl;
    // for USRP device start tx stream UHD_INIT_TIME_SEC sec in the future
    std::cout << "no input yet, from example, seems no issue command is needed" << std::endl;
}

void RadioUHD::deactivateRecv(void) {
    uhd::stream_cmd_t stream_cmd_1(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    const int flags = 0;

    stream_cmd_1.stream_now = (flags & SOAPY_SDR_HAS_TIME) == 0;
    stream_cmd_1.time_spec = uhd::time_spec_t::from_ticks(0, 1e9);
    rxs->issue_stream_cmd(stream_cmd_1);
}

void RadioUHD::deactivateXmit(void) {
    std::cout << "no input yet, from example, seems no issue command is needed" << std::endl;
}

RadioUHD::~RadioUHD(void)
{
    deactivateRecv();
    deactivateXmit();
}

int RadioUHD::recv(void* const* buffs, int samples, long long& frameTime)
{
    uhd::rx_streamer::sptr &stream = rxs;
    int flags(0);
    uhd::rx_streamer::buffs_type stream_buffs(buffs, stream->get_num_channels());
    uhd::rx_metadata_t md;

    int r = stream->recv(stream_buffs, samples, md, 1, (flags & SOAPY_SDR_ONE_PACKET) != 0);

    flags = 0;
    if (md.has_time_spec) flags |= SOAPY_SDR_HAS_TIME;
    if (md.end_of_burst) flags |= SOAPY_SDR_END_BURST;
    if (md.more_fragments) flags |= SOAPY_SDR_MORE_FRAGMENTS;
    frameTime = md.time_spec.to_ticks(1e9);

    if (r < 0) {
        MLPD_ERROR("Time: %lld, readStream error: %d - %s, flags: %d\n",
                   frameTime, r, SoapySDR::errToStr(r), flags);
        MLPD_TRACE("Samples: %d, Frame time: %lld\n", samples, frameTime);
    } else if (r < samples) {
        MLPD_WARN("Time: %lld, readStream returned less than requested "
                  "samples: %d : %d, flags: %d\n",
                  frameTime, r, samples, flags);
    }

    switch (md.error_code)
    {
        case uhd::rx_metadata_t::ERROR_CODE_NONE: return r;
        case uhd::rx_metadata_t::ERROR_CODE_OVERFLOW: return SOAPY_SDR_OVERFLOW;
        case uhd::rx_metadata_t::ERROR_CODE_TIMEOUT: return SOAPY_SDR_TIMEOUT;
        case uhd::rx_metadata_t::ERROR_CODE_BAD_PACKET: return SOAPY_SDR_CORRUPTION;
        case uhd::rx_metadata_t::ERROR_CODE_ALIGNMENT: return SOAPY_SDR_CORRUPTION;
        case uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND: return SOAPY_SDR_STREAM_ERROR;
        case uhd::rx_metadata_t::ERROR_CODE_BROKEN_CHAIN: return SOAPY_SDR_STREAM_ERROR;
    }
}

int RadioUHD::xmit(
        const void* const* buffs, int samples, int flags, long long& frameTime)
{
    uhd::tx_streamer::sptr &stream = txs;
    int soapyFlags[]
            = { 0, SOAPY_SDR_HAS_TIME, SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
                SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST };
    int flag_args = soapyFlags[flags];
    uhd::tx_metadata_t md;

    md.has_time_spec = (flag_args & SOAPY_SDR_HAS_TIME) != 0;
    md.end_of_burst = (flag_args & SOAPY_SDR_END_BURST) != 0;
    md.time_spec = uhd::time_spec_t::from_ticks(frameTime, 1e9);

    uhd::tx_streamer::buffs_type stream_buffs(buffs, stream->get_num_channels());
    int r = stream-> send (stream_buffs, samples, md, 1);

    if (r == 0) return SOAPY_SDR_TIMEOUT;

    if (r != samples)
        std::cerr << "unexpected writeStream error " << SoapySDR::errToStr(r)
                  << std::endl;
    return (r);
}

int RadioUHD::getTriggers(void) const
{
    return 0;
}

void RadioUHD::reset_DATA_clk_domain(void)
{
    return;
}
