/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
*/

#include "config.h"
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Time.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/stream.hpp>
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

//class Radio {
//private:
//    SoapySDR::Device* dev;
//    SoapySDR::Stream* rxs;
//    SoapySDR::Stream* txs;
//    void reset_DATA_clk_domain(void);
//    void dev_init(Config* _cfg, int ch, double rxgain, double txgain);
//    friend class ClientRadioSet;
//    friend class BaseRadioSet;
//
//public:
//    Radio(const SoapySDR::Kwargs& args, const char soapyFmt[],
//        const std::vector<size_t>& channels, double rate);
//    ~Radio(void);
//    int recv(void* const* buffs, int samples, long long& frameTime);
//    int activateRecv(
//        const long long rxTime = 0, const size_t numSamps = 0, int flags = 0);
//    void deactivateRecv(void);
//    int xmit(
//        const void* const* buffs, int samples, int flags, long long& frameTime);
//    void activateXmit(void);
//    void deactivateXmit(void);
//    int getTriggers(void) const;
//    void drain_buffers(std::vector<void*> buffs, int symSamp);
//};

class Radio {
private:
    uhd::usrp::multi_usrp::sptr* dev;
    uhd::rx_streamer::sptr* rxs;
    uhd::tx_streamer::sptr* txs;
    uhd::stream_cmd_t stream_cmd_rx;
    uhd::stream_cmd_t stream_cmd_tx;
    uhd::rx_metadata_t rmd;
    uhd::tx_metadata_t tmd;

    void reset_DATA_clk_domain(void);
    void dev_init(Config* _cfg, int ch, double rxgain, double txgain);
    friend class ClientRadioSet;
    friend class BaseRadioSet;

public:
    void drain_buffers(std::vector<void*> buffs, int symSamp);
    Radio(const std::map< std::string, std::string >& args, const char uhdFmt[],
          const std::vector<size_t>& channels, double rate);
    void activateXmit(void);


    ~Radio(void);
    int recv(void* const* buffs, int samples, long long& frameTime);
    int activateRecv(
            const long long rxTime = 0, const size_t numSamps = 0, int flags = 0);
    void deactivateRecv(void);
    int xmit(
            const void* const* buffs, int samples, int flags, long long& frameTime);

    void deactivateXmit(void);
    int getTriggers(void) const;
};
