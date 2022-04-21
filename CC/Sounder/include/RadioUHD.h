/*
 Copyright (c) 2018-2019, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

*/

#include "config.h"
#include <uhd/usrp/multi_usrp.hpp>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>


#ifndef RADIO_UHD_H
#define RADIO_UHD_H


class RadioUHD {
private:
    uhd::usrp::multi_usrp::sptr dev;
    uhd::rx_streamer::sptr rxs;
    uhd::tx_streamer::sptr txs;

    void reset_DATA_clk_domain(void);
    void dev_init(Config* _cfg, int ch, double rxgain, double txgain);
    friend class ClientRadioSetUHD;
    friend class BaseRadioSetUHD;

public:
    void drain_buffers(std::vector<void*> buffs, int symSamp);
    RadioUHD(const std::map< std::string, std::string >& args, const char uhdFmt[],
          const std::vector<size_t>& channels);


    void activateXmit(void);

    ~RadioUHD(void);
    int recv(void* const* buffs, int samples, long long& frameTime);
    int activateRecv(
            const long long rxTime = 0, const size_t numSamps = 0, int flags = 0);
    void deactivateRecv(void);
    int xmit(
            const void* const* buffs, int samples, int flags, long long& frameTime);

    void deactivateXmit(void);
    int getTriggers(void) const;
};

#endif /* RADIO_UHD_H */

