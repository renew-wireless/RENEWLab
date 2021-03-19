/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
*/

#include "config.h"
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Time.hpp>

class Radio {
private:
    SoapySDR::Device* dev;
    SoapySDR::Stream* rxs;
    SoapySDR::Stream* txs;
    void reset_DATA_clk_domain(void);
    void dev_init(Config* _cfg, int ch, double rxgain, double txgain);
    friend class ClientRadioSet;
    friend class BaseRadioSet;

public:
    Radio(const SoapySDR::Kwargs& args, const char soapyFmt[],
        const std::vector<size_t>& channels, double rate);
    ~Radio(void);
    int recv(void* const* buffs, int samples, long long& frameTime);
    int activateRecv(
        const long long rxTime = 0, const size_t numSamps = 0, int flags = 0);
    void deactivateRecv(void);
    int xmit(
        const void* const* buffs, int samples, int flags, long long& frameTime);
    void activateXmit(void);
    void deactivateXmit(void);
    int getTriggers(void) const;
    void drain_buffers(std::vector<void*> buffs, int symSamp);
};
