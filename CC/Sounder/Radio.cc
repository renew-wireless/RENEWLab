#include "include/Radio.h"
#include "include/macros.h"
#include <SoapySDR/Errors.hpp>
#include <iostream>

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
    while (r != -1) {
        r = dev->readStream(rxs, buffs.data(), symSamp, flags, frameTime, 0);
        i++;
    }
    //std::cout << "Number of reads needed to drain: " << i << std::endl;
}

Radio::Radio(const SoapySDR::Kwargs& args, const char soapyFmt[],
    const std::vector<size_t>& channels, double rate)
{
    dev = SoapySDR::Device::make(args);
    for (auto ch : channels) {
        dev->setSampleRate(SOAPY_SDR_RX, ch, rate);
        dev->setSampleRate(SOAPY_SDR_TX, ch, rate);
    }
    rxs = dev->setupStream(SOAPY_SDR_RX, soapyFmt, channels);
    txs = dev->setupStream(SOAPY_SDR_TX, soapyFmt, channels);
    reset_DATA_clk_domain();
}

Radio::~Radio(void)
{
    deactivateRecv();
    deactivateXmit();
    dev->closeStream(rxs);
    dev->closeStream(txs);
    SoapySDR::Device::unmake(dev);
}

int Radio::recv(void* const* buffs, int samples, long long& frameTime)
{
    int flags(0);
    return dev->readStream(rxs, buffs, samples, flags, frameTime, 1000000);
}

int Radio::activateRecv(const long long rxTime, const size_t numSamps)
{
    return dev->activateStream(rxs, 0, rxTime, numSamps);
}

void Radio::deactivateRecv(void)
{
    dev->deactivateStream(rxs);
}

int Radio::xmit(const void* const* buffs, int samples, int flags, long long& frameTime)
{
    int soapyFlags[] = {
        0,
        SOAPY_SDR_HAS_TIME,
        SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
        SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST
    };
    int flag_args = soapyFlags[flags];
    int r = dev->writeStream(txs, buffs, samples, flag_args, frameTime, 1000000);
    if (r != samples)
        std::cerr << "unexpected writeStream error " << SoapySDR::errToStr(r) << std::endl;
    return (r);
}

void Radio::activateXmit(void)
{
    dev->activateStream(txs);
}

void Radio::deactivateXmit(void)
{
    dev->deactivateStream(txs);
}

int Radio::getTriggers(void) const
{
    return (dev->readRegister("IRIS30", 92));
}

void Radio::reset_DATA_clk_domain(void)
{
    // What does 29 mean?
    dev->writeRegister("IRIS30", RF_RST_REG, (1 << 29) | 1);
    dev->writeRegister("IRIS30", RF_RST_REG, (1 << 29));
    dev->writeRegister("IRIS30", RF_RST_REG, 0);
}
