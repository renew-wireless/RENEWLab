#include "include/Radio.h"
#include "include/macros.h"
#include <SoapySDR/Errors.hpp>
#include <iostream>

void Radio::dev_init(Config* _cfg, int ch, double rxgain, double txgain)
{
    // these params are sufficient to set before DC offset and IQ imbalance calibration
    dev->setAntenna(SOAPY_SDR_RX, ch, "TRX");
    dev->setBandwidth(SOAPY_SDR_RX, ch, _cfg->bwFilter);
    dev->setBandwidth(SOAPY_SDR_TX, ch, _cfg->bwFilter);

    dev->setFrequency(SOAPY_SDR_RX, ch, "RF", _cfg->radioRfFreq);
    dev->setFrequency(SOAPY_SDR_RX, ch, "BB", _cfg->nco);
    dev->setFrequency(SOAPY_SDR_TX, ch, "RF", _cfg->radioRfFreq);
    dev->setFrequency(SOAPY_SDR_TX, ch, "BB", _cfg->nco);

    // Unified gains for both lime and frontend
    //dev->setGain(SOAPY_SDR_RX, ch, rxgain); // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
    //dev->setGain(SOAPY_SDR_TX, ch, txgain); // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
    dev->setGain(SOAPY_SDR_RX, ch, "LNA", std::min(30.0, rxgain)); // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
    dev->setGain(SOAPY_SDR_RX, ch, "TIA", 0);
    dev->setGain(SOAPY_SDR_RX, ch, "PGA", 0);
    dev->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
    dev->setGain(SOAPY_SDR_RX, ch, "ATTN", 0);
    dev->setGain(SOAPY_SDR_TX, ch, "PAD", std::min(42.0, txgain)); // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
    dev->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);
    dev->setGain(SOAPY_SDR_TX, ch, "PA2", 0);
    dev->setGain(SOAPY_SDR_TX, ch, "ATTN", -6);

    // DC Offset
    dev->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
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
    if (dev == NULL)
        throw std::invalid_argument("error making SoapySDR::Device\n");
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
    return std::stoi(dev->readSetting("TRIGGER_COUNT"));
}

void Radio::reset_DATA_clk_domain(void)
{
    dev->writeSetting("RESET_DATA_LOGIC", "");
}
