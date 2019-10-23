#include <SoapySDR/Device.hpp>

class Radio {
private:
    SoapySDR::Device* dev;
    SoapySDR::Stream* rxs;
    SoapySDR::Stream* txs;
    void reset_DATA_clk_domain(void);
    friend class ClientRadioSet;
    friend class BaseRadioSet;

public:
    Radio(const SoapySDR::Kwargs& args, const char soapyFmt[], const std::vector<size_t>& channels, double rate);
    ~Radio(void);
    int recv(void* const* buffs, int samples, long long& frameTime);
    int activateRecv(const long long rxTime = 0, const size_t numSamps = 0);
    void deactivateRecv(void);
    int xmit(const void* const* buffs, int samples, int flags, long long& frameTime);
    void activateXmit(void);
    void deactivateXmit(void);
    int getTriggers(void) const;
    void drain_buffers(std::vector<void*> buffs, int symSamp);
};
