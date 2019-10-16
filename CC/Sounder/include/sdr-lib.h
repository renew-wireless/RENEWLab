#include "config.h"
#include <SoapySDR/Device.hpp>
#include <chrono>
#include <complex>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

class Radio {
private:
    SoapySDR::Device* dev;
    SoapySDR::Stream* rxs;
    SoapySDR::Stream* txs;
    friend class RadioConfig;

public:
    Radio(const SoapySDR::Kwargs& args, const char soapyFmt[], const std::vector<size_t>& channels);
    ~Radio(void);
    int recv(void* const* buffs, int samples, long long& frameTime);
    int xmit(const void* const* buffs, int samples, int flags, long long& frameTime);
    int getTriggers(void) const;
    void drain_buffers(std::vector<void*> buffs, int symSamp);
};

class RadioConfig {
public:
    RadioConfig(Config* cfg);
    ~RadioConfig();
    void radioTx(const void* const* buffs);
    void radioRx(void* const* buffs);
    int radioTx(size_t, const void* const* buffs, int flags, long long& frameTime);
    int radioRx(size_t, void* const* buffs, long long& frameTime);
    Radio* getRadio(int i);
    void radioStart();
    void radioStop();
    void radioConfigure();

private:
    std::vector<Radio*> radios;
    // use for create pthread
    struct RadioConfigContext {
        RadioConfig* ptr;
        int tid;
        int cell;
    };
    void initBSRadio(RadioConfigContext* context);

    static void* initBSRadio_launch(void* in_context);
    void readSensors();

    void radioTrigger();
    void initAGC(SoapySDR::Device* iclSdrs);
    void sync_delays(int cellIdx);
    SoapySDR::Device* baseRadio(int cellId);
    void collectCSI(bool&);
    Config* _cfg;
    std::vector<SoapySDR::Device*> hubs;
    std::vector<std::vector<Radio*>> bsRadios; // [cell, iris]
    std::vector<int> nBsAntennas;
#if 0
    std::vector<SoapySDR::Device*> clSdrs;
    std::vector<SoapySDR::Stream*> clTxStreams;
    std::vector<SoapySDR::Stream*> clRxStreams;
    SoapySDR::Device* ref;
    SoapySDR::Stream* refRxStream;
    std::vector<std::complex<int16_t>> buff;
    std::vector<uint32_t> pilot_uint32;
    std::vector<uint32_t> dummy_uint32;
    int nClAntennas;
#endif
    std::atomic<int> remainingJobs;
};
