#include "config.h"
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>
#include <chrono>
#include <complex>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

struct Radio {
    SoapySDR::Device* dev;
    SoapySDR::Stream* rxs;
    SoapySDR::Stream* txs;
};

class RadioConfig {
public:
    RadioConfig(Config* cfg);
    static void* initBSRadio_launch(void* in_context);
    void radioConfigure();
    void radioStart();
    void radioStop();
    void readSensors();
    void radioTx(const void* const* buffs);
    void radioRx(void* const* buffs);
    int radioTx(int, const void* const* buffs, int flags, long long& frameTime);
    int radioRx(int, void* const* buffs, long long& frameTime);
    void sync_delays(int cellIdx);

    ~RadioConfig();
    std::vector<struct Radio> radios;
    // use for create pthread
    struct RadioConfigContext {
        RadioConfig* ptr;
        int tid;
        int cell;
    };
    void initBSRadio(RadioConfigContext* context);

private:
    void collectCSI(bool&);
    static void drain_buffers(SoapySDR::Device* ibsSdrs, SoapySDR::Stream* istream, std::vector<void*> buffs, int symSamp);
    Config* _cfg;
    std::vector<SoapySDR::Device*> hubs;
    std::vector<std::vector<SoapySDR::Device*>> bsSdrs; // [cell, iris]
    std::vector<std::vector<SoapySDR::Stream*>> bsTxStreams;
    std::vector<std::vector<SoapySDR::Stream*>> bsRxStreams;
    int nClSdrs;
    std::vector<int> nBsSdrs;
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
