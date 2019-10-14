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
    ~Radio(void);
};

class RadioConfig {
public:
    RadioConfig(Config* cfg);
    ~RadioConfig();
    void radioTx(const void* const* buffs);
    void radioRx(void* const* buffs);
    int radioTx(size_t, const void* const* buffs, int flags, long long& frameTime);
    int radioRx(size_t, void* const* buffs, long long& frameTime);
    void radioStart();
    void radioStop();
    void radioConfigure();
    void dciqCalibrationProc(size_t);
    std::vector<struct Radio> radios;

private:
    // use for create pthread
    struct RadioConfigContext {
        RadioConfig* ptr;
        int tid;
        int cell;
    };
    void initBSRadioPreCal(RadioConfigContext* context);
    void initBSRadioPostCal(RadioConfigContext* context);

    static void* initBSRadioPreCal_launch(void* in_context);
    static void* initBSRadioPostCal_launch(void* in_context);
    void readSensors();

    void radioTrigger();
    void initAGC(SoapySDR::Device* iclSdrs);
    void sync_delays(int cellIdx);
    SoapySDR::Device* baseRadio(int cellId);
    void collectCSI(bool&);
    static void drain_buffers(SoapySDR::Device* ibsSdrs, SoapySDR::Stream* istream, std::vector<void*> buffs, int symSamp);
    static void dciqMinimize(SoapySDR::Device *, SoapySDR::Device *, size_t, int, double, double);
    static void setIQBalance(SoapySDR::Device *, size_t, int, int, int);
    static void adjustCalibrationGains(std::vector<SoapySDR::Device *>, SoapySDR::Device *, size_t, double);
    std::vector<std::complex<float>> snoopSamples(size_t, size_t, size_t);
    static std::vector<std::complex<float>> snoopSamples(SoapySDR::Device *, size_t, size_t);
    Config* _cfg;
    std::vector<SoapySDR::Device*> hubs;
    std::vector<std::vector<struct Radio>> bsRadios; // [cell, iris]
    std::vector<int> nBsAntennas;
    std::vector<int> nBsRadios;
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
