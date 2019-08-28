#include <iostream>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Time.hpp>
#include <cstdlib>
#include <cstddef>
#include <chrono>
#include <string>
#include <cstdint>
#include <complex>
#include <csignal>
#include "config.h"


class RadioConfig
{
public:
    RadioConfig(Config *cfg);
    static void *initBSRadio(void * context);
    void radioConfigure();
    void radioStart();
    void radioStop();
    void readSensors();
    void radioTx(const void *const *buffs);
    void radioRx(void *const *buffs);
    int radioTx(int, const void *const *buffs, int flags, long long & frameTime);
    int radioRx(int, void *const *buffs, long long & frameTime);
    void collectCSI(bool&);
    static void drain_buffers(SoapySDR::Device * ibsSdrs, SoapySDR::Stream * istream, std::vector<void *> buffs, int symSamp);
    void sync_delays(int cellIdx);

    ~RadioConfig();
    std::vector<SoapySDR::Device *> devs;
    std::vector<SoapySDR::Stream *> rxss;
    std::vector<SoapySDR::Stream *> txss;
    // use for create pthread 
    struct RadioConfigContext
    {
        RadioConfig *ptr;
        int tid;
        int cell;
    };

private:
    Config *_cfg;
    std::vector<SoapySDR::Device *> hubs;
    std::vector<std::vector<SoapySDR::Device *>> bsSdrs; // [cell, iris]
    std::vector<SoapySDR::Device *> clSdrs; 
    SoapySDR::Device *ref; 
    SoapySDR::Stream * refRxStream; 
    std::vector<std::vector<SoapySDR::Stream *>> bsTxStreams;
    std::vector<std::vector<SoapySDR::Stream *>> bsRxStreams;
    std::vector<SoapySDR::Stream *> clTxStreams;
    std::vector<SoapySDR::Stream *> clRxStreams;
    std::vector<std::complex<int16_t>> buff;
    std::vector<int> nBsSdrs;
    std::vector<int> nBsAntennas;
    std::vector<uint32_t> pilot_uint32;
    std::vector<uint32_t> dummy_uint32;

    int nClSdrs;
    int nClAntennas;
    std::atomic<int> remainingJobs;
    RadioConfigContext *context;
};
