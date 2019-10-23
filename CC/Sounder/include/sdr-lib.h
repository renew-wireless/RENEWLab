#include "config.h"
#include <SoapySDR/Device.hpp>
#include <chrono>
#include <complex>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

class Radio;

class ClientRadioSet {
public:
    ClientRadioSet(Config* cfg);
    ~ClientRadioSet(void);
    int triggers(int i);
    int radioRx(size_t radio_id, void* const* buffs, int numSamps, long long& frameTime);
    int radioTx(size_t radio_id, const void* const* buffs, int numSamps, int flags, long long& frameTime);
    void radioStop(void);

private:
    void initAGC(SoapySDR::Device* iclSdrs);
    Config* _cfg;
    std::vector<Radio*> radios;
};

class BaseRadioSet {
public:
    BaseRadioSet(Config* cfg);
    ~BaseRadioSet(void);
    void radioTx(const void* const* buffs);
    void radioRx(void* const* buffs);
    int radioTx(size_t radio_id, const void* const* buffs, int flags, long long& frameTime);
    int radioRx(size_t radio_id, void* const* buffs, long long& frameTime);
    void radioStart(void);
    void radioStop(void);

private:
    // use for create pthread
    struct BaseRadioContext {
        BaseRadioSet* brs;
        std::atomic_int* threadCount;
        int tid;
        int cell;
    };
    void init(BaseRadioContext* context);
    void configure(BaseRadioContext* context);

    static void* init_launch(void* in_context);
    static void* configure_launch(void* in_context);

    void radioTrigger(void);
    void sync_delays(int cellIdx);
    SoapySDR::Device* baseRadio(size_t cellId);
    void collectCSI(bool&);
    static void dciqMinimize(SoapySDR::Device*, SoapySDR::Device*, int, size_t, double, double);
    static void setIQBalance(SoapySDR::Device*, int, size_t, int, int);
    static void adjustCalibrationGains(std::vector<SoapySDR::Device*>, SoapySDR::Device*, size_t, double, bool plot = false);
    static std::vector<std::complex<float>> snoopSamples(SoapySDR::Device*, size_t, size_t);
    void dciqCalibrationProc(size_t);
    void readSensors(void);

    Config* _cfg;
    std::vector<SoapySDR::Device*> hubs;
    std::vector<std::vector<Radio*>> bsRadios; // [cell, iris]
};
