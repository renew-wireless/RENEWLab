#include "config.h"
#include <SoapySDR/Device.hpp>
#include <chrono>
#include <complex>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>

class Radio {
private:
    SoapySDR::Device* dev;
    SoapySDR::Stream* rxs;
    SoapySDR::Stream* txs;
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

class ClientRadioSet {
public:
    ClientRadioSet(Config* cfg);
    ~ClientRadioSet(void);
    Radio* getRadio(int i);
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
    SoapySDR::Device* baseRadio(int cellId);
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
