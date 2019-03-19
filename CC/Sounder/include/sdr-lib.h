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
    void radioStart();
    void radioStop();
    void readSensors();
    void radioTx(void ** buffs);
    void radioRx(void ** buffs);
    void radioTx(int, void ** buffs, int flags, long long & frameTime);
    int radioRx(int, void ** buffs, long long & frameTime);
    void radioSched(std::vector<std::string> sched);
    void reciprocityCalProcedure(std::vector<void *> &tx, std::vector<void *> &rx);
    void sampleDelayCalibrate();
    ~RadioConfig();
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
    int nClSdrs;
    int nClAntennas;
};
