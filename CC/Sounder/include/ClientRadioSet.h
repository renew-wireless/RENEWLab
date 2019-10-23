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
