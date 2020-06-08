#include "config.h"
#include <SoapySDR/Device.hpp>

class Radio;

class ClientRadioSet {
public:
    ClientRadioSet(Config* cfg);
    ~ClientRadioSet(void);
    int triggers(int i);
    int radioRxSched(size_t radio_id, const long long rxTime = 0, const size_t numSamps = 0, int flags = 0);
    int radioRx(size_t radio_id, void* const* buffs, int numSamps, long long& frameTime);
    int radioTx(size_t radio_id, const void* const* buffs, int numSamps, int flags, long long& frameTime);
    void radioStop(void);
    bool getRadioNotFound() { return radioNotFound; }

private:
    Config* _cfg;
    std::vector<Radio*> radios;
    bool radioNotFound;
};
