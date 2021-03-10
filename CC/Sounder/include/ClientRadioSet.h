#include "config.h"
#include <SoapySDR/Device.hpp>

#ifndef CLIENT_RADIO_SET_H
#define CLIENT_RADIO_SET_H

#pragma once

class Radio;

class ClientRadioSet {
public:
    ClientRadioSet(Config* cfg);
    ~ClientRadioSet(void);
    int triggers(int i);
    int radioRx(size_t radio_id, void* const* buffs, int numSamps,
        long long& frameTime);
    int radioTx(size_t radio_id, const void* const* buffs, int numSamps,
        int flags, long long& frameTime);
    void radioStop(void);
    bool getRadioNotFound() { return radioNotFound; }

private:
    struct ClientRadioContext {
        ClientRadioSet* crs;
        std::atomic_ulong* thread_count;
        size_t tid;
    };
    void init(ClientRadioContext* context);
    static void* init_launch(void* in_context);

    Config* _cfg;
    std::vector<Radio*> radios;
    bool radioNotFound;
};

#endif /* CLIENT_RADIO_SET_H */
