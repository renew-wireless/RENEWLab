/**
 * @file ClientRadioSetUHD.h
 * @brief Declaration file for the ClientRadioSetUHD class.
 */
#ifndef CLIENT_RADIO_SET_UHD_H_
#define CLIENT_RADIO_SET_UHD_H_

#include <atomic>
#include <cstddef>

#include "RadioUHD.h"
#include "config.h"

#include "SoapySDR/Errors.hpp"
#include "SoapySDR/Formats.hpp"
#include "SoapySDR/Time.hpp"

#include "nlohmann/json.hpp"

class ClientRadioSetUHD {
 public:
  ClientRadioSetUHD(Config* cfg);
  ~ClientRadioSetUHD(void);
  int triggers(int i);
  int radioRx(size_t radio_id, void* const* buffs, int numSamps,
              long long& frameTime);
  int radioTx(size_t radio_id, const void* const* buffs, int numSamps,
              int flags, long long& frameTime);
  void radioStop(void);
  bool getRadioNotFound() { return radioNotFound; }

 private:
  struct ClientRadioContext {
    ClientRadioSetUHD* crs;
    std::atomic_ulong* thread_count;
    size_t tid;
  };
  void init(ClientRadioContext* context);
  static void* init_launch(void* in_context);

  Config* _cfg;
  RadioUHD* radios;
  bool radioNotFound;
};

#endif /* CLIENT_RADIO_SET_UHD_H_ */
