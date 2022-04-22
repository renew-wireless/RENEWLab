/**
 * @file BaseRadioSetUHD.h
 * @brief Declaration file for the BaseRadioSetUHD class.
 */

#ifndef BASE_RADIO_SET_UHD_H_
#define BASE_RADIO_SET_UHD_H_

#include "SoapySDR/Formats.hpp"
#include "SoapySDR/Time.hpp"
#include "uhd/usrp/multi_usrp.hpp"
#include "RadioUHD.h"

#include "config.h"
#include "nlohmann/json.hpp"

class BaseRadioSetUHD {
 public:
  BaseRadioSetUHD(Config* cfg);
  ~BaseRadioSetUHD(void);
  void radioTx(const void* const* buffs);
  void radioRx(void* const* buffs);
  int radioTx(size_t radio_id, size_t cell_id, const void* const* buffs,
              int flags, long long& frameTime);
  int radioRx(size_t radio_id, size_t cell_id, void* const* buffs,
              long long& frameTime);
  int radioRx(size_t radio_id, size_t cell_id, void* const* buffs, int numSamps,
              long long& frameTime);
  void radioStart(void);
  void radioStop(void);
  bool getRadioNotFound() { return radioNotFound; }

 private:
  // use for create pthread
  struct BaseRadioContext {
    BaseRadioSetUHD* brs;
    std::atomic_ulong* thread_count;
    size_t tid;
    size_t cell;
  };
  void init(BaseRadioContext* context);
  void configure(BaseRadioContext* context);

  static void* init_launch(void* in_context);
  static void* configure_launch(void* in_context);

  void radioTriggerUHD(void);
  void sync_delays(size_t cellIdx);
  //    SoapySDR::Device* baseRadio(size_t cellId);
  uhd::usrp::multi_usrp::sptr baseRadio(size_t cellId);
  int syncTimeOffsetUHD(bool, bool);
  void dciqCalibrationProcUHD(size_t){};
  void readSensors(void);

  Config* _cfg;
  //    std::vector<SoapySDR::Device*> hubs;
  uhd::usrp::multi_usrp::sptr hubs;
  //    std::vector<std::vector<Radio*>> bsRadios; // [cell, iris]
  RadioUHD* bsRadios;
  bool radioNotFound;
};

#endif /* BASE_RADIO_SET_UHD_H_ */
