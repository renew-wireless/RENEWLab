/**
 * @file BaseRadioSet.h
 * @brief Declaration file for the BaseRadioSet class.
 */
#ifndef BASE_RADIO_SET_H_
#define BASE_RADIO_SET_H_

#include <atomic>
#include <cstddef>
#include <vector>

#include "Radio.h"
#include "SoapySDR/Device.hpp"
#include "config.h"

class BaseRadioSet {
 public:
  BaseRadioSet(Config* cfg, const bool calibrate_proc);
  ~BaseRadioSet(void);
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
  void adjustDelays(void);

 private:
  // use for create pthread
  struct BaseRadioContext {
    BaseRadioSet* brs;
    std::atomic_ulong* thread_count;
    size_t tid;
    size_t cell;
  };
  void init(BaseRadioContext* context);
  void configure(BaseRadioContext* context);

  static void* init_launch(void* in_context);
  static void* configure_launch(void* in_context);

  void radioTrigger(void);
  void sync_delays(size_t cellIdx);
  SoapySDR::Device* baseRadio(size_t cellId);
  int syncTimeOffset();
  void dciqCalibrationProc(size_t);
  void readSensors(void);

  Config* _cfg;
  std::vector<SoapySDR::Device*> hubs;
  std::vector<std::vector<Radio*>> bsRadios;  // [cell, iris]
  std::vector<int> trigger_offsets_;
  bool radioNotFound;
};

#endif  // BASE_RADIO_SET_H_
