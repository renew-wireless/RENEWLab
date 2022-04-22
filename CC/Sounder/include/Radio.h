/** @file Radio.h
  * @brief Declaration file for the Radio class.
  * 
  * Copyright (c) 2018-2022, Rice University 
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#ifndef RADIO_H_
#define RADIO_H_

#include <cstdlib>
#include <vector>

#include "SoapySDR/Device.hpp"
#include "SoapySDR/Types.hpp"
#include "config.h"

class Radio {
 public:
  inline SoapySDR::Device* RawDev() const { return dev_; };

  Radio(const SoapySDR::Kwargs& args, const char soapyFmt[],
        const std::vector<size_t>& channels);
  ~Radio(void);
  int recv(void* const* buffs, int samples, long long& frameTime);
  int activateRecv(const long long rxTime = 0, const size_t numSamps = 0,
                   int flags = 0);
  void deactivateRecv(void);
  int xmit(const void* const* buffs, int samples, int flags,
           long long& frameTime);
  void activateXmit(void);
  void deactivateXmit(void);
  int getTriggers(void) const;
  void drain_buffers(std::vector<void*> buffs, int symSamp);

  void reset_DATA_clk_domain(void);
  void dev_init(Config* _cfg, int ch, double rxgain, double txgain);

 private:
  SoapySDR::Device* dev_;
  SoapySDR::Stream* rxs_;
  SoapySDR::Stream* txs_;
};

#endif  // RADIO_H_