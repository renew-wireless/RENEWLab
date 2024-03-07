/** @file RadioUHD.h
  * @brief Declaration file for the RadioUHD class.
  *
  * Copyright (c) 2018-2022, Rice University
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#ifndef RADIOUHD_H_
#define RADIOUHD_H_

#include <cstdlib>
#include <string>
#include <vector>

#include "config.h"
#include "uhd/usrp/multi_usrp.hpp"

class RadioUHD {
 public:
  RadioUHD(const std::map<std::string, std::string>& args, const char uhdFmt[],
           const std::vector<size_t>& channels, Config* _cfg);

  //Radio(const SoapySDR::Kwargs& args, const char soapyFmt[],
  //      const std::vector<size_t>& channels);
  ~RadioUHD(void);

  void drain_buffers(std::vector<void*> buffs, int symSamp);
  void activateXmit(void);
  int recv(void* const* buffs, int samples, long long& frameTime);
  int activateRecv(const long long rxTime = 0, const size_t numSamps = 0,
                   int flags = 0);
  void deactivateRecv(void);
  int xmit(const void* const* buffs, int samples, int flags,
           long long& frameTime);

  void deactivateXmit(void);
  int getTriggers(void) const;

  void dev_init(Config* _cfg, int ch, double rxgain, double txgain);
  void reset_DATA_clk_domain(void);
  inline uhd::usrp::multi_usrp::sptr RawDev() const { return dev_; };

 private:
  uhd::usrp::multi_usrp::sptr dev_ = nullptr;
  uhd::rx_streamer::sptr rxs_ = nullptr;
  uhd::tx_streamer::sptr txs_ = nullptr;
};

#endif /* RADIOUHD_H_ */
