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
 private:
  uhd::usrp::multi_usrp::sptr dev_;
  uhd::rx_streamer::sptr rxs_;
  uhd::tx_streamer::sptr txs_;

 public:
  inline uhd::usrp::multi_usrp::sptr RawDev() const { return dev_; };

  void drain_buffers(std::vector<void*> buffs, int symSamp);
  RadioUHD(const std::map<std::string, std::string>& args, const char uhdFmt[],
           const std::vector<size_t>& channels, Config* _cfg);
  void activateXmit(void);
  ~RadioUHD(void);
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
};

#endif /* RADIOUHD_H_ */
