/** @file Radio.cc
  * @brief Defination file for the Radio class.
  *
  * Copyright (c) 2018-2022, Rice University
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
  * ----------------------------------------------------------
  * Initialize and Configure an SDR
  * ----------------------------------------------------------
*/
#include "include/Radio.h"

#include <iostream>

#include "SoapySDR/Errors.hpp"
#include "include/logger.h"
#include "include/macros.h"

void Radio::dev_init(Config* _cfg, int ch, double rxgain, double txgain) {
  // Set sampling rate
  dev_->setSampleRate(SOAPY_SDR_RX, ch, _cfg->rate());
  dev_->setSampleRate(SOAPY_SDR_TX, ch, _cfg->rate());

  // these params are sufficient to set before DC offset and IQ imbalance calibration
  if (!kUseSoapyUHD) {
    dev_->setAntenna(SOAPY_SDR_RX, ch, "TRX");
    dev_->setBandwidth(SOAPY_SDR_RX, ch, _cfg->bw_filter());
    dev_->setBandwidth(SOAPY_SDR_TX, ch, _cfg->bw_filter());
    dev_->setFrequency(SOAPY_SDR_RX, ch, "BB", _cfg->nco());
    dev_->setFrequency(SOAPY_SDR_TX, ch, "BB", _cfg->nco());
  } else {
    MLPD_INFO("Init USRP channel: %d\n", ch);
    dev_->setAntenna(SOAPY_SDR_TX, ch, "TX/RX");
    dev_->setAntenna(SOAPY_SDR_RX, ch, "RX2");  // or "TX/RX"
    dev_->setFrequency(SOAPY_SDR_RX, ch, "BB", 0);
    dev_->setFrequency(SOAPY_SDR_TX, ch, "BB", 0);
  }

  dev_->setFrequency(SOAPY_SDR_RX, ch, "RF", _cfg->radio_rf_freq());
  dev_->setFrequency(SOAPY_SDR_TX, ch, "RF", _cfg->radio_rf_freq());

  if (kUseSoapyUHD == false) {
    // Unified gains for both lime and frontend
    if (_cfg->single_gain()) {
      dev_->setGain(SOAPY_SDR_RX, ch,
                    rxgain);  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
      dev_->setGain(SOAPY_SDR_TX, ch,
                    txgain);  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
      MLPD_INFO("Tx gain: %lf, Rx gain: %lf\n", dev_->getGain(SOAPY_SDR_TX, ch),
                dev_->getGain(SOAPY_SDR_RX, ch));
    } else {
      dev_->setGain(
          SOAPY_SDR_RX, ch, "LNA",
          std::min(30.0, rxgain));  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
      dev_->setGain(SOAPY_SDR_RX, ch, "TIA", 0);
      dev_->setGain(SOAPY_SDR_RX, ch, "PGA", 0);
      dev_->setGain(SOAPY_SDR_RX, ch, "LNA2",
                    17);  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
      dev_->setGain(SOAPY_SDR_RX, ch, "ATTN",
                    _cfg->radio_rf_freq() < 3e9 ? -12 : 0);
      dev_->setGain(
          SOAPY_SDR_TX, ch, "PAD",
          std::min(42.0, txgain));  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
      dev_->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);
      dev_->setGain(SOAPY_SDR_TX, ch, "PA2", 0);
      dev_->setGain(SOAPY_SDR_TX, ch, "ATTN", -6);
    }
  } else {
    dev_->setGain(SOAPY_SDR_RX, ch, "PGA0", std::min(31.5, rxgain));
    dev_->setGain(SOAPY_SDR_TX, ch, "PGA0", std::min(31.5, txgain));
  }

  // DC Offset for Iris
  if (!kUseSoapyUHD) dev_->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
}

void Radio::drain_buffers(std::vector<void*> buffs, int symSamp) {
  /*
     *  "Drain" rx buffers during initialization
     *  Input:
     *      buffs   - Vector to which we will write received IQ samples
     *      symSamp - Number of samples
     *
     *  Output:
     *      None
     */
  long long frameTime = 0;
  int flags = 0, r = 0, i = 0;
  while (r != -1) {
    r = dev_->readStream(rxs_, buffs.data(), symSamp, flags, frameTime, 0);
    i++;
  }
  MLPD_TRACE("Number of reads needed to drain: %d\n", i);
}

Radio::Radio(const SoapySDR::Kwargs& args, const char soapyFmt[],
             const std::vector<size_t>& channels) {
  dev_ = SoapySDR::Device::make(args);
  if (dev_ == nullptr) {
    throw std::invalid_argument("error making SoapySDR::Device\n");
  }

  /* Moved to dev_init function (seems to fix the rate issue)
    for (auto ch : channels) {
        dev_->setSampleRate(SOAPY_SDR_RX, ch, rate);
        dev_->setSampleRate(SOAPY_SDR_TX, ch, rate);
    }*/
  rxs_ = dev_->setupStream(SOAPY_SDR_RX, soapyFmt, channels);
  txs_ = dev_->setupStream(SOAPY_SDR_TX, soapyFmt, channels);

  if (!kUseSoapyUHD) {
    reset_DATA_clk_domain();
  }
}

Radio::~Radio(void) {
  deactivateRecv();
  deactivateXmit();
  dev_->closeStream(rxs_);
  rxs_ = nullptr;
  dev_->closeStream(txs_);
  txs_ = nullptr;
  SoapySDR::Device::unmake(dev_);
  dev_ = nullptr;
}

int Radio::recv(void* const* buffs, int samples, long long& frameTime) {
  int flags(0);
  int r = dev_->readStream(rxs_, buffs, samples, flags, frameTime, 1000000);
  if (r < 0) {
    MLPD_ERROR("Time: %lld, readStream error: %d - %s, flags: %d\n", frameTime,
               r, SoapySDR::errToStr(r), flags);
    MLPD_TRACE("Samples: %d, Frame time: %lld\n", samples, frameTime);
  } else if (r < samples) {
    MLPD_WARN(
        "Time: %lld, readStream returned less than requested "
        "samples: %d : %d, flags: %d\n",
        frameTime, r, samples, flags);
  }

  return r;
}

int Radio::activateRecv(const long long rxTime, const size_t numSamps,
                        int flags) {
  int soapyFlags[] = {0, SOAPY_SDR_HAS_TIME,
                      SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
                      SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST};
  int flag_args = soapyFlags[flags];
  // for USRP device start rx stream UHD_INIT_TIME_SEC sec in the future
  if (!kUseSoapyUHD) {
    return dev_->activateStream(rxs_, flag_args, rxTime, numSamps);
  } else {
    return dev_->activateStream(rxs_, SOAPY_SDR_HAS_TIME,
                                UHD_INIT_TIME_SEC * 1e9, 0);
  }
}

void Radio::deactivateRecv(void) { dev_->deactivateStream(rxs_); }

int Radio::xmit(const void* const* buffs, int samples, int flags,
                long long& frameTime) {
  int soapyFlags[] = {0, SOAPY_SDR_HAS_TIME,
                      SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
                      SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST};
  int flag_args = soapyFlags[flags];
  int r =
      dev_->writeStream(txs_, buffs, samples, flag_args, frameTime, 1000000);
  if (r != samples) {
    std::cerr << "unexpected writeStream error " << SoapySDR::errToStr(r)
              << std::endl;
  }
  return (r);
}

void Radio::activateXmit(void) {
  // for USRP device start tx stream UHD_INIT_TIME_SEC sec in the future
  if (!kUseSoapyUHD) {
    dev_->activateStream(txs_);
  } else {
    dev_->activateStream(txs_, SOAPY_SDR_HAS_TIME, UHD_INIT_TIME_SEC * 1e9, 0);
  }
}

void Radio::deactivateXmit(void) { dev_->deactivateStream(txs_); }

int Radio::getTriggers(void) const {
  return std::stoi(dev_->readSetting("TRIGGER_COUNT"));
}

void Radio::reset_DATA_clk_domain(void) {
  dev_->writeSetting("RESET_DATA_LOGIC", "");
}
