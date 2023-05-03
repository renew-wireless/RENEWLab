/** @file ClientRadioSetUHD.cc
  * @brief Defination file for the ClientRadioSetUHD class.
  *
  * Copyright (c) 2018-2022, Rice University
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
  * ----------------------------------------------------------
  *  Initializes and Configures Client Radios
  * ----------------------------------------------------------
  */
#include "include/ClientRadioSetUHD.h"

#include "SoapySDR/Formats.h"
#include "SoapySDR/Time.hpp"
#include "include/RadioUHD.h"
#include "include/comms-lib.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"

ClientRadioSetUHD::ClientRadioSetUHD(Config* cfg) : _cfg(cfg) {
  size_t num_radios = _cfg->num_cl_sdrs();
  //load channels
  auto channels = Utils::strToChannels(_cfg->cl_channel());
  // Update for UHD multi USRP
  radioNotFound = false;
  std::atomic_ulong thread_count = ATOMIC_VAR_INIT(num_radios);
  for (size_t i = 0; i < num_radios; i++) {
    ClientRadioContext* context = new ClientRadioContext;
    context->crs = this;
    context->thread_count = &thread_count;
    context->tid = i;
#if defined(THREADED_INIT)
    pthread_t init_thread_;

    if (pthread_create(&init_thread_, NULL, ClientRadioSetUHD::init_launch,
                       context) != 0) {
      delete context;
      throw std::runtime_error("ClientRadioSet - init thread create failed");
    }
#else
    init(context);
#endif
  }
  // Wait for init
  while (thread_count.load() > 0) {
  }

  if (num_radios != radio_->RawDev()->get_num_mboards()) {
    radioNotFound = true;
  }

  // update for UHD multi USRP
  for (size_t i = 0; i < radio_->RawDev()->get_num_mboards(); i++) {
    auto dev = radio_->RawDev();
    std::cout << _cfg->cl_sdr_ids().at(i) << ": Front end "
              << dev->get_usrp_rx_info()["frontend"] << std::endl;
  }

  for (auto ch : channels) {
    if (ch < radio_->RawDev()->get_rx_num_channels()) {
      printf("RX Channel %zu\n", ch);
      printf("Actual RX sample rate: %fMSps...\n",
             (radio_->RawDev()->get_rx_rate(ch) / 1e6));
      printf("Actual RX frequency: %fGHz...\n",
             (radio_->RawDev()->get_rx_freq(ch) / 1e9));
      printf("Actual RX gain: %f...\n", (radio_->RawDev()->get_rx_gain(ch)));

      printf("Actual RX bandwidth: %fM...\n",
             (radio_->RawDev()->get_rx_bandwidth(ch) / 1e6));
      printf("Actual RX antenna: %s...\n",
             (radio_->RawDev()->get_rx_antenna(ch).c_str()));
    }
  }

  for (auto ch : channels) {
    if (ch < radio_->RawDev()->get_tx_num_channels()) {
      printf("TX Channel %zu\n", ch);
      printf("Actual TX sample rate: %fMSps...\n",
             (radio_->RawDev()->get_tx_rate(ch) / 1e6));
      printf("Actual TX frequency: %fGHz...\n",
             (radio_->RawDev()->get_tx_freq(ch) / 1e9));
      printf("Actual TX gain: %f...\n", (radio_->RawDev()->get_tx_gain(ch)));
      printf("Actual TX bandwidth: %fM...\n",
             (radio_->RawDev()->get_tx_bandwidth(ch) / 1e6));
      printf("Actual TX antenna: %s...\n",
             (radio_->RawDev()->get_tx_antenna(ch).c_str()));
    }
  }

  // Update for UHD multi USRP
  if (radioNotFound == true) {
    std::cout << "some client serials were not "
                 "discovered in the network!"
              << std::endl;
  } else {
    const auto cl_timing_source = _cfg->getClTimingSrc(0);
    const auto cl_clock_source = _cfg->getClClockSrc(0);
    radio_->RawDev()->set_time_source(cl_timing_source);
    radio_->RawDev()->set_clock_source(cl_clock_source);
    radio_->RawDev()->set_time_unknown_pps(uhd::time_spec_t(0.0));
    /*MLPD_INFO(
        "USRP UE Clock source requested %s, actual %s, Time Source requested "
        "%s, actual  %s\n",
        cl_clock_source.c_str(),
        radio_->RawDev()
            ->get_clock_source(uhd::usrp::multi_usrp::ALL_MBOARDS)
            .c_str(),
        cl_clock_source.c_str(),
        radio_->RawDev()
            ->get_time_source(uhd::usrp::multi_usrp::ALL_MBOARDS)
            .c_str());
    */
    radio_->activateRecv();
    radio_->activateXmit();
    MLPD_INFO("%s done!\n", __func__);
  }
}

void* ClientRadioSetUHD::init_launch(void* in_context) {
  ClientRadioContext* context =
      reinterpret_cast<ClientRadioContext*>(in_context);
  context->crs->init(context);
  return 0;
}

void ClientRadioSetUHD::init(ClientRadioContext* context) {
  int i = context->tid;
  std::atomic_ulong* thread_count = context->thread_count;
  delete context;

  MLPD_TRACE("Deleting context for tid: %d\n", i);

  bool has_runtime_error(false);
  auto channels = Utils::strToChannels(_cfg->cl_channel());
  MLPD_TRACE("ClientRadioSet setting up radio: %zu : %zu\n", (i + 1),
             _cfg->num_cl_sdrs());

  // update for UHD multi USRP
  std::map<std::string, std::string> args;
  args["timeout"] = "1000000";
  args["driver"] = "uhd";
  args["serial"] = _cfg->cl_sdr_ids().at(i);

  try {
    radio_ = nullptr;
    radio_ = new RadioUHD(args, SOAPY_SDR_CS16, channels, _cfg);
  } catch (std::runtime_error& err) {
    has_runtime_error = true;

    if (radio_ != nullptr) {
      MLPD_TRACE("Radio not used due to exception\n");
      radio_ = nullptr;
    }
  } catch (...) {
    MLPD_WARN("Unknown exception\n");
    if (radio_ != nullptr) {
      delete radio_;
      radio_ = nullptr;
    }
    throw;
  }
  if (has_runtime_error == false) {
    for (auto ch : channels) {
      auto new_ch = _cfg->cl_channel();
      double rxgain = _cfg->cl_rxgain_vec().at(ch).at(
          i);  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
      double txgain = _cfg->cl_txgain_vec().at(ch).at(
          i);  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]

      auto total_rx_channel = 2;
      radio_->dev_init_set_freq(_cfg, total_rx_channel);
      radio_->dev_init(_cfg, ch, rxgain, txgain);
    }
  }
  MLPD_TRACE("ClientRadioSet: Init complete\n");
  assert(thread_count->load() != 0);
  thread_count->store(thread_count->load() - 1);
  std::cout << "Client Init success" << std::endl;
}

ClientRadioSetUHD::~ClientRadioSetUHD(void) {
  if (radio_ != nullptr) {
    delete radio_;
    radio_ = nullptr;
  }
}

void ClientRadioSetUHD::radioStop(void) {
  radio_->deactivateRecv();
  MLPD_TRACE("Stopping radio...\n");
}

int ClientRadioSetUHD::triggers(int i) {
  if (i > (int)radio_->RawDev()->get_num_mboards()) {
    std::cout << "invalid radio id " << i << std::endl;
    return 0;
  }
  return (radio_->getTriggers());
}

int ClientRadioSetUHD::radioRx(size_t radio_id, void* const* buffs,
                               int numSamps, long long& frameTime) {
  if (radio_id < radio_->RawDev()->get_num_mboards()) {
    int ret(0);
    // update for UHD multi USRP
    long long frameTimeNs(0);
    ret = radio_->recv(buffs, numSamps, frameTimeNs);

    frameTime = SoapySDR::timeNsToTicks(frameTimeNs, _cfg->rate());

    if (kDebugRadio) {
      if (frameTimeNs < 2e9) {
        std::cout << "client " << radio_id << " received " << ret << " at "
                  << frameTimeNs << std::endl;
      }
    }
    return ret;
  }
  std::cout << "invalid radio id " << radio_id << std::endl;
  return 0;
}

int ClientRadioSetUHD::radioTx(size_t radio_id, const void* const* buffs,
                               int numSamps, int flags, long long& frameTime) {
  if (radio_id > radio_->RawDev()->get_num_mboards()) {
    std::cout << "invalid radio id " << radio_id << std::endl;
    return 0;
  }
  long long frameTimeNs = SoapySDR::ticksToTimeNs(frameTime, _cfg->rate());
  return radio_->xmit(buffs, numSamps, flags, frameTimeNs);
}
