/*
 Copyright (c) 2018-2019, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------
 Initializes and Configures Client Radios
----------------------------------------------------------
*/
#include "include/ClientRadioSetUHD.h"

#include "include/RadioUHD.h"
#include "include/comms-lib.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"

static void freeRadios(RadioUHD*& radios) { delete radios; }

ClientRadioSetUHD::ClientRadioSetUHD(Config* cfg) : _cfg(cfg) {
  size_t num_radios = _cfg->num_cl_sdrs();
  //load channels
  std::cout << "channel is: " << _cfg->cl_channel() << std::endl;
  auto channels = Utils::strToChannels(_cfg->cl_channel());
  // Update for UHD multi USRP
  radioNotFound = false;
  std::atomic_ulong thread_count = ATOMIC_VAR_INIT(num_radios);
  for (size_t i = 0; i < num_radios; i++) {
    ClientRadioContext* context = new ClientRadioContext;
    context->crs = this;
    context->thread_count = &thread_count;
    context->tid = i;
#ifdef THREADED_INIT
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
  // commented out for now, will add this when future doing Multi USRP
  // Strip out broken radios.
  // Update for UHD multi USRP
  //    for (size_t i = 0; i < num_radios; i++) {
  //        if (radios.at(i) == nullptr) {
  //            radioNotFound = true;
  //            radioSerialNotFound.push_back(_cfg->cl_sdr_ids().at(i));
  //            while (num_radios != 0 && radios.at(num_radios - 1) == NULL) {
  //                --num_radios;
  //                radios.pop_back();
  //            }
  //            if (i < num_radios) {
  //                radios.at(i) = radios.at(--num_radios);
  //                radios.pop_back();
  //            }
  //        }
  //    }
  //    radios.shrink_to_fit();

  if (num_radios != radios->dev->get_num_mboards()) {
    radioNotFound = true;
  }

  // update for UHD multi USRP
  for (size_t i = 0; i < radios->dev->get_num_mboards(); i++) {
    auto dev = radios->dev;
    std::cout << _cfg->cl_sdr_ids().at(i) << ": Front end "
              << dev->get_usrp_rx_info()["frontend"] << std::endl;
  }

  for (auto ch : channels) {
    if (ch < radios->dev->get_rx_num_channels()) {
      printf("RX Channel %zu\n", ch);
      printf("Actual RX sample rate: %fMSps...\n",
             (radios->dev->get_rx_rate(ch) / 1e6));
      printf("Actual RX frequency: %fGHz...\n",
             (radios->dev->get_rx_freq(ch) / 1e9));
      printf("Actual RX gain: %f...\n", (radios->dev->get_rx_gain(ch)));

      printf("Actual RX bandwidth: %fM...\n",
             (radios->dev->get_rx_bandwidth(ch) / 1e6));
      printf("Actual RX antenna: %s...\n",
             (radios->dev->get_rx_antenna(ch).c_str()));
    }
  }

  for (auto ch : channels) {
    if (ch < radios->dev->get_tx_num_channels()) {
      printf("TX Channel %zu\n", ch);
      printf("Actual TX sample rate: %fMSps...\n",
             (radios->dev->get_tx_rate(ch) / 1e6));
      printf("Actual TX frequency: %fGHz...\n",
             (radios->dev->get_tx_freq(ch) / 1e9));
      printf("Actual TX gain: %f...\n", (radios->dev->get_tx_gain(ch)));
      printf("Actual TX bandwidth: %fM...\n",
             (radios->dev->get_tx_bandwidth(ch) / 1e6));
      printf("Actual TX antenna: %s...\n",
             (radios->dev->get_tx_antenna(ch).c_str()));
    }
  }
  std::cout << "ClientRadioSetUHD Init Check" << std::endl;
  std::cout << std::endl;

  //    }

  // Update for UHD multi USRP
  if (radioNotFound == true) {
    std::cout << "some client serials were not "
                 "discovered in the network!"
              << std::endl;
  } else {
    //beaconSize + 82 (BS FE delay) + 68 (path delay) + 17 (correlator delay) + 82 (Client FE Delay)
    std::cout << "checking tx advance " << _cfg->tx_advance(0) << std::endl;
    radios->dev->set_time_source("internal");
    radios->dev->set_clock_source("internal");
    radios->dev->set_time_unknown_pps(uhd::time_spec_t(0.0));
    radios->activateRecv();
    radios->activateXmit();

    std::cout << "sync check" << std::endl;
    std::cout << std::endl;
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
  //    SoapySDR::Kwargs args;
  args["timeout"] = "1000000";
  args["driver"] = "uhd";
  args["addr"] = _cfg->cl_sdr_ids().at(i);
  std::cout << "address i is " << i << std::endl;

  try {
    radios = nullptr;
    radios = new RadioUHD(args, SOAPY_SDR_CS16, channels);
  } catch (std::runtime_error& err) {
    has_runtime_error = true;

    if (radios != nullptr) {
      MLPD_TRACE("Radio not used due to exception\n");
      //            delete radios.at(i);
      radios = nullptr;
    }
  } catch (...) {
    MLPD_WARN("Unknown exception\n");
    if (radios != nullptr) {
      delete radios;
      radios = nullptr;
    }
    throw;
  }
  if (has_runtime_error == false) {
    //        auto dev = radios.at(i)->dev;
    //        SoapySDR::Kwargs info = dev->getHardwareInfo();
    for (auto ch : channels) {
      std::cout << "check ch: " << ch << std::endl;
      auto new_ch = _cfg->cl_channel();
      double rxgain = _cfg->cl_rxgain_vec().at(ch).at(
          i);  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
      double txgain = _cfg->cl_txgain_vec().at(ch).at(
          i);  // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
      radios->dev_init(_cfg, ch, rxgain, txgain);
    }

    // Init AGC only for Iris device
    //        if (kUseUHD == false) {
    //            initAGC(dev, _cfg);
    //        }
  }
  MLPD_TRACE("ClientRadioSet: Init complete\n");
  assert(thread_count->load() != 0);
  thread_count->store(thread_count->load() - 1);
  std::cout << "Client Init success" << std::endl;
}

ClientRadioSetUHD::~ClientRadioSetUHD(void) { freeRadios(radios); }

void ClientRadioSetUHD::radioStop(void) {
  radios->deactivateRecv();
  MLPD_TRACE("Stopping radio...\n");
}

int ClientRadioSetUHD::triggers(int i) {
  if (i > (int)radios->dev->get_num_mboards()) {
    std::cout << "invalid radio id " << i << std::endl;
    return 0;
  }
  return (radios->getTriggers());
}

int ClientRadioSetUHD::radioRx(size_t radio_id, void* const* buffs,
                               int numSamps, long long& frameTime) {
  if (radio_id < radios->dev->get_num_mboards()) {
    int ret(0);
    // update for UHD multi USRP
    long long frameTimeNs(0);
    ret = radios->recv(buffs, numSamps, frameTimeNs);

    frameTime = SoapySDR::timeNsToTicks(frameTimeNs, _cfg->rate());

#if DEBUG_RADIO
    if (frameTimeNs < 2e9)
      std::cout << "client " << radio_id << " received " << ret << " at "
                << frameTimeNs << std::endl;
#endif
    //        double a = radios->dev->get_time_now().get_real_secs();
    //        std::cout << "time: " << a <<std::endl;
    //        std::cout << "full seconds: " << radios->dev->get_time_now().get_full_secs() << std::endl;
    //        std::cout << "frac seconds: " << radios->dev->get_time_now().get_frac_secs() << std::endl;

    //        long long b = check_dev->getHardwareTime();
    //        double utime = b / 1e9;
    //        std::cout << "sopay way time in ns: " << b << std::endl;
    //        std::cout << "soapy way time: " << utime <<std::endl;
    //        }
    return ret;
  }
  std::cout << "invalid radio id " << radio_id << std::endl;
  return 0;
}

int ClientRadioSetUHD::radioTx(size_t radio_id, const void* const* buffs,
                               int numSamps, int flags, long long& frameTime) {
  if (radio_id > radios->dev->get_num_mboards()) {
    std::cout << "invalid radio id " << radio_id << std::endl;
    return 0;
  }
  long long frameTimeNs = SoapySDR::ticksToTimeNs(frameTime, _cfg->rate());
  return radios->xmit(buffs, numSamps, flags, frameTimeNs);
}
