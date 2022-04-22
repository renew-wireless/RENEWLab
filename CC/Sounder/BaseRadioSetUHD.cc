/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
            Doug Moore: dougm@rice.edu
	    Oscar Bejarano: ob4@rice.edu
 
---------------------------------------------------------------------
 Initializes and Configures Radios in the massive-MIMO base station 
---------------------------------------------------------------------
*/
#include "include/BaseRadioSetUHD.h"

#include "include/RadioUHD.h"
#include "include/comms-lib.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"

using json = nlohmann::json;
static constexpr int kMaxOffsetDiff = 6;

BaseRadioSetUHD::BaseRadioSetUHD(Config* cfg) : _cfg(cfg) {
  std::vector<size_t> num_bs_antenntas(_cfg->num_cells());
  radioNotFound = false;
  std::vector<std::string> radio_serial_not_found;

  // need to be further modified
  for (size_t c = 0; c < _cfg->num_cells(); c++) {
    size_t num_radios = _cfg->n_bs_sdrs()[c];
    num_bs_antenntas[c] = num_radios * _cfg->bs_channel().length();
    MLPD_TRACE("Setting up radio: %zu, cells: %zu\n", num_radios,
               _cfg->num_cells());

    std::atomic_ulong thread_count = ATOMIC_VAR_INIT(num_radios);

    MLPD_TRACE("Init base radios: %zu\n", num_radios);
    for (size_t i = 0; i < num_radios; i++) {
      BaseRadioContext* context = new BaseRadioContext;
      context->brs = this;
      context->thread_count = &thread_count;
      context->tid = i;
      context->cell = c;
#ifdef THREADED_INIT
      pthread_t init_thread_;

      if (pthread_create(&init_thread_, NULL, BaseRadioSetUHD::init_launch,
                         context) != 0) {
        delete context;
        throw std::runtime_error("BaseRadioSet - init thread create failed");
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
    // elimated this part
    //        for (size_t i = 0; i < num_radios; i++) {
    //            if (bsRadios == NULL) {
    //                radioNotFound = true;
    //                radio_serial_not_found.push_back(
    //                    _cfg->bs_sdr_ids().at(c).at(i));
    //                while (num_radios != 0
    //                    && bsRadios.at(c).at(num_radios - 1) == NULL) {
    //                    --num_radios;
    //                    bsRadios.at(c).pop_back();
    //                }
    //                if (i < num_radios) {
    //                    bsRadios.at(c).at(i) = bsRadios.at(c).at(--num_radios);
    //                    bsRadios.at(c).pop_back();
    //                }
    //            }
    //        }
    //        bsRadios.at(c).shrink_to_fit();
    _cfg->n_bs_sdrs().at(c) = num_radios;
    if (radioNotFound == true) {
      break;
    }

    // Perform DC Offset & IQ Imbalance Calibration
    if (_cfg->imbalance_cal_en() == true) {
      if (_cfg->bs_channel().find('A') != std::string::npos)
        dciqCalibrationProcUHD(0);
      if (_cfg->bs_channel().find('B') != std::string::npos)
        dciqCalibrationProcUHD(1);
    }

    thread_count.store(num_radios);
    for (size_t i = 0; i < num_radios; i++) {
      BaseRadioContext* context = new BaseRadioContext;
      context->brs = this;
      context->thread_count = &thread_count;
      context->tid = i;
      context->cell = c;
#ifdef THREADED_INIT
      pthread_t configure_thread_;
      if (pthread_create(&configure_thread_, NULL,
                         BaseRadioSetUHD::configure_launch, context) != 0) {
        delete context;
        throw std::runtime_error(
            "BaseRadioSet - configure thread create failed");
      }
#else
      configure(context);
#endif
    }

    while (thread_count.load() > 0) {
    }

    auto channels = Utils::strToChannels(_cfg->bs_channel());

    for (size_t i = 0; i < bsRadios->RawDev()->get_num_mboards(); i++) {
      auto dev = bsRadios->RawDev();
      std::cout << _cfg->bs_sdr_ids().at(c).at(i) << ": Front end "
                << dev->get_usrp_rx_info()["frontend"] << std::endl;
      for (auto ch : channels) {
        if (ch < dev->get_rx_num_channels()) {
          printf("RX Channel %zu\n", ch);
          printf("Actual RX sample rate: %fMSps...\n",
                 (dev->get_rx_rate(ch) / 1e6));
          printf("Actual RX frequency: %fGHz...\n",
                 (dev->get_rx_freq(ch) / 1e9));
          printf("Actual RX gain: %f...\n", (dev->get_rx_gain(ch)));
          printf("Actual RX bandwidth: %fM...\n",
                 (dev->get_rx_bandwidth(ch) / 1e6));
          printf("Actual RX antenna: %s...\n",
                 (dev->get_rx_antenna(ch).c_str()));
        }
      }

      for (auto ch : channels) {
        if (ch < dev->get_tx_num_channels()) {
          printf("TX Channel %zu\n", ch);
          printf("Actual TX sample rate: %fMSps...\n",
                 (dev->get_tx_rate(ch) / 1e6));
          printf("Actual TX frequency: %fGHz...\n",
                 (dev->get_tx_freq(ch) / 1e9));
          printf("Actual TX gain: %f...\n", (dev->get_tx_gain(ch)));
          printf("Actual TX bandwidth: %fM...\n",
                 (dev->get_tx_bandwidth(ch) / 1e6));
          printf("Actual TX antenna: %s...\n",
                 (dev->get_tx_antenna(ch).c_str()));
        }
      }
      std::cout << std::endl;
    }
  }

  if (radioNotFound == true) {
    for (auto st = radio_serial_not_found.begin();
         st != radio_serial_not_found.end(); st++)
      std::cout << "\033[1;31m" << *st << "\033[0m" << std::endl;
    std::cout << "\033[1;31mERROR: the above base station serials were not "
                 "discovered in the network!\033[0m"
              << std::endl;
  } else {
    if (_cfg->sample_cal_en() == true) {
      //            bool adjust = false;
      int cal_cnt = 0;
      int offset_diff = 1;

      while (offset_diff > 0) {
        if (++cal_cnt > 10) {
          std::cout << "10 attemps of sample offset calibration, "
                       "stopping..."
                    << std::endl;
          break;
        }
        offset_diff =
            syncTimeOffsetUHD(false, true);  // run 1: find offsets and adjust
      }
      usleep(100000);
      offset_diff = syncTimeOffsetUHD(false, false);  // run 2: verify
      if (offset_diff > 1)
        std::cout << "Failed ";
      else
        std::cout << "Successful ";
      std::cout << "sample offset calibration!" << std::endl;
    }

    nlohmann::json tddConf;
    tddConf["tdd_enabled"] = true;
    tddConf["frame_mode"] = "free_running";
    tddConf["max_frame"] = _cfg->max_frame();
    tddConf["symbol_size"] = _cfg->samps_per_slot();

    // write TDD schedule and beacons to FPFA buffers only for Iris
    for (size_t c = 0; c < _cfg->num_cells(); c++) {
      bsRadios->RawDev()->set_time_source("external", 0);
      bsRadios->RawDev()->set_clock_source("external", 0);
      uhd::time_spec_t time = uhd::time_spec_t::from_ticks(0, 1e9);
      bsRadios->RawDev()->set_time_next_pps(time);

      // Wait for pps sync pulse
      std::this_thread::sleep_for(std::chrono::seconds(2));
      // Activate Rx and Tx streamers
      bsRadios->activateRecv();
      bsRadios->activateXmit();
    }
    MLPD_INFO("%s done!\n", __func__);
  }
}

BaseRadioSetUHD::~BaseRadioSetUHD(void) { delete bsRadios; }

void* BaseRadioSetUHD::init_launch(void* in_context) {
  BaseRadioContext* context = reinterpret_cast<BaseRadioContext*>(in_context);
  context->brs->init(context);
  return 0;
}

void BaseRadioSetUHD::init(BaseRadioContext* context) {
  int i = context->tid;
  int c = context->cell;
  std::atomic_ulong* thread_count = context->thread_count;
  delete context;

  MLPD_TRACE("Deleting context for tid: %d\n", i);

  auto channels = Utils::strToChannels(_cfg->bs_channel());
  std::map<std::string, std::string> args;

  args["driver"] = "uhd";
  args["addr"] = _cfg->bs_sdr_ids().at(c).at(i);
  std::cout << "Init bsRadios: " << args["addr"] << std::endl;
  //    }
  args["timeout"] = "1000000";
  try {
    bsRadios = nullptr;
    bsRadios = new RadioUHD(args, SOAPY_SDR_CS16, channels);
  } catch (std::runtime_error& err) {
    std::cerr << "Ignoring uhd device " << _cfg->bs_sdr_ids().at(c).at(i)
              << std::endl;
    //        }
    if (bsRadios != nullptr) {
      MLPD_TRACE("Deleting radio ptr due to exception\n");
      delete bsRadios;
      bsRadios = nullptr;
    }
  }
  MLPD_TRACE("BaseRadioSet: Init complete\n");
  assert(thread_count->load() != 0);
  thread_count->store(thread_count->load() - 1);
}

void* BaseRadioSetUHD::configure_launch(void* in_context) {
  BaseRadioContext* context = reinterpret_cast<BaseRadioContext*>(in_context);
  context->brs->configure(context);
  return 0;
}

void BaseRadioSetUHD::configure(BaseRadioContext* context) {
  //    int i = context->tid;
  //    int c = context->cell;
  std::atomic_ulong* thread_count = context->thread_count;
  delete context;

  //load channels
  auto channels = Utils::strToChannels(_cfg->bs_channel());
  RadioUHD* bsRadio = bsRadios;
  uhd::usrp::multi_usrp::sptr dev = bsRadio->RawDev();
  for (auto ch : channels) {
    double rxgain = _cfg->rx_gain().at(ch);
    double txgain = _cfg->tx_gain().at(ch);
    bsRadios->dev_init(_cfg, ch, rxgain, txgain);
  }
  assert(thread_count->load() != 0);
  thread_count->store(thread_count->load() - 1);
}

uhd::usrp::multi_usrp::sptr BaseRadioSetUHD::baseRadio(size_t cellId) {
  // future edit when doing multi USRP
  std::cout << "cell ID is " << cellId << std::endl;
  return NULL;
}

void BaseRadioSetUHD::sync_delays(size_t cellIdx) {
  /*
     * Compute Sync Delays
     */
  uhd::usrp::multi_usrp::sptr base = baseRadio(cellIdx);
}

void BaseRadioSetUHD::radioTriggerUHD(void) {
  //    for (size_t c = 0; c < _cfg->num_cells(); c++) {
  //        SoapySDR::Device* base = baseRadio(c);
  //        if (base != NULL)
  //            base->writeSetting("TRIGGER_GEN", "");
  //    }
}

void BaseRadioSetUHD::radioStart() {
  //    if (!kUseUHD)
  //        radioTrigger();
}

void BaseRadioSetUHD::readSensors() {
  uhd::usrp::multi_usrp::sptr dev = bsRadios->RawDev();
  for (size_t i = 0; i < dev->get_num_mboards(); i++) {
    std::cout << dev->get_mboard_sensor_names(i).at(0) << std::endl;
  }
}

void BaseRadioSetUHD::radioStop(void) {
  std::string tddConfStr = "{\"tdd_enabled\":false}";
  //    for (size_t c = 0; c < _cfg->num_cells(); c++) {
  //        for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
  //            if (!kUseUHD) {
  //                SoapySDR::Device* dev = bsRadios.at(c).at(i)->dev;
  //                dev->writeSetting("TDD_CONFIG", tddConfStr);
  //                dev->writeSetting("TDD_MODE", "false");
  //            }
  bsRadios->reset_DATA_clk_domain();
  //        }
  //    }
}

void BaseRadioSetUHD::radioTx(const void* const* buffs) {
  long long frameTime(0);
  //    for (size_t c = 0; c < _cfg->num_cells(); c++) {
  //        for (size_t i = 0; i < bsRadios.at(c).size(); i++) {
  std::cout << "running tx 1" << std::endl;
  bsRadios->xmit(buffs, _cfg->samps_per_slot(), 0, frameTime);
  //        }
  //    }
}

int BaseRadioSetUHD::radioTx(size_t radio_id, size_t cell_id,
                             const void* const* buffs, int flags,
                             long long& frameTime) {
  (void)radio_id;
  (void)cell_id;
  int w;
  // for UHD device xmit from host using frameTimeNs
  //    if (!kUseUHD) {
  //        w = bsRadios.at(cell_id).at(radio_id)->xmit(
  //            buffs, _cfg->samps_per_symbol(), flags, frameTime);
  //    } else {
  // future edit on multi usrp:
  //    std::cout << "radio id is " << radio_id << std::endl;
  //    std::cout << "cell id is " << cell_id << std::endl;
  long long frameTimeNs = SoapySDR::ticksToTimeNs(frameTime, _cfg->rate());
  w = bsRadios->xmit(buffs, _cfg->samps_per_slot(), flags, frameTimeNs);
//    }
#if DEBUG_RADIO
  size_t chanMask;
  long timeoutUs(0);
  auto* dev = bsRadios->dev;
  auto* txs = bsRadios->txs;
//    int s = dev->readStreamStatus(txs, chanMask, flags, frameTime, timeoutUs);
//    std::cout << "cell " << cell_id << " radio " << radio_id << " tx returned "
//              << w << " and status " << s << std::endl;
#endif
  //    std::cout<<"running tx 2" << std::endl;
  return w;
}

void BaseRadioSetUHD::radioRx(void* const* buffs) {
  long long frameTime(0);
  void* const* buff;

  //    for (size_t c = 0; c < _cfg->num_cells(); c++) {
  for (size_t i = 0; i < bsRadios->RawDev()->get_num_mboards(); i++) {
    buff = buffs + (i * 2);
    bsRadios->recv(buff, _cfg->samps_per_slot(), frameTime);
  }
  //        }
  //    }
}

int BaseRadioSetUHD::radioRx(size_t radio_id, size_t cell_id,
                             void* const* buffs, long long& frameTime) {
  return this->radioRx(radio_id, cell_id, buffs, _cfg->samps_per_slot(),
                       frameTime);
}

int BaseRadioSetUHD::radioRx(size_t radio_id, size_t cell_id,
                             void* const* buffs, int numSamps,
                             long long& frameTime) {
  int ret = 0;

  //    std::cout<< "radio id is "<<radio_id<<std::endl;
  if (radio_id < bsRadios->RawDev()->get_num_mboards()) {
    long long frameTimeNs = 0;
    ret = bsRadios->recv(buffs, numSamps, frameTimeNs);
    //        std::cout<<"bsRadios->dev->get_num_mboards() " << bsRadios->dev->get_num_mboards() << std::endl;

    // for UHD device recv using ticks
    frameTime = SoapySDR::timeNsToTicks(frameTimeNs, _cfg->rate());
#if DEBUG_RADIO
    if (ret != numSamps)
      std::cout << "recv returned " << ret << " from radio " << radio_id
                << ", in cell " << cell_id << ". Expected: " << numSamps
                << std::endl;
    else
      std::cout << "radio " << radio_id << " in cell " << cell_id
                << ". Received " << ret << " at " << frameTime << std::endl;
#endif
  } else {
    MLPD_WARN("Invalid radio id: %zu in cell %zu\n", radio_id, cell_id);
    ret = 0;
  }
  return ret;
}

int BaseRadioSetUHD::syncTimeOffsetUHD(bool measure_ref_radio,
                                       bool adjust_trigger) {
  std::cout << "sync time being called" << std::endl;
  int num_radios = _cfg->n_bs_sdrs().at(0) - 1;
  if (num_radios < 1) {
    std::cout << "No need to sample calibrate with one Iris! skipping ..."
              << std::endl;
    return -1;
  }
  int seqLen = 160;  // Sequence length
  std::vector<std::vector<float>> pilot =
      CommsLib::getSequence(CommsLib::LTS_SEQ, seqLen);
  auto pilot_cint16 = Utils::float_to_cint16(pilot);

  // Prepend/Append vectors with prefix/postfix number of null samples
  std::vector<std::complex<int16_t>> prefix_vec(_cfg->prefix(), 0);
  std::vector<std::complex<int16_t>> postfix_vec(
      _cfg->samps_per_slot() - _cfg->prefix() - seqLen, 0);
  pilot_cint16.insert(pilot_cint16.begin(), prefix_vec.begin(),
                      prefix_vec.end());
  pilot_cint16.insert(pilot_cint16.end(), postfix_vec.begin(),
                      postfix_vec.end());
  // Transmitting from only one chain, create a null vector for chainB
  std::vector<std::complex<int16_t>> dummy_cint16(pilot_cint16.size(), 0);
  size_t num_samps = _cfg->samps_per_slot();

  std::vector<void*> txbuff(2);
  txbuff[0] = pilot_cint16.data();
  if (_cfg->bs_sdr_ch() == 2) txbuff[1] = dummy_cint16.data();

  std::vector<std::complex<int16_t>> dummyBuff0(num_samps);
  std::vector<std::complex<int16_t>> dummyBuff1(num_samps);
  std::vector<void*> dummybuffs(2);
  dummybuffs[0] = dummyBuff0.data();
  dummybuffs[1] = dummyBuff1.data();

  long long txTime(0);
  long long rxTime(0);

  RadioUHD* ref_radio = bsRadios;
  uhd::usrp::multi_usrp::sptr ref_dev = ref_radio->RawDev();

  int offset_diff = num_samps;
  if (measure_ref_radio == true) {
    std::vector<std::complex<int16_t>> rx_buff_ref;
    rx_buff_ref.resize(num_samps);

    ref_radio->drain_buffers(dummybuffs, num_samps);
    RadioUHD* front_radio = bsRadios;
    uhd::usrp::multi_usrp::sptr front_dev = front_radio->RawDev();
    //        front_dev->setGain(SOAPY_SDR_TX, 0, "PAD", _cfg->cal_tx_gain().at(0));
    front_dev->set_tx_gain(0, "PAD", _cfg->cal_tx_gain().at(0));
    front_radio->activateXmit();
    int ret = front_radio->xmit(txbuff.data(), num_samps, 3, txTime);
    if (ret < 0) std::cout << "bad write" << std::endl;
    ret = ref_radio->activateRecv(rxTime, num_samps, 3);
    if (ret < 0) std::cout << "bad activate at front node" << std::endl;
    radioTriggerUHD();
    std::vector<void*> rxbuffref(2);
    rxbuffref[0] = rx_buff_ref.data();
    if (_cfg->bs_sdr_ch() == 2) rxbuffref[1] = dummyBuff0.data();
    ret = ref_radio->recv(rxbuffref.data(), num_samps, rxTime);
    auto rx = Utils::cint16_to_cfloat(rx_buff_ref);
    //std::cout << "ref=[";
    //for (size_t s = 0; s < num_samps; s++)
    //  std::cout << rx_buff_ref.at(s).real() << "+1j*"
    //            << rx_buff_ref.at(s).imag() << " ";
    //std::cout << "];" << std::endl;
    int peak = CommsLib::findLTS(rx, seqLen);
    int offset = peak < seqLen ? 0 : peak - seqLen;
    if (offset > 0) {
      offset_diff = _cfg->prefix() - offset;
      std::cout << "Ref offset from prefix is " << offset_diff << "."
                << std::endl;
      /*std::cout << "adjusting delay of ref node by " << offset_diff
                      << std::endl;*/
      if (adjust_trigger) {
        int abs_delta = std::abs(offset_diff);
        if (abs_delta > _cfg->prefix())
          std::cout << "Likely bad receive!" << std::endl;
        else {
          while (abs_delta > 0) {
            //                        ref_dev->writeSetting("ADJUST_DELAYS",
            //                                              offset_diff > 0 ? "1" : "-1");
            --abs_delta;
          }
        }
      }
    }
    front_radio->deactivateXmit();
    //        front_dev->setGain(SOAPY_SDR_TX, 0, "PAD", _cfg->tx_gain().at(0));
    front_dev->set_tx_gain(0, "PAD", _cfg->cal_tx_gain().at(0));
    ref_radio->deactivateRecv();
    ref_radio->drain_buffers(dummybuffs, num_samps);
  } else {
    std::vector<std::vector<std::complex<int16_t>>> buff;
    buff.resize(num_radios);
    for (int i = 0; i < num_radios; i++) {
      buff[i].resize(num_samps);
    }

    //        ref_dev->setGain(SOAPY_SDR_TX, 0, "PAD", _cfg->cal_tx_gain().at(0));
    ref_dev->set_tx_gain(0, "PAD", _cfg->cal_tx_gain().at(0));
    for (int i = 0; i < num_radios; i++)
      bsRadios->drain_buffers(dummybuffs, num_samps);

    ref_radio->activateXmit();
    int ret = ref_radio->xmit(txbuff.data(), num_samps, 3, txTime);
    if (ret < 0) std::cout << "bad write" << std::endl;

    // All write, or prepare to receive.
    for (int j = 0; j < num_radios; j++) {
      ret = bsRadios->activateRecv(rxTime, num_samps, 3);
      if (ret < 0) std::cout << "bad activate at node " << j << std::endl;
    }

    radioTriggerUHD();

    for (int i = 0; i < num_radios; i++) {
      std::vector<void*> rxbuff(2);
      rxbuff[0] = buff[i].data();
      //rxbuff[1] = ant == 2 ? buff[(i*M+j)*ant+1].data() : dummyBuff.data();
      if (_cfg->bs_sdr_ch() == 2) rxbuff[1] = dummyBuff0.data();
      int ret = bsRadios->recv(rxbuff.data(), num_samps, rxTime);
      if (ret < 0) std::cout << "bad read at node " << i << std::endl;
    }

    int min_offset = num_samps;
    int max_offset = 0;
    std::vector<int> offset(num_radios, 0);

    for (int i = 0; i < num_radios; i++) {
      // std::cout << "s" << i << "=[";
      // for (size_t s = 0; s < num_samps; s++)
      //     std::cout << buff[i].at(s).real() << "+1j*" << buff[i].at(s).imag()
      //               << " ";
      // std::cout << "];" << std::endl;
      auto rx = Utils::cint16_to_cfloat(buff[i]);
      int peak = CommsLib::findLTS(rx, seqLen);
      offset[i] = peak < seqLen ? 0 : peak - seqLen;
      //std::cout << i << " " << offset[i] << std::endl;
      if (offset[i] != 0) {
        min_offset = std::min(offset[i], min_offset);
        max_offset = std::max(offset[i], max_offset);
      }
    }
    std::cout << "Min/Max offset is " << min_offset << "/" << max_offset << "."
              << std::endl;
    offset_diff = max_offset - min_offset;
    if (std::abs(offset_diff) > kMaxOffsetDiff) {
      std::cout << "Offset Diff is " << max_offset - min_offset
                << ". Likely bad receive!" << std::endl;
    } else if (adjust_trigger && offset_diff > 1) {
      // adjusting trigger delays based on lts peak index
      std::vector<int> offset_sorted(offset.begin(), offset.end());
      std::sort(offset_sorted.begin(), offset_sorted.end());
      int median_offset = offset_sorted.at(num_radios / 2);
      for (int i = 0; i < num_radios; i++) {
        RadioUHD* bsRadio = bsRadios;
        uhd::usrp::multi_usrp::sptr dev = bsRadio->RawDev();
        //int delta = median_offset - offset[i];
        int delta = median_offset - offset[i];
        /*std::cout << "adjusting delay of node " << i << " by " << delta
                          << std::endl;*/
        int abs_delta = std::abs(delta);
        while (abs_delta > 0) {
          //                    dev->writeSetting("ADJUST_DELAYS", delta > 0 ? "1" : "-1");
          --abs_delta;
        }
      }
    }

    ref_radio->deactivateXmit();
    //        ref_dev->setGain(SOAPY_SDR_TX, 0, "PAD", _cfg->tx_gain().at(0));
    ref_dev->set_tx_gain(0, "PAD", _cfg->cal_tx_gain().at(0));
    for (int i = 0; i < num_radios; i++) {
      RadioUHD* bsRadio = bsRadios;
      bsRadio->deactivateRecv();
      bsRadio->drain_buffers(dummybuffs, num_samps);
    }
  }
  return offset_diff;
}
