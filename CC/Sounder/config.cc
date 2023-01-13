/*
 Copyright (c) 2018-2022, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
---------------------------------------------------------------------
 Reads configuration parameters from file 
---------------------------------------------------------------------
*/

#include "include/config.h"

#include "include/comms-lib.h"
#include "include/constants.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"
#include "nlohmann/json.hpp"
using json = nlohmann::json;

static size_t kFpgaTxRamSize = 4096;
static size_t kMaxSupportedFFTSize = 2048;
static size_t kMinSupportedFFTSize = 64;
static size_t kMaxSupportedCPSize = 128;

Config::Config(const std::string& jsonfile, const std::string& directory,
               const bool bs_only, const bool client_only)
    : directory_(directory) {
  std::string conf_str;
  Utils::loadTDDConfig(jsonfile, conf_str);
  // Enable comments in json file
  const auto tddConf = json::parse(conf_str, nullptr, true, true);
  std::stringstream ss;
  ss << "  Config: " << tddConf << "\n" << std::endl;
  MLPD_INFO("\nInput config:\n\n%s", ss.str().c_str());
  ss.str(std::string());
  ss.clear();

  if (bs_only && client_only == true) {
    MLPD_ERROR("Client-Only and BS-Only can't both be enabled!\n");
    exit(1);
  }

  internal_measurement_ = tddConf.value("internal_measurement", false);
  /* Used for internal measurements. If internal_measurement is enabled,
       the default is to use a reference node for reciprocity calibration.
       Users have the option of "disabling" the reference node to get a full
       matrix (send pilot from all base station boards and receive on all
       base station boards). The guard interval multiplier extends the number
       of G's in a schedule (between pilots).
     */

  ref_node_enable_ = tddConf.value("reference_node_enable", true);
  guard_mult_ = tddConf.value("meas_guard_interval_mult", 1);

  sample_cal_en_ = tddConf.value("calibrate_digital", false);
  imbalance_cal_en_ = tddConf.value("calibrate_analog", false);

  num_bs_sdrs_all_ = 0;
  num_bs_antennas_all_ = 0;
  num_cl_sdrs_ = 0;

  bs_channel_ = tddConf.value("channel", "A");
  if ((bs_channel_ != "A") && (bs_channel_ != "B") && (bs_channel_ != "AB")) {
    throw std::invalid_argument("error channel config: not any of A/B/AB!\n");
  }
  bs_sdr_ch_ = (bs_channel_ == "AB") ? 2 : 1;

  cl_channel_ = tddConf.value("ue_channel", "A");
  if (cl_channel_ != "A" && cl_channel_ != "B" && cl_channel_ != "AB")
    throw std::invalid_argument("error channel config: not any of A/B/AB!\n");
  cl_sdr_ch_ = (cl_channel_ == "AB") ? 2 : 1;

  auto serials_file = tddConf.value("serial_file", "./files/topology.json");
  loadTopology(serials_file, bs_only, client_only);
  std::cout << "Topology: "
            << "\n"
            << " Number of cells: " << num_cells_ << "\n"
            << " Number of BS sdrs in cell 0: " << n_bs_sdrs_.at(0) << "\n"
            << " Client SDRs: " << num_cl_sdrs_ << std::endl;

  static const int kMaxTxGainBS = 81;
  // common (BaseStation config overrides these)
  freq_ = tddConf.value("frequency", 2.5e9);
  rate_ = tddConf.value("sample_rate", 5e6);
  nco_ = tddConf.value("nco_frequency", 0.75 * rate_);
  symbol_per_slot_ = tddConf.value("ofdm_symbol_per_slot", 1);
  fft_size_ = tddConf.value("fft_size", 0);
  cp_size_ = tddConf.value("cp_size", 0);
  dl_pilots_en_ = tddConf.value("enable_dl_pilots", false);
  prefix_ = tddConf.value("ofdm_tx_zero_prefix", 0);
  postfix_ = tddConf.value("ofdm_tx_zero_postfix", 0);
  symbol_data_subcarrier_num_ = tddConf.value("ofdm_data_num", fft_size_);
  pilot_seq_ = tddConf.value("pilot_seq", "lts");
  data_mod_ = tddConf.value("modulation", "QPSK");
  single_gain_ = tddConf.value("single_gain", true);

  if (tddConf.value("tx_gain_a", 20) > kMaxTxGainBS) {
    std::string msg = "ERROR: BaseStation ChanA - Maximum TX gain value is ";
    msg += std::to_string(kMaxTxGainBS);
    throw std::invalid_argument(msg);
  } else {
    tx_gain_.push_back(tddConf.value("tx_gain_a", 20));
  }

  if (tddConf.value("tx_gain_b", 20) > kMaxTxGainBS) {
    std::string msg = "ERROR: BaseStation ChanB - Maximum TX gain value is ";
    msg += std::to_string(kMaxTxGainBS);
    throw std::invalid_argument(msg);
  } else {
    tx_gain_.push_back(tddConf.value("tx_gain_b", 20));
  }

  rx_gain_.push_back(tddConf.value("rx_gain_a", 20));
  rx_gain_.push_back(tddConf.value("rx_gain_b", 20));
  cal_tx_gain_.push_back(tddConf.value("cal_tx_gain_a", tx_gain_.at(0)));
  cal_tx_gain_.push_back(tddConf.value("cal_tx_gain_b", tx_gain_.at(1)));
  tx_gain_.shrink_to_fit();
  rx_gain_.shrink_to_fit();
  cal_tx_gain_.shrink_to_fit();

  beam_sweep_ = tddConf.value("beamsweep", false);
  beacon_ant_ = tddConf.value("beacon_antenna", 0);
  beacon_radio_ = beacon_ant_ / bs_sdr_ch_;
  beacon_ch_ = beacon_ant_ % bs_sdr_ch_;
  max_frame_ = tddConf.value("max_frame", 0);
  bs_hw_framer_ = tddConf.value("bs_hw_framer", true);

  // Load/Build BS and Client SDRs' Schedules
  bs_array_frames_.resize(num_cells_);
  if (internal_measurement_ == true) {
    genBsSchedule(ref_node_enable_ ? CALIB_STAR_TOPO : CALIB_FULLY_CONN);
    genClientSchedule(ref_node_enable_ ? CALIB_STAR_TOPO : CALIB_FULLY_CONN);
  } else {
    auto jBsFrames = tddConf.value("frame_schedule", json::array());
    std::vector<std::string> frames;
    frames.assign(jBsFrames.begin(), jBsFrames.end());
    assert(frames.size() == num_cells_);
    for (size_t cell_id = 0; cell_id < num_cells_; cell_id++) {
      bs_array_frames_.at(cell_id).resize(n_bs_sdrs_.at(cell_id),
                                          frames.at(cell_id));
    }
    genBsSchedule(dl_pilots_en_ ? DL_SOUNDING : USER_INPUT);
    // read commons from client json config
    if (client_serial_present_ == false) {
      const size_t ref_cell_id = 0;
      num_cl_antennas_ =
          std::count(bs_array_frames_.at(ref_cell_id).at(0).begin(),
                     bs_array_frames_.at(ref_cell_id).at(0).end(), 'P');
      num_cl_sdrs_ = num_cl_antennas_ / cl_sdr_ch_;
    }
    if (dl_pilots_en_ == true) {
      genClientSchedule(DL_SOUNDING);
    } else {
      if (tddConf.find("ue_frame_schedule") == tddConf.end()) {
        genClientSchedule(USER_INPUT);
      } else {
        auto jClFrames = tddConf.value("ue_frame_schedule", json::array());
        cl_frames_.assign(jClFrames.begin(), jClFrames.end());
        assert(cl_frames_.size() == num_cl_sdrs_);
        for (size_t i = 0; i < num_cl_sdrs_; i++) {
          std::cout << "Client " << i << " schedule: " << cl_frames_.at(i)
                    << std::endl;
        }
      }
    }
  }
  cl_pilot_slots_ = Utils::loadSlots(cl_frames_, 'P');
  cl_ul_slots_ = Utils::loadSlots(cl_frames_, 'U');
  cl_dl_slots_ = Utils::loadSlots(cl_frames_, 'D');

  std::cout << "Slots: " << slot_per_frame_ << "\n"
            << " Pilots: " << pilot_slot_per_frame_ << "\n"
            << " Noise: " << noise_slot_per_frame_ << "\n"
            << " UL Slots: " << ul_slot_per_frame_ << "\n"
            << " DL Slots: " << dl_slot_per_frame_ << "\n"
            << " Client SDRs: " << num_cl_sdrs_ << std::endl;

  // Clients
  cl_data_mod_ = tddConf.value("ue_modulation", "QPSK");

  cl_agc_en_ = tddConf.value("agc_en", false);
  cl_agc_gain_init_ = tddConf.value("agc_gain_init", 70);  // 0 to 108
  cl_power_ramp_ = tddConf.value("ue_power_ramp", false);
  cl_power_ramp_lo_ = tddConf.value("ue_ramp_min_gain", 10);
  cl_power_ramp_hi_ = tddConf.value("ue_ramp_max_gain", 42);
  frame_mode_ = tddConf.value("frame_mode", "continuous_resync");
  hw_framer_ = tddConf.value("ue_hw_framer", false);
  auto tx_advance = tddConf.value("tx_advance", json::array());
  if (tx_advance.empty() == true) {
    tx_advance_.resize(num_cl_sdrs_, 250);
  } else {
    if (client_present_ && tx_advance.size() != num_cl_sdrs_) {
      MLPD_ERROR("tx_advance size must be same as the number of clients!\n");
      exit(1);
    }
    tx_advance_.assign(tx_advance.begin(), tx_advance.end());
  }
  auto corr_scale = tddConf.value("corr_scale", json::array());
  if (corr_scale.empty() == true) {
    corr_scale_.resize(num_cl_sdrs_, 1);
  } else {
    if (client_present_ && corr_scale.size() != num_cl_sdrs_) {
      MLPD_ERROR("tx_advance size must be same as the number of clients!\n");
      exit(1);
    }
    corr_scale_.assign(corr_scale.begin(), corr_scale.end());
  }
  ul_data_frame_num_ = tddConf.value("ul_data_frame_num", 1);
  dl_data_frame_num_ = tddConf.value("dl_data_frame_num", 1);

  // Help verify whether gain exceeds max value
  struct compare {
    const int key_;
    compare(int const& i) : key_(i) {}
    bool operator()(int const& i) { return (i > key_); }
  };
  cl_txgain_vec_.resize(2);
  cl_rxgain_vec_.resize(2);
  auto jClTxgainA_vec = tddConf.value("ue_tx_gain_a", json::array());
  cl_txgain_vec_.at(0).assign(jClTxgainA_vec.begin(), jClTxgainA_vec.end());
  auto jClRxgainA_vec = tddConf.value("ue_rx_gain_a", json::array());
  cl_rxgain_vec_.at(0).assign(jClRxgainA_vec.begin(), jClRxgainA_vec.end());
  auto jClTxgainB_vec = tddConf.value("ue_tx_gain_b", json::array());
  cl_txgain_vec_.at(1).assign(jClTxgainB_vec.begin(), jClTxgainB_vec.end());
  auto jClRxgainB_vec = tddConf.value("ue_rx_gain_b", json::array());
  cl_rxgain_vec_.at(1).assign(jClRxgainB_vec.begin(), jClRxgainB_vec.end());

  max_tx_gain_ue_ = tddConf.value("maxTxGainUE", 109);
  compare find_guilty(max_tx_gain_ue_);
  if (std::any_of(cl_txgain_vec_.at(0).begin(), cl_txgain_vec_.at(0).end(),
                  find_guilty)) {
    std::string msg = "ERROR: UE ChanA - Maximum TX gain value is ";
    msg += std::to_string(max_tx_gain_ue_);
    throw std::invalid_argument(msg);
  }
  if (std::any_of(cl_txgain_vec_.at(1).begin(), cl_txgain_vec_.at(1).end(),
                  find_guilty)) {
    std::string msg = "ERROR: UE ChanB - Maximum TX gain value is ";
    msg += std::to_string(max_tx_gain_ue_);
    throw std::invalid_argument(msg);
  }

  bw_filter_ = rate_ + 2 * nco_;
  radio_rf_freq_ = freq_ - nco_;
  ofdm_symbol_size_ = fft_size_ + cp_size_;
  slot_samp_size_ = symbol_per_slot_ * ofdm_symbol_size_;
  samps_per_slot_ = slot_samp_size_ + prefix_ + postfix_;
  assert((internal_measurement_ && num_cl_antennas_ == 0) || (dl_pilots_en_) ||
         (num_cl_sdrs_ > 0 && slot_per_frame_ == cl_frames_.at(0).size()));

  ul_data_slot_present_ =
      (internal_measurement_ == false) &&
      ((bs_present_ == true && (ul_slots_.at(0).empty() == false)) ||
       (client_present_ == true && cl_ul_slots_.at(0).empty() == false));

  dl_data_slot_present_ =
      (internal_measurement_ == false) &&
      ((bs_present_ == true && dl_slots_.empty() == false &&
        (dl_slots_.at(0).empty() == false)) ||
       (client_present_ == true && cl_dl_slots_.at(0).empty() == false));

  tx_scale_ = tddConf.value("tx_scale", 0);
  this->genPilots();

  this->loadULData();
  this->loadDLData();

  bool recording =
      pilot_slot_per_frame_ + ul_slot_per_frame_ + dl_slot_per_frame_ > 0;
  if (recording == true) {
    // set trace file path
    time_t now = time(0);
    tm* ltm = localtime(&now);
    std::string filename;
    if (internal_measurement_ && num_cl_antennas_ == 0) {
      filename =
          directory_ + "/trace-internal-meas-" +
          std::to_string(1900 + ltm->tm_year) + "-" +
          std::to_string(1 + ltm->tm_mon) + "-" + std::to_string(ltm->tm_mday) +
          "-" + std::to_string(ltm->tm_hour) + "-" +
          std::to_string(ltm->tm_min) + "-" + std::to_string(ltm->tm_sec) +
          "_" + std::to_string(num_cells_) + "_" +
          std::to_string(num_bs_antennas_all_) + ".hdf5";
    } else if (internal_measurement_ && num_cl_antennas_ > 0) {
      filename =
          directory_ + "/trace-reciprocity-" +
          std::to_string(1900 + ltm->tm_year) + "-" +
          std::to_string(1 + ltm->tm_mon) + "-" + std::to_string(ltm->tm_mday) +
          "-" + std::to_string(ltm->tm_hour) + "-" +
          std::to_string(ltm->tm_min) + "-" + std::to_string(ltm->tm_sec) +
          "_" + std::to_string(num_cells_) + "_" +
          std::to_string(num_bs_antennas_all_) + "x" +
          std::to_string(num_cl_antennas_) + ".hdf5";
    } else {
      std::string ul_present_str = (ul_data_slot_present_ ? "uplink-" : "");
      std::string dl_present_str = (dl_data_slot_present_ ? "downlink-" : "");
      filename =
          directory_ + "/trace-" + ul_present_str + dl_present_str +
          std::to_string(1900 + ltm->tm_year) + "-" +
          std::to_string(1 + ltm->tm_mon) + "-" + std::to_string(ltm->tm_mday) +
          "-" + std::to_string(ltm->tm_hour) + "-" +
          std::to_string(ltm->tm_min) + "-" + std::to_string(ltm->tm_sec) +
          "_" + std::to_string(num_cells_) + "_" +
          std::to_string(num_bs_antennas_all_) + "x" +
          std::to_string(num_cl_antennas_) + ".hdf5";
    }
    trace_file_ = tddConf.value("trace_file", filename);
    recorder_thread_num_ = tddConf.value(
        "recorder_thread",
        bs_present_ || (client_present_ && cl_dl_slots_.at(0).size() > 0)
            ? RECORDER_THREAD_NUM
            : 0);
    reader_thread_num_ = (client_present_ && ul_slot_per_frame_ > 0) +
                         (bs_present_ && dl_slot_per_frame_ > 0);
  } else {
    recorder_thread_num_ = 0;
    reader_thread_num_ = 0;
  }

  // Multi-threading settings
  unsigned num_cores = this->getCoreCount();
  MLPD_INFO("Cores found %u ... \n", num_cores);
  if (bs_present_ == true &&
      pilot_slot_per_frame_ + ul_slot_per_frame_ + dl_slot_per_frame_ > 0) {
    bs_rx_thread_num_ =
        (num_cores >= (2 * RX_THREAD_NUM))
            ? std::min(RX_THREAD_NUM, static_cast<int>(num_bs_sdrs_all_))
            : 1;
    if (internal_measurement_ == true && ref_node_enable_ == true) {
      bs_rx_thread_num_ = 2;
    }
  } else {
    bs_rx_thread_num_ = 0;
  }

  if (client_present_ == true && cl_dl_slots_.at(0).size() > 0) {
    cl_rx_thread_num_ = num_cl_sdrs_;
  } else {
    cl_rx_thread_num_ = 0;
  }

  core_alloc_ = num_cores >= (1 + recorder_thread_num_ + reader_thread_num_ +
                              bs_rx_thread_num_ + num_cl_sdrs_);

  if (core_alloc_ == true) {
    if (bs_present_ == true) {
      MLPD_INFO("Allocating %zu cores to receive threads ... \n",
                bs_rx_thread_num_);
      MLPD_INFO("Allocating %zu cores to record threads ... \n",
                recorder_thread_num_);
      MLPD_INFO("Allocating %zu cores to read threads ... \n",
                reader_thread_num_);
    }
    if (client_present_ == true) {
      MLPD_INFO("Allocating %zu cores to client threads ... \n", num_cl_sdrs_);
    }
  }

  tx_frame_delta_ =
      std::ceil(TIME_DELTA_MS / (1e3 * this->getFrameDurationSec()));
  std::printf(
      "Config: %zu BS, %zu BS radios (total), %zu UE antennas, %zu pilot "
      "symbols per "
      "frame,\n\t%zu uplink data symbols per frame, %zu downlink data "
      "symbols per frame,\n\t%zu OFDM subcarriers (%zu data subcarriers), "
      "modulation %s, frame time %.3f usec \n",
      num_cells_, num_bs_sdrs_all_, num_cl_antennas_, pilot_slot_per_frame_,
      ul_slot_per_frame_, dl_slot_per_frame_, fft_size_,
      symbol_data_subcarrier_num_, data_mod_.c_str(),
      this->getFrameDurationSec() * 1e6);
  std::printf(
      "Thread Config: %zu BS receive threads, %zu Client receive "
      "threads, %zu recording threads, %zu reading thread\n",
      bs_rx_thread_num_, num_cl_sdrs_, recorder_thread_num_,
      reader_thread_num_);
  running_.store(true);
}

void Config::loadTopology(std::string serials_file, const bool bs_only,
                          const bool client_only) {
  // Load serials file (loads hub, sdr, and rrh serials)
  std::string serials_str;
  Utils::loadTDDConfig(serials_file, serials_str);
  std::stringstream ss;
  if (serials_str.empty() == false) {
    const auto j_serials = json::parse(serials_str, nullptr, true, true);
    if (j_serials.find("BaseStations") != j_serials.end()) {
      json j_bs_serials;
      ss << j_serials.value("BaseStations", j_bs_serials);
      j_bs_serials = json::parse(ss);
      ss.str(std::string());
      ss.clear();
      num_cells_ = j_bs_serials.size();
      bs_sdr_ids_.resize(num_cells_);
      calib_ids_.resize(num_cells_);
      n_bs_sdrs_.resize(num_cells_);
      n_bs_antennas_.resize(num_cells_);

      for (size_t i = 0; i < num_cells_; i++) {
        const auto j_serials = json::parse(serials_str, nullptr, true, true);
        json serials_conf;
        std::string cell_str = "BS" + std::to_string(i);
        ss << j_bs_serials.value(cell_str, serials_conf);
        serials_conf = json::parse(ss);
        ss.str(std::string());
        ss.clear();

        auto hub_serial = serials_conf.value("hub", "");
        hub_ids_.push_back(hub_serial);
        auto sdr_serials = serials_conf.value("sdr", json::array());
        bs_sdr_ids_.at(i).assign(sdr_serials.begin(), sdr_serials.end());

        // Append calibration node
        if ((internal_measurement_ == true && ref_node_enable_ == true) ||
            sample_cal_en_ == true) {
          calib_ids_.at(i) = serials_conf.value("reference", "");
          if (calib_ids_.at(i).empty()) {
            MLPD_ERROR("No calibration node ID found in topology file!\n");
            exit(1);
          }
          std::cout << "Calibration Node: " << calib_ids_.at(i) << std::endl;
          bs_sdr_ids_.at(i).push_back(calib_ids_.at(i));
        }

        n_bs_sdrs_.at(i) = bs_sdr_ids_.at(i).size();
        n_bs_antennas_.at(i) = bs_sdr_ch_ * n_bs_sdrs_.at(i);
        num_bs_sdrs_all_ += n_bs_sdrs_.at(i);
        num_bs_antennas_all_ += n_bs_antennas_.at(i);
        cal_ref_sdr_id_ = n_bs_sdrs_.at(i) - 1;
        MLPD_INFO(
            "Loading devices - cell %zu, sdrs %zu, antennas: %zu, "
            "total bs srds: %zu\n",
            i, n_bs_sdrs_.at(i), n_bs_antennas_.at(i), num_bs_sdrs_all_);
      }
      // Print Topology
      std::cout << "Topology: " << std::endl;
      for (size_t i = 0; i < bs_sdr_ids_.size(); i++) {
        std::cout << "BS" + std::to_string(i) + " Hub:"
                  << (hub_ids_.empty() == false ? hub_ids_.at(i) : "")
                  << std::endl;
        for (size_t j = 0; j < bs_sdr_ids_.at(i).size(); j++) {
          std::cout << " \t- " << bs_sdr_ids_.at(i).at(j) << std::endl;
        }
      }

      // Array with cummulative sum of SDRs in cells
      n_bs_sdrs_agg_.resize(num_cells_ + 1);
      n_bs_sdrs_agg_.at(0) = 0;  //n_bs_sdrs_[0];
      for (size_t i = 0; i < num_cells_; i++) {
        n_bs_sdrs_agg_.at(i + 1) = n_bs_sdrs_agg_.at(i) + n_bs_sdrs_.at(i);
      }

    } else {
      num_cells_ = 0;
      bs_present_ = false;
      if (internal_measurement_ == true) {
        MLPD_ERROR("No BS devices are present in the serial file!");
        exit(1);
      }
    }

    // read client serials
    client_serial_present_ = (j_serials.find("Clients") != j_serials.end());
    if (client_serial_present_) {
      json j_ue_serials;
      ss << j_serials.value("Clients", j_ue_serials);
      j_ue_serials = json::parse(ss);
      ss.str(std::string());
      ss.clear();
      auto ue_serials = j_ue_serials.value("sdr", json::array());
      cl_sdr_ids_.assign(ue_serials.begin(), ue_serials.end());
      num_cl_sdrs_ = cl_sdr_ids_.size();
      num_cl_antennas_ = num_cl_sdrs_ * cl_sdr_ch_;
    } else {
      num_cl_sdrs_ = num_cl_antennas_ = 0;
    }

    client_present_ = !bs_only && client_serial_present_ && num_cl_sdrs_ > 0;
    bs_present_ = !client_only && num_bs_sdrs_all_ > 0;
  } else {
    std::cout << "Serial file empty! Exitting.." << std::endl;
    exit(1);
  }
}

void Config::genBsSchedule(BsSchedType type) {
  size_t num_channels = bs_channel_.size();
  switch (type) {
    case CALIB_STAR_TOPO:
      for (size_t c = 0; c < num_cells_; c++) {
        cal_ref_sdr_id_ = n_bs_sdrs_[c] - 1;
        bs_array_frames_[c].resize(n_bs_sdrs_[c]);
        size_t beacon_slot = 0;
        if (num_cl_antennas_ > 0)
          beacon_slot = 1;  // add a "B" in the front for UE sync
        size_t frame_length =
            beacon_slot + n_bs_antennas_[c] + num_cl_antennas_;
        bs_array_frames_[c][cal_ref_sdr_id_] = std::string(frame_length, 'G');
        bs_array_frames_[c][cal_ref_sdr_id_].replace(
            beacon_slot + num_channels * cal_ref_sdr_id_, 1, "P");

        for (size_t i = 0; i < n_bs_sdrs_[c]; i++) {
          if (i != cal_ref_sdr_id_) {
            bs_array_frames_[c][i] = std::string(frame_length, 'G');
            if (num_cl_antennas_ > 0) bs_array_frames_[c][i].replace(0, 1, "B");
            for (size_t ch = 0; ch < num_channels; ch++) {
              bs_array_frames_[c][i].replace(
                  beacon_slot + i * num_channels + ch, 1, "P");
              bs_array_frames_[c][cal_ref_sdr_id_].replace(
                  beacon_slot + num_channels * i + ch, 1, "R");
            }
            bs_array_frames_[c][i].replace(
                beacon_slot + num_channels * cal_ref_sdr_id_, 1, "R");
            for (size_t p = 0; p < num_cl_antennas_; p++)
              bs_array_frames_[c][i].replace(
                  beacon_slot + num_channels * n_bs_sdrs_[c] + p, 1, "R");
          }
        }
      }
      break;
    case CALIB_FULLY_CONN:
      // For full matrix measurements (all bs nodes transmit and receive)
      for (size_t c = 0; c < num_cells_; c++) {
        cal_ref_sdr_id_ = n_bs_sdrs_[c] - 1;
        bs_array_frames_[c].resize(n_bs_sdrs_[c]);
        size_t frame_length = num_channels * n_bs_sdrs_[c] * guard_mult_;
        for (size_t i = 0; i < n_bs_sdrs_[c]; i++) {
          bs_array_frames_[c][i] = std::string(frame_length, 'G');
        }
        for (size_t i = 0; i < n_bs_sdrs_[c]; i++) {
          for (size_t ch = 0; ch < num_channels; ch++) {
            bs_array_frames_[c][i].replace(guard_mult_ * i * num_channels + ch,
                                           1, "P");
          }
          for (size_t k = 0; k < n_bs_sdrs_[c]; k++) {
            if (i != k) {
              for (size_t ch = 0; ch < num_channels; ch++) {
                bs_array_frames_[c][k].replace(
                    guard_mult_ * i * num_channels + ch, 1, "R");
              }
            }
          }
        }
      }
      break;
    case DL_SOUNDING:
      for (size_t cell_id = 0; cell_id < num_cells_; cell_id++) {
        auto& bs_array_frame = bs_array_frames_.at(cell_id);
        const auto& num_cell_bs_antennas = n_bs_antennas_.at(cell_id);
        const auto& num_cell_bs_radios = n_bs_sdrs_.at(cell_id);
        bs_array_frame.resize(num_cell_bs_radios);
        std::cout << "BS antennas...." << num_cell_bs_antennas << std::endl;
        std::cout << "BS radios......" << num_cell_bs_radios << std::endl;

        // If downlink pilots enabled
        const size_t beacon_slot = 0;

        const size_t num_uplink_pilots = num_cl_antennas_;
        std::cout << "UE antennas...." << num_uplink_pilots << std::endl;
        //Produces 1 less than value
        const size_t num_guard_after_beacon = 3;
        const size_t num_guard_after_ul_pilots = 2;
        const size_t num_guard_after_dl_pilots = 2;
        const size_t num_dl_pilots_per_ant = 1;
        const size_t frame_length =
            num_guard_after_beacon + num_uplink_pilots +
            num_guard_after_ul_pilots +
            (num_cell_bs_antennas * num_dl_pilots_per_ant) +
            num_guard_after_dl_pilots;

        for (size_t i = 0; i < num_cell_bs_radios; i++) {
          auto& frame = bs_array_frame.at(i);
          frame = std::string(frame_length, 'G');
          //Add beacon
          frame.at(beacon_slot) = 'B';
          // Add uplink pilots (1 per Ue antenna)
          for (size_t uplink_pilot = 0; uplink_pilot < num_uplink_pilots;
               uplink_pilot++) {
            frame.at(beacon_slot + num_guard_after_beacon + uplink_pilot) = 'P';
          }
          // Add downlink pilots (1 per bs antenna)
          for (size_t ch = 0; ch < num_channels; ch++) {
            frame.at(beacon_slot + num_guard_after_beacon + num_uplink_pilots +
                     num_guard_after_ul_pilots + ((i * num_channels) + ch)) =
                'D';
          }
          std::cout << frame << std::endl;
        }
      }
      break;
    default:
      break;
  }
  for (size_t c = 0; c < num_cells_; c++) {
    for (std::string const& s : bs_array_frames_[c])
      std::cout << s << std::endl;
  }
  const size_t ref_cell_id = 0;
  // Assume all BS nodes will transmit the same number of downlink/uplink pilots, etc. Grab the first one
  // TODO: Extend to multi-cell
  pilot_slots_ = Utils::loadSlots(bs_array_frames_.at(ref_cell_id), 'P');
  noise_slots_ = Utils::loadSlots(bs_array_frames_.at(ref_cell_id), 'N');
  ul_slots_ = Utils::loadSlots(bs_array_frames_.at(ref_cell_id), 'U');
  dl_slots_ = Utils::loadSlots(bs_array_frames_.at(ref_cell_id), 'D');
  slot_per_frame_ = bs_array_frames_.at(ref_cell_id).at(0).size();
  pilot_slot_per_frame_ = pilot_slots_.at(ref_cell_id).size();
  noise_slot_per_frame_ = noise_slots_.at(ref_cell_id).size();
  ul_slot_per_frame_ = ul_slots_.at(ref_cell_id).size();
  dl_slot_per_frame_ = dl_slots_.at(ref_cell_id).size();
}

void Config::genClientSchedule(BsSchedType type) {
  const size_t ref_cell_id = 0;
  size_t num_channels = bs_channel_.size();
  switch (type) {
    case CALIB_STAR_TOPO:
      // Two pilots (up/down) plus additional user pilots
      pilot_slot_per_frame_ = 2 + num_cl_antennas_;
      if (num_cl_antennas_ > 0) {
        cl_frames_.resize(num_cl_sdrs_);
        std::string empty_frame = std::string(slot_per_frame_, 'G');
        for (size_t i = 0; i < num_cl_sdrs_; i++) {
          cl_frames_.at(i) = empty_frame;
          for (size_t n = 0; n < n_bs_sdrs_.at(ref_cell_id); n++) {
            if (n != cal_ref_sdr_id_) {
              for (size_t ch = 0; ch < num_channels; ch++) {
                size_t slot = 1 + n * num_channels + ch;
                std::cout << "Replacing slot " << slot << std::endl;
                // downlink symbol for UEs
                cl_frames_.at(i).replace(slot, 1, "D");
              }
            }
          }
          for (size_t ch = 0; ch < cl_sdr_ch_; ch++) {
            size_t slot = 1 + n_bs_antennas_[ref_cell_id] + cl_sdr_ch_ * i + ch;
            cl_frames_.at(i).replace(slot, 1, "P");
          }
          std::cout << "Client " << i << " schedule: " << cl_frames_.at(i)
                    << std::endl;
        }
      }
      break;
    case CALIB_FULLY_CONN:
      if (num_cl_antennas_ > 0) {
        std::cout << "Client Schedule is not supported!" << std::endl;
        exit(1);
      }
      pilot_slot_per_frame_ = n_bs_antennas_.at(ref_cell_id);
      break;
    case DL_SOUNDING: {
      cl_frames_.resize(num_cl_sdrs_);
      std::string empty_frame = std::string(slot_per_frame_, 'G');
      //Include all the D's
      for (const auto& dl_slot_sdr : dl_slots_) {
        for (const auto& dl_ind : dl_slot_sdr) {
          empty_frame.at(dl_ind) = 'D';
        }
      }
      //Include all the U's
      const size_t ref_sdr = 0;
      for (const auto& ul_ind : ul_slots_.at(ref_sdr)) {
        empty_frame.at(ul_ind) = 'U';
      }

      //Add the per sdr ul pilot schedule
      for (size_t i = 0; i < num_cl_sdrs_; i++) {
        cl_frames_.at(i) = empty_frame;
        //Look at the P index array.
        const auto& ul_pilots = pilot_slots_.at(ref_sdr);
        size_t cl_ant_num = cl_sdr_ch_ * i;
        size_t ul_pilot_idx = 0;
        for (const auto& ul_pilot : ul_pilots) {
          if (ul_pilot_idx == cl_ant_num) {
            cl_frames_.at(i).at(ul_pilot) = 'P';
            cl_ant_num++;
            //Exit when the last pilot channel has been found
            if (cl_ant_num >= (cl_sdr_ch_ * (i + 1))) {
              break;
            }
          }
          ul_pilot_idx++;
        }

        std::cout << "Client " << i << " schedule: " << cl_frames_.at(i)
                  << std::endl;
      }
      break;
    }
    default:
      cl_frames_.resize(num_cl_sdrs_);
      for (size_t i = 0; i < cl_frames_.size(); i++) {
        cl_frames_.at(i) = bs_array_frames_.at(ref_cell_id).at(0);
        size_t frame_len = bs_array_frames_.at(ref_cell_id).at(0).size();
        for (size_t s = 0; s < frame_len; s++) {
          char c = cl_frames_.at(i).at(s);
          if (c == 'B') {
            // Dummy RX used in PHY scheduler
            cl_frames_.at(i).replace(s, 1, "G");
          } else if (c == 'P' and
                     ((cl_sdr_ch_ == 1 and pilot_slots_.at(0).at(i) != s) or
                      (cl_sdr_ch_ == 2 and
                       (pilot_slots_.at(0).at(2 * i) != s and
                        pilot_slots_.at(0).at(i * 2 + 1) != s)))) {
            cl_frames_.at(i).replace(s, 1, "G");
          } else if (c != 'P' && c != 'U' && c != 'D') {
            cl_frames_.at(i).replace(s, 1, "G");
          }
        }
        std::cout << "Client " << i << " schedule: " << cl_frames_.at(i)
                  << std::endl;
      }
  }
}

void Config::genPilots() {
  std::vector<std::complex<int16_t>> prefix_zpad(prefix_, 0);
  std::vector<std::complex<int16_t>> postfix_zpad(postfix_, 0);

  // compose Beacon slot:
  // STS Sequence (for AGC) + GOLD Sequence (for Sync)
  // 15reps of STS(16) + 2reps of gold_ifft(128)
  srand(time(NULL));
  const int seq_len = 128;
  std::vector<std::vector<float>> gold_ifft =
      CommsLib::getSequence(CommsLib::GOLD_IFFT);
  auto gold_ifft_ci16 = Utils::float_to_cint16(gold_ifft);
  gold_cf32_.clear();
  for (size_t i = 0; i < seq_len; i++) {
    gold_cf32_.push_back(std::complex<float>(gold_ifft[0][i], gold_ifft[1][i]));
  }

  std::vector<std::vector<float>> sts_seq =
      CommsLib::getSequence(CommsLib::STS_SEQ);
  auto sts_seq_ci16 = Utils::float_to_cint16(sts_seq);

  // Populate STS (stsReps repetitions)
  int stsReps = 15;
  for (int i = 0; i < stsReps; i++) {
    beacon_ci16_.insert(beacon_ci16_.end(), sts_seq_ci16.begin(),
                        sts_seq_ci16.end());
  }

  // Populate gold sequence (two reps, 128 each)
  int goldReps = 2;
  for (int i = 0; i < goldReps; i++) {
    beacon_ci16_.insert(beacon_ci16_.end(), gold_ifft_ci16.begin(),
                        gold_ifft_ci16.end());
  }

  beacon_size_ = beacon_ci16_.size();

  if (slot_samp_size_ < beacon_size_) {
    std::string msg = "Minimum supported slot_samp_size is ";
    msg += std::to_string(beacon_size_);
    msg += ". Current slot_samp_size is " + std::to_string(slot_samp_size_);
    throw std::invalid_argument(msg);
  }

  beacon_ = Utils::cint16_to_uint32(beacon_ci16_, false, "QI");
  coeffs_ = Utils::cint16_to_uint32(gold_ifft_ci16, true, "QI");

  std::vector<std::complex<int16_t>> post_beacon_zpad(
      slot_samp_size_ - beacon_size_, 0);
  beacon_ci16_.insert(beacon_ci16_.begin(), prefix_zpad.begin(),
                      prefix_zpad.end());
  beacon_ci16_.insert(beacon_ci16_.end(), post_beacon_zpad.begin(),
                      post_beacon_zpad.end());
  beacon_ci16_.insert(beacon_ci16_.end(), postfix_zpad.begin(),
                      postfix_zpad.end());

  neg_beacon_ci16_.resize(beacon_ci16_.size());
  for (size_t i = 0; i < beacon_ci16_.size(); i++) {
    neg_beacon_ci16_.at(i) = std::complex<int16_t>(0, 0) - beacon_ci16_.at(i);
  }

  // compose pilot slot
  if (fft_size_ > kMaxSupportedFFTSize) {
    fft_size_ = kMaxSupportedFFTSize;
    std::cout << "Unsupported fft size! Setting fft size to "
              << kMaxSupportedFFTSize << "..." << std::endl;
  }

  if (fft_size_ < kMinSupportedFFTSize) {
    fft_size_ = kMinSupportedFFTSize;
    std::cout << "Unsupported fft size! Setting fft size to "
              << kMinSupportedFFTSize << "..." << std::endl;
  }

  if (cp_size_ > kMaxSupportedCPSize) {
    cp_size_ = 0;
    std::cout << "Invalid cp size! Setting cp size to " << cp_size_ << "..."
              << std::endl;
  }

  if (fft_size_ == 64) {
    pilot_sym_f_ = CommsLib::getSequence(CommsLib::LTS_SEQ_F);
    pilot_sym_t_ = CommsLib::getSequence(CommsLib::LTS_SEQ);
    symbol_data_subcarrier_num_ = Consts::kNumMappedSubcarriers_80211;
  } else {  // Construct Zadoff-Chu-based pilot
    pilot_sym_f_ = CommsLib::getSequence(CommsLib::LTE_ZADOFF_CHU_F,
                                         symbol_data_subcarrier_num_);
    pilot_sym_t_ = CommsLib::getSequence(CommsLib::LTE_ZADOFF_CHU,
                                         symbol_data_subcarrier_num_);
  }

  auto iq_tmp_ci16 = Utils::float_to_cint16(pilot_sym_t_);
  auto iq_cf = Utils::cint16_to_cfloat(iq_tmp_ci16);
  float max_amp = 0;
  for (size_t i = 0; i < iq_cf.size(); i++) {
    float this_amp = std::abs(iq_cf.at(i));
    if (this_amp > max_amp) max_amp = this_amp;
  }
  std::printf("Max pilot amplitude = %.2f\n", max_amp);
  // 6dB Power backoff value to avoid clipping in the data due to high PAPR
  static constexpr float ofdm_pwr_scale_lin = 4;
  if (tx_scale_ == 0) {
    tx_scale_ = 1 / (ofdm_pwr_scale_lin * max_amp);
  }
  for (size_t i = 0; i < iq_cf.size(); i++) {
    iq_cf.at(i) *= tx_scale_;
  }
  auto iq_ci16 = Utils::cfloat_to_cint16(iq_cf);
  iq_ci16.insert(iq_ci16.begin(), iq_ci16.end() - cp_size_, iq_ci16.end());

  pilot_ci16_.clear();
  pilot_ci16_.insert(pilot_ci16_.begin(), prefix_zpad.begin(),
                     prefix_zpad.end());
  for (size_t i = 0; i < symbol_per_slot_; i++)
    pilot_ci16_.insert(pilot_ci16_.end(), iq_ci16.begin(), iq_ci16.end());
  pilot_ci16_.insert(pilot_ci16_.end(), postfix_zpad.begin(),
                     postfix_zpad.end());

  pilot_ = Utils::cint16_to_uint32(pilot_ci16_, false, "QI");

  size_t remain_size =
      kFpgaTxRamSize - pilot_.size();  // 4096 is the size of TX_RAM in the FPGA
  for (size_t j = 0; j < remain_size; j++) pilot_.push_back(0);
#if DEBUG_PRINT
  for (size_t j = 0; j < pilot_ci16_.size(); j++) {
    std::cout << "Pilot[" << j << "]: \t " << pilot_ci16_.at(j) << std::endl;
  }
#endif

  data_ind_ = CommsLib::getDataSc(fft_size_, symbol_data_subcarrier_num_);
  pilot_sc_ = CommsLib::getPilotScValue(fft_size_, symbol_data_subcarrier_num_);
  pilot_sc_ind_ =
      CommsLib::getPilotScIndex(fft_size_, symbol_data_subcarrier_num_);
}

void Config::loadULData() {
  // compose data slot
  if (ul_data_slot_present_) {
    txdata_time_dom_.resize(num_cl_antennas_);
    txdata_freq_dom_.resize(num_cl_antennas_);
    // For now, we're reading one frame worth of data
    for (size_t i = 0; i < num_cl_sdrs_; i++) {
      std::string filename_tag = cl_data_mod_ + "_" +
                                 std::to_string(symbol_data_subcarrier_num_) +
                                 "_" + std::to_string(fft_size_) + "_" +
                                 std::to_string(symbol_per_slot_) + "_" +
                                 std::to_string(cl_ul_slots_[i].size()) + "_" +
                                 std::to_string(ul_data_frame_num_) + "_" +
                                 cl_channel_ + "_" + std::to_string(i) + ".bin";

      std::string filename_ul_data_f =
          directory_ + "/ul_data_f_" + filename_tag;
      ul_tx_fd_data_files_.push_back("ul_data_f_" + filename_tag);
      std::string filename_ul_data_t =
          directory_ + "/ul_data_t_" + filename_tag;
      ul_tx_td_data_files_.push_back(filename_ul_data_t);
    }
  }
}

void Config::loadDLData() {
  // compose data slot
  if (dl_data_slot_present_) {
    dl_txdata_time_dom_.resize(num_bs_antennas_all_);
    dl_txdata_freq_dom_.resize(num_bs_antennas_all_);
    // For now, we're reading one frame worth of data
    for (size_t i = 0; i < num_bs_sdrs_all_; i++) {
      std::string filename_tag =
          data_mod_ + "_" + std::to_string(symbol_data_subcarrier_num_) + "_" +
          std::to_string(fft_size_) + "_" + std::to_string(symbol_per_slot_) +
          "_" + std::to_string(dl_slot_per_frame_) + "_" +
          std::to_string(dl_data_frame_num_) + "_" + bs_channel_ + "_" +
          std::to_string(i) + ".bin";

      std::string filename_dl_data_f =
          directory_ + "/dl_data_f_" + filename_tag;
      dl_tx_fd_data_files_.push_back("dl_data_f_" + filename_tag);

      std::string filename_dl_data_t =
          directory_ + "/dl_data_t_" + filename_tag;
      dl_tx_td_data_files_.push_back(filename_dl_data_t);
    }
  }
}

size_t Config::getNumAntennas() {
  size_t ret;
  if (this->bs_present_ == false) {
    ret = 1;
  } else {
    ret = (n_bs_sdrs_.at(0) * bs_channel_.length());
  }
  return ret;
}

size_t Config::getMaxNumAntennas() {
  size_t ret;
  /* Max number of antennas across cells */
  if (this->bs_present_ == false) {
    ret = 1;
  } else {
    size_t max_num_sdr = 0;
    for (size_t i = 0; i < this->num_cells_; i++) {
      if (max_num_sdr < this->n_bs_sdrs_.at(i)) {
        max_num_sdr = this->n_bs_sdrs_.at(i);
      }
      if (internal_measurement_ == true && ref_node_enable_ == true) {
        max_num_sdr--;  // exclude the ref sdr
      }
    }
    ret = (max_num_sdr * bs_channel_.length());
  }
  return ret;
}

size_t Config::getNumBsSdrs() {
  size_t sdr_num;
  /* Total number of Sdrs across cells */
  if (this->bs_present_ == false) {
    sdr_num = 1;
  } else {
    sdr_num = num_bs_sdrs_all_;
    for (size_t i = 0; i < num_cells_; i++) {
      if (internal_measurement_ == true && ref_node_enable_ == true) sdr_num--;
    }
  }
  return sdr_num;
}

size_t Config::getTotNumAntennas() {
  size_t ret;
  if (this->bs_present_ == false) {
    ret = 0;
  } else {
    ret = this->getNumBsSdrs() * bs_channel_.length();
  }
  return ret;
}

size_t Config::getNumRecordedSdrs() {
  size_t ret = 0;
  /* Total number of antennas across cells */
  if (this->bs_present_ == true) {
    ret += getNumBsSdrs();
  }
  if (this->client_present_ == true) {
    // Only consider clients that have 'D' in their schedule
    for (size_t i = 0; i < cl_dl_slots_.size(); i++) {
      if (cl_dl_slots_.at(i).size() > 0) ret++;
    }
  }
  return ret;
}

Config::~Config() {}

int Config::getClientId(size_t radio_id, size_t slot_id) {
  // TODO: Consider cell_id
  std::vector<size_t>::iterator it;
  it = find(pilot_slots_.at(radio_id).begin(), pilot_slots_.at(radio_id).end(),
            slot_id);
  if (it != pilot_slots_.at(radio_id).end()) {
    return (int)(it - pilot_slots_.at(radio_id).begin());
  }
  return -1;
}

int Config::getNoiseSlotIndex(size_t radio_id, size_t slot_id) {
  // TODO: Consider cell_id
  std::vector<size_t>::iterator it;
  it = find(noise_slots_.at(radio_id).begin(), noise_slots_.at(radio_id).end(),
            slot_id);
  if (it != noise_slots_.at(radio_id).end())
    return (int)(it - noise_slots_.at(radio_id).begin());
  return -1;
}

int Config::getUlSlotIndex(size_t radio_id, size_t slot_id) {
  // TODO: Consider cell_id
  std::vector<size_t>::iterator it;
  it = find(ul_slots_.at(radio_id).begin(), ul_slots_.at(radio_id).end(),
            slot_id);
  if (it != ul_slots_.at(radio_id).end()) {
    return (int)(it - ul_slots_.at(radio_id).begin());
  }
  return -1;
}

int Config::getDlSlotIndex(size_t radio_id, size_t slot_id) {
  std::vector<size_t>::iterator it;
  it = find(cl_dl_slots_.at(radio_id).begin(), cl_dl_slots_.at(radio_id).end(),
            slot_id);
  if (it != cl_dl_slots_.at(radio_id).end())
    return (int)(it - cl_dl_slots_.at(radio_id).begin());
  return -1;
}

bool Config::isPilot(size_t cell_id, size_t radio_id, size_t slot_id) {
  try {
    return bs_array_frames_.at(cell_id).at(radio_id).at(slot_id) == 'P';
  } catch (const std::out_of_range&) {
    return false;
  }
}

bool Config::isNoise(size_t cell_id, size_t radio_id, size_t slot_id) {
  try {
    return bs_array_frames_.at(cell_id).at(radio_id).at(slot_id) == 'N';
  } catch (const std::out_of_range&) {
    return false;
  }
}

bool Config::isUlData(size_t cell_id, size_t radio_id, size_t slot_id) {
  try {
    return bs_array_frames_.at(cell_id).at(radio_id).at(slot_id) == 'U';
  } catch (const std::out_of_range&) {
    return false;
  }
}

bool Config::isDlData(size_t radio_id, size_t slot_id) {
  try {
    return cl_frames_.at(radio_id).at(slot_id) == 'D';
  } catch (const std::out_of_range&) {
    return false;
  }
}

unsigned Config::getCoreCount() {
  unsigned n_cores = std::thread::hardware_concurrency();
#if DEBUG_PRINT
  std::cout << "number of CPU cores " << std::to_string(n_cores) << std::endl;
#endif
  return n_cores;
}

extern "C" {
__attribute__((visibility("default"))) Config* Config_new(char* filename,
                                                          char* storepath,
                                                          bool bs_only,
                                                          bool client_only) {
  Config* cfg = new Config(filename, storepath, bs_only, client_only);
  return cfg;
}
}
