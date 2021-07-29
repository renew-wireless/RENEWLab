/*
 Copyright (c) 2018-2019, Rice University 
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
{
    std::string conf_str;
    Utils::loadTDDConfig(jsonfile, conf_str);
    // Enable comments in json file
    const auto jConf = json::parse(conf_str, nullptr, true, true);
    std::stringstream ss;
    json tddConf;

    ss << jConf.value("BaseStations", tddConf);
    tddConf = json::parse(ss);
    ss.str(std::string());
    ss.clear();

    json tddConfCl;
    ss << jConf.value("Clients", tddConfCl);
    tddConfCl = json::parse(ss);
    ss.str(std::string());
    ss.clear();

    if (bs_only && client_only == true) {
        MLPD_ERROR("Client-Only and BS-Only can't both be enabled!\n");
        exit(1);
    }

    if (tddConf.empty() == true) {
        MLPD_ERROR("Both \"BaseStations\" and \"Clients\" must be present in "
                   "JSON file\n");
        exit(1);
    } else if (tddConfCl.empty() == true) {
        internal_measurement_ = tddConf.value("internal_measurement", false);
        if (!internal_measurement_) {
            MLPD_ERROR(
                "Both \"BaseStations\" and \"Clients\" must be present in "
                "JSON file\n");
            exit(1);
        }
    } else {
        ss << "  BaseStations: " << tddConf << "\n\n";
        ss << "  Clients: " << tddConfCl << "\n" << std::endl;
        MLPD_INFO("\nInput config:\n\n%s", ss.str().c_str());
        ss.str(std::string());
        ss.clear();
    }
    bs_present_ = !client_only;
    client_present_ = !bs_only && !internal_measurement_;

    static const int kMaxTxGainBS = 81;
    // common (BaseStation config overrides these)
    freq_ = tddConf.value("frequency", 2.5e9);
    rate_ = tddConf.value("rate", 5e6);
    nco_ = tddConf.value("nco_frequency", 0.75 * rate_);
    symbol_per_slot_ = tddConf.value("ofdm_symbol_per_slot", 1);
    fft_size_ = tddConf.value("fft_size", 0);
    cp_size_ = tddConf.value("cp_size", 0);
    prefix_ = tddConf.value("prefix", 0);
    postfix_ = tddConf.value("postfix", 0);
    symbol_data_subcarrier_num_
        = tddConf.value("ofdm_data_subcarrier_num", fft_size_);
    tx_scale_ = tddConf.value("tx_scale", 0.5);
    pilot_seq_ = tddConf.value("pilot_seq", "lts");
    data_mod_ = tddConf.value("modulation", "QPSK");
    bs_channel_ = tddConf.value("channel", "A");
    if ((bs_channel_ != "A") && (bs_channel_ != "B") && (bs_channel_ != "AB")) {
        throw std::invalid_argument(
            "error channel config: not any of A/B/AB!\n");
    }
    single_gain_ = tddConf.value("single_gain", true);

    if (tddConf.value("txgainA", 20) > kMaxTxGainBS) {
        std::string msg
            = "ERROR: BaseStation ChanA - Maximum TX gain value is ";
        msg += std::to_string(kMaxTxGainBS);
        throw std::invalid_argument(msg);
    } else {
        tx_gain_.push_back(tddConf.value("txgainA", 20));
    }

    if (tddConf.value("txgainB", 20) > kMaxTxGainBS) {
        std::string msg
            = "ERROR: BaseStation ChanB - Maximum TX gain value is ";
        msg += std::to_string(kMaxTxGainBS);
        throw std::invalid_argument(msg);
    } else {
        tx_gain_.push_back(tddConf.value("txgainB", 20));
    }

    rx_gain_.push_back(tddConf.value("rxgainA", 20));
    rx_gain_.push_back(tddConf.value("rxgainB", 20));
    cal_tx_gain_.push_back(tddConf.value("calTxGainA", 10));
    cal_tx_gain_.push_back(tddConf.value("calTxGainB", 10));
    tx_gain_.shrink_to_fit();
    rx_gain_.shrink_to_fit();
    cal_tx_gain_.shrink_to_fit();

    sample_cal_en_ = tddConf.value("sample_calibrate", false);
    imbalance_cal_en_ = tddConf.value("imbalance_calibrate", false);
    beam_sweep_ = tddConf.value("beamsweep", false);
    beacon_ant_ = tddConf.value("beacon_antenna", 0);
    max_frame_ = tddConf.value("max_frame", 0);

    // Load serials file (loads hub, sdr, and rrh serials)
    std::string serials_str;
    auto serials_file_ = tddConf.value("serials_file", "./files/topology.json");
    Utils::loadTDDConfig(serials_file_, serials_str);
    const auto j_serials = json::parse(serials_str, nullptr, true, true);
    json serials_conf;

    num_cells_ = tddConf.value("cells", 1);
    MLPD_TRACE("Number cells: %zu\n", num_cells_);
    hub_ids_.resize(num_cells_);
    bs_sdr_ids_.resize(num_cells_);
    calib_ids_.resize(num_cells_);
    n_bs_sdrs_.resize(num_cells_);
    n_bs_antennas_.resize(num_cells_);
    num_bs_sdrs_all_ = 0;

    /* Used for internal measurements. If internal_measurement is enabled,
       the default is to use a reference node for reciprocity calibration.
       Users have the option of "disabling" the reference node to get a full
       matrix (send pilot from all base station boards and receive on all
       base station boards). The guard interval multiplier extends the number
       of G's in a schedule (between pilots).
     */
    internal_measurement_ = tddConf.value("internal_measurement", false);
    ref_node_enable_ = tddConf.value("reference_node_enable", true);
    guard_mult_ = tddConf.value("meas_guard_interval_mult", 1);
    for (size_t i = 0; i < num_cells_; i++) {
        std::string cell_str = "Cell" + std::to_string(i);
        ss << j_serials[0].value(cell_str, serials_conf);
        serials_conf = json::parse(ss);
        ss.str(std::string());
        ss.clear();

        hub_ids_.at(i) = serials_conf.value("hub", "");
        auto sdr_serials = serials_conf.value("sdr", json::array());
        bs_sdr_ids_.at(i).assign(sdr_serials.begin(), sdr_serials.end());

        // Append calibration node
        if (internal_measurement_ && ref_node_enable_) {
            calib_ids_.at(i) = serials_conf.value("reference node", "");
            if (calib_ids_.at(i).empty()) {
                MLPD_ERROR("No calibration node ID found in topology file!\n");
                exit(1);
            }
            std::cout << "Calibration Node: " << calib_ids_.at(i) << std::endl;
            bs_sdr_ids_.at(i).push_back(calib_ids_.at(i));
        }

        n_bs_sdrs_.at(i) = bs_sdr_ids_.at(i).size();
        n_bs_antennas_.at(i) = bs_channel_.length() * n_bs_sdrs_.at(i);
        num_bs_sdrs_all_ += bs_sdr_ids_.at(i).size();
        MLPD_TRACE("Loading devices - cell %zu, sdrs %zu, antennas: %zu, "
                   "total bs srds: %zu\n",
            i, n_bs_sdrs_.at(i), n_bs_antennas_.at(i), num_bs_sdrs_all_);
    }

    // Print Topology
    std::cout << "Topology: " << std::endl;
    for (size_t i = 0; i < bs_sdr_ids_.size(); i++) {
        std::cout << "Cell" + std::to_string(i) + " Hub:" <<hub_ids_.at(i) << std::endl;
        for (size_t j = 0; j < bs_sdr_ids_.at(i).size(); j++) {
            std::cout << " \t- " << bs_sdr_ids_.at(i).at(j) << std::endl;
        }
    }

    // Array with cummulative sum of SDRs in cells
    n_bs_sdrs_agg_.resize(num_cells_ + 1);
    n_bs_sdrs_agg_.at(0) = 0; //n_bs_sdrs_[0];
    for (size_t i = 0; i < num_cells_; i++) {
        n_bs_sdrs_agg_.at(i + 1) = n_bs_sdrs_agg_.at(i) + n_bs_sdrs_.at(i);
    }

    // Schedule for internal measurements
    if (internal_measurement_ == true) {
        calib_frames_.resize(num_cells_);
        for (size_t c = 0; c < num_cells_; c++) {
            cal_ref_sdr_id_ = n_bs_sdrs_[c] - 1;
            calib_frames_[c].resize(n_bs_sdrs_[c]);
            size_t num_channels = bs_channel_.size();
            size_t frame_length
                = num_channels * n_bs_sdrs_[c] * guard_mult_;

            for (size_t i = 0; i < n_bs_sdrs_[c]; i++) {
                calib_frames_[c][i] = std::string(frame_length, 'G');
            }
            for (size_t i = 0; i < n_bs_sdrs_[c]; i++) {
                for (size_t ch = 0; ch < num_channels; ch++) {
                        calib_frames_[c][i].replace(
                            guard_mult_ * i * num_channels + ch, 1, "P");
                }
                for (size_t k = 0; k < n_bs_sdrs_[c]; k++) {
                    if (i != k) {
                        for (size_t ch = 0; ch < num_channels; ch++) {
                            calib_frames_[c][k].replace(
                                guard_mult_ * i * num_channels + ch, 1, "R");
                        }
                    }
                }
            }
#if DEBUG_PRINT
            for (auto i = calib_frames_[c].begin(); i != calib_frames_[c].end(); ++i)
                std::cout << *i << ' ' << std::endl;
#endif
        }
        slot_per_frame_ = calib_frames_.at(0).size();
        if (ref_node_enable_ == true) {
            pilot_slot_per_frame_ = 2; // Two pilots (up/down)
        } else {
            pilot_slot_per_frame_ = 2 * n_bs_sdrs_[0]; // Two pilots per board (up/down)
        }
        noise_slot_per_frame_ = 0;
        ul_slot_per_frame_ = 0;
        dl_slot_per_frame_ = 0;
    } else {
        auto jBsFrames = tddConf.value("frame_schedule", json::array());
        frames_.assign(jBsFrames.begin(), jBsFrames.end());
        assert(frames_.size() == num_cells_);
        pilot_slots_ = Utils::loadSlots(frames_, 'P');
        noise_slots_ = Utils::loadSlots(frames_, 'N');
        ul_slots_ = Utils::loadSlots(frames_, 'U');
        dl_slots_ = Utils::loadSlots(frames_, 'D');
        slot_per_frame_ = frames_.at(0).size();
        pilot_slot_per_frame_ = pilot_slots_.at(0).size();
        noise_slot_per_frame_ = noise_slots_.at(0).size();
        ul_slot_per_frame_ = ul_slots_.at(0).size();
        dl_slot_per_frame_ = dl_slots_.at(0).size();
        // read commons from client json config
        if (client_present_ == false) {
            num_cl_sdrs_ = num_cl_antennas_
                = std::count(frames_.at(0).begin(), frames_.at(0).end(), 'P');
        }
    }
    //}

    // Clients
    if (internal_measurement_ == false) {
        // read commons from Client json config
        if (tddConf.find("frequency") == tddConf.end())
            freq_ = tddConfCl.value("frequency", 2.5e9);
        if (tddConf.find("rate") == tddConf.end())
            rate_ = tddConfCl.value("rate", 5e6);
        if (tddConf.find("nco_frequency") == tddConf.end())
            nco_ = tddConfCl.value("nco_frequency", 0.75 * rate_);
        if (tddConf.find("ofdm_symbol_per_slot") == tddConf.end())
            symbol_per_slot_ = tddConfCl.value("ofdm_symbol_per_slot", 1);
        if (tddConf.find("fft_size") == tddConf.end())
            fft_size_ = tddConfCl.value("fft_size", 0);
        if (tddConf.find("cp_size") == tddConf.end())
            cp_size_ = tddConfCl.value("cp_size", 0);
        if (tddConf.find("prefix") == tddConf.end())
            prefix_ = tddConfCl.value("prefix", 0);
        if (tddConf.find("postfix") == tddConf.end())
            postfix_ = tddConfCl.value("postfix", 0);
        if (tddConf.find("ofdm_data_subcarrier_num") == tddConf.end())
            symbol_data_subcarrier_num_
                = tddConfCl.value("ofdm_data_subcarrier_num", fft_size_);
        if (tddConf.find("tx_scale") == tddConf.end())
            tx_scale_ = tddConfCl.value("tx_scale", 0.5);
        if (tddConf.find("pilot_seq") == tddConf.end())
            pilot_seq_ = tddConfCl.value("pilot_seq", "lts");
        if (tddConf.find("single_gain") == tddConf.end())
            single_gain_ = tddConfCl.value("single_gain", true);
        if (tddConf.find("modulation") == tddConf.end())
            data_mod_ = tddConfCl.value("modulation", "QPSK");

        auto jClSdrs = tddConfCl.value("sdr_id", json::array());
        num_cl_sdrs_ = jClSdrs.size();
        cl_sdr_ids_.assign(jClSdrs.begin(), jClSdrs.end());
        cl_channel_ = tddConfCl.value("channel", "A");
        if (cl_channel_ != "A" && cl_channel_ != "B" && cl_channel_ != "AB")
            throw std::invalid_argument(
                "error channel config: not any of A/B/AB!\n");
        cl_sdr_ch_ = (cl_channel_ == "AB") ? 2 : 1;
        num_cl_antennas_ = num_cl_sdrs_ * cl_sdr_ch_;
        cl_agc_en_ = tddConfCl.value("agc_en", false);
        cl_agc_gain_init_ = tddConfCl.value("agc_gain_init", 70); // 0 to 108
        frame_mode_ = tddConfCl.value("frame_mode", "continuous_resync");
        hw_framer_ = tddConfCl.value("hw_framer", false);
        tx_advance_ = tddConfCl.value("tx_advance", 250); // 250
        ul_data_frame_num_ = tddConfCl.value("ul_data_frame_num", 1);

        // Help verify whether gain exceeds max value
        struct compare {
            const int key_;
            compare(int const& i)
                : key_(i)
            {
            }
            bool operator()(int const& i) { return (i > key_); }
        };
        cl_txgain_vec_.resize(2);
        cl_rxgain_vec_.resize(2);
        auto jClTxgainA_vec = tddConfCl.value("txgainA", json::array());
        cl_txgain_vec_.at(0).assign(
            jClTxgainA_vec.begin(), jClTxgainA_vec.end());
        auto jClRxgainA_vec = tddConfCl.value("rxgainA", json::array());
        cl_rxgain_vec_.at(0).assign(
            jClRxgainA_vec.begin(), jClRxgainA_vec.end());
        auto jClTxgainB_vec = tddConfCl.value("txgainB", json::array());
        cl_txgain_vec_.at(1).assign(
            jClTxgainB_vec.begin(), jClTxgainB_vec.end());
        auto jClRxgainB_vec = tddConfCl.value("rxgainB", json::array());
        cl_rxgain_vec_.at(1).assign(
            jClRxgainB_vec.begin(), jClRxgainB_vec.end());

        max_tx_gain_ue_ = tddConfCl.value("maxTxGainUE", 81);
        compare find_guilty(max_tx_gain_ue_);
        if (std::any_of(cl_txgain_vec_.at(0).begin(),
                cl_txgain_vec_.at(0).end(), find_guilty)) {
            std::string msg = "ERROR: UE ChanA - Maximum TX gain value is ";
            msg += std::to_string(max_tx_gain_ue_);
            throw std::invalid_argument(msg);
        }
        if (std::any_of(cl_txgain_vec_.at(1).begin(),
                cl_txgain_vec_.at(1).end(), find_guilty)) {
            std::string msg = "ERROR: UE ChanB - Maximum TX gain value is ";
            msg += std::to_string(max_tx_gain_ue_);
            throw std::invalid_argument(msg);
        }

        auto jClFrames = tddConfCl.value("frame_schedule", json::array());
        assert(jClSdrs.size() == jClFrames.size());
        cl_frames_.assign(jClFrames.begin(), jClFrames.end());
        cl_pilot_slots_ = Utils::loadSlots(cl_frames_, 'P');
        cl_ul_slots_ = Utils::loadSlots(cl_frames_, 'U');
        cl_dl_slots_ = Utils::loadSlots(cl_frames_, 'D');
    }

    bw_filter_ = rate_ + 2 * nco_;
    radio_rf_freq_ = freq_ - nco_;
    ofdm_symbol_size_ = fft_size_ + cp_size_;
    slot_samp_size_ = symbol_per_slot_ * ofdm_symbol_size_;
    samps_per_slot_ = slot_samp_size_ + prefix_ + postfix_;
    assert(internal_measurement_ || slot_per_frame_ == cl_frames_.at(0).size());

    ul_data_slot_present_ = (internal_measurement_ == false)
        && ((bs_present_ && (ul_slots_.at(0).empty() == false))
            || (client_present_ && !cl_ul_slots_.at(0).empty()));

    std::vector<std::complex<int16_t>> prefix_zpad(prefix_, 0);
    std::vector<std::complex<int16_t>> postfix_zpad(postfix_, 0);

    // compose Beacon slot:
    // STS Sequence (for AGC) + GOLD Sequence (for Sync)
    // 15reps of STS(16) + 2reps of gold_ifft(128)
    srand(time(NULL));
    const int seq_len = 128;
    std::vector<std::vector<float>> gold_ifft
        = CommsLib::getSequence(CommsLib::GOLD_IFFT);
    auto gold_ifft_ci16 = Utils::float_to_cint16(gold_ifft);
    gold_cf32_.clear();
    for (size_t i = 0; i < seq_len; i++) {
        gold_cf32_.push_back(
            std::complex<float>(gold_ifft[0][i], gold_ifft[1][i]));
    }

    std::vector<std::vector<float>> sts_seq
        = CommsLib::getSequence(CommsLib::STS_SEQ);
    auto sts_seq_ci16 = Utils::float_to_cint16(sts_seq);

    // Populate STS (stsReps repetitions)
    int stsReps = 15;
    for (int i = 0; i < stsReps; i++) {
        beacon_ci16_.insert(
            beacon_ci16_.end(), sts_seq_ci16.begin(), sts_seq_ci16.end());
    }

    // Populate gold sequence (two reps, 128 each)
    int goldReps = 2;
    for (int i = 0; i < goldReps; i++) {
        beacon_ci16_.insert(
            beacon_ci16_.end(), gold_ifft_ci16.begin(), gold_ifft_ci16.end());
    }

    beacon_size_ = beacon_ci16_.size();

    if (samps_per_slot_ < (beacon_size_ + prefix_ + postfix_)) {
        std::string msg = "Minimum supported slot_samp_size is ";
        msg += std::to_string(beacon_size_);
        throw std::invalid_argument(msg);
    }

    beacon_ = Utils::cint16_to_uint32(beacon_ci16_, false, "QI");
    coeffs_ = Utils::cint16_to_uint32(gold_ifft_ci16, true, "QI");

    std::vector<std::complex<int16_t>> post_beacon_zpad(
        slot_samp_size_ - beacon_size_, 0);
    beacon_ci16_.insert(
        beacon_ci16_.begin(), prefix_zpad.begin(), prefix_zpad.end());
    beacon_ci16_.insert(
        beacon_ci16_.end(), post_beacon_zpad.begin(), post_beacon_zpad.end());
    beacon_ci16_.insert(
        beacon_ci16_.end(), postfix_zpad.begin(), postfix_zpad.end());

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
    } else if (pilot_seq_ == "zadoff-chu") {
        pilot_sym_f_ = CommsLib::getSequence(
            CommsLib::LTE_ZADOFF_CHU_F, symbol_data_subcarrier_num_);
        pilot_sym_t_ = CommsLib::getSequence(
            CommsLib::LTE_ZADOFF_CHU, symbol_data_subcarrier_num_);
    } else
        std::cout
            << pilot_seq_
            << " is not supported! Choose either LTS (64-fft) or zaddof-chu."
            << std::endl;

    auto iq_ci16 = Utils::float_to_cint16(pilot_sym_t_);
    iq_ci16.insert(iq_ci16.begin(), iq_ci16.end() - cp_size_, iq_ci16.end());

    pilot_ci16_.clear();
    pilot_ci16_.insert(
        pilot_ci16_.begin(), prefix_zpad.begin(), prefix_zpad.end());
    for (size_t i = 0; i < symbol_per_slot_; i++)
        pilot_ci16_.insert(pilot_ci16_.end(), iq_ci16.begin(), iq_ci16.end());
    pilot_ci16_.insert(
        pilot_ci16_.end(), postfix_zpad.begin(), postfix_zpad.end());

    pilot_ = Utils::cint16_to_uint32(pilot_ci16_, false, "QI");
    pilot_cf32_ = Utils::uint32tocfloat(pilot_, "QI");
    size_t remain_size = kFpgaTxRamSize
        - pilot_.size(); // 4096 is the size of TX_RAM in the FPGA
    for (size_t j = 0; j < remain_size; j++)
        pilot_.push_back(0);
#if DEBUG_PRINT
    for (size_t j = 0; j < pilot_ci16_.size(); j++) {
        std::cout << "Pilot[" << j << "]: \t " << pilot_ci16_.at(j)
                  << std::endl;
    }
#endif

    data_ind_ = CommsLib::getDataSc(fft_size_, symbol_data_subcarrier_num_);
    pilot_sc_
        = CommsLib::getPilotScValue(fft_size_, symbol_data_subcarrier_num_);
    pilot_sc_ind_
        = CommsLib::getPilotScIndex(fft_size_, symbol_data_subcarrier_num_);
    if (bs_present_ == true) {
        // set trace file path
        time_t now = time(0);
        tm* ltm = localtime(&now);
        int cell_num = num_cells_;
        size_t ant_num = getTotNumAntennas();
        std::string filename;
        if (internal_measurement_) {
            filename = directory + "/trace-internal-meas-"
                + std::to_string(1900 + ltm->tm_year) + "-"
                + std::to_string(1 + ltm->tm_mon) + "-"
                + std::to_string(ltm->tm_mday) + "-"
                + std::to_string(ltm->tm_hour) + "-"
                + std::to_string(ltm->tm_min) + "-"
                + std::to_string(ltm->tm_sec) + "_" + std::to_string(cell_num)
                + "x" + std::to_string(ant_num) + ".hdf5";
        } else {
            std::string ul_present_str
                = (ul_data_slot_present_ ? "uplink-" : "");
            filename = directory + "/trace-" + ul_present_str
                + std::to_string(1900 + ltm->tm_year) + "-"
                + std::to_string(1 + ltm->tm_mon) + "-"
                + std::to_string(ltm->tm_mday) + "-"
                + std::to_string(ltm->tm_hour) + "-"
                + std::to_string(ltm->tm_min) + "-"
                + std::to_string(ltm->tm_sec) + "_" + std::to_string(cell_num)
                + "x" + std::to_string(ant_num) + "x"
                + std::to_string(num_cl_antennas_) + ".hdf5";
        }
        trace_file_ = tddConf.value("trace_file", filename);
    }

    // Multi-threading settings
    unsigned num_cores = this->getCoreCount();
    MLPD_INFO("Cores found %u ... \n", num_cores);
    core_alloc_ = num_cores > RX_THREAD_NUM;
    if ((bs_present_ == true)
        && (pilot_slot_per_frame_ + ul_slot_per_frame_ > 0)) {
        task_thread_num_ = tddConf.value("task_thread", TASK_THREAD_NUM);
        rx_thread_num_ = (num_cores >= (2 * RX_THREAD_NUM))
            ? std::min(RX_THREAD_NUM, static_cast<int>(num_bs_sdrs_all_))
            : 1;
        if (internal_measurement_ == true) {        // OBCH how many threads if reading from all boards???
            rx_thread_num_ = 2;
        }
        if ((client_present_ == true)
            && (num_cores
                < (1 + task_thread_num_ + rx_thread_num_ + num_cl_sdrs_))) {
            core_alloc_ = false;
        }
    } else {
        rx_thread_num_ = 0;
        task_thread_num_ = 0;
        if (client_present_ && num_cores <= 1 + num_cl_sdrs_)
            core_alloc_ = false;
    }
    if ((bs_present_ == true) && (core_alloc_ == true)) {
        MLPD_INFO(
            "Allocating %d cores to receive threads ... \n", rx_thread_num_);
        MLPD_INFO(
            "Allocating %d cores to record threads ... \n", task_thread_num_);
    }

    if ((client_present_ == true) && (core_alloc_ == true)) {
        MLPD_INFO(
            "Allocating %zu cores to client threads ... \n", num_cl_sdrs_);
    }
    running_.store(true);
    MLPD_INFO("Configuration file was successfully parsed!\n");
}

void Config::loadULData(const std::string& directory)
{
    // compose data slot
    if (ul_data_slot_present_) {
        std::vector<std::complex<float>> prefix_zpad_t(prefix_, 0);
        std::vector<std::complex<float>> postfix_zpad_t(postfix_, 0);
        txdata_time_dom_.resize(num_cl_antennas_);
        txdata_freq_dom_.resize(num_cl_antennas_);
        // For now, we're reading one frame worth of data
        for (size_t i = 0; i < num_cl_sdrs_; i++) {
            std::string filename_tag = data_mod_ + "_"
                + std::to_string(symbol_data_subcarrier_num_) + "_"
                + std::to_string(fft_size_) + "_"
                + std::to_string(symbol_per_slot_) + "_"
                + std::to_string(cl_ul_slots_[i].size()) + "_"
                + std::to_string(ul_data_frame_num_) + "_" + cl_channel_ + "_"
                + std::to_string(i) + ".bin";

            std::string filename_ul_data_f
                = directory + "/ul_data_f_" + filename_tag;
            std::printf(
                "Loading UL frequency-domain data for radio %zu to %s\n", i,
                filename_ul_data_f.c_str());
            tx_fd_data_files_.push_back("ul_data_f_" + filename_tag);
            FILE* fp_tx_f = std::fopen(filename_ul_data_f.c_str(), "rb");
            if (!fp_tx_f) {
                throw std::runtime_error(
                    filename_ul_data_f + std::string(" not found!"));
            }

            std::string filename_ul_data_t
                = directory + "/ul_data_t_" + filename_tag;
            std::printf("Loading UL time-domain data for radio %zu to %s\n", i,
                filename_ul_data_t.c_str());
            tx_td_data_files_.push_back(filename_ul_data_t);
            FILE* fp_tx_t = std::fopen(filename_ul_data_t.c_str(), "rb");
            if (!fp_tx_t) {
                throw std::runtime_error(
                    filename_ul_data_t + std::string(" not found!"));
            }

            // Frame * UL Slots * Channel * Samples
            for (size_t u = 0; u < cl_ul_slots_[i].size(); u++) {
                for (size_t h = 0; h < cl_sdr_ch_; h++) {
                    size_t ant_i = i * cl_sdr_ch_ + h;

                    std::vector<std::complex<float>> data_freq_dom(
                        fft_size_ * symbol_per_slot_);
                    size_t read_num
                        = std::fread(data_freq_dom.data(), 2 * sizeof(float),
                            fft_size_ * symbol_per_slot_, fp_tx_f);
                    if (read_num != (fft_size_ * symbol_per_slot_)) {
                        MLPD_WARN(
                            "BAD Read of Uplink Freq-Domain Data: %zu/%zu\n",
                            read_num, fft_size_ * symbol_per_slot_);
                    }
                    txdata_freq_dom_[ant_i].insert(
                        txdata_freq_dom_[ant_i].end(), data_freq_dom.begin(),
                        data_freq_dom.end());

                    std::vector<std::complex<float>> data_time_dom(
                        samps_per_slot_);
                    read_num = std::fread(data_time_dom.data(),
                        2 * sizeof(float), samps_per_slot_, fp_tx_t);
                    if (read_num != samps_per_slot_) {
                        MLPD_WARN(
                            "BAD Read of Uplink Time-Domain Data: %zu/%zu\n",
                            read_num, samps_per_slot_);
                    }
                    txdata_time_dom_[ant_i].insert(
                        txdata_time_dom_[ant_i].end(), data_time_dom.begin(),
                        data_time_dom.end());
                }
            }
            std::fclose(fp_tx_f);
            std::fclose(fp_tx_t);
        }
    }
}

size_t Config::getNumAntennas()
{
    size_t ret;
    if (this->bs_present_ == false) {
        ret = 1;
    } else {
        ret = (n_bs_sdrs_.at(0) * bs_channel_.length());
    }
    return ret;
}

size_t Config::getMaxNumAntennas()
{
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
                max_num_sdr--; // exclude the ref sdr
            }
        }
        ret = (max_num_sdr * bs_channel_.length());
    }
    return ret;
}

size_t Config::getTotNumAntennas()
{
    size_t ret;
    /* Total number of antennas across cells */
    if (this->bs_present_ == false) {
        ret = 1;
    } else {
        size_t totNumSdr = 0;
        for (size_t i = 0; i < num_cells_; i++) {
            totNumSdr += n_bs_sdrs_.at(i);
            if (internal_measurement_ == true && ref_node_enable_ == true)
                totNumSdr--;
        }
        ret = totNumSdr * bs_channel_.length();
    }
    return ret;
}

Config::~Config() {}

int Config::getClientId(int frame_id, int slot_id)
{
    if (internal_measurement_)
        return slot_id;
    std::vector<size_t>::iterator it;
    int fid = frame_id % frames_.size();
    it = find(
        pilot_slots_.at(fid).begin(), pilot_slots_.at(fid).end(), slot_id);
    if (it != pilot_slots_.at(fid).end()) {
        return (it - pilot_slots_.at(fid).begin());
    }
    return -1;
}

int Config::getNoiseSlotIndex(int frame_id, int slot_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % frames_.size();
    it = find(
        noise_slots_.at(fid).begin(), noise_slots_.at(fid).end(), slot_id);
    if (it != noise_slots_.at(fid).end())
        return (it - noise_slots_.at(fid).begin());
    return -1;
}

int Config::getUlSlotIndex(int frame_id, int slot_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % frames_.size();
    it = find(ul_slots_.at(fid).begin(), ul_slots_.at(fid).end(), slot_id);
    if (it != ul_slots_.at(fid).end()) {
        return (it - ul_slots_.at(fid).begin());
    }
    return -1;
}

int Config::getDlSlotIndex(int frame_id, int slot_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % frames_.size();
    it = find(dl_slots_.at(fid).begin(), dl_slots_.at(fid).end(), slot_id);
    if (it != dl_slots_.at(fid).end())
        return (it - dl_slots_.at(fid).begin());
    return -1;
}

bool Config::isPilot(int frame_id, int slot_id)
{
    try {
        return frames_[frame_id % frames_.size()].at(slot_id) == 'P';
    } catch (const std::out_of_range&) {
        return false;
    }
}

bool Config::isNoise(int frame_id, int slot_id)
{
    try {
        return frames_[frame_id % frames_.size()].at(slot_id) == 'N';
    } catch (const std::out_of_range&) {
        return false;
    }
}

bool Config::isData(int frame_id, int slot_id)
{
    try {
        return frames_[frame_id % frames_.size()].at(slot_id) == 'U';
    } catch (const std::out_of_range&) {
        return false;
    }
}

bool Config::isRx(int frame_id, int slot_id)
{
    // Generic RX (assuming cell == 0)
    try {
        return calib_frames_[0][frame_id % calib_frames_[0].size()].at(slot_id) == 'R' || calib_frames_[0][frame_id % calib_frames_[0].size()].at(slot_id) == 'P';
    } catch (const std::out_of_range&) {
        return false;
    }
}

unsigned Config::getCoreCount()
{
    unsigned n_cores = std::thread::hardware_concurrency();
#if DEBUG_PRINT
    std::cout << "number of CPU cores " << std::to_string(n_cores) << std::endl;
#endif
    return n_cores;
}

extern "C" {
__attribute__((visibility("default"))) Config* Config_new(
    char* filename, char* storepath, bool bs_only, bool client_only)
{

    Config* cfg = new Config(filename, storepath, bs_only, client_only);
    return cfg;
}
}
