/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
---------------------------------------------------------------------
 Reads configuration parameters from file 
---------------------------------------------------------------------
*/

#include "include/config.h"
#include "include/comms-lib.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"

static size_t kFpgaTxRamSize = 4096;
static size_t kMaxSupportedFFTSize = 2048;
static size_t kMinSupportedFFTSize = 64;
static size_t kMaxSupportedCPSize = 128;

Config::Config(const std::string& jsonfile)
{
    std::string conf;
    Utils::loadTDDConfig(jsonfile, conf);
    const auto jConf = json::parse(conf);
    std::stringstream ss;
    json tddConf;

    ss << jConf.value("BaseStations", tddConf);
    tddConf = json::parse(ss);
    bs_present_ = (tddConf.empty() == false);
    if (bs_present_ == true) {
        ss.str(std::string());
        ss.clear();
        ss << tddConf << std::endl << std::endl;
        MLPD_INFO("Base Stations present: %s", ss.str().c_str());
    }
    ss.str(std::string());
    ss.clear();

    json tddConfCl;
    ss << jConf.value("Clients", tddConfCl);
    tddConfCl = json::parse(ss);
    client_present_ = (tddConfCl.empty() == false);
    if (client_present_ == true) {
        ss.str(std::string());
        ss.clear();
        ss << tddConfCl << std::endl << std::endl;
        MLPD_INFO("Clients present: %s", ss.str().c_str());
    }
    ss.str(std::string());
    ss.clear();

    // common (BaseStation config overrides these)
    if (bs_present_ == true) {
        freq_ = tddConf.value("frequency", 2.5e9);
        rate_ = tddConf.value("rate", 5e6);
        nco_ = tddConf.value("nco_frequency", 0.75 * rate_);
        bw_filter_ = rate_ + 2 * nco_;
        radio_rf_freq_ = freq_ - nco_;
        symbol_per_subframe_ = tddConf.value("ofdm_symbol_per_subframe", 1);
        fft_size_ = tddConf.value("fft_size", 0);
        cp_size_ = tddConf.value("cp_size", 0);
        prefix_ = tddConf.value("prefix", 0);
        postfix_ = tddConf.value("postfix", 0);
        ofdm_symbol_size_ = fft_size_ + cp_size_;
        subframe_size_ = symbol_per_subframe_ * ofdm_symbol_size_;
        samps_per_symbol_ = subframe_size_ + prefix_ + postfix_;
        symbol_data_subcarrier_num_
            = tddConf.value("ofdm_data_subcarrier_num", fft_size_);
        tx_scale_ = tddConf.value("tx_scale", 0.5);
        beacon_seq_ = tddConf.value("beacon_seq", "gold_ifft");
        pilot_seq_ = tddConf.value("pilot_seq", "lts");
        data_mod_ = tddConf.value("modulation", "QPSK");

        // BS
        if (kUseUHD == false) {
            hub_file_ = tddConf.value("hub_id", "hub_serials.txt");
        }
        auto sdr_id_files = tddConf.value("sdr_id", json::array());
        num_cells_ = sdr_id_files.size();
        bs_sdr_file_.assign(sdr_id_files.begin(), sdr_id_files.end());
        bs_channel_ = tddConf.value("channel", "A");
        if ((bs_channel_ != "A") && (bs_channel_ != "B")
            && (bs_channel_ != "AB")) {
            throw std::invalid_argument(
                "error channel config: not any of A/B/AB!\n");
        }
        single_gain_ = tddConf.value("single_gain", true);
        tx_gain_.push_back(tddConf.value("txgainA", 20));
        rx_gain_.push_back(tddConf.value("rxgainA", 20));
        tx_gain_.push_back(tddConf.value("txgainB", 20));
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

        MLPD_TRACE("Number cells: %zu\n", num_cells_);
        bs_sdr_ids_.resize(num_cells_);
        n_bs_sdrs_.resize(num_cells_);
        n_bs_antennas_.resize(num_cells_);
        num_bs_sdrs_all_ = 0;
        for (size_t i = 0u; i < num_cells_; i++) {
            Utils::loadDevices(bs_sdr_file_.at(i), bs_sdr_ids_.at(i));
            n_bs_sdrs_.at(i) = bs_sdr_ids_.at(i).size();
            n_bs_antennas_.at(i) = bs_channel_.length() * n_bs_sdrs_.at(i);
            num_bs_sdrs_all_ += bs_sdr_ids_.at(i).size();
            MLPD_TRACE("Loading devices - cell %zu, sdrs %zu, antennas: %zu, "
                       "total bs srds: %zu\n",
                i, n_bs_sdrs_.at(i), n_bs_antennas_.at(i), num_bs_sdrs_all_);
        }

        // Array with cummulative sum of SDRs in cells
        n_bs_sdrs_agg_.resize(num_cells_ + 1);
        n_bs_sdrs_agg_.at(0) = 0; //n_bs_sdrs_[0];
        for (size_t i = 0; i < num_cells_; i++) {
            n_bs_sdrs_agg_.at(i + 1) = n_bs_sdrs_agg_.at(i) + n_bs_sdrs_.at(i);
        }

        if (kUseUHD == false)
            Utils::loadDevices(hub_file_, hub_ids_);
        reciprocal_calib_ = tddConf.value("reciprocal_calibration", false);
        cal_ref_sdr_id_ = tddConf.value("ref_sdr_index", num_bs_sdrs_all_ - 1);

        if (reciprocal_calib_ == true) {
            calib_frames_.resize(num_cells_);
            for (size_t c = 0; c < num_cells_; c++) {
                calib_frames_[c].resize(n_bs_sdrs_[c]);
                size_t num_channels = bs_channel_.size();
                size_t frame_length
                    = num_channels * n_bs_sdrs_[c] - (num_channels - 1);
                calib_frames_[c][cal_ref_sdr_id_]
                    = std::string(frame_length, 'G');
                calib_frames_[c][cal_ref_sdr_id_].replace(
                    num_channels * cal_ref_sdr_id_, 1, "P");
                for (size_t i = 0; i < n_bs_sdrs_[c]; i++) {
                    if (i != cal_ref_sdr_id_) {
                        calib_frames_[c][i] = std::string(frame_length, 'G');
                        for (size_t ch = 0; ch < num_channels; ch++) {
                            calib_frames_[c][i].replace(
                                i * num_channels + ch, 1, "P");
                            calib_frames_[c][cal_ref_sdr_id_].replace(
                                num_channels * i + ch, 1, "R");
                        }
                        calib_frames_[c][i].replace(
                            num_channels * cal_ref_sdr_id_, 1, "R");
                    }
                }
            }
            symbols_per_frame_ = calib_frames_.at(0).size();
            pilot_syms_per_frame_ = 2; // up and down reciprocity pilots
            noise_syms_per_frame_ = 0;
            ul_syms_per_frame_ = 0;
            dl_syms_per_frame_ = 0;
        } else {
            auto jBsFrames = tddConf.value("frame_schedule", json::array());
            frames_.assign(jBsFrames.begin(), jBsFrames.end());
            assert(frames_.size() == num_cells_);
            pilot_symbols_ = Utils::loadSymbols(frames_, 'P');
            noise_symbols_ = Utils::loadSymbols(frames_, 'N');
            ul_symbols_ = Utils::loadSymbols(frames_, 'U');
            dl_symbols_ = Utils::loadSymbols(frames_, 'D');
            symbols_per_frame_ = frames_.at(0).size();
            pilot_syms_per_frame_ = pilot_symbols_.at(0).size();
            noise_syms_per_frame_ = noise_symbols_.at(0).size();
            ul_syms_per_frame_ = ul_symbols_.at(0).size();
            dl_syms_per_frame_ = dl_symbols_.at(0).size();
            // read commons from client json config
            if (client_present_ == false) {
                num_cl_sdrs_ = num_cl_antennas_ = std::count(
                    frames_.at(0).begin(), frames_.at(0).end(), 'P');
            }
        }
    }

    MLPD_TRACE("Starting clients -- %zu", num_bs_sdrs_all_);

    // Clients
    assert(!(client_present_ == true && reciprocal_calib_ == true));
    if (client_present_ == true) {
        auto jClSdrs = tddConfCl.value("sdr_id", json::array());
        // auto jClSdrs = tddConfCl.value("sdr_ip", json::array());
        num_cl_sdrs_ = jClSdrs.size();
        cl_sdr_ids_.assign(jClSdrs.begin(), jClSdrs.end());
        // cl_sdr_ips.assign(jClSdrs.begin(), jClSdrs.end());
        cl_channel_ = tddConfCl.value("channel", "A");
        if (cl_channel_ != "A" && cl_channel_ != "B" && cl_channel_ != "AB")
            throw std::invalid_argument(
                "error channel config: not any of A/B/AB!\n");
        cl_sdr_ch_ = (cl_channel_ == "AB") ? 2 : 1;
        num_cl_antennas_ = num_cl_sdrs_ * cl_sdr_ch_;
        cl_agc_en_ = tddConfCl.value("agc_en", false);
        cl_agc_gain_init_ = tddConfCl.value("agc_gain_init", 70); // 0 to 108
        frame_mode_ = tddConfCl.value("frame_mode", "continuous_resync");
        hw_framer_ = tddConfCl.value("hw_framer", true);
        tx_advance_ = tddConfCl.value("tx_advance", 250); // 250

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

        auto jClFrames = tddConfCl.value("frame_schedule", json::array());
        assert(jClSdrs.size() == jClFrames.size());
        cl_frames_.assign(jClFrames.begin(), jClFrames.end());
        cl_pilot_symbols_ = Utils::loadSymbols(cl_frames_, 'P');
        cl_ul_symbols_ = Utils::loadSymbols(cl_frames_, 'U');
        cl_dl_symbols_ = Utils::loadSymbols(cl_frames_, 'D');

        // read commons from Client json config
        if (bs_present_ == false) {
            freq_ = tddConfCl.value("frequency", 2.5e9);
            rate_ = tddConfCl.value("rate", 5e6);
            nco_ = tddConfCl.value("nco_frequency", 0.75 * rate_);
            bw_filter_ = rate_ + 2 * nco_;
            radio_rf_freq_ = freq_ - nco_;
            symbol_per_subframe_
                = tddConfCl.value("ofdm_symbol_per_subframe", 1);
            fft_size_ = tddConfCl.value("fft_size", 0);
            cp_size_ = tddConfCl.value("cp_size", 0);
            prefix_ = tddConfCl.value("prefix", 0);
            postfix_ = tddConfCl.value("postfix", 0);
            ofdm_symbol_size_ = fft_size_ + cp_size_;
            subframe_size_ = symbol_per_subframe_ * ofdm_symbol_size_;
            samps_per_symbol_ = subframe_size_ + prefix_ + postfix_;
            tx_scale_ = tddConfCl.value("tx_scale", 0.5);
            beacon_seq_ = tddConfCl.value("beacon_seq", "gold_ifft");
            pilot_seq_ = tddConfCl.value("pilot_seq", "lts");
            symbols_per_frame_ = cl_frames_.at(0).size();
            single_gain_ = tddConfCl.value("single_gain", true);
            data_mod_ = tddConfCl.value("modulation", "QPSK");
        }
    }

    ul_data_sym_present_ = (reciprocal_calib_ == false)
        && ((bs_present_ && (ul_symbols_.at(0).empty() == false))
               || (client_present_ && !cl_ul_symbols_.at(0).empty()));

    std::vector<std::complex<int16_t>> prefix_zpad(prefix_, 0);
    std::vector<std::complex<int16_t>> postfix_zpad(postfix_, 0);

    // compose Beacon subframe:
    // STS Sequence (for AGC) + GOLD Sequence (for Sync)
    // 15reps of STS(16) + 2reps of gold_ifft(128)
    srand(time(NULL));
    const int seqLen = 128;
    std::vector<std::vector<double>> gold_ifft
        = CommsLib::getSequence(CommsLib::GOLD_IFFT);
    std::vector<std::complex<int16_t>> gold_ifft_ci16
        = Utils::double_to_cint16(gold_ifft);
    gold_cf32_.clear();
    for (size_t i = 0; i < seqLen; i++) {
        gold_cf32_.push_back(
            std::complex<float>(gold_ifft[0][i], gold_ifft[1][i]));
    }

    std::vector<std::vector<double>> sts_seq
        = CommsLib::getSequence(CommsLib::STS_SEQ);
    std::vector<std::complex<int16_t>> sts_seq_ci16
        = Utils::double_to_cint16(sts_seq);

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

    if (samps_per_symbol_ < (beacon_size_ + prefix_ + postfix_)) {
        std::string msg = "Minimum supported subframe_size is ";
        msg += std::to_string(beacon_size_);
        throw std::invalid_argument(msg);
    }

    beacon_ = Utils::cint16_to_uint32(beacon_ci16_, false, "QI");
    coeffs_ = Utils::cint16_to_uint32(gold_ifft_ci16, true, "QI");

    std::vector<std::complex<int16_t>> post_beacon_zpad(
        subframe_size_ - beacon_size_, 0);
    beacon_ci16_.insert(
        beacon_ci16_.begin(), prefix_zpad.begin(), prefix_zpad.end());
    beacon_ci16_.insert(
        beacon_ci16_.end(), post_beacon_zpad.begin(), post_beacon_zpad.end());
    beacon_ci16_.insert(
        beacon_ci16_.end(), postfix_zpad.begin(), postfix_zpad.end());

    // compose pilot subframe
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
        pilot_sym_ = CommsLib::getSequence(CommsLib::LTS_SEQ);
    } else if (pilot_seq_ == "zadoff-chu") {
        pilot_sym_ = CommsLib::getSequence(
            CommsLib::LTE_ZADOFF_CHU, symbol_data_subcarrier_num_);
    } else
        std::cout
            << pilot_seq_
            << " is not supported! Choose either LTS (64-fft) or zaddof-chu."
            << std::endl;

    auto iq_ci16 = Utils::double_to_cint16(pilot_sym_);
    iq_ci16.insert(iq_ci16.begin(), iq_ci16.end() - cp_size_, iq_ci16.end());

    pilot_ci16_.clear();
    pilot_ci16_.insert(
        pilot_ci16_.begin(), prefix_zpad.begin(), prefix_zpad.end());
    for (size_t i = 0; i < symbol_per_subframe_; i++)
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

    // compose data subframe
    if (ul_data_sym_present_) {
        int mod_type = data_mod_ == "64QAM"
            ? CommsLib::QAM64
            : (data_mod_ == "16QAM" ? CommsLib::QAM16 : CommsLib::QPSK);
        std::cout << mod_type << std::endl;
        int mod_order = 1 << mod_type;
        std::cout << mod_order << std::endl;

        data_ind_ = CommsLib::getDataSc(fft_size_);
        pilot_sc_ = CommsLib::getPilotSc(fft_size_);
        std::vector<std::complex<float>> prefix_zpad_f(prefix_, 0);
        std::vector<std::complex<float>> postfix_zpad_f(postfix_, 0);
        size_t nDataScs = data_ind_.size();
        for (size_t i = 0; i < num_cl_antennas_; i++) {
            std::vector<std::complex<float>> data_cf;
            std::vector<std::complex<float>> data_freq_dom;
            data_cf.insert(
                data_cf.begin(), prefix_zpad_f.begin(), prefix_zpad_f.end());
            std::vector<std::vector<int>> dataBits;
            dataBits.resize(symbol_per_subframe_);
            for (size_t s = 0; s < symbol_per_subframe_; s++) {
                for (size_t c = 0; c < nDataScs; c++) {
                    dataBits[s].push_back(rand() % mod_order);
                }
                std::vector<std::complex<float>> mod_data
                    = CommsLib::modulate(dataBits[s], mod_type);
#if DEBUG_PRINT
                std::cout << "Modulation output: " << mod_data[0] << " "
                          << mod_data[1] << std::endl;
#endif
                std::vector<std::complex<float>> ofdmSym(fft_size_);
                int sc = 0;
                for (size_t c = 0; c < nDataScs; c++) {
                    sc = data_ind_[c];
                    ofdmSym[sc] = mod_data[c];
                }
#if DEBUG_PRINT
                std::cout << "Data symbol: " << ofdmSym[sc - 2] << " "
                          << ofdmSym[sc - 1] << std::endl;
#endif
                for (size_t c = 0; c < pilot_sc_.at(0).size(); c++) {
                    sc = pilot_sc_.at(0).at(c);
                    ofdmSym[sc] = pilot_sc_.at(1).at(c);
                }
#if DEBUG_PRINT
                std::cout << "Pilot symbol: " << ofdmSym[pilot_sc_.at(0).at(0)]
                          << " " << ofdmSym[pilot_sc_.at(0).at(1)] << std::endl;
#endif
                auto txSym = CommsLib::IFFT(
                    ofdmSym, fft_size_, 1.f / fft_size_, false);
                txSym.insert(txSym.begin(), txSym.end() - cp_size_,
                    txSym.end()); // add CP
#if DEBUG_PRINT
                std::cout << "IFFT output: " << txSym[0] << " " << txSym[64]
                          << std::endl;
#endif
                data_cf.insert(data_cf.end(), txSym.begin(), txSym.end());
                data_freq_dom.insert(
                    data_freq_dom.end(), ofdmSym.begin(), ofdmSym.end());
            }
            data_cf.insert(
                data_cf.end(), postfix_zpad_f.begin(), postfix_zpad_f.end());
            tx_data_.push_back(data_cf);
            txdata_time_dom_.push_back(data_cf);
            txdata_freq_dom_.push_back(data_freq_dom);
        }
#if DEBUG_PRINT
        for (size_t i = 0; i < tx_data_.size(); i++) {
            for (size_t j = 0; j < tx_data_.at(i).size(); j++) {
                std::cout << "Values[" << i << "][" << j << "]: \t "
                          << tx_data_.at(i).at(j) << std::endl;
            }
        }
        for (size_t i = 0; i < txdata_freq_dom_.size(); i++) {
            for (size_t j = 0; j < txdata_freq_dom_.at(i).size(); j++) {
                std::cout << "FREQ DOMAIN Values[" << i << "][" << j << "]: \t "
                          << txdata_freq_dom_.at(i).at(j) << std::endl;
            }
        }
#endif
    }

    if (bs_present_ == true) {
        // set trace file path
        time_t now = time(0);
        tm* ltm = localtime(&now);
        int cell_num = num_cells_;
        size_t ant_num = getTotNumAntennas();
        std::string filename;
        if (reciprocal_calib_) {
            filename = "logs/trace-reciprocal-calib-"
                + std::to_string(1900 + ltm->tm_year) + "-"
                + std::to_string(1 + ltm->tm_mon) + "-"
                + std::to_string(ltm->tm_mday) + "-"
                + std::to_string(ltm->tm_hour) + "-"
                + std::to_string(ltm->tm_min) + "-"
                + std::to_string(ltm->tm_sec) + "_" + std::to_string(cell_num)
                + "x" + std::to_string(ant_num) + ".hdf5";
        } else {
            std::string ul_present_str
                = (ul_data_sym_present_ ? "uplink-" : "");
            filename = "logs/trace-" + ul_present_str
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
        && (pilot_syms_per_frame_ + ul_syms_per_frame_ > 0)) {
        task_thread_num_ = tddConf.value("task_thread", TASK_THREAD_NUM);
        rx_thread_num_ = (num_cores >= (2 * RX_THREAD_NUM))
            ? std::min(RX_THREAD_NUM, static_cast<int>(num_bs_sdrs_all_))
            : 1;
        if (reciprocal_calib_ == true) {
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
            if (reciprocal_calib_ == true) {
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
            if (reciprocal_calib_ == true)
                totNumSdr--;
        }
        ret = totNumSdr * bs_channel_.length();
    }
    return ret;
}

Config::~Config() {}

int Config::getClientId(int frame_id, int symbol_id)
{
    if (reciprocal_calib_)
        return symbol_id;
    std::vector<size_t>::iterator it;
    int fid = frame_id % frames_.size();
    it = find(pilot_symbols_.at(fid).begin(), pilot_symbols_.at(fid).end(),
        symbol_id);
    if (it != pilot_symbols_.at(fid).end()) {
        return (it - pilot_symbols_.at(fid).begin());
    }
    return -1;
}

int Config::getNoiseSFIndex(int frame_id, int symbol_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % frames_.size();
    it = find(noise_symbols_.at(fid).begin(), noise_symbols_.at(fid).end(),
        symbol_id);
    if (it != noise_symbols_.at(fid).end())
        return (it - noise_symbols_.at(fid).begin());
    return -1;
}

int Config::getUlSFIndex(int frame_id, int symbol_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % frames_.size();
    it = find(
        ul_symbols_.at(fid).begin(), ul_symbols_.at(fid).end(), symbol_id);
    if (it != ul_symbols_.at(fid).end()) {
        return (it - ul_symbols_.at(fid).begin());
    }
    return -1;
}

int Config::getDlSFIndex(int frame_id, int symbol_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % frames_.size();
    it = find(
        dl_symbols_.at(fid).begin(), dl_symbols_.at(fid).end(), symbol_id);
    if (it != dl_symbols_.at(fid).end())
        return (it - dl_symbols_.at(fid).begin());
    return -1;
}

bool Config::isPilot(int frame_id, int symbol_id)
{
    try {
        return frames_[frame_id % frames_.size()].at(symbol_id) == 'P';
    } catch (const std::out_of_range&) {
        return false;
    }
}

bool Config::isNoise(int frame_id, int symbol_id)
{
    try {
        return frames_[frame_id % frames_.size()].at(symbol_id) == 'N';
    } catch (const std::out_of_range&) {
        return false;
    }
}

bool Config::isData(int frame_id, int symbol_id)
{
    try {
        return frames_[frame_id % frames_.size()].at(symbol_id) == 'U';
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
__attribute__((visibility("default"))) Config* Config_new(char* filename)
{

    Config* cfg = new Config(filename);
    return cfg;
}
}
