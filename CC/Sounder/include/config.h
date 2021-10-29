/*
 Copyright (c) 2018-2021, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
 
---------------------------------------------------------------------
 Reads Configuration Parameters from file 
---------------------------------------------------------------------
*/

#ifndef CONFIG_HEADER
#define CONFIG_HEADER

#include <algorithm>
#include <atomic>
#include <complex.h>
#include <vector>

class Config {
public:
    Config(const std::string&, const std::string&, const bool, const bool);
    ~Config();

    //Accessors
    inline bool bs_present(void) const { return this->bs_present_; }
    inline bool client_present(void) const { return this->client_present_; }
    inline size_t num_bs_sdrs_all(void) const { return this->num_bs_sdrs_all_; }
    inline size_t num_bs_antennas_all(void) const
    {
        return this->num_bs_antennas_all_;
    }
    inline size_t num_cl_sdrs(void) const { return this->num_cl_sdrs_; }
    inline size_t core_alloc(void) const { return this->core_alloc_; }
    inline int slot_samp_size(void) const { return this->slot_samp_size_; }
    inline size_t samps_per_slot(void) const { return this->samps_per_slot_; }
    inline size_t slot_per_frame(void) const { return this->slot_per_frame_; }
    inline size_t symbol_per_slot(void) const { return this->symbol_per_slot_; }
    inline bool ul_data_slot_present(void) const
    {
        return this->ul_data_slot_present_;
    }
    inline bool dl_data_slot_present(void) const
    {
        return this->dl_data_slot_present_;
    }
    inline size_t num_cells(void) const { return this->num_cells_; }
    inline size_t guard_mult(void) const { return this->guard_mult_; }
    inline bool bs_hw_framer(void) const { return this->bs_hw_framer_; }
    inline bool hw_framer(void) const { return this->hw_framer_; }
    inline int prefix(void) const { return this->prefix_; }
    inline int postfix(void) const { return this->postfix_; }
    inline int beacon_size(void) const { return this->beacon_size_; }
    inline double bw_filter(void) const { return this->bw_filter_; }
    inline double freq(void) const { return this->freq_; }
    inline double nco(void) const { return this->nco_; }
    inline double radio_rf_freq(void) const { return this->radio_rf_freq_; }
    inline bool single_gain(void) const { return this->single_gain_; }
    inline bool cl_agc_en(void) const { return this->cl_agc_en_; }
    inline int cl_agc_gain_init(void) const { return this->cl_agc_gain_init_; }
    inline bool imbalance_cal_en(void) const { return this->imbalance_cal_en_; }
    inline bool sample_cal_en(void) const { return this->sample_cal_en_; }
    inline size_t max_frame(void) const { return this->max_frame_; }
    inline size_t ul_data_frame_num(void) const
    {
        return this->ul_data_frame_num_;
    }
    inline size_t dl_data_frame_num(void) const
    {
        return this->dl_data_frame_num_;
    }
    inline bool beam_sweep(void) const { return this->beam_sweep_; }
    inline size_t beacon_ant(void) const { return this->beacon_ant_; }
    inline size_t num_cl_antennas(void) const { return this->num_cl_antennas_; }
    inline size_t fft_size(void) const { return this->fft_size_; }
    inline size_t cp_size(void) const { return this->cp_size_; }
    inline size_t symbol_data_subcarrier_num(void) const
    {
        return this->symbol_data_subcarrier_num_;
    }
    inline size_t pilot_slot_per_frame(void) const
    {
        return this->pilot_slot_per_frame_;
    }
    inline size_t noise_slot_per_frame(void) const
    {
        return this->noise_slot_per_frame_;
    }
    inline size_t ul_slot_per_frame(void) const
    {
        return this->ul_slot_per_frame_;
    }
    inline size_t dl_slot_per_frame(void) const
    {
        return this->dl_slot_per_frame_;
    }
    inline const std::vector<std::vector<size_t>>& dl_slots(void) const
    {
        return this->dl_slots_;
    }
    inline double rate(void) const { return this->rate_; }
    inline int tx_advance(void) const { return this->tx_advance_; }
    inline size_t cl_sdr_ch(void) const { return this->cl_sdr_ch_; }
    inline size_t bs_sdr_ch(void) const { return this->bs_sdr_ch_; }

    inline bool running(void) const { return this->running_.load(); }
    inline void running(bool value) { this->running_ = value; }

    inline const std::string& frame_mode(void) const
    {
        return this->frame_mode_;
    }
    inline const std::string& bs_channel(void) const
    {
        return this->bs_channel_;
    }
    inline const std::string& trace_file(void) const
    {
        return this->trace_file_;
    }
    inline const std::string& cl_channel(void) const
    {
        return this->cl_channel_;
    }
    inline const std::string& beacon_seq(void) const
    {
        return this->beacon_seq_;
    }
    inline const std::string& pilot_seq(void) const { return this->pilot_seq_; }
    inline const std::string& data_mod(void) const { return this->data_mod_; }

    inline const std::vector<size_t>& n_bs_sdrs_agg(void) const
    {
        return this->n_bs_sdrs_agg_;
    }
    inline bool internal_measurement(void) const
    {
        return this->internal_measurement_;
    }
    inline bool ref_node_enable(void) const { return this->ref_node_enable_; }
    inline size_t cal_ref_sdr_id(void) const { return this->cal_ref_sdr_id_; }
    inline const std::vector<std::vector<std::string>>& calib_frames(void) const
    {
        return this->calib_frames_;
    }

    //TODO split the following (4) in accessor and setter
    inline std::vector<std::complex<int16_t>>& beacon_ci16(void)
    {
        return this->beacon_ci16_;
    }
    inline std::vector<std::vector<std::complex<float>>>& tx_data(void)
    {
        return this->tx_data_;
    };
    inline std::vector<std::complex<int16_t>>& pilot_ci16(void)
    {
        return this->pilot_ci16_;
    }
    inline std::vector<std::complex<float>>& pilot_cf32(void)
    {
        return this->pilot_cf32_;
    }
    inline std::vector<size_t>& n_bs_sdrs(void) { return this->n_bs_sdrs_; }

    inline const std::vector<std::string>& cl_frames(void) const
    {
        return this->cl_frames_;
    }
    inline const std::vector<std::vector<size_t>>& cl_pilot_slots(void) const
    {
        return this->cl_pilot_slots_;
    }
    inline const std::vector<std::vector<size_t>>& cl_ul_slots(void) const
    {
        return this->cl_ul_slots_;
    }
    inline const std::vector<std::vector<size_t>>& cl_dl_slots(void) const
    {
        return this->cl_dl_slots_;
    }
    inline const std::vector<std::string>& cl_sdr_ids(void) const
    {
        return this->cl_sdr_ids_;
    }
    inline const std::vector<std::string>& ul_tx_fd_data_files(void) const
    {
        return this->ul_tx_fd_data_files_;
    }
    inline const std::vector<std::string>& ul_tx_td_data_files(void) const
    {
        return this->ul_tx_td_data_files_;
    }
    inline const std::vector<std::string>& dl_tx_fd_data_files(void) const
    {
        return this->dl_tx_fd_data_files_;
    }
    inline const std::vector<std::string>& dl_tx_td_data_files(void) const
    {
        return this->dl_tx_td_data_files_;
    }

    inline const std::vector<size_t>& data_ind(void) const
    {
        return this->data_ind_;
    }
    inline const std::vector<uint32_t>& coeffs(void) const
    {
        return this->coeffs_;
    }
    inline const std::vector<uint32_t>& pilot(void) const
    {
        return this->pilot_;
    }
    inline const std::vector<std::vector<double>>& cl_txgain_vec(void) const
    {
        return this->cl_txgain_vec_;
    }
    inline const std::vector<std::vector<double>>& cl_rxgain_vec(void) const
    {
        return this->cl_rxgain_vec_;
    }
    inline const std::vector<uint32_t>& beacon(void) const
    {
        return this->beacon_;
    }

    inline std::vector<std::vector<float>>& pilot_sym_t(void)
    {
        return this->pilot_sym_t_;
    };
    inline std::vector<std::vector<float>>& pilot_sym_f(void)
    {
        return this->pilot_sym_f_;
    };
    inline std::vector<std::complex<float>>& pilot_sc(void)
    {
        return this->pilot_sc_;
    };
    inline std::vector<size_t>& pilot_sc_ind(void)
    {
        return this->pilot_sc_ind_;
    };

    inline const std::vector<std::string>& frames(void) const
    {
        return this->frames_;
    }

    inline const std::vector<std::vector<std::string>>& bs_sdr_ids(void) const
    {
        return this->bs_sdr_ids_;
    }
    inline const std::vector<std::complex<float>>& gold_cf32(void) const
    {
        return this->gold_cf32_;
    }

    inline size_t cl_rx_thread_num(void) const
    {
        return this->cl_rx_thread_num_;
    }
    inline size_t bs_rx_thread_num(void) const
    {
        return this->bs_rx_thread_num_;
    }
    inline size_t task_thread_num(void) const { return this->task_thread_num_; }

    inline const std::vector<std::string>& hub_ids(void) const
    {
        return this->hub_ids_;
    }

    inline const std::vector<std::string>& calib_ids(void) const
    {
        return this->calib_ids_;
    }

    inline const std::vector<double>& tx_gain(void) const
    {
        return this->tx_gain_;
    }
    inline const std::vector<double>& rx_gain(void) const
    {
        return this->rx_gain_;
    }
    inline const std::vector<double>& cal_tx_gain(void) const
    {
        return this->cal_tx_gain_;
    }

    inline std::vector<std::vector<std::complex<int16_t>>>& txdata_time_dom(
        void)
    {
        return this->txdata_time_dom_;
    }
    inline const std::vector<std::vector<std::complex<float>>>& txdata_freq_dom(
        void) const
    {
        return this->txdata_freq_dom_;
    }

    inline std::vector<std::vector<std::complex<int16_t>>>& dl_txdata_time_dom(
        void)
    {
        return this->dl_txdata_time_dom_;
    }
    inline const std::vector<std::vector<std::complex<float>>>&
    dl_txdata_freq_dom(void) const
    {
        return this->dl_txdata_freq_dom_;
    }

    inline size_t getPackageDataLength() const
    {
        return (2 * this->samps_per_slot_ * sizeof(short));
    }

    /// Return the frame duration in seconds
    inline double getFrameDurationSec() const
    {
        return ((this->symbol_per_slot_ * this->samps_per_slot_) / this->rate_);
    }

    size_t getNumAntennas();
    size_t getMaxNumAntennas();
    size_t getNumBsSdrs();
    size_t getTotNumAntennas();
    size_t getNumRecordedSdrs();
    int getClientId(int, int);
    int getNoiseSlotIndex(int, int);
    int getUlSlotIndex(int, int);
    int getDlSlotIndex(int, int);
    bool isPilot(int, int);
    bool isNoise(int, int);
    bool isUlData(int, int);
    bool isDlData(int, int);
    unsigned getCoreCount();
    void loadULData(const std::string&);
    void loadDLData(const std::string&);

private:
    bool bs_present_;
    bool client_present_;

    // common features
    double freq_;
    double nco_; // baseband frequency controlled by NCO
    double rate_;
    double
        radio_rf_freq_; // RF frequency set frame_modeon the radio after NCO adjustments
    double bw_filter_;
    size_t fft_size_;
    size_t cp_size_;
    size_t ofdm_symbol_size_;
    size_t symbol_data_subcarrier_num_;
    size_t symbol_per_slot_;
    size_t slot_samp_size_;
    size_t samps_per_slot_;
    size_t prefix_;
    size_t postfix_;
    size_t slot_per_frame_;
    size_t pilot_slot_per_frame_;
    size_t noise_slot_per_frame_;
    size_t ul_slot_per_frame_;
    size_t dl_slot_per_frame_; // No accessor
    float tx_scale_; // No accessor
    std::string pilot_seq_;
    std::string beacon_seq_;
    bool ul_data_slot_present_;
    bool dl_data_slot_present_;
    std::string data_mod_;

    // BS features
    size_t num_cells_;
    size_t guard_mult_;
    std::vector<std::string> bs_sdr_file_; // No accessor
    std::string hub_file_; // No accessor
    std::string ref_sdr;
    size_t bs_sdr_ch_;
    std::vector<std::vector<std::string>> bs_sdr_ids_;
    std::vector<std::string> hub_ids_;
    std::vector<std::string> calib_ids_;
    std::vector<std::complex<float>> gold_cf32_;
    std::vector<uint32_t> beacon_;
    std::vector<std::complex<int16_t>> beacon_ci16_;
    int beacon_size_;
    size_t beacon_ant_;
    bool beam_sweep_;
    std::vector<size_t> n_bs_sdrs_;
    std::vector<size_t> n_bs_antennas_; //No accessor
    std::vector<size_t> n_bs_sdrs_agg_;
    size_t num_bs_sdrs_all_;
    size_t num_bs_antennas_all_;
    std::string bs_channel_;
    std::vector<std::string> frames_;
    std::string frame_mode_;
    bool bs_hw_framer_;
    bool hw_framer_;
    size_t max_frame_;
    size_t ul_data_frame_num_;
    size_t dl_data_frame_num_;
    std::vector<std::vector<size_t>>
        pilot_slots_; // Accessed through getClientId
    std::vector<std::vector<size_t>> noise_slots_;
    std::vector<std::vector<size_t>>
        ul_slots_; // Accessed through getUlSFIndex()
    std::vector<std::vector<size_t>> dl_slots_;
    bool single_gain_;
    std::vector<double> tx_gain_;
    std::vector<double> rx_gain_;
    std::vector<double> cal_tx_gain_;
    bool sample_cal_en_;
    bool imbalance_cal_en_;
    std::string trace_file_;
    std::vector<std::vector<std::string>> calib_frames_;
    bool internal_measurement_;
    bool ref_node_enable_;
    size_t cal_ref_sdr_id_;

    // Clients features
    std::vector<std::string> cl_sdr_ids_;
    size_t max_tx_gain_ue_;
    size_t num_cl_sdrs_;
    size_t cl_sdr_ch_;
    size_t num_cl_antennas_;
    std::string cl_channel_;
    bool cl_agc_en_;
    int cl_agc_gain_init_;
    int tx_advance_;
    std::vector<size_t> data_ind_;
    std::vector<uint32_t> coeffs_;
    std::vector<std::complex<int16_t>> pilot_ci16_;
    std::vector<std::complex<float>> pilot_cf32_;
    std::vector<uint32_t> pilot_;
    std::vector<std::complex<float>> pilot_sc_;
    std::vector<size_t> pilot_sc_ind_;
    std::vector<std::vector<float>> pilot_sym_t_;
    std::vector<std::vector<float>> pilot_sym_f_;
    std::vector<std::vector<std::complex<float>>> tx_data_;
    std::vector<std::vector<std::complex<float>>> txdata_freq_dom_;
    std::vector<std::vector<std::complex<int16_t>>> txdata_time_dom_;
    std::vector<std::vector<std::complex<float>>> dl_txdata_freq_dom_;
    std::vector<std::vector<std::complex<int16_t>>> dl_txdata_time_dom_;

    std::vector<std::string> cl_frames_;
    std::vector<std::vector<size_t>> cl_pilot_slots_;
    std::vector<std::vector<size_t>> cl_ul_slots_;
    std::vector<std::vector<size_t>> cl_dl_slots_;

    std::vector<std::vector<double>> cl_txgain_vec_;
    std::vector<std::vector<double>> cl_rxgain_vec_;
    std::vector<std::string> ul_tx_td_data_files_;
    std::vector<std::string> ul_tx_fd_data_files_;
    std::vector<std::string> dl_tx_td_data_files_;
    std::vector<std::string> dl_tx_fd_data_files_;

    std::atomic<bool> running_;
    bool core_alloc_;
    size_t bs_rx_thread_num_;
    size_t cl_rx_thread_num_;
    size_t task_thread_num_;
};

#endif /* CONFIG_HEADER */
