/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
 
---------------------------------------------------------------------
 Reads Configuration Parameters from file 
---------------------------------------------------------------------
*/

#ifndef CONFIG_HEADER
#define CONFIG_HEADER

#include <atomic>
#include <complex.h>
#include <vector>
#ifdef JSON
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#endif

class Config {
public:
    Config(const std::string&);
    ~Config();

    //Accessors
    inline bool bs_present(void) const { return this->bs_present_; }
    inline bool client_present(void) const { return this->client_present_; }
    inline size_t num_bs_sdrs_all(void) const { return this->num_bs_sdrs_all_; }
    inline size_t num_cl_sdrs(void) const { return this->num_cl_sdrs_; }
    inline size_t core_alloc(void) const { return this->core_alloc_; }
    inline int subframe_size(void) const { return this->subframe_size_; }
    inline int samps_per_symbol(void) const { return this->samps_per_symbol_; }
    inline size_t symbols_per_frame(void) const
    {
        return this->symbols_per_frame_;
    }
    inline bool ul_data_sym_present(void) const
    {
        return this->ul_data_sym_present_;
    }
    inline size_t num_cells(void) const { return this->num_cells_; }
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
    inline bool beam_sweep(void) const { return this->beam_sweep_; }
    inline size_t beacon_ant(void) const { return this->beacon_ant_; }
    inline size_t num_cl_antennas(void) const { return this->num_cl_antennas_; }
    inline size_t fft_size(void) const { return this->fft_size_; }
    inline size_t cp_size(void) const { return this->cp_size_; }
    inline size_t symbol_data_subcarrier_num(void) const
    {
        return this->symbol_data_subcarrier_num_;
    }
    inline size_t pilot_syms_per_frame(void) const
    {
        return this->pilot_syms_per_frame_;
    }
    inline size_t noise_syms_per_frame(void) const
    {
        return this->noise_syms_per_frame_;
    }
    inline size_t ul_syms_per_frame(void) const
    {
        return this->ul_syms_per_frame_;
    }
    inline double rate(void) const { return this->rate_; }
    inline int tx_advance(void) const { return this->tx_advance_; }
    inline size_t cl_sdr_ch(void) const { return this->cl_sdr_ch_; }

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
    inline bool reciprocal_calib(void) const { return this->reciprocal_calib_; }
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
    inline std::vector<std::complex<float>>& pilot_cf32(void)
    {
        return this->pilot_cf32_;
    }
    inline std::vector<size_t>& n_bs_sdrs(void) { return this->n_bs_sdrs_; }

    inline const std::vector<std::string>& cl_frames(void) const
    {
        return this->cl_frames_;
    }
    inline const std::vector<std::vector<size_t>>& cl_pilot_symbols(void) const
    {
        return this->cl_pilot_symbols_;
    }
    inline const std::vector<std::vector<size_t>>& cl_ul_symbols(void) const
    {
        return this->cl_ul_symbols_;
    }
    inline const std::vector<std::vector<size_t>>& cl_dl_symbols(void) const
    {
        return this->cl_dl_symbols_;
    }
    inline const std::vector<std::string>& cl_sdr_ids(void) const
    {
        return this->cl_sdr_ids_;
    }

    inline const std::vector<int>& data_ind(void) const
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

    inline std::vector<std::vector<double>>& pilot_sym(void)
    {
        return this->pilot_sym_;
    };
    inline std::vector<std::vector<int>>& pilot_sc(void)
    {
        return this->pilot_sc_;
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

    inline unsigned int rx_thread_num(void) const
    {
        return this->rx_thread_num_;
    }
    inline unsigned int task_thread_num(void) const
    {
        return this->task_thread_num_;
    }

    inline const std::vector<std::string>& hub_ids(void) const
    {
        return this->hub_ids_;
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

    inline const std::vector<std::vector<std::complex<float>>>& txdata_time_dom(
        void) const
    {
        return this->txdata_time_dom_;
    }
    inline const std::vector<std::vector<std::complex<float>>>& txdata_freq_dom(
        void) const
    {
        return this->txdata_freq_dom_;
    }

    inline size_t getPackageDataLength() const
    {
        return (2 * this->samps_per_symbol_ * sizeof(short));
    }
    size_t getNumAntennas();
    size_t getMaxNumAntennas();
    size_t getTotNumAntennas();
    int getClientId(int, int);
    int getNoiseSFIndex(int, int);
    int getUlSFIndex(int, int);
    int getDlSFIndex(int, int);
    bool isPilot(int, int);
    bool isNoise(int, int);
    bool isData(int, int);
    unsigned getCoreCount();

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
    size_t symbol_per_subframe_;
    size_t subframe_size_;
    size_t samps_per_symbol_;
    size_t prefix_;
    size_t postfix_;
    size_t symbols_per_frame_;
    size_t pilot_syms_per_frame_;
    size_t noise_syms_per_frame_;
    size_t ul_syms_per_frame_;
    size_t dl_syms_per_frame_; // No accessor
    float tx_scale_; // No accessor
    std::string pilot_seq_;
    std::string beacon_seq_;
    bool ul_data_sym_present_;
    std::string data_mod_;

    // BS features
    size_t num_cells_;
    std::vector<std::string> bs_sdr_file_; // No accessor
    std::string hub_file_; // No accessor
    std::string ref_sdr;
    std::vector<std::vector<std::string>> bs_sdr_ids_;
    std::vector<std::string> hub_ids_;
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
    std::string bs_channel_;
    std::vector<std::string> frames_;
    std::string frame_mode_;
    bool hw_framer_;
    size_t max_frame_;
    std::vector<std::vector<size_t>>
        pilot_symbols_; // Accessed through getClientId
    std::vector<std::vector<size_t>> noise_symbols_;
    std::vector<std::vector<size_t>>
        ul_symbols_; // Accessed through getUlSFIndex()
    std::vector<std::vector<size_t>> dl_symbols_; // No accessor
    bool single_gain_;
    std::vector<double> tx_gain_;
    std::vector<double> rx_gain_;
    std::vector<double> cal_tx_gain_;
    bool sample_cal_en_;
    bool imbalance_cal_en_;
    std::string trace_file_;
    std::vector<std::vector<std::string>> calib_frames_;
    bool reciprocal_calib_;
    size_t cal_ref_sdr_id_;

    // Clients features
    std::vector<std::string> cl_sdr_ids_;
    size_t num_cl_sdrs_;
    size_t cl_sdr_ch_;
    size_t num_cl_antennas_;
    std::string cl_channel_;
    bool cl_agc_en_;
    int cl_agc_gain_init_;
    int tx_advance_;
    std::vector<int> data_ind_;
    std::vector<uint32_t> coeffs_;
    std::vector<std::complex<int16_t>> pilot_ci16_;
    std::vector<std::complex<float>> pilot_cf32_;
    std::vector<uint32_t> pilot_;
    std::vector<std::vector<int>> pilot_sc_;
    std::vector<std::vector<double>> pilot_sym_;
    std::vector<std::vector<std::complex<float>>> tx_data_;
    std::vector<std::vector<std::complex<float>>> txdata_freq_dom_;
    std::vector<std::vector<std::complex<float>>> txdata_time_dom_;

    std::vector<std::string> cl_frames_;
    std::vector<std::vector<size_t>> cl_pilot_symbols_;
    std::vector<std::vector<size_t>> cl_ul_symbols_;
    std::vector<std::vector<size_t>> cl_dl_symbols_;

    std::vector<std::vector<double>> cl_txgain_vec_;
    std::vector<std::vector<double>> cl_rxgain_vec_;

    std::atomic<bool> running_;
    bool core_alloc_;
    unsigned int rx_thread_num_;
    unsigned int task_thread_num_;
};

#endif /* CONFIG_HEADER */
