/*
 Copyright (c) 2018-2022, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
---------------------------------------------------------------------
 Generates uplink and downlink data bits and the corresponding 
 modulated data in both frequency- and time-domain data.
---------------------------------------------------------------------
*/

#include "include/data_generator.h"

#include "include/comms-lib.h"
#include "include/utils.h"

void DataGenerator::GenerateData(const std::string& directory) {
  srand(time(NULL));

  std::vector<std::complex<float>> prefix_zpad_t(cfg_->prefix(), 0);
  std::vector<std::complex<float>> postfix_zpad_t(cfg_->postfix(), 0);
  for (size_t i = 0; i < cfg_->num_cl_sdrs(); i++) {
    if (cfg_->cl_ul_slots().at(i).size() > 0) {
      int mod_type = cfg_->cl_data_mod() == "64QAM"
                         ? CommsLib::QAM64
                         : (cfg_->cl_data_mod() == "16QAM" ? CommsLib::QAM16
                                                           : CommsLib::QPSK);
      int mod_order = 1 << mod_type;

      std::string filename_tag =
          cfg_->cl_data_mod() + "_" +
          std::to_string(cfg_->symbol_data_subcarrier_num()) + "_" +
          std::to_string(cfg_->fft_size()) + "_" +
          std::to_string(cfg_->symbol_per_slot()) + "_" +
          std::to_string(cfg_->cl_ul_slots()[i].size()) + "_" +
          std::to_string(cfg_->ul_data_frame_num()) + "_" + cfg_->cl_channel() +
          "_" + std::to_string(i) + ".bin";

      std::string filename_ul_data_b = directory + "/ul_data_b_" + filename_tag;
      std::printf("Saving UL data bits for radio %zu to %s\n", i,
                  filename_ul_data_b.c_str());
      FILE* fp_tx_b = std::fopen(filename_ul_data_b.c_str(), "wb");
      std::string filename_ul_data_f = directory + "/ul_data_f_" + filename_tag;
      std::printf("Saving UL frequency-domain data for radio %zu to %s\n", i,
                  filename_ul_data_f.c_str());
      FILE* fp_tx_f = std::fopen(filename_ul_data_f.c_str(), "wb");
      std::string filename_ul_data_t = directory + "/ul_data_t_" + filename_tag;
      std::printf("Saving UL time-domain data for radio %zu to %s\n", i,
                  filename_ul_data_t.c_str());
      FILE* fp_tx_t = std::fopen(filename_ul_data_t.c_str(), "wb");
      // Frame * UL Slots * Channel * Samples
      for (size_t f = 0; f < cfg_->ul_data_frame_num(); f++) {
        for (size_t u = 0; u < cfg_->cl_ul_slots().at(i).size(); u++) {
          for (size_t h = 0; h < cfg_->cl_sdr_ch(); h++) {
            std::vector<std::complex<float>> data_freq_dom;
            std::vector<std::complex<float>> data_time_dom;
            data_time_dom.insert(data_time_dom.begin(), prefix_zpad_t.begin(),
                                 prefix_zpad_t.end());
            for (size_t s = 0; s < cfg_->symbol_per_slot(); s++) {
              std::vector<uint8_t> data_bits;
              for (size_t c = 0; c < cfg_->data_ind().size(); c++) {
                data_bits.push_back((uint8_t)(rand() % mod_order));
              }
              std::fwrite(data_bits.data(), cfg_->symbol_data_subcarrier_num(),
                          sizeof(uint8_t), fp_tx_b);
              std::vector<std::complex<float>> mod_data =
                  CommsLib::modulate(data_bits, mod_type);
              std::vector<std::complex<float>> ofdm_sym(cfg_->fft_size(), 0);
              size_t sc = 0;
              for (size_t c = 0; c < cfg_->data_ind().size(); c++) {
                sc = cfg_->data_ind()[c];
                ofdm_sym[sc] = mod_data[c];
              }
              for (size_t c = 0; c < cfg_->pilot_sc().size(); c++) {
                sc = cfg_->pilot_sc_ind().at(c);
                ofdm_sym[sc] = cfg_->pilot_sc().at(c);
              }
              auto tx_sym = CommsLib::IFFT(ofdm_sym, cfg_->fft_size(),
                                           1.f / cfg_->fft_size(), false);
              tx_sym.insert(tx_sym.begin(), tx_sym.end() - cfg_->cp_size(),
                            tx_sym.end());  // add CP
              data_time_dom.insert(data_time_dom.end(), tx_sym.begin(),
                                   tx_sym.end());
              data_freq_dom.insert(data_freq_dom.end(), ofdm_sym.begin(),
                                   ofdm_sym.end());
            }
            data_time_dom.insert(data_time_dom.end(), postfix_zpad_t.begin(),
                                 postfix_zpad_t.end());
            for (size_t id = 0; id < data_time_dom.size(); id++) {
              data_time_dom.at(id) *= cfg_->tx_scale();
              if (data_time_dom.at(id).real() > 1.f ||
                  data_time_dom.at(id).imag() > 1.f) {
                std::printf(
                    "Saturation detected in frame %zu slot %zu channel %zu "
                    "sample %zu\n",
                    f, u, h, id);
              }
            }
            auto data_time_dom_ci16 = Utils::cfloat_to_cint16(data_time_dom);
            std::fwrite(data_freq_dom.data(),
                        cfg_->fft_size() * cfg_->symbol_per_slot(),
                        sizeof(float) * 2, fp_tx_f);
            std::fwrite(data_time_dom_ci16.data(), cfg_->samps_per_slot(),
                        sizeof(int16_t) * 2, fp_tx_t);
          }
        }
      }
      std::fclose(fp_tx_b);
      std::fclose(fp_tx_f);
      std::fclose(fp_tx_t);
    }
  }

  // Generate Downlink Data
  if (cfg_->dl_slot_per_frame() > 0) {
    int mod_type =
        cfg_->data_mod() == "64QAM"
            ? CommsLib::QAM64
            : (cfg_->data_mod() == "16QAM" ? CommsLib::QAM16 : CommsLib::QPSK);
    int mod_order = 1 << mod_type;
    for (size_t i = 0; i < cfg_->num_bs_sdrs_all(); i++) {
      std::string filename_tag =
          cfg_->data_mod() + "_" +
          std::to_string(cfg_->symbol_data_subcarrier_num()) + "_" +
          std::to_string(cfg_->fft_size()) + "_" +
          std::to_string(cfg_->symbol_per_slot()) + "_" +
          std::to_string(cfg_->dl_slot_per_frame()) + "_" +
          std::to_string(cfg_->dl_data_frame_num()) + "_" + cfg_->bs_channel() +
          "_" + std::to_string(i) + ".bin";

      std::string filename_dl_data_b = directory + "/dl_data_b_" + filename_tag;
      std::printf("Saving DL data bits to %s\n", filename_dl_data_b.c_str());
      FILE* fp_tx_b = std::fopen(filename_dl_data_b.c_str(), "wb");
      std::string filename_dl_data_f = directory + "/dl_data_f_" + filename_tag;
      std::printf("Saving DL frequency-domain data to %s\n",
                  filename_dl_data_f.c_str());
      FILE* fp_tx_f = std::fopen(filename_dl_data_f.c_str(), "wb");
      std::string filename_dl_data_t = directory + "/dl_data_t_" + filename_tag;
      std::printf("Saving DL time-domain data to %s\n",
                  filename_dl_data_t.c_str());
      FILE* fp_tx_t = std::fopen(filename_dl_data_t.c_str(), "wb");
      // Frame * DL Slots * Antennas * Samples
      for (size_t f = 0; f < cfg_->dl_data_frame_num(); f++) {
        for (size_t u = 0; u < cfg_->dl_slot_per_frame(); u++) {
          for (size_t h = 0; h < cfg_->bs_sdr_ch(); h++) {
            std::vector<std::complex<float>> data_freq_dom;
            std::vector<std::complex<float>> data_time_dom;
            data_time_dom.insert(data_time_dom.begin(), prefix_zpad_t.begin(),
                                 prefix_zpad_t.end());
            for (size_t s = 0; s < cfg_->symbol_per_slot(); s++) {
              std::vector<uint8_t> data_bits;
              for (size_t c = 0; c < cfg_->data_ind().size(); c++) {
                data_bits.push_back((uint8_t)(rand() % mod_order));
              }
              std::fwrite(data_bits.data(), cfg_->symbol_data_subcarrier_num(),
                          sizeof(uint8_t), fp_tx_b);
              std::vector<std::complex<float>> mod_data =
                  CommsLib::modulate(data_bits, mod_type);
              std::vector<std::complex<float>> ofdm_sym(cfg_->fft_size(), 0);
              size_t sc = 0;
              for (size_t c = 0; c < cfg_->data_ind().size(); c++) {
                sc = cfg_->data_ind()[c];
                ofdm_sym[sc] = mod_data[c];
              }
              for (size_t c = 0; c < cfg_->pilot_sc().size(); c++) {
                sc = cfg_->pilot_sc_ind().at(c);
                ofdm_sym[sc] = cfg_->pilot_sc().at(c);
              }
              auto tx_sym = CommsLib::IFFT(ofdm_sym, cfg_->fft_size(),
                                           1.f / cfg_->fft_size(), false);
              tx_sym.insert(tx_sym.begin(), tx_sym.end() - cfg_->cp_size(),
                            tx_sym.end());  // add CP
              data_time_dom.insert(data_time_dom.end(), tx_sym.begin(),
                                   tx_sym.end());
              data_freq_dom.insert(data_freq_dom.end(), ofdm_sym.begin(),
                                   ofdm_sym.end());
            }
            data_time_dom.insert(data_time_dom.end(), postfix_zpad_t.begin(),
                                 postfix_zpad_t.end());
            auto data_time_dom_ci16 = Utils::cfloat_to_cint16(data_time_dom);
            std::fwrite(data_freq_dom.data(),
                        cfg_->fft_size() * cfg_->symbol_per_slot(),
                        sizeof(float) * 2, fp_tx_f);
            std::fwrite(data_time_dom_ci16.data(), cfg_->samps_per_slot(),
                        sizeof(int16_t) * 2, fp_tx_t);
          }
        }
      }
      std::fclose(fp_tx_b);
      std::fclose(fp_tx_f);
      std::fclose(fp_tx_t);
    }
  }
}
