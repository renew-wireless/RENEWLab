/*
 Helper functions for signal processing and communications
 Generate training sequence for pilots and preambles.

 Supports:
 STS - 802.11 Short training sequence. Generates one symbol, 16 complex I/Q samples.
 LTS - 802.11 Long training sequence. Generates 2.5 symbols, cp length of 32 samples, 
       for a total of 160 complex I/Q samples.
 LTE Zadoff Chu - Generates the 25th root length-63 Zadoff-Chu sequence.
       Total of 63-long complex IQ samples.
 Gold IFFT - Total of 128-long complex IQ samples including a 32-sample cyclic prefix
 Hadamard - Real valued sequence. Possible lenghts: {2, 4, 8, 16, 32, 64}

---------------------------------------------------------------------
 Copyright (c) 2018-2022, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
*/

#include "include/comms-lib.h"

#include <limits.h>

#include <queue>

#include "include/constants.h"
#include "include/utils.h"

static constexpr float kShortMaxFloat = SHRT_MAX;

int CommsLib::findLTS(const std::vector<std::complex<float>>& iq, int seqLen) {
  /*
     * Find 802.11-based LTS (Long Training Sequence)
     * Input:
     *     iq        - IQ complex samples (vector)
     *     seqLen    - Length of sequence
     * Output:
     *     best_peak - LTS peak index (correlation peak)
     */

  float lts_thresh = 0.8;
  std::vector<std::vector<float>> lts_seq;
  int best_peak;

  // Original LTS sequence
  lts_seq = CommsLib::getSequence(LTS_SEQ, seqLen);

  // Re-arrange into complex vector, flip, and compute conjugate
  const size_t lts_symbol_len = Consts::kFftSize_80211;
  std::vector<std::complex<float>> lts_sym(lts_symbol_len);
  std::vector<std::complex<float>> lts_sym_conj(lts_sym.size());
  for (size_t i = 0; i < lts_sym.size(); i++) {
    // lts_seq is a 2x160 matrix (real/imag by seqLen=160 elements)
    // grab one symbol and flip around
    lts_sym[i] = std::complex<float>(lts_seq[0][seqLen - 1 - i],
                                     lts_seq[1][seqLen - 1 - i]);
    // conjugate
    lts_sym_conj[i] = std::conj(lts_sym[i]);
  }

  // Equivalent to numpy's sign function
  auto iq_sign = CommsLib::csign(iq);

  // Convolution
  auto lts_corr = CommsLib::convolve(iq_sign, lts_sym_conj);
  std::vector<float> lts_corr_abs(lts_corr.size());
  std::transform(lts_corr.begin(), lts_corr.end(), lts_corr_abs.begin(),
                 computeAbs);
  double lts_limit =
      lts_thresh * *std::max_element(lts_corr_abs.begin(), lts_corr_abs.end());

  // Find all peaks, and pairs that are lts_sym.size() samples apart
  std::queue<int> valid_peaks;
  for (size_t i = lts_sym.size(); i < lts_corr.size(); i++) {
    if (lts_corr_abs[i] > lts_limit &&
        lts_corr_abs[i - lts_sym.size()] > lts_limit)
      valid_peaks.push(i);
  }

  // Use first LTS found
  if (valid_peaks.empty()) {
    best_peak = -1;
  } else {
    best_peak = valid_peaks.front();
  }

  return best_peak;
}

size_t CommsLib::find_pilot_seq(const std::vector<std::complex<float>>& iq,
                                const std::vector<std::complex<float>>& pilot,
                                size_t seq_len) {
  // Re-arrange into complex vector, flip, and compute conjugate
  std::vector<std::complex<float>> pilot_conj;
  for (size_t i = 0; i < seq_len; i++) {
    // conjugate
    pilot_conj.push_back(std::conj(pilot[seq_len - i - 1]));
  }

  // Equivalent to numpy's sign function
  auto iq_sign = CommsLib::csign(iq);

  // Convolution
  auto pilot_corr = CommsLib::convolve(iq_sign, pilot_conj);

  std::vector<float> pilot_corr_abs(pilot_corr.size());
  for (size_t i = 0; i < pilot_corr_abs.size(); i++)
    pilot_corr_abs[i] = computePower(pilot_corr[i]);

  // Find all peaks
  auto best_peak =
      std::max_element(pilot_corr_abs.begin(), pilot_corr_abs.end()) -
      pilot_corr_abs.begin();
  return best_peak;
}

int CommsLib::find_beacon(const std::complex<int16_t>* raw_samples,
                          size_t check_window) {
  //Allocate memory, only used for beacon detection (consider making this static)
  std::vector<std::complex<float>> beacon_compare(
      check_window, std::complex<float>(0.0f, 0.0f));

  // convert entire frame data to complex float for sync detection
  for (size_t i = 0; i < check_window; i++) {
    beacon_compare.at(i) = (std::complex<float>(
        static_cast<float>(raw_samples[i].real()) / kShortMaxFloat,
        static_cast<float>(raw_samples[i].imag()) / kShortMaxFloat));
  }
  return CommsLib::find_beacon(beacon_compare);
}

int CommsLib::find_beacon(const std::vector<std::complex<float>>& iq) {
  int best_peak;
  std::queue<int> valid_peaks;

  // Original LTS sequence
  int seqLen = 128;
  auto gold_seq = CommsLib::getSequence(GOLD_IFFT);
  struct timespec tv, tv2;

  // Re-arrange into complex vector, flip, and compute conjugate
  std::vector<std::complex<float>> gold_sym(seqLen);
  std::vector<std::complex<float>> gold_sym_conj(gold_sym.size());
  for (int i = 0; i < seqLen; i++) {
    // grab one symbol and flip around
    gold_sym[i] = std::complex<float>(gold_seq[0][seqLen - 1 - i],
                                      gold_seq[1][seqLen - 1 - i]);
    // conjugate
    gold_sym_conj[i] = std::conj(gold_sym[i]);
  }

  // Convolution
  clock_gettime(CLOCK_MONOTONIC, &tv);
  auto gold_corr = CommsLib::convolve(iq, gold_sym_conj);
  clock_gettime(CLOCK_MONOTONIC, &tv2);
#ifdef TEST_BENCH
  double diff1 =
      ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec)) / 1e3;
#endif

  size_t gold_corr_size = gold_corr.size();
  size_t gold_corr_size_2 = gold_corr_size + seqLen;
  std::vector<float> gold_corr_2(gold_corr_size_2);
  clock_gettime(CLOCK_MONOTONIC, &tv);
  for (size_t i = seqLen; i < gold_corr_size; i++) {
    gold_corr_2[i] =
        computePower(gold_corr[i] * std::conj(gold_corr[i - seqLen]));
  }
  clock_gettime(CLOCK_MONOTONIC, &tv2);
#ifdef TEST_BENCH
  double diff2 =
      ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec)) / 1e3;
#endif

  std::vector<float> const1(seqLen, 1);

  std::vector<float> corr_abs(gold_corr_size);
  clock_gettime(CLOCK_MONOTONIC, &tv);
  std::transform(gold_corr.begin(), gold_corr.end(), corr_abs.begin(),
                 computePower);
  auto corr_abs_filt = CommsLib::convolve<float>(corr_abs, const1);
  corr_abs_filt.push_back(0);
  clock_gettime(CLOCK_MONOTONIC, &tv2);
#ifdef TEST_BENCH
  double diff3 =
      ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec)) / 1e3;
#endif

  std::vector<float> thresh(corr_abs_filt.begin(), corr_abs_filt.end());
  //std::vector<double> thresh(corr_abs_filt.size());
  //std::transform(corr_abs_filt.begin(), corr_abs_filt.end(), thresh.begin(),
  //    std::bind(std::multiplies<double>(), std::placeholders::_1, 0.0078)); // divide by 128
  // Find all peaks, and pairs that are lts_sym.size() samples apart
  for (size_t i = seqLen; i < gold_corr_size; i++) {
    if (gold_corr_2[i] > thresh[i] / 128) valid_peaks.push(i - seqLen);
  }

#ifdef TEST_BENCH
  std::cout << "Convolution took " << diff1 << " usec" << std::endl;
  std::cout << "Corr Abs took " << diff2 << " usec" << std::endl;
  std::cout << "Thresh calc took " << diff3 << " usec" << std::endl;
  printf("Saving Corr data\n");
  std::string filename = "corr_data.bin";
  FILE* fc = fopen(filename.c_str(), "wb");
  float* cdata_ptr = (float*)gold_corr_2.data();
  fwrite(cdata_ptr, gold_corr_2.size(), sizeof(float), fc);
  fclose(fc);
  filename = "thresh_data.bin";
  FILE* fp = fopen(filename.c_str(), "wb");
  float* tdata_ptr = (float*)thresh.data();
  fwrite(tdata_ptr, thresh.size(), sizeof(float), fp);
  fclose(fp);
#endif

  valid_peaks.push(0);
  // Use first LTS found
  if (valid_peaks.empty()) {
    best_peak = -1;
  } else {
    best_peak = valid_peaks.front();
  }

  return best_peak;
}

std::vector<std::complex<float>> CommsLib::csign(
    const std::vector<std::complex<float>>& iq) {
  /*
     * Return element-wise indication of the sign of a number (for complex vector).
     *
     * For complex-valued inputs:
     *     sign(x.real) + 0j if x.real != 0 else sign(x.imag) + 0j
     *
     * where sign(x) is given by
     *     -1 if x < 0, 0 if x==0, 1 if x > 0
     */
  std::vector<std::complex<float>> iq_sign;
  for (int i = 0; i < static_cast<int>(iq.size()); i++) {
    // sign(x.real) + 0j if x.real != 0 else sign(x.imag) + 0j
    std::complex<float> x = iq[i];
    if (x.real() != 0) {
      iq_sign.push_back((x.real() > 0) ? 1 : (x.real() < 0) ? -1 : 0);
    } else {
      iq_sign.push_back((x.imag() > 0) ? 1 : (x.imag() < 0) ? -1 : 0);
    }
  }
  return iq_sign;
}

float CommsLib::find_max_abs(const std::vector<std::complex<float>>& in) {
  float max_val = 0;
  for (size_t j = 0; j < in.size(); j++) {
    auto cur_val = std::abs(in[j]);
    if (cur_val > max_val) {
      max_val = cur_val;
    }
  }
  return max_val;
}

std::vector<float> CommsLib::magnitudeFFT(
    std::vector<std::complex<float>> const& samps,
    std::vector<float> const& win, size_t fftSize) {
  std::vector<std::complex<float>> preFFT(samps.size());

  for (size_t n = 0; n < fftSize; n++) {
    preFFT[n] = samps[n] * win[n];
  }

  std::vector<std::complex<float>> fftSamps = CommsLib::FFT(preFFT, fftSize);

  // compute magnitudes
  std::vector<float> fftMag;
  fftMag.reserve(fftSize);
  for (size_t n = fftSize / 2; n < fftSize; n++) {
    fftMag.push_back(std::norm(fftSamps[n]));
  }
  for (size_t n = 0; n < fftSize / 2; n++) {
    fftMag.push_back(std::norm(fftSamps[n]));
  }
  std::reverse(
      fftMag.begin(),
      fftMag
          .end());  // not sure why we need reverse here, but this seems to give the right spectrum
  return fftMag;
}

// Take fftSize samples of (1 - cos(x)) / 2 from 0 up to 2pi
std::vector<float> CommsLib::hannWindowFunction(size_t fftSize) {
  std::vector<float> winFcn(1, 0);
  double step = 2 * M_PI / fftSize;

  // Compute the samples for the first half.
  for (size_t n = 1; n < fftSize / 2; n++) {
    winFcn.push_back((1 - std::cos(step * n)) / 2);
  }
  // If a sample lies at the center, just use (1-cos(pi))/2 == 1.
  if (fftSize % 2 == 0) winFcn.push_back(1);
  // The second half is a mirror image of the first, so just copy.
  for (size_t n = fftSize / 2 + 1; n < fftSize; n++)
    winFcn.push_back(winFcn[fftSize - n]);
  return winFcn;
}

double CommsLib::windowFunctionPower(std::vector<float> const& win) {
  double windowPower = (0);
  size_t N = win.size();
  for (size_t n = 0; n < win.size(); n++) {
    windowPower += std::norm(win[n]);
  }
  windowPower = std::sqrt(windowPower / N);
  return 20 * std::log10(N * windowPower);
}

template <typename T>
T CommsLib::findTone(std::vector<T> const& magnitude, double winGain,
                     double fftBin, size_t fftSize, const size_t delta) {
  /*
     * Find the tone level at a specific interval in the input Power Spectrum
     * fftBins assumed interval is [-0.5, 0.5] which is coverted to [0, fftSize-1]
     */
  // make sure we don't exceed array bounds
  size_t first =
      std::max<size_t>(0, std::lround((fftBin + 0.5) * fftSize) - delta);
  size_t last = std::min<size_t>(fftSize - 1,
                                 std::lround((fftBin + 0.5) * fftSize) + delta);
  T refLevel = magnitude[last];
  for (size_t n = first; n < last; n++) {
    if (magnitude[n] > refLevel) refLevel = magnitude[n];
  }
  return 10 * std::max(std::log10(refLevel), (T)(-20.0)) - (T)winGain;
}

float CommsLib::measureTone(std::vector<std::complex<float>> const& samps,
                            std::vector<float> const& win, double winGain,
                            double fftBin, size_t fftSize, const size_t delta) {
  return findTone(magnitudeFFT(samps, win, fftSize), winGain, fftBin, fftSize,
                  delta);
}

std::vector<size_t> CommsLib::getDataSc(size_t fftSize, size_t DataScNum,
                                        size_t PilotScOffset) {
  std::vector<size_t> data_sc;
  if (fftSize == Consts::kFftSize_80211) {
    // We follow 802.11 PHY format here
    size_t sc_ind[48] = {1,  2,  3,  4,  5,  6,  8,  9,  10, 11, 12, 13,
                         14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26,
                         38, 39, 40, 41, 42, 44, 45, 46, 47, 48, 49, 50,
                         51, 52, 53, 54, 55, 56, 58, 59, 60, 61, 62, 63};
    data_sc.assign(sc_ind, sc_ind + 48);
  } else {  // Allocate the center subcarriers as data
    size_t start_sc = (fftSize - DataScNum) / 2;
    size_t stop_sc = start_sc + DataScNum;
    for (size_t i = start_sc; i < stop_sc; i++)
      if ((i - start_sc) % kPilotSubcarrierSpacing != PilotScOffset)
        data_sc.push_back(i);
  }
  return data_sc;
}

std::vector<size_t> CommsLib::getNullSc(size_t fftSize, size_t DataScNum) {
  std::vector<size_t> null_sc;
  if (fftSize == Consts::kFftSize_80211) {
    // We follow 802.11 PHY format here
    int null[12] = {0, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37};
    null_sc.assign(null, null + 12);
  } else {  // Allocate the boundary subcarriers as null
    size_t start_sc = (fftSize - DataScNum) / 2;
    size_t stop_sc = start_sc + DataScNum;
    for (size_t i = 0; i < start_sc; i++) null_sc.push_back(i);
    for (size_t i = stop_sc; i < fftSize; i++) null_sc.push_back(i);
  }
  return null_sc;
}

std::vector<std::complex<float>> CommsLib::getPilotScValue(
    size_t fftSize, size_t DataScNum, size_t PilotScOffset) {
  std::vector<std::complex<float>> pilot_sc;
  if (fftSize == Consts::kFftSize_80211) {
    // We follow 802.11 PHY format here
    std::complex<float> sc_val[4] = {1, 1, -1, 1};
    pilot_sc.assign(sc_val, sc_val + 4);
  } else {
    size_t start_sc = (fftSize - DataScNum) / 2;
    size_t stop_sc = start_sc + DataScNum;
    std::vector<std::vector<float>> pilot_sym =
        CommsLib::getSequence(CommsLib::LTE_ZADOFF_CHU, DataScNum);
    std::vector<std::complex<float>> pilot_sym_cf32;
    for (size_t i = 0; i < pilot_sym.at(0).size(); i++) {
      pilot_sym_cf32.push_back(
          std::complex<float>(pilot_sym.at(0).at(i), pilot_sym.at(1).at(i)));
    }
    auto pilot_fft = CommsLib::FFT(pilot_sym_cf32, fftSize);
    for (size_t i = start_sc + PilotScOffset; i < stop_sc;
         i += kPilotSubcarrierSpacing) {
      pilot_sc.push_back(pilot_fft.at(i));
    }
  }
  return pilot_sc;
}

std::vector<size_t> CommsLib::getPilotScIndex(size_t fftSize, size_t DataScNum,
                                              size_t PilotScOffset) {
  std::vector<size_t> pilot_sc;
  if (fftSize == Consts::kFftSize_80211) {
    // We follow 802.11 standard here
    int sc_ind[4] = {7, 21, 43, 57};
    pilot_sc.assign(sc_ind, sc_ind + 4);
  } else {  // consider center subcarriers
    size_t start_sc = (fftSize - DataScNum) / 2;
    size_t stop_sc = start_sc + DataScNum;
    // pilot at the center of each RB
    for (size_t i = start_sc + PilotScOffset; i < stop_sc;
         i += kPilotSubcarrierSpacing) {
      pilot_sc.push_back(i);
    }
  }
  return pilot_sc;
}

std::vector<std::complex<float>> CommsLib::IFFT(
    const std::vector<std::complex<float>>& in, int fftSize, float scale,
    bool normalize) {
  std::vector<std::complex<float>> out(in.size());

  void* fft_in = mufft_alloc(fftSize * sizeof(std::complex<float>));
  void* fft_out = mufft_alloc(fftSize * sizeof(std::complex<float>));
  mufft_plan_1d* mufftplan =
      mufft_create_plan_1d_c2c(fftSize, MUFFT_INVERSE, MUFFT_FLAG_CPU_ANY);

  memcpy(fft_in, in.data(), fftSize * sizeof(std::complex<float>));
  mufft_execute_plan_1d(mufftplan, fft_out, fft_in);
  memcpy(out.data(), fft_out, fftSize * sizeof(std::complex<float>));
  float max_val = 1;
  if (normalize) {
    for (int i = 0; i < fftSize; i++) {
      if (std::abs(out[i]) > max_val) max_val = std::abs(out[i]);
    }
  }
#if DEBUG_PRINT
  std::cout << "IFFT output is normalized with " << std::to_string(max_val)
            << std::endl;
#endif
  for (int i = 0; i < fftSize; i++) out[i] = (out[i] / max_val) * scale;

  mufft_free_plan_1d(mufftplan);
  mufft_free(fft_in);
  mufft_free(fft_out);
  return out;
}

std::vector<std::complex<float>> CommsLib::FFT(
    const std::vector<std::complex<float>>& in, int fftSize) {
  std::vector<std::complex<float>> out(in.size());

  void* fft_in = mufft_alloc(fftSize * sizeof(std::complex<float>));
  void* fft_out = mufft_alloc(fftSize * sizeof(std::complex<float>));
  mufft_plan_1d* mufftplan =
      mufft_create_plan_1d_c2c(fftSize, MUFFT_FORWARD, MUFFT_FLAG_CPU_ANY);

  memcpy(fft_in, in.data(), fftSize * sizeof(std::complex<float>));
  mufft_execute_plan_1d(mufftplan, fft_out, fft_in);
  memcpy(out.data(), fft_out, fftSize * sizeof(std::complex<float>));

  mufft_free_plan_1d(mufftplan);
  mufft_free(fft_in);
  mufft_free(fft_out);
  return out;
}

std::vector<std::complex<float>> CommsLib::modulate(std::vector<uint8_t> in,
                                                    int type) {
  std::vector<std::complex<float>> out(in.size());
  if (type == QPSK) {
    float qpsk_table[2][4];  // = init_qpsk();
    float scale = 1 / sqrt(2);
    float mod_qpsk[2] = {-scale, scale};
    for (int i = 0; i < 4; i++) {
      qpsk_table[0][i] = mod_qpsk[i / 2];
      qpsk_table[1][i] = mod_qpsk[i % 2];
    }
    for (size_t i = 0; i < in.size(); i++) {
      if (in[i] < 4u)
        out[i] =
            std::complex<float>(qpsk_table[0][in[i]], qpsk_table[1][in[i]]);
      else {
        std::cout << "Error: No compatible input vector!" << std::endl;
        break;
      }
    }
  } else if (type == QAM16) {
    float qam16_table[2][16];  //= init_qam16();
    float scale = 1 / sqrt(10);
    float mod_16qam[4] = {-3 * scale, -1 * scale, 3 * scale, scale};
    for (int i = 0; i < 16; i++) {
      qam16_table[0][i] = mod_16qam[i / 4];
      qam16_table[1][i] = mod_16qam[i % 4];
    }
    for (size_t i = 0; i < in.size(); i++) {
      if (in[i] < 16u)
        out[i] =
            std::complex<float>(qam16_table[0][in[i]], qam16_table[1][in[i]]);
      else {
        std::cout << "Error: No compatible input vector!" << std::endl;
        break;
      }
    }
  } else if (type == QAM64) {
    float qam64_table[2][64];  // = init_qam64();
    float scale = 1 / sqrt(42);
    float mod_64qam[8] = {-7 * scale, -5 * scale, -3 * scale, -1 * scale,
                          scale,      3 * scale,  5 * scale,  7 * scale};
    for (int i = 0; i < 64; i++) {
      qam64_table[0][i] = mod_64qam[i / 8];
      qam64_table[1][i] = mod_64qam[i % 8];
    }
    for (size_t i = 0; i < in.size(); i++) {
      if (in[i] < 64u)
        out[i] =
            std::complex<float>(qam64_table[0][in[i]], qam64_table[1][in[i]]);
      else {
        std::cout << "Error: No compatible input vector!" << std::endl;
        break;
      }
    }
  } else {
    // Not Supported
    std::cout << "Modulation Type " << type << " not supported!" << std::endl;
  }
  return out;
}

std::vector<std::vector<float>> CommsLib::getSequence(size_t type,
                                                      size_t seq_len) {
  std::vector<std::vector<float>> matrix;

  if (type == STS_SEQ) {
    // STS - 802.11 Short training sequence (one symbol)
    matrix.resize(2);
    const size_t sts_seq_len = 16;

    // Define freq-domain STS according to
    // https://standards.ieee.org/standard/802_11a-1999.html
    std::vector<std::complex<float>> sts_freq(
        Consts::sts_seq, Consts::sts_seq + Consts::kFftSize_80211);

    // Perform ifft-shift on sts_freq
    std::vector<std::complex<float>> sts_freq_shifted;
    sts_freq_shifted.insert(sts_freq_shifted.end(), sts_freq.begin() + 32,
                            sts_freq.end());
    sts_freq_shifted.insert(sts_freq_shifted.end(), sts_freq.begin(),
                            sts_freq.begin() + 32);

    std::vector<std::complex<float>> sts_iq =
        CommsLib::IFFT(sts_freq_shifted, Consts::kFftSize_80211, 1);

    size_t out_seq_len = seq_len > 0 ? seq_len : sts_seq_len;
    size_t frac_seq_len = out_seq_len % sts_seq_len;
    matrix[0].resize(out_seq_len);
    matrix[1].resize(out_seq_len);
    for (size_t i = 0; i < out_seq_len; i++) {
      matrix[0][i] = sts_iq[(i + frac_seq_len) % sts_seq_len].real();
      matrix[1][i] = sts_iq[(i + frac_seq_len) % sts_seq_len].imag();
    }
  } else if (type == LTS_SEQ || type == LTS_SEQ_F) {
    matrix.resize(2);
    const size_t lts_seq_len = Consts::kFftSize_80211;

    std::vector<std::complex<float>> lts_freq(Consts::lts_seq,
                                              Consts::lts_seq + lts_seq_len);

    // Perform ifft-shift on lts_freq
    std::vector<std::complex<float>> lts_freq_shifted;
    lts_freq_shifted.insert(lts_freq_shifted.end(), lts_freq.begin() + 32,
                            lts_freq.end());
    lts_freq_shifted.insert(lts_freq_shifted.end(), lts_freq.begin(),
                            lts_freq.begin() + 32);

    if (type == LTS_SEQ_F) {
      matrix[0].resize(lts_seq_len);
      matrix[1].resize(lts_seq_len);
      for (size_t i = 0; i < lts_seq_len; i++) {
        matrix[0][i] = lts_freq_shifted[i].real();
        matrix[1][i] = lts_freq_shifted[i].imag();
      }
    } else {
      std::vector<std::complex<float>> lts_iq = CommsLib::IFFT(
          lts_freq_shifted, lts_seq_len, 1.f / lts_seq_len, false);

      size_t out_seq_len = seq_len > 0 ? seq_len : lts_seq_len;
      size_t frac_seq_len = out_seq_len % lts_seq_len;
      matrix[0].resize(out_seq_len);
      matrix[1].resize(out_seq_len);
      for (size_t i = 0; i < out_seq_len; i++) {
        matrix[0][i] = lts_iq[(i + frac_seq_len) % lts_seq_len].real();
        matrix[1][i] = lts_iq[(i + frac_seq_len) % lts_seq_len].imag();
      }
    }
  } else if (type == LTE_ZADOFF_CHU || type == LTE_ZADOFF_CHU_F) {
    // https://www.etsi.org/deliver/etsi_ts/136200_136299/136211/10.01.00_60/ts_136211v100100p.pdf
    // ETSI TS 136 211 V10.1.0 (sec. 5.5)
    matrix.resize(2);
    double u = 1;  // Cell ID 1
    double v = 0;
    size_t M = Consts::prime[308];
    for (size_t j = 0; j < 308; j++) {
      if (Consts::prime[j] < seq_len && Consts::prime[j + 1] > seq_len) {
        M = Consts::prime[j];
        break;
      }
    }
    float qh = M * (u + 1) / 31;
    float q = std::floor(qh + 0.5) + v * std::pow(-1, std::floor(2 * qh));
    std::vector<std::complex<float>> zc_freq;
    for (size_t i = 0; i < seq_len; i++) {
      size_t m = i % M;
      float re = std::cos(-M_PI * q * m * (m + 1) / M);
      float im = std::sin(-M_PI * q * m * (m + 1) / M);
      zc_freq.push_back(std::complex<float>(re, im));
    }

    // Fill in center subcarriers
    auto zc_iq_len = (size_t)std::pow(2, std::ceil(std::log2(seq_len)));
    auto zero_sc_len = zc_iq_len - seq_len;
    if (zero_sc_len > 0) {
      std::vector<std::complex<float>> zeros(zero_sc_len / 2,
                                             std::complex<float>(0, 0));
      zc_freq.insert(zc_freq.begin(), zeros.begin(), zeros.end());
      zc_freq.insert(zc_freq.end(), zeros.begin(), zeros.end());
    }

    matrix[0].resize(zc_iq_len);
    matrix[1].resize(zc_iq_len);
    if (type == LTE_ZADOFF_CHU_F) {
      for (size_t i = 0; i < zc_iq_len; i++) {
        matrix[0][i] = zc_freq[i].real();
        matrix[1][i] = zc_freq[i].imag();
      }
    } else {
      std::vector<std::complex<float>> zc_iq =
          CommsLib::IFFT(zc_freq, zc_iq_len, 1.f / zc_iq_len, false);

      for (size_t i = 0; i < zc_iq_len; i++) {
        matrix[0][i] = zc_iq[i].real();
        matrix[1][i] = zc_iq[i].imag();
      }
    }
  } else if (type == GOLD_IFFT) {
    // Gold IFFT Sequence - seq_length=128, cp=0, upsample=1
    matrix.resize(2);
    const size_t gold_seq_len = 128;

    // Use row 52 in gold-127
    std::vector<int> gold_code(Consts::gold_code, Consts::gold_code + 127);

    // Insert 0 at center freq, construct inter-leaved quad code
    gold_code.insert(gold_code.begin() + 63, 0);

    std::vector<std::complex<float>> gold_freq(2 * gold_seq_len);
    for (size_t i = 0; i < gold_seq_len; i++) {
      gold_freq[2 * i] = std::complex<float>(gold_code[i], gold_code[i]);
    }

    // Perform ifft-shift on gold_freq
    std::vector<std::complex<float>> gold_freq_shifted;
    gold_freq_shifted.insert(gold_freq_shifted.end(),
                             gold_freq.begin() + gold_seq_len, gold_freq.end());
    gold_freq_shifted.insert(gold_freq_shifted.end(), gold_freq.begin(),
                             gold_freq.begin() + gold_seq_len);

    std::vector<std::complex<float>> gold_ifft_iq =
        CommsLib::IFFT(gold_freq_shifted, 2 * gold_seq_len, 1);

    matrix[0].resize(gold_seq_len);
    matrix[1].resize(gold_seq_len);
    for (size_t i = 0; i < gold_seq_len; i++) {
      matrix[0][i] = gold_ifft_iq[i].real();
      matrix[1][i] = gold_ifft_iq[i].imag();
    }

  } else if (type == HADAMARD) {
    // Hadamard - using Sylvester's construction for powers of 2.
    matrix.resize(seq_len);
    ;
    if ((seq_len & (seq_len - 1)) == 0) {
      for (size_t i = 0; i < seq_len; i++) {
        matrix[i].resize(seq_len);
        for (size_t j = 0; j < seq_len; j++) matrix[i][j] = hadamard2(i, j);
      }
    }
  }
#if DEBUG_PRINT
  std::cout << "Num elements in first vector: \t " << matrix[0].size()
            << "   Number of rows: " << matrix.size() << std::endl;
  for (int i = 0; i < matrix.size(); i++) {
    for (int j = 0; j < matrix[i].size(); j++) {
      std::cout << "Values[" << i << "][" << j << "]: \t " << matrix[i][j]
                << std::endl;
    }
  }
#endif
  return matrix;
}

/*
int main(int argc, char* argv[])
{
    std::vector<std::vector<double>> sequence;
    int type = CommsLib::LTE_ZADOFF_CHU; //atoi(argv[1]);
    int N
        = 304; //atoi(argv[2]); 	// If Hadamard, possible N: {2, 4, 8, 16, 32, 64}
    sequence = CommsLib::getSequence(type, N);

    std::vector<std::complex<double>> sequence_c;
    for (int i = 0; i < sequence[0].size(); i++) {
        sequence_c.push_back(
            std::complex<double>(sequence[0][i], sequence[1][i]));
    }
    // double peak = CommsLib::findLTS(sequence_c, N);
    // std::cout << "LTS PEAK: " << peak << std::endl;

    return 0;
}
*/
