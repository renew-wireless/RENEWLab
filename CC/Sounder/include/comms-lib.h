/*

 Communications Library:
   a) Generate pilot/preamble sequences
   b) OFDM modulation

---------------------------------------------------------------------
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
	    Oscar Bejarano: obejarano@rice.edu
---------------------------------------------------------------------
*/

#include "fft.h"
#include <algorithm>
#include <complex.h>
#include <fstream> // std::ifstream
#include <iostream>
#include <math.h>
#include <stdio.h> /* for fprintf */
#include <stdlib.h>
#include <string.h> /* for memcpy */
#include <thread>
#include <unistd.h>
#include <vector>

static inline double computeAbs(std::complex<double> x) { return std::abs(x); }

//template <typename T>
//static inline T computeAbs(std::complex<T> x) { return std::abs(x); }
static inline double computePower(std::complex<double> x)
{
    return std::pow(std::abs(x), 2);
}
static inline double computeSquare(double x) { return x * x; }

class CommsLib {
public:
    enum SequenceType { STS_SEQ, LTS_SEQ, LTE_ZADOFF_CHU, GOLD_IFFT, HADAMARD };

    enum ModulationOrder { QPSK = 2, QAM16 = 4, QAM64 = 6 };

    CommsLib(std::string);
    ~CommsLib();

    static std::vector<std::vector<double>> getSequence(
        size_t type, size_t seq_len = 0);
    static std::vector<std::complex<float>> modulate(std::vector<int>, int);
    static std::vector<int> getDataSc(int fftSize);
    static std::vector<int> getNullSc(int fftSize);
    static std::vector<std::vector<int>> getPilotSc(int fftSize);
    static std::vector<std::complex<float>> FFT(
        std::vector<std::complex<float>>, int);
    static std::vector<std::complex<float>> IFFT(
        std::vector<std::complex<float>>, int, float scale = 0.5,
        bool normalize = true);

    static int findLTS(const std::vector<std::complex<double>>& iq, int seqLen);
    static size_t find_pilot_seq(std::vector<std::complex<double>> iq,
        std::vector<std::complex<double>> pilot, size_t seqLen);
    template <typename T>
    //static std::vector<T> convolve(std::vector<T> const& f, std::vector<T> const& g);
    static std::vector<T> convolve(
        std::vector<T> const& f, std::vector<T> const& g)
    {
        /* Convolution of two vectors
         * Source:
         * https://stackoverflow.com/questions/24518989/how-to-perform-1-dimensional-valid-convolution
         */
        int const nf = f.size();
        int const ng = g.size();
        int const n = nf + ng - 1;
        std::vector<T> out(n, 0);
        for (auto i(0); i < n; ++i) {
            int const jmn = (i >= ng - 1) ? i - (ng - 1) : 0;
            int const jmx = (i < nf - 1) ? i : nf - 1;
            for (auto j(jmn); j <= jmx; ++j) {
                out[i] += f[j] * g[i - j];
            }
        }
        return out;
    }
    static float find_max_abs(std::vector<std::complex<float>> in);
    static std::vector<std::complex<double>> csign(
        std::vector<std::complex<double>> iq);
    static inline int hadamard2(int i, int j)
    {
        return (__builtin_parity(i & j) != 0 ? -1 : 1);
    }
    static std::vector<float> magnitudeFFT(
        std::vector<std::complex<float>> const&, std::vector<float> const&,
        size_t);
    static std::vector<float> hannWindowFunction(size_t);
    static double windowFunctionPower(std::vector<float> const&);
    template <typename T>
    static T findTone(
        std::vector<T> const&, double, double, size_t, const size_t delta = 10);
    static float measureTone(std::vector<std::complex<float>> const&,
        std::vector<float> const&, double, double, size_t,
        const size_t delta = 10);

    // Functions using AVX
    static int find_beacon(const std::vector<std::complex<double>>& iq);
    static int find_beacon_avx(const std::vector<std::complex<float>>& iq,
        const std::vector<std::complex<float>>& seq);
    static std::vector<float> correlate_avx_s(
        std::vector<float> const& f, std::vector<float> const& g);
    static std::vector<int16_t> correlate_avx_si(
        std::vector<int16_t> const& f, std::vector<int16_t> const& g);
    static std::vector<float> abs2_avx(
        std::vector<std::complex<float>> const& f);
    static std::vector<int32_t> abs2_avx(
        std::vector<std::complex<int16_t>> const& f);
    static std::vector<std::complex<float>> auto_corr_mult_avx(
        std::vector<std::complex<float>> const& f, const int dly,
        const bool conj = true);
    static std::vector<std::complex<int16_t>> auto_corr_mult_avx(
        std::vector<std::complex<int16_t>> const& f, const int dly,
        const bool conj = true);
    static std::vector<std::complex<float>> correlate_avx(
        std::vector<std::complex<float>> const& f,
        std::vector<std::complex<float>> const& g);
    static std::vector<std::complex<float>> complex_mult_avx(
        std::vector<std::complex<float>> const& f,
        std::vector<std::complex<float>> const& g, const bool conj);
    static std::vector<std::complex<int16_t>> complex_mult_avx(
        std::vector<std::complex<int16_t>> const& f,
        std::vector<std::complex<int16_t>> const& g, const bool conj);
    static std::vector<std::complex<int16_t>> correlate_avx(
        std::vector<std::complex<int16_t>> const& f,
        std::vector<std::complex<int16_t>> const& g);
    //private:
    //    static inline float** init_qpsk();
    //    static inline float** init_qam16();
    //    static inline float** init_qam64();
};
