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
#include <cstring>
#include <fstream> // std::ifstream
#include <iostream>
#include <math.h>
#include <stdio.h> /* for fprintf */
#include <stdlib.h>
#include <string.h> /* for memcpy */
#include <thread>
#include <unistd.h>
#include <vector>

static constexpr size_t kPilotSubcarrierSpacing = 12;
static constexpr size_t kDefaultPilotScOffset = 6;

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
    enum SequenceType {
        STS_SEQ,
        LTS_SEQ,
        LTS_SEQ_F,
        LTE_ZADOFF_CHU,
        LTE_ZADOFF_CHU_F,
        GOLD_IFFT,
        HADAMARD
    };

    enum ModulationOrder { QPSK = 2, QAM16 = 4, QAM64 = 6 };

    CommsLib(std::string);
    ~CommsLib();

    static std::vector<std::vector<float>> getSequence(
        size_t type, size_t seq_len = 0);
    static std::vector<std::complex<float>> modulate(std::vector<uint8_t>, int);
    static std::vector<size_t> getDataSc(size_t fftSize, size_t DataScNum,
        size_t PilotScOffset = kDefaultPilotScOffset);
    static std::vector<size_t> getNullSc(size_t fftSize, size_t DataScNum);
    static std::vector<std::complex<float>> getPilotScValue(size_t fftSize,
        size_t DataScNum, size_t PilotScOffset = kDefaultPilotScOffset);
    static std::vector<size_t> getPilotScIndex(size_t fftSize, size_t DataScNum,
        size_t PilotScOffset = kDefaultPilotScOffset);
    static std::vector<std::complex<float>> FFT(
        const std::vector<std::complex<float>>&, int);
    static std::vector<std::complex<float>> IFFT(
        const std::vector<std::complex<float>>&, int, float scale = 0.5,
        bool normalize = true);

    static int findLTS(const std::vector<std::complex<float>>& iq, int seqLen);
    static size_t find_pilot_seq(const std::vector<std::complex<float>>& iq,
        const std::vector<std::complex<float>>& pilot, size_t seqLen);
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
    static float find_max_abs(const std::vector<std::complex<float>>& in);
    static std::vector<std::complex<float>> csign(
        const std::vector<std::complex<float>>& iq);
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
    static int find_beacon(const std::vector<std::complex<float>>& iq);
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
    static std::vector<std::complex<int16_t>> correlate_avx(
        std::vector<std::complex<int16_t>> const& f,
        std::vector<std::complex<int16_t>> const& g);
    static std::vector<std::complex<float>> complex_mult_avx(
        std::vector<std::complex<float>> const& f,
        std::vector<std::complex<float>> const& g, const bool conj);
    static std::vector<std::complex<int16_t>> complex_mult_avx(
        std::vector<std::complex<int16_t>> const& f,
        std::vector<std::complex<int16_t>> const& g, const bool conj);
    //private:
    //    static inline float** init_qpsk();
    //    static inline float** init_qam16();
    //    static inline float** init_qam64();
    const std::complex<float> sts_seq[64] = { { 0, 0 }, { 0, 0 }, { 0, 0 },
        { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 1, 1 }, { 0, 0 },
        { 0, 0 }, { 0, 0 }, { -1, -1 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 1, 1 },
        { 0, 0 }, { 0, 0 }, { 0, 0 }, { -1, -1 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
        { -1, -1 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 1, 1 }, { 0, 0 }, { 0, 0 },
        { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -1, -1 }, { 0, 0 },
        { 0, 0 }, { 0, 0 }, { -1, -1 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 1, 1 },
        { 0, 0 }, { 0, 0 }, { 0, 0 }, { 1, 1 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
        { 1, 1 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 1, 1 }, { 0, 0 }, { 0, 0 },
        { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } };
    const std::complex<float> lts_seq[64] = { { 0, 0 }, { 0, 0 }, { 0, 0 },
        { 0, 0 }, { 0, 0 }, { 0, 0 }, { 1, 0 }, { 1, 0 }, { -1, 0 }, { -1, 0 },
        { 1, 0 }, { 1, 0 }, { -1, 0 }, { 1, 0 }, { -1, 0 }, { 1, 0 }, { 1, 0 },
        { 1, 0 }, { 1, 0 }, { 1, 0 }, { 1, 0 }, { -1, 0 }, { -1, 0 }, { 1, 0 },
        { 1, 0 }, { -1, 0 }, { 1, 0 }, { -1, 0 }, { 1, 0 }, { 1, 0 }, { 1, 0 },
        { 1, 0 }, { 0, 0 }, { 1, 0 }, { -1, 0 }, { -1, 0 }, { 1, 0 }, { 1, 0 },
        { -1, 0 }, { 1, 0 }, { -1, 0 }, { 1, 0 }, { -1, 0 }, { -1, 0 },
        { -1, 0 }, { -1, 0 }, { -1, 0 }, { 1, 0 }, { 1, 0 }, { -1, 0 },
        { -1, 0 }, { 1, 0 }, { -1, 0 }, { 1, 0 }, { -1, 0 }, { 1, 0 }, { 1, 0 },
        { 1, 0 }, { 1, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } };
    const size_t prime[309] = { 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41,
        43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109, 113,
        127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193,
        197, 199, 211, 223, 227, 229, 233, 239, 241, 251, 257, 263, 269, 271,
        277, 281, 283, 293, 307, 311, 313, 317, 331, 337, 347, 349, 353, 359,
        367, 373, 379, 383, 389, 397, 401, 409, 419, 421, 431, 433, 439, 443,
        449, 457, 461, 463, 467, 479, 487, 491, 499, 503, 509, 521, 523, 541,
        547, 557, 563, 569, 571, 577, 587, 593, 599, 601, 607, 613, 617, 619,
        631, 641, 643, 647, 653, 659, 661, 673, 677, 683, 691, 701, 709, 719,
        727, 733, 739, 743, 751, 757, 761, 769, 773, 787, 797, 809, 811, 821,
        823, 827, 829, 839, 853, 857, 859, 863, 877, 881, 883, 887, 907, 911,
        919, 929, 937, 941, 947, 953, 967, 971, 977, 983, 991, 997, 1009, 1013,
        1019, 1021, 1031, 1033, 1039, 1049, 1051, 1061, 1063, 1069, 1087, 1091,
        1093, 1097, 1103, 1109, 1117, 1123, 1129, 1151, 1153, 1163, 1171, 1181,
        1187, 1193, 1201, 1213, 1217, 1223, 1229, 1231, 1237, 1249, 1259, 1277,
        1279, 1283, 1289, 1291, 1297, 1301, 1303, 1307, 1319, 1321, 1327, 1361,
        1367, 1373, 1381, 1399, 1409, 1423, 1427, 1429, 1433, 1439, 1447, 1451,
        1453, 1459, 1471, 1481, 1483, 1487, 1489, 1493, 1499, 1511, 1523, 1531,
        1543, 1549, 1553, 1559, 1567, 1571, 1579, 1583, 1597, 1601, 1607, 1609,
        1613, 1619, 1621, 1627, 1637, 1657, 1663, 1667, 1669, 1693, 1697, 1699,
        1709, 1721, 1723, 1733, 1741, 1747, 1753, 1759, 1777, 1783, 1787, 1789,
        1801, 1811, 1823, 1831, 1847, 1861, 1867, 1871, 1873, 1877, 1879, 1889,
        1901, 1907, 1913, 1931, 1933, 1949, 1951, 1973, 1979, 1987, 1993, 1997,
        1999, 2003, 2011, 2017, 2027, 2029, 2039 };
    const int gold_code[127] = { 1, -1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, 1, 1, -1, 1, 1, 1, -1, 1, -1, -1, -1, 1, -1, -1, -1, -1, -1,
        1, -1, 1, -1, -1, 1, -1, -1, 1, 1, 1, -1, -1, 1, 1, -1, 1, 1, 1, 1, 1,
        -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, 1, 1, -1, 1, -1, -1, 1, 1, 1, -1, 1,
        1, -1, -1, -1, 1, -1, -1, 1, -1, -1, -1, -1, -1, -1, -1, 1, -1, -1, 1,
        1, -1, -1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1,
        -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, -1, -1 };
};
