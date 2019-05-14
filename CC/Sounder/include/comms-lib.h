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


#include <algorithm>
#include <iostream>
#include <complex.h>
#include <stdio.h>  /* for fprintf */
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <fstream>      // std::ifstream
#include <math.h>
#include <string.h> /* for memcpy */
#include "fft.h"

class CommsLib
{
public:

    enum SequenceType
    {
	STS_SEQ,
	LTS_SEQ,
	LTE_ZADOFF_CHU,
	GOLD_IFFT,
	HADAMARD	
    };

    enum ModulationOrder
    {
        QPSK = 2,
        QAM16 = 4,
        QAM64 = 6
    };

    CommsLib(std::string);
    ~CommsLib();

    static std::vector<std::vector<double> > getSequence(int N, int type);
    static std::vector<std::complex<float>> modulate(std::vector<int>, int);
    static std::vector<int> getDataSc(int fftSize);
    static std::vector<int> getNullSc(int fftSize);
    static std::vector<std::vector<int>> getPilotSc(int fftSize);
    static std::vector<std::complex<float>> FFT(std::vector<std::complex<float>>, int);
    static std::vector<std::complex<float>> IFFT(std::vector<std::complex<float>>, int);

    static double findLTS(std::vector<std::complex<double>> iq);
    static std::vector<double> convolve(std::vector<std::complex<double>> const &f, std::vector<std::complex<double>> const &g);
    static std::vector<std::complex<double>> csign(std::vector<std::complex<double>> iq);
    static void meshgrid(std::vector<double> x_in, std::vector<double> y_in, std::vector<std::vector<double>> &x, std::vector<std::vector<double>> &y);
//private:
//    static inline float** init_qpsk();
//    static inline float** init_qam16();
//    static inline float** init_qam64();

};
