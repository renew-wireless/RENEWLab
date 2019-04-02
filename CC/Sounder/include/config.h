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

#include <algorithm>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <complex.h>
#include <fstream>      // std::ifstream
#include <stdio.h>  /* for fprintf */
#include <unistd.h>
#include "macros.h"
#include "utils.h"
#ifdef JSON
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#endif

class Config
{
public:

    std::string conf;
    bool bsPresent;
    bool clPresent;

    // common features

    double freq;
    double rate;
    int sampsPerSymbol;
    int prefix;
    int postfix;
    int symbolsPerFrame;
    int pilotSymsPerFrame;
    int ulSymsPerFrame;
    int dlSymsPerFrame;
    int fftSize;
    int cpSize;
    std::string pilot_seq;
    std::string beacon_seq;
    
    // BS features
    int nCells;
    std::vector<std::string> bs_sdr_file;
    std::string hub_file;
    std::string ref_sdr;
    std::vector<std::vector<std::string>> bs_sdr_ids;
    std::vector<std::string> hub_ids;
    int beacon_ant;
    bool beamsweep;
    std::vector<int> nBsSdrs;
    std::vector<int> nBsAntennas;
    int bsSdrCh;
    int framePeriod;
    std::vector<std::string> frames;
    std::vector<std::vector<size_t>> pilotSymbols;
    std::vector<std::vector<size_t>> ULSymbols;
    std::vector<std::vector<size_t>> DLSymbols;
    double txgainA;
    double rxgainA;
    double txgainB;
    double rxgainB;

    // Clients features
    int nClSdrs;
    std::vector<std::string> cl_sdr_ids;
    int clSdrCh;
    bool clAgcEn;
    std::string clDataMod;
    std::vector<int> data_ind;
    std::vector<uint32_t> coeffs;
    std::vector<std::complex<int16_t>> coeffs_ci16;
    std::vector<uint32_t> beacon;
    std::vector<std::complex<int16_t>> beacon_ci16;
    std::vector<std::complex<int16_t>> pilot_ci16;
    std::vector<uint32_t> pilot;
    std::vector<std::vector<int>> pilot_sc;
    std::vector<std::vector<double>> pilot_double;
    std::vector<std::vector<std::complex<float>>> txdata;
    std::vector<std::vector<std::complex<float>>> txdata_freq_dom;
    std::vector<std::vector<std::complex<float>>> txdata_time_dom;

    std::vector<std::string> clFrames;
    std::vector<std::vector<size_t>> clPilotSymbols;
    std::vector<std::vector<size_t>> clULSymbols;
    std::vector<std::vector<size_t>> clDLSymbols;
    // TODO clients gain can be set for each separately
    double clTxgainA;
    double clRxgainA;
    double clTxgainB;
    double clRxgainB;
    

    const int maxFrame = 1 << 31;
    const int data_offset = sizeof(int) * 4;
    // header 4 int for: frame_id, subframe_id, cell_id, ant_id
    // ushort for: I/Q samples
    int getPackageLength() { return sizeof(int) * 4 + sizeof(ushort) * sampsPerSymbol * 2; }
    int getNumAntennas(); //{ return nRadios*nChannels; }
    int getClientId(int, int);
    int getUlSFIndex(int, int);
    int getDlSFIndex(int, int);
    bool isPilot(int, int);
    bool isData(int, int);


    Config(std::string);
    ~Config();
};

#endif
