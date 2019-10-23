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
    std::string conf;
    bool bsPresent;
    bool clPresent;

    // common features

    double freq;
    double nco; // baseband frequency controlled by NCO
    double rate;
    double radioRfFreq; // RF frequency set on the radio after NCO adjustments
    double bwFilter;
    int sampsPerSymbol;
    int prefix;
    int postfix;
    int symbolsPerFrame;
    size_t pilotSymsPerFrame;
    size_t ulSymsPerFrame;
    int dlSymsPerFrame;
    int fftSize;
    int cpSize;
    float tx_scale;
    std::string pilot_seq;
    std::string beacon_seq;

    // BS features
    size_t nCells;
    std::vector<std::string> bs_sdr_file;
    std::string hub_file;
    std::string ref_sdr;
    std::vector<std::vector<std::string>> bs_sdr_ids;
    std::vector<std::string> hub_ids;
    std::vector<uint32_t> beacon;
    std::vector<std::complex<int16_t>> beacon_ci16;
    int beaconSize;
    size_t beacon_ant;
    bool beamsweep;
    std::vector<size_t> nBsSdrs;
    std::vector<size_t> nBsAntennas;
    std::string bsChannel;
    std::vector<std::string> frames;
    std::string frame_mode;
    size_t max_frame;
    std::vector<std::vector<size_t>> pilotSymbols;
    std::vector<std::vector<size_t>> ULSymbols;
    std::vector<std::vector<size_t>> DLSymbols;
    double txgain[2];
    double rxgain[2];
    double calTxGain[2];
    bool sampleCalEn;
    bool imbalanceCalEn;
    std::string trace_file;

    // Clients features
    unsigned int nClSdrs;
    std::vector<std::string> cl_sdr_ids;
    int clSdrCh;
    std::string clChannel;
    bool clAgcEn;
    int clAgcGainInit;
    std::string clDataMod;
    std::vector<int> data_ind;
    std::vector<uint32_t> coeffs;
    std::vector<std::complex<int16_t>> coeffs_ci16;
    std::vector<std::complex<int16_t>> pilot_ci16;
    std::vector<uint32_t> pilot;
    std::vector<std::vector<int>> pilot_sc;
    std::vector<std::vector<double>> pilot_double;
    //std::vector<std::vector<std::vector<std::complex<float>>>> txdata;
    std::vector<std::vector<std::complex<float>>> txdata;
    std::vector<std::vector<std::complex<float>>> txdata_freq_dom;
    std::vector<std::vector<std::complex<float>>> txdata_time_dom;

    std::vector<std::string> clFrames;
    std::vector<std::vector<size_t>> clPilotSymbols;
    std::vector<std::vector<size_t>> clULSymbols;
    std::vector<std::vector<size_t>> clDLSymbols;

    std::vector<double> clTxgain_vec[2];
    std::vector<double> clRxgain_vec[2];

    size_t getPackageLength()
    {
        return sizeof(int) * 4 + sizeof(ushort) * (size_t)sampsPerSymbol * 2;
    }
    size_t getNumAntennas();
    int getClientId(int, int);
    int getUlSFIndex(int, int);
    int getDlSFIndex(int, int);
    bool isPilot(int, int);
    bool isData(int, int);
    unsigned getCoreCount();

    std::atomic<bool> running;
    bool core_alloc;
    unsigned int rx_thread_num;
    unsigned int task_thread_num;

    Config(const std::string&);
    ~Config();
};

#endif
