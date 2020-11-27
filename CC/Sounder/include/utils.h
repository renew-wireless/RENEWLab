/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
 
---------------------------------------------------------------------
 Utils functions  
---------------------------------------------------------------------
*/

#ifndef UTILS_HEADER
#define UTILS_HEADER

#include <atomic>
#include <chrono>
#include <complex>
#include <condition_variable>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream> // std::ifstream
#include <iostream>
#include <mutex>
#include <pthread.h>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

int pin_thread_to_core(int core_id, pthread_t& thread_to_pin);
int pin_to_core(int core_id);

class Utils {
public:
    Utils();
    ~Utils();

    static std::vector<size_t> strToChannels(const std::string& channel);
    static std::vector<std::complex<int16_t>> double_to_cint16(
        std::vector<std::vector<double>> in);
    static std::vector<std::complex<float>> doubletocfloat(
        std::vector<std::vector<double>> in);
    static std::vector<std::complex<float>> uint32tocfloat(
        std::vector<uint32_t> in, const std::string& order);
    static std::vector<uint32_t> cint16_to_uint32(
        std::vector<std::complex<int16_t>> in, bool conj,
        const std::string& order);
    static std::vector<std::vector<size_t>> loadSymbols(
        std::vector<std::string> frames, char sym);
    static void loadDevices(
        const std::string& filename, std::vector<std::string>& data);
    static void loadData(const char* filename,
        std::vector<std::complex<int16_t>>& data, int samples);
    static void loadData(
        const char* filename, std::vector<unsigned>& data, int samples);
    static void loadTDDConfig(
        const std::string& filename, std::string& jconfig);
    static std::vector<std::string> split(const std::string& s, char delimiter);
    static void printVector(std::vector<std::complex<int16_t>>& data);
};
#endif /* UTILS_HEADER */
