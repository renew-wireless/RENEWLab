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

#include <pthread.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <complex>
#include <condition_variable>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>  // std::ifstream
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

int pin_thread_to_core(int core_id, pthread_t& thread_to_pin);
int pin_to_core(int core_id);

class Utils {
 public:
  Utils();
  ~Utils();

  static std::vector<size_t> strToChannels(const std::string& channel);
  static std::vector<std::complex<float>> cint16_to_cfloat(
      const std::vector<std::complex<int16_t>>& in);
  static std::vector<std::complex<int16_t>> cfloat_to_cint16(
      const std::vector<std::complex<float>>& in);
  static std::vector<std::complex<int16_t>> float_to_cint16(
      const std::vector<std::vector<float>>& in);
  // static std::vector<std::complex<float>> doubletocfloat(
  //    const std::vector<std::vector<double>>& in);
  static std::vector<std::complex<float>> uint32tocfloat(
      const std::vector<uint32_t>& in, const std::string& order);
  static std::vector<uint32_t> cint16_to_uint32(
      const std::vector<std::complex<int16_t>>& in, bool conj,
      const std::string& order);
  static std::vector<std::vector<size_t>> loadSlots(
      const std::vector<std::string>& frames, char sym);
  static void loadDevices(const std::string& filename,
                          std::vector<std::string>& data);
  static void loadData(const char* filename,
                       std::vector<std::complex<int16_t>>& data, int samples);
  static void loadData(const char* filename, std::vector<unsigned>& data,
                       int samples);
  static void loadTDDConfig(const std::string& filename, std::string& jconfig);
  static std::vector<std::string> split(const std::string& s, char delimiter);
  static void printVector(std::vector<std::complex<int16_t>>& data);
  static void WriteVector(const std::string filename, const std::string desc,
                          const std::vector<int> vec_data);
  static std::vector<int> ReadVector(const std::string filename,
                                     const bool skip_line);
};
#endif /* UTILS_HEADER */
