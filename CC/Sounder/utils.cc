/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------

 utility functions for file and text processing.

---------------------------------------------------------------------
*/

#include "include/utils.h"

int pin_to_core(int core_id) {
  pthread_t current_thread = pthread_self();
  return pin_thread_to_core(core_id, current_thread);
}

int pin_thread_to_core(int core_id, pthread_t& thread_to_pin) {
  int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
  if (core_id < 0 || core_id >= num_cores) return -1;

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);

  return pthread_setaffinity_np(thread_to_pin, sizeof(cpu_set_t), &cpuset);
}

std::vector<size_t> Utils::strToChannels(const std::string& channel) {
  std::vector<size_t> channels;
  if (channel == "A")
    channels = {0};
  else if (channel == "B")
    channels = {1};
  else if (channel == "AB")
    channels = {0, 1};
  else if (channel == "C")
    channels = {2};
  else if (channel == "D")
    channels = {3};
  else if (channel == "CD")
    channels = {2, 3};
  else if (channel == "ABCD")
    channels = {0, 1, 2, 3};
  return (channels);
}

std::vector<std::complex<int16_t>> Utils::cfloat_to_cint16(
    const std::vector<std::complex<float>>& in) {
  size_t len = in.size();
  std::vector<std::complex<int16_t>> out(len, 0);
  for (size_t i = 0; i < len; i++)
    out.at(i) = std::complex<int16_t>((int16_t)(in.at(i).real() * 32768),
                                      (int16_t)(in.at(i).imag() * 32768));
  return out;
}

std::vector<std::complex<int16_t>> Utils::float_to_cint16(
    const std::vector<std::vector<float>>& in) {
  std::vector<std::complex<int16_t>> out;
  // check if input format is valid
  if (in.size() != 2 && in[0].size() != in[1].size()) return out;
  size_t len = in[0].size();
  out.resize(len, 0);
  for (size_t i = 0; i < len; i++)
    out[i] = std::complex<int16_t>((int16_t)(in[0][i] * 32768),
                                   (int16_t)(in[1][i] * 32768));
  return out;
}

std::vector<std::complex<float>> Utils::cint16_to_cfloat(
    const std::vector<std::complex<int16_t>>& in) {
  int len = in.size();
  std::vector<std::complex<float>> out(len);
  for (int i = 0; i < len; i++)
    out[i] =
        std::complex<float>(in[i].real() / 32768.0, in[i].imag() / 32768.0);
  return out;
}

std::vector<std::complex<float>> Utils::uint32tocfloat(
    const std::vector<uint32_t>& in, const std::string& order) {
  int len = in.size();
  std::vector<std::complex<float>> out(len, 0);
  for (size_t i = 0; i < in.size(); i++) {
    int16_t arr_hi_int = (int16_t)(in[i] >> 16);
    int16_t arr_lo_int = (int16_t)(in[i] & 0x0FFFF);

    float arr_hi = (float)arr_hi_int / 32768.0;
    float arr_lo = (float)arr_lo_int / 32768.0;

    if (order == "IQ") {
      std::complex<float> csamp(arr_hi, arr_lo);
      out[i] = csamp;
    } else if (order == "QI") {
      std::complex<float> csamp(arr_lo, arr_hi);
      out[i] = csamp;
    }
  }
  return out;
}

std::vector<uint32_t> Utils::cint16_to_uint32(
    const std::vector<std::complex<int16_t>>& in, bool conj,
    const std::string& order) {
  std::vector<uint32_t> out(in.size(), 0);
  for (size_t i = 0; i < in.size(); i++) {
    uint16_t re = (uint16_t)in[i].real();
    uint16_t im = (uint16_t)(conj ? -in[i].imag() : in[i].imag());
    if (order == "IQ")
      out[i] = (uint32_t)re << 16 | im;
    else if (order == "QI")
      out[i] = (uint32_t)im << 16 | re;
  }
  return out;
}

std::vector<std::vector<size_t>> Utils::loadSlots(
    const std::vector<std::string>& frames, char s) {
  const size_t num_frames = frames.size();
  std::vector<std::vector<size_t>> slot_id(num_frames);

  for (size_t frame_id = 0; frame_id < num_frames; frame_id++) {
    const auto& input_frame = frames.at(frame_id);
    auto& output_frame = slot_id.at(frame_id);

    size_t slot_idx = 0;
    for (const char& id : input_frame) {
      if (id == s) {
        output_frame.emplace_back(slot_idx);
      }
      slot_idx++;
    }
  }
  return slot_id;
}

void Utils::loadDevices(const std::string& filename,
                        std::vector<std::string>& data) {
  std::string line;
  std::ifstream myfile(filename, std::ifstream::in);
  if (myfile.is_open()) {
    size_t num_dev = 0;
    while (getline(myfile, line)) {
      std::string item;
      bool word_found = false;
      for (char const& ch : line) {
        if (!word_found && ch == ' ')
          continue;
        else if (word_found && ch == ' ')
          break;
        else {
          word_found = true;
          item += ch;
        }
      }
      if (item.empty() || item.at(0) == '#') {
        continue;
      }
      data.push_back(item);
      std::cout << item << '\n';
      num_dev++;
    }
    std::cout << "Number of valid devices loaded from " << filename << ": "
              << num_dev << std::endl;
    myfile.close();
  }

  else {
    std::cerr << "Unable to open device file " << filename << std::endl;
    exit(1);
  }
}

void Utils::loadData(const char* filename,
                     std::vector<std::complex<int16_t>>& data, int samples) {
  FILE* fp = fopen(filename, "r");
  data.resize(samples);
  float real, imag;
  for (int i = 0; i < samples; i++) {
    if (2 != fscanf(fp, "%f %f", &real, &imag)) break;
    data[i] =
        std::complex<int16_t>(int16_t(real * 32768), int16_t(imag * 32768));
  }

  fclose(fp);
}

void Utils::loadData(const char* filename, std::vector<unsigned>& data,
                     int samples) {
  FILE* fp = fopen(filename, "r");
  data.resize(samples);
  for (int i = 0; i < samples; i++) {
    if (1 != fscanf(fp, "%u", &data[i])) break;
  }

  fclose(fp);
}

void Utils::loadTDDConfig(const std::string& filename, std::string& jconfig) {
  std::string line;
  std::ifstream configFile(filename);
  if (configFile.is_open()) {
    while (getline(configFile, line)) {
      jconfig += line;
    }
    configFile.close();
  }

  else {
    std::cerr << "Unable to open config file " << filename << std::endl;
    exit(1);
  }
}

std::vector<std::string> Utils::split(const std::string& s, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

void Utils::printVector(std::vector<std::complex<int16_t>>& data) {
  for (size_t i = 0; i < data.size(); i++) {
    std::cout << real(data.at(i)) << " " << imag(data.at(i)) << std::endl;
  }
}
