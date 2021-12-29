/*
 Copyright (c) 2018-2020
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
----------------------------------------------------------------------
 Class to handle writting data to an hdf5 file
---------------------------------------------------------------------
*/
#ifndef SOUNDER_RECORDER_WORKER_H_
#define SOUNDER_RECORDER_WORKER_H_

#include "config.h"
#include "hdf5_lib.h"
#include "receiver.h"

namespace Sounder {
class RecorderWorker {
 public:
  RecorderWorker(Config* in_cfg, size_t antenna_offset, size_t num_antennas);
  ~RecorderWorker();

  void init(void);
  void finalize(void);
  void record(int tid, Packet* pkt);

  inline size_t num_antennas(void) { return num_antennas_; }
  inline size_t antenna_offset(void) { return antenna_offset_; }

 private:
  Config* cfg_;
  H5std_string hdf5_name_;
  Hdf5Lib* hdf5_;
  std::vector<std::string> datasets;

  size_t max_frame_number_;

  size_t antenna_offset_;
  size_t num_antennas_;
};
}; /* End namespace Sounder */

#endif /* SOUNDER_RECORDER_WORKER_H_ */
