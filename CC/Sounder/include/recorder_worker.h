/*
 Copyright (c) 2018-2020
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 
----------------------------------------------------------------------
 Class to handle writting data to an hdf5 file
---------------------------------------------------------------------
*/
#ifndef SOUDER_RECORDER_WORKER_H_
#define SOUDER_RECORDER_WORKER_H_

#include "H5Cpp.h"
#include "config.h"
#include "receiver.h"

namespace Sounder {
class RecorderWorker {
public:
    RecorderWorker(Config* in_cfg, size_t antenna_offset, size_t num_antennas);
    ~RecorderWorker();

    void init(void);
    void finalize(void);
    herr_t record(int tid, Package* pkg);

    inline size_t num_antennas(void) { return num_antennas_; }
    inline size_t antenna_offset(void) { return antenna_offset_; }

private:
    // pilot dataset size increment
    static const int kConfigPilotExtentStep;
    // data dataset size increment
    static const int kConfigDataExtentStep;

    void gc(void);
    herr_t initHDF5();
    void openHDF5();
    void closeHDF5();
    void finishHDF5();

    Config* cfg_;
    H5std_string hdf5_name_;

    H5::H5File* file_;
    // Group* group;
    H5::DSetCreatPropList pilot_prop_;
    H5::DSetCreatPropList noise_prop_;
    H5::DSetCreatPropList data_prop_;

    H5::DataSet* pilot_dataset_;
    H5::DataSet* noise_dataset_;
    H5::DataSet* data_dataset_;

    size_t frame_number_pilot_;
    size_t frame_number_noise_;
    size_t frame_number_data_;

    size_t max_frame_number_;

    size_t antenna_offset_;
    size_t num_antennas_;
};
}; /* End namespace Sounder */

#endif /* SOUDER_RECORDER_WORKER_H_ */
