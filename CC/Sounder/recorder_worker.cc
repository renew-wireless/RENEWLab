/*
 Copyright (c) 2018-2022, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 Class to handle writting data to an hdf5 file
---------------------------------------------------------------------
*/

#include "include/recorder_worker.h"

#include "include/logger.h"
#include "include/macros.h"
#include "include/utils.h"

namespace Sounder {

RecorderWorker::RecorderWorker(Config* in_cfg, size_t antenna_offset,
                               size_t num_antennas)
    : cfg_(in_cfg) {
  antenna_offset_ = antenna_offset;
  num_antennas_ = num_antennas;
  unsigned int end_antenna = (this->antenna_offset_ + this->num_antennas_) - 1;

  this->hdf5_name_ = this->cfg_->trace_file();
  size_t found_index = this->hdf5_name_.find_last_of('.');
  std::string append = "_" + std::to_string(this->antenna_offset_) + "_" +
                       std::to_string(end_antenna);
  this->hdf5_name_.insert(found_index, append);
}

RecorderWorker::~RecorderWorker() { this->finalize(); }

void RecorderWorker::init(void) {
  this->hdf5_ = std::make_unique<Hdf5Lib>(this->hdf5_name_, "Data");
  // Write Atrributes
  // ******* COMMON ******** //
  // TX/RX Frequencyfile
  this->hdf5_->write_attribute("FREQ", this->cfg_->freq());

  // BW
  this->hdf5_->write_attribute("RATE", this->cfg_->rate());

  // Number of samples for prefix (padding)
  this->hdf5_->write_attribute("PREFIX_LEN", this->cfg_->prefix());

  // Number of samples for postfix (padding)
  this->hdf5_->write_attribute("POSTFIX_LEN", this->cfg_->postfix());

  // Number of samples on each symbol including prefix and postfix
  this->hdf5_->write_attribute("SLOT_SAMP_LEN", this->cfg_->samps_per_slot());

  // Size of FFT
  this->hdf5_->write_attribute("FFT_SIZE", this->cfg_->fft_size());

  // Number of data subcarriers in ofdm symbols
  this->hdf5_->write_attribute("DATA_SUBCARRIER_NUM",
                               this->cfg_->symbol_data_subcarrier_num());

  // Length of cyclic prefix
  this->hdf5_->write_attribute("CP_LEN", this->cfg_->cp_size());

  // Downlink Pilots Enabled Flag
  this->hdf5_->write_attribute("DL_PILOTS_EN", this->cfg_->dl_pilots_en());

  // Beacon sequence type (string)
  this->hdf5_->write_attribute("BEACON_SEQ_TYPE", this->cfg_->beacon_seq());

  // Pilot sequence type (string)
  this->hdf5_->write_attribute("PILOT_SEQ_TYPE", this->cfg_->pilot_seq());

  // ******* Base Station ******** //
  // Hub IDs (vec of strings)
  this->hdf5_->write_attribute("BS_HUB_ID", this->cfg_->hub_ids());

  // BS SDR IDs
  // *** first, how many boards in each cell? ***
  std::vector<std::string> bs_sdr_num_per_cell(this->cfg_->bs_sdr_ids().size());
  for (size_t i = 0; i < bs_sdr_num_per_cell.size(); ++i) {
    bs_sdr_num_per_cell[i] =
        std::to_string(this->cfg_->bs_sdr_ids().at(i).size());
  }
  this->hdf5_->write_attribute("BS_SDR_NUM_PER_CELL", bs_sdr_num_per_cell);

  // *** second, reshape matrix into vector ***
  std::vector<std::string> bs_sdr_id;
  for (auto&& v : this->cfg_->bs_sdr_ids()) {
    bs_sdr_id.insert(bs_sdr_id.end(), v.begin(), v.end());
  }
  this->hdf5_->write_attribute("BS_SDR_ID", bs_sdr_id);

  // Number of Base Station Cells
  this->hdf5_->write_attribute("BS_NUM_CELLS", this->cfg_->num_cells());

  // How many RF channels per Iris board are enabled ("single" or "dual")
  this->hdf5_->write_attribute("BS_CH_PER_RADIO",
                               this->cfg_->bs_channel().length());

  // Frame schedule (vec of strings for now, this should change to matrix when we go to multi-cell)
  this->hdf5_->write_attribute("BS_FRAME_SCHED", this->cfg_->frames());

  // RX Gain RF channel A
  this->hdf5_->write_attribute("BS_RX_GAIN_A", this->cfg_->rx_gain().at(0));

  // TX Gain RF channel A
  this->hdf5_->write_attribute("BS_TX_GAIN_A", this->cfg_->tx_gain().at(0));

  // RX Gain RF channel B
  this->hdf5_->write_attribute("BS_RX_GAIN_B", this->cfg_->rx_gain().at(1));

  // TX Gain RF channel B
  this->hdf5_->write_attribute("BS_TX_GAIN_B", this->cfg_->tx_gain().at(1));

  // Beamsweep (true or false)
  this->hdf5_->write_attribute("BS_BEAMSWEEP",
                               this->cfg_->beam_sweep() ? 1 : 0);

  // Beacon Antenna
  this->hdf5_->write_attribute("BS_BEACON_ANT", this->cfg_->beacon_ant());

  // Number of antennas on Base Station (per cell)
  std::vector<std::string> bs_ant_num_per_cell(this->cfg_->bs_sdr_ids().size());
  for (size_t i = 0; i < bs_ant_num_per_cell.size(); ++i) {
    bs_ant_num_per_cell[i] =
        std::to_string(this->cfg_->bs_sdr_ids().at(i).size() *
                       this->cfg_->bs_channel().length());
  }
  this->hdf5_->write_attribute("BS_ANT_NUM_PER_CELL", bs_ant_num_per_cell);

  //If the antennas are non consective this will be an issue.
  this->hdf5_->write_attribute("ANT_OFFSET", this->antenna_offset_);
  this->hdf5_->write_attribute("ANT_NUM", this->num_antennas_);
  this->hdf5_->write_attribute("ANT_TOTAL", this->cfg_->getTotNumAntennas());

  // Number of symbols in a frame
  this->hdf5_->write_attribute("BS_FRAME_LEN", this->cfg_->slot_per_frame());

  // Number of uplink symbols per frame
  this->hdf5_->write_attribute("UL_SLOTS", this->cfg_->ul_slot_per_frame());

  // Reciprocal Calibration Mode
  bool reciprocity_cal =
      this->cfg_->internal_measurement() && this->cfg_->ref_node_enable();
  this->hdf5_->write_attribute("RECIPROCAL_CALIB", reciprocity_cal ? 1 : 0);

  // All combinations of TX/RX boards in the base station
  bool full_matrix_meas =
      this->cfg_->internal_measurement() && !this->cfg_->ref_node_enable();
  this->hdf5_->write_attribute("FULL_MATRIX_MEAS", full_matrix_meas ? 1 : 0);

  // ******* Clients ******** //
  // Freq. Domain Pilot symbols
  std::vector<double> split_vec_pilot_f(2 *
                                        this->cfg_->pilot_sym_f().at(0).size());
  for (size_t i = 0; i < this->cfg_->pilot_sym_f().at(0).size(); i++) {
    split_vec_pilot_f[2 * i + 0] = this->cfg_->pilot_sym_f().at(0).at(i);
    split_vec_pilot_f[2 * i + 1] = this->cfg_->pilot_sym_f().at(1).at(i);
  }
  this->hdf5_->write_attribute("OFDM_PILOT_F", split_vec_pilot_f);

  // Time Domain Pilot symbols
  std::vector<double> split_vec_pilot(2 *
                                      this->cfg_->pilot_sym_t().at(0).size());
  for (size_t i = 0; i < this->cfg_->pilot_sym_t().at(0).size(); i++) {
    split_vec_pilot[2 * i + 0] = this->cfg_->pilot_sym_t().at(0).at(i);
    split_vec_pilot[2 * i + 1] = this->cfg_->pilot_sym_t().at(1).at(i);
  }
  this->hdf5_->write_attribute("OFDM_PILOT", split_vec_pilot);

  // Number of Pilots
  this->hdf5_->write_attribute("PILOT_NUM", this->cfg_->pilot_slot_per_frame());

  // Data subcarriers
  if (this->cfg_->data_ind().size() > 0)
    this->hdf5_->write_attribute("OFDM_DATA_SC", this->cfg_->data_ind());

  // Pilot subcarriers (indexes)
  if (this->cfg_->pilot_sc_ind().size() > 0)
    this->hdf5_->write_attribute("OFDM_PILOT_SC", this->cfg_->pilot_sc_ind());
  if (this->cfg_->pilot_sc().size() > 0)
    this->hdf5_->write_attribute("OFDM_PILOT_SC_VALS", this->cfg_->pilot_sc());

  // Number of Client Antennas
  this->hdf5_->write_attribute("CL_NUM", this->cfg_->num_cl_antennas());

  // Data modulation
  this->hdf5_->write_attribute("CL_MODULATION", this->cfg_->cl_data_mod());

  if (this->cfg_->internal_measurement() == false ||
      this->cfg_->num_cl_antennas() > 0) {
    // Client antenna polarization
    this->hdf5_->write_attribute("CL_CH_PER_RADIO", this->cfg_->cl_sdr_ch());

    // Client AGC enable flag
    this->hdf5_->write_attribute("CL_AGC_EN", this->cfg_->cl_agc_en() ? 1 : 0);

    // RX Gain RF channel A
    this->hdf5_->write_attribute("CL_RX_GAIN_A",
                                 this->cfg_->cl_rxgain_vec().at(0));

    // TX Gain RF channel A
    this->hdf5_->write_attribute("CL_TX_GAIN_A",
                                 this->cfg_->cl_txgain_vec().at(0));

    // RX Gain RF channel B
    this->hdf5_->write_attribute("CL_RX_GAIN_B",
                                 this->cfg_->cl_rxgain_vec().at(1));

    // TX Gain RF channel B
    this->hdf5_->write_attribute("CL_TX_GAIN_B",
                                 this->cfg_->cl_txgain_vec().at(1));

    // Client frame schedule (vec of strings)
    this->hdf5_->write_attribute("CL_FRAME_SCHED", this->cfg_->cl_frames());

    // Set of client SDR IDs (vec of strings)
    this->hdf5_->write_attribute("CL_SDR_ID", this->cfg_->cl_sdr_ids());
  }

  if (this->cfg_->ul_data_slot_present()) {
    // Number of frames for UL data recorded in bit source files
    this->hdf5_->write_attribute("UL_DATA_FRAME_NUM",
                                 this->cfg_->ul_data_frame_num());

    // Names of Files including uplink tx frequency-domain data
    if (this->cfg_->ul_tx_fd_data_files().size() > 0) {
      this->hdf5_->write_attribute("TX_FD_DATA_FILENAMES",
                                   this->cfg_->ul_tx_fd_data_files());
    }
  }
  // ********************* //

  // dataset dimension
  hsize_t IQ = 2 * this->cfg_->samps_per_slot();
  std::array<hsize_t, kDsDimsNum> cdims = {
      1, 1, 1, 1, IQ};  // recording chunk size, TODO: optimize size

  if (this->cfg_->bs_rx_thread_num() > 0 &&
      this->cfg_->pilot_slot_per_frame() > 0) {
    // pilots
    std::cout << " HEREEEEEEEEEEEEEE: " << this->cfg_->pilot_slot_per_frame() << std::endl;
    datasets.push_back("Pilot_Samples");
    std::array<hsize_t, kDsDimsNum> dims_pilot = {
        MAX_FRAME_INC, this->cfg_->num_cells(),
        this->cfg_->pilot_slot_per_frame(), this->num_antennas_, IQ};
    this->hdf5_->createDataset(datasets.back(), dims_pilot, cdims);
  }
  if (this->cfg_->noise_slot_per_frame() > 0) {
    // noise
    datasets.push_back("Noise_Samples");
    std::array<hsize_t, kDsDimsNum> dims_noise = {
        MAX_FRAME_INC, this->cfg_->num_cells(),
        this->cfg_->noise_slot_per_frame(), this->num_antennas_, IQ};
    this->hdf5_->createDataset(datasets.back(), dims_noise, cdims);
  }

  if (this->cfg_->bs_rx_thread_num() > 0 &&
      this->cfg_->ul_slot_per_frame() > 0) {
    // UL data
    datasets.push_back("UplinkData");
    std::array<hsize_t, kDsDimsNum> dims_ul_data = {
        MAX_FRAME_INC, this->cfg_->num_cells(), this->cfg_->ul_slot_per_frame(),
        this->num_antennas_, IQ};
    this->hdf5_->createDataset(datasets.back(), dims_ul_data, cdims);
  }

  if (this->cfg_->cl_rx_thread_num() > 0 &&
      cfg_->cl_dl_slots().at(0).empty() == false) {
    // DL
    datasets.push_back("DownlinkData");
    std::array<hsize_t, kDsDimsNum> dims_dl_data = {
        MAX_FRAME_INC, this->cfg_->num_cells(),
        this->cfg_->cl_dl_slots().at(0).size(), this->cfg_->num_cl_antennas(),
        IQ};
    this->hdf5_->createDataset(datasets.back(), dims_dl_data, cdims);
  }

  this->hdf5_->setTargetPrimaryDimSize(MAX_FRAME_INC);
  this->hdf5_->setMaxPrimaryDimSize(cfg_->max_frame());
  this->hdf5_->openDataset();
}

void RecorderWorker::finalize(void) {
  this->hdf5_->closeDataset();
  this->hdf5_->closeFile();
}

void RecorderWorker::record(int tid, Packet* pkt, NodeType node_type) {
  (void)tid;
  /* TODO: remove TEMP check */
  size_t end_antenna = (this->antenna_offset_ + this->num_antennas_) - 1;

  if ((pkt->ant_id < this->antenna_offset_) || (pkt->ant_id > end_antenna)) {
    MLPD_ERROR("Antenna id is not within range of this recorder %d, %zu:%zu",
               pkt->ant_id, this->antenna_offset_, end_antenna);
  }
  assert((pkt->ant_id >= this->antenna_offset_) &&
         (pkt->ant_id <= end_antenna));

  //Generates a ton of messages
  //MLPD_TRACE( "Tid: %d -- frame_id %u, antenna: %u\n", tid, pkt->frame_id, pkt->ant_id);

#if DEBUG_PRINT
  printf(
      "record            frame %d, symbol %d, cell %d, ant %d "
      "samples: %d "
      "%d %d %d %d %d %d %d ....\n",
      pkt->frame_id, pkt->slot_id, pkt->cell_id, pkt->ant_id, pkt->data[1],
      pkt->data[2], pkt->data[3], pkt->data[4], pkt->data[5], pkt->data[6],
      pkt->data[7], pkt->data[8]);
#endif
  hsize_t IQ = 2 * this->cfg_->samps_per_slot();
  if ((this->cfg_->max_frame()) != 0 &&
      (pkt->frame_id > this->cfg_->max_frame())) {
    this->hdf5_->closeDataset();
    MLPD_TRACE("Closing file due to frame id %d : %zu max\n", pkt->frame_id,
               this->cfg_->max_frame());
  } else {
    // Update the max frame number.
    // Note that the 'frame_id' might be out of order.
    this->max_frame_number_ = this->hdf5_->getTargetPrimaryDimSize();
    if (pkt->frame_id >= this->max_frame_number_) {
      // Open the hdf5 file if we haven't.
      this->hdf5_->closeDataset();
      this->hdf5_->openDataset();
      this->hdf5_->setTargetPrimaryDimSize(this->max_frame_number_ +
                                           MAX_FRAME_INC);
    }

    uint32_t antenna_index = pkt->ant_id - this->antenna_offset_;
    std::array<hsize_t, kDsDimsNum> hdfoffset = {pkt->frame_id, pkt->cell_id, 0,
                                                 antenna_index, 0};
    std::array<hsize_t, kDsDimsNum> count = {1, 1, 1, 1, IQ};
    if (this->cfg_->internal_measurement() == true || this->cfg_->dl_pilots_en() == true) {
      if (node_type == kClient) {
        this->hdf5_->extendDataset(std::string("DownlinkData"), pkt->frame_id);
        hdfoffset[kDsDimSymbol] = this->cfg_->getDlSlotIndex(0, pkt->slot_id);
        this->hdf5_->writeDataset(std::string("DownlinkData"), hdfoffset, count,
                                  pkt->data);
      } else {
        this->hdf5_->extendDataset(std::string("Pilot_Samples"), pkt->frame_id);
        hdfoffset[kDsDimSymbol] = pkt->slot_id;
        this->hdf5_->writeDataset(std::string("Pilot_Samples"), hdfoffset,
                                  count, pkt->data);
      }
    } else if (this->cfg_->isPilot(pkt->frame_id, pkt->slot_id) == true) {
      this->hdf5_->extendDataset(std::string("Pilot_Samples"), pkt->frame_id);
      hdfoffset[kDsDimSymbol] =
          this->cfg_->getClientId(pkt->frame_id, pkt->slot_id);
      this->hdf5_->writeDataset(std::string("Pilot_Samples"), hdfoffset, count,
                                pkt->data);
    } else if (this->cfg_->isUlData(pkt->frame_id, pkt->slot_id) == true) {
      this->hdf5_->extendDataset(std::string("UplinkData"), pkt->frame_id);
      hdfoffset[kDsDimSymbol] =
          this->cfg_->getUlSlotIndex(pkt->frame_id, pkt->slot_id);
      this->hdf5_->writeDataset(std::string("UplinkData"), hdfoffset, count,
                                pkt->data);

    } else if (this->cfg_->isDlData(0, pkt->slot_id) == true) {
      this->hdf5_->extendDataset(std::string("DownlinkData"), pkt->frame_id);
      hdfoffset[kDsDimSymbol] = this->cfg_->getDlSlotIndex(0, pkt->slot_id);
      this->hdf5_->writeDataset(std::string("DownlinkData"), hdfoffset, count,
                                pkt->data);
    } else if (this->cfg_->isNoise(pkt->frame_id, pkt->slot_id) == true) {
      this->hdf5_->extendDataset(std::string("Noise_Samples"), pkt->frame_id);
      hdfoffset[kDsDimSymbol] =
          this->cfg_->getNoiseSlotIndex(pkt->frame_id, pkt->slot_id);
      this->hdf5_->writeDataset(std::string("Noise_Samples"), hdfoffset, count,
                                pkt->data);
    }
  } /* End else */
}
};  //End namespace Sounder
