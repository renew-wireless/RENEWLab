/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 Record received frames from massive-mimo base station in HDF5 format
---------------------------------------------------------------------
*/

#include "include/recorder.h"
#include "include/logger.h"
#include "include/macros.h"
#include "include/signalHandler.hpp"
#include "include/utils.h"

// buffer length of each rx thread
const int Recorder::kSampleBufferFrameNum = 80;
// dequeue bulk size, used to reduce the overhead of dequeue in main thread
const int Recorder::KDequeueBulkSize = 5;
// pilot dataset size increment
const int Recorder::kConfigPilotExtentStep = 400;
// data dataset size increment
const int Recorder::kConfigDataExtentStep = 400;

#if (DEBUG_PRINT)
const int kDsSim = 5;
#endif

Recorder::Recorder(Config* in_cfg)
{
    cfg_ = in_cfg;
    file_ = nullptr;
    pilot_dataset_ = nullptr;
    data_dataset_ = nullptr;
    size_t rx_thread_num = cfg_->rx_thread_num();
    size_t task_thread_num = cfg_->task_thread_num();
    size_t ant_per_rx_thread
        = cfg_->bs_present() ? cfg_->getTotNumAntennas() / rx_thread_num : 1;
    rx_thread_buff_size_
        = kSampleBufferFrameNum * cfg_->symbols_per_frame() * ant_per_rx_thread;

    task_queue_
        = moodycamel::ConcurrentQueue<Event_data>(rx_thread_buff_size_ * 36);
    message_queue_
        = moodycamel::ConcurrentQueue<Event_data>(rx_thread_buff_size_ * 36);

    MLPD_TRACE("Recorder construction - rx thread: %zu, task tread %zu, chunk "
               "size: %zu\n",
        rx_thread_num, task_thread_num, rx_thread_buff_size_);

    if (rx_thread_num > 0) {
        // initialize rx buffers
        rx_buffer_ = new SampleBuffer[rx_thread_num];
        size_t intsize = sizeof(std::atomic_int);
        size_t arraysize = (rx_thread_buff_size_ + intsize - 1) / intsize;
        size_t packageLength = sizeof(Package) + cfg_->getPackageDataLength();
        for (size_t i = 0; i < rx_thread_num; i++) {
            rx_buffer_[i].buffer.resize(rx_thread_buff_size_ * packageLength);
            rx_buffer_[i].pkg_buf_inuse = new std::atomic_int[arraysize];
            std::fill_n(rx_buffer_[i].pkg_buf_inuse, arraysize, 0);
        }
    }

    // Receiver object will be used for both BS and clients
    try {
        receiver_.reset(new Receiver(rx_thread_num, cfg_, &message_queue_));
    } catch (std::exception& e) {
        std::cout << e.what() << '\n';
        gc();
        throw runtime_error("Error Setting up the Receiver");
    }

    if (task_thread_num > 0) {
        pthread_attr_t detached_attr;
        pthread_attr_init(&detached_attr);
        pthread_attr_setdetachstate(&detached_attr, PTHREAD_CREATE_DETACHED);

        for (size_t i = 0; i < task_thread_num; i++) {
            EventHandlerContext* context = new EventHandlerContext;
            pthread_t task_thread;
            context->obj_ptr = this;
            context->id = i;
            MLPD_TRACE("Launching task thread with id: %zu\n", i);
            if (pthread_create(&task_thread, &detached_attr,
                    Recorder::taskThread_launch, context)
                != 0) {
                delete context;
                gc();
                throw runtime_error("Task thread create failed");
            }
        }
    }
}

void Recorder::gc(void)
{
    MLPD_TRACE("Garbage collect\n");
    this->receiver_.reset();
    if (this->cfg_->rx_thread_num() > 0) {
        for (size_t i = 0; i < this->cfg_->rx_thread_num(); i++) {
            delete[] this->rx_buffer_[i].pkg_buf_inuse;
        }
        delete[] this->rx_buffer_;
    }

    if (this->pilot_dataset_ != nullptr) {
        MLPD_TRACE("Pilot Dataset exists during garbage collection\n");
        this->pilot_dataset_->close();
        delete this->pilot_dataset_;
        this->pilot_dataset_ = nullptr;
    }

    if (this->data_dataset_ != nullptr) {
        MLPD_TRACE("Data dataset exists during garbage collection\n");
        this->data_dataset_->close();
        delete this->data_dataset_;
        this->data_dataset_ = nullptr;
    }

    if (this->file_ != nullptr) {
        MLPD_TRACE("File exists exists during garbage collection\n");
        this->file_->close();
        delete this->file_;
        this->file_ = nullptr;
    }
}

void Recorder::finishHDF5()
{
    MLPD_TRACE("Finish HD5F file\n");
    delete this->file_;
    this->file_ = nullptr;
}

static void write_attribute(H5::Group& g, const char name[], double val)
{
    hsize_t dims[] = { 1 };
    H5::DataSpace attr_ds = H5::DataSpace(1, dims);
    H5::Attribute att
        = g.createAttribute(name, H5::PredType::NATIVE_DOUBLE, attr_ds);
    att.write(H5::PredType::NATIVE_DOUBLE, &val);
}

static void write_attribute(
    H5::Group& g, const char name[], const std::vector<double>& val)
{
    size_t size = val.size();
    hsize_t dims[] = { size };
    H5::DataSpace attr_ds = H5::DataSpace(1, dims);
    H5::Attribute att
        = g.createAttribute(name, H5::PredType::NATIVE_DOUBLE, attr_ds);
    att.write(H5::PredType::NATIVE_DOUBLE, &val[0]);
}

static void write_attribute(H5::Group& g, const char name[],
    const std::vector<std::complex<float>>& val)
{
    size_t size = val.size();
    hsize_t dims[] = { 2 * size };
    H5::DataSpace attr_ds = H5::DataSpace(1, dims);
    H5::Attribute att
        = g.createAttribute(name, H5::PredType::NATIVE_DOUBLE, attr_ds);
    double val_pair[2 * size];
    for (size_t j = 0; j < size; j++) {
        val_pair[2 * j + 0] = std::real(val[j]);
        val_pair[2 * j + 1] = std::imag(val[j]);
    }
    att.write(H5::PredType::NATIVE_DOUBLE, &val_pair[0]);
}

static void write_attribute(H5::Group& g, const char name[], size_t val)
{
    hsize_t dims[] = { 1 };
    H5::DataSpace attr_ds = H5::DataSpace(1, dims);
    H5::Attribute att
        = g.createAttribute(name, H5::PredType::STD_U32BE, attr_ds);
    att.write(H5::PredType::NATIVE_UINT, &val);
}

static void write_attribute(H5::Group& g, const char name[], int val)
{
    hsize_t dims[] = { 1 };
    H5::DataSpace attr_ds = H5::DataSpace(1, dims);
    H5::Attribute att
        = g.createAttribute(name, H5::PredType::STD_I32BE, attr_ds);
    att.write(H5::PredType::NATIVE_INT, &val);
}

static void write_attribute(
    H5::Group& g, const char name[], const std::vector<int>& val)
{
    size_t size = val.size();
    hsize_t dims[] = { size };
    H5::DataSpace attr_ds = H5::DataSpace(1, dims);
    H5::Attribute att
        = g.createAttribute(name, H5::PredType::STD_I32BE, attr_ds);
    att.write(H5::PredType::NATIVE_INT, &val[0]);
}

static void write_attribute(
    H5::Group& g, const char name[], const std::string& val)
{
    hsize_t dims[] = { 1 };
    H5::DataSpace attr_ds = H5::DataSpace(1, dims);
    H5::StrType strdatatype(
        H5::PredType::C_S1, H5T_VARIABLE); // of variable length characters
    H5::Attribute att = g.createAttribute(name, strdatatype, attr_ds);
    att.write(strdatatype, val);
}

static void write_attribute(
    H5::Group& g, const char name[], const std::vector<std::string>& val)
{
    if (val.empty())
        return;
    size_t size = val.size();
    H5::StrType strdatatype(
        H5::PredType::C_S1, H5T_VARIABLE); // of variable length characters
    hsize_t dims[] = { size };
    H5::DataSpace attr_ds = H5::DataSpace(1, dims);
    H5::Attribute att = g.createAttribute(name, strdatatype, attr_ds);
    const char* cStrArray[size];

    for (size_t i = 0; i < size; ++i)
        cStrArray[i] = val[i].c_str();
    att.write(strdatatype, cStrArray);
}

enum {
    DS_FRAME_NUMBER,
    DS_NCELLS,
    DS_SYMS_PER_FRAME,
    DS_NANTENNAS,
    DS_PKG_DATA_LEN,
    DS_DIM
};
typedef hsize_t DataspaceIndex[DS_DIM];

herr_t Recorder::initHDF5(const std::string& hdf5)
{
    MLPD_INFO("Init HD5F file: %s\n", hdf5.c_str());
    this->hdf5_name_ = hdf5;

    // dataset dimension
    hsize_t IQ = 2 * this->cfg_->samps_per_symbol();
    DataspaceIndex cdims
        = { 1, 1, 1, 1, IQ }; // pilot chunk size, TODO: optimize size
    this->frame_number_pilot_ = MAX_FRAME_INC; //this->cfg_->maxFrame;
    DataspaceIndex dims_pilot = { this->frame_number_pilot_,
        this->cfg_->num_cells(), this->cfg_->pilot_syms_per_frame(),
        this->cfg_->getMaxNumAntennas(), IQ };
    DataspaceIndex max_dims_pilot = { H5S_UNLIMITED, this->cfg_->num_cells(),
        this->cfg_->pilot_syms_per_frame(), this->cfg_->getMaxNumAntennas(),
        IQ };

    this->frame_number_data_ = MAX_FRAME_INC; //this->cfg_->maxFrame;
    DataspaceIndex dims_data = { this->frame_number_data_,
        this->cfg_->num_cells(), this->cfg_->ul_syms_per_frame(),
        this->cfg_->getMaxNumAntennas(), IQ };
    DataspaceIndex max_dims_data = { H5S_UNLIMITED, this->cfg_->num_cells(),
        this->cfg_->ul_syms_per_frame(), this->cfg_->getMaxNumAntennas(), IQ };

    try {
        H5::Exception::dontPrint();

        this->file_ = new H5::H5File(this->hdf5_name_, H5F_ACC_TRUNC);
        auto mainGroup = this->file_->createGroup("/Data");
        this->pilot_prop_.setChunk(DS_DIM, cdims);

        H5::DataSpace pilot_dataspace(DS_DIM, dims_pilot, max_dims_pilot);
        this->file_->createDataSet("/Data/Pilot_Samples",
            H5::PredType::STD_I16BE, pilot_dataspace, this->pilot_prop_);
        pilot_dataspace.close();

        // ******* COMMON ******** //
        // TX/RX Frequency
        write_attribute(mainGroup, "FREQ", this->cfg_->freq());

        // BW
        write_attribute(mainGroup, "RATE", this->cfg_->rate());

        // Number of samples on each symbol (excluding prefix/postfix)
        write_attribute(
            mainGroup, "SYMBOL_LEN_NO_PAD", this->cfg_->subframe_size());

        // Number of samples for prefix (padding)
        write_attribute(mainGroup, "PREFIX_LEN", this->cfg_->prefix());

        // Number of samples for postfix (padding)
        write_attribute(mainGroup, "POSTFIX_LEN", this->cfg_->postfix());

        // Number of samples on each symbol including prefix and postfix
        write_attribute(
            mainGroup, "SYMBOL_LEN", this->cfg_->samps_per_symbol());

        // Size of FFT
        write_attribute(mainGroup, "FFT_SIZE", this->cfg_->fft_size());

        // Length of cyclic prefix
        write_attribute(mainGroup, "CP_LEN", this->cfg_->cp_size());

        // Beacon sequence type (string)
        write_attribute(mainGroup, "BEACON_SEQ_TYPE", this->cfg_->beacon_seq());

        // Pilot sequence type (string)
        write_attribute(mainGroup, "PILOT_SEQ_TYPE", this->cfg_->pilot_seq());

        // ******* Base Station ******** //

        // Hub IDs (vec of strings)
        write_attribute(mainGroup, "BS_HUB_ID", this->cfg_->hub_ids());

        // BS SDR IDs
        // *** first, how many boards in each cell? ***
        std::vector<std::string> bs_sdr_num_per_cell(
            this->cfg_->bs_sdr_ids().size());
        for (size_t i = 0; i < bs_sdr_num_per_cell.size(); ++i) {
            bs_sdr_num_per_cell[i]
                = std::to_string(this->cfg_->bs_sdr_ids().at(i).size());
        }
        write_attribute(mainGroup, "BS_SDR_NUM_PER_CELL", bs_sdr_num_per_cell);

        // *** second, reshape matrix into vector ***
        std::vector<std::string> bs_sdr_id;
        for (auto&& v : this->cfg_->bs_sdr_ids()) {
            bs_sdr_id.insert(bs_sdr_id.end(), v.begin(), v.end());
        }
        write_attribute(mainGroup, "BS_SDR_ID", bs_sdr_id);

        // Number of Base Station Cells
        write_attribute(
            mainGroup, "BS_NUM_CELLS", (int)this->cfg_->num_cells());

        // How many RF channels per Iris board are enabled ("single" or "dual")
        write_attribute(mainGroup, "BS_CH_PER_RADIO",
            (int)this->cfg_->bs_channel().length());

        // Frame schedule (vec of strings for now, this should change to matrix when we go to multi-cell)
        write_attribute(mainGroup, "BS_FRAME_SCHED", this->cfg_->frames());

        // RX Gain RF channel A
        write_attribute(mainGroup, "BS_RX_GAIN_A", this->cfg_->rx_gain().at(0));

        // TX Gain RF channel A
        write_attribute(mainGroup, "BS_TX_GAIN_A", this->cfg_->tx_gain().at(0));

        // RX Gain RF channel B
        write_attribute(mainGroup, "BS_RX_GAIN_B", this->cfg_->rx_gain().at(1));

        // TX Gain RF channel B
        write_attribute(mainGroup, "BS_TX_GAIN_B", this->cfg_->tx_gain().at(1));

        // Beamsweep (true or false)
        write_attribute(
            mainGroup, "BS_BEAMSWEEP", this->cfg_->beam_sweep() ? 1 : 0);

        // Beacon Antenna
        write_attribute(
            mainGroup, "BS_BEACON_ANT", (int)this->cfg_->beacon_ant());

        // Number of antennas on Base Station (per cell)
        std::vector<std::string> bs_ant_num_per_cell(
            this->cfg_->bs_sdr_ids().size());
        for (size_t i = 0; i < bs_ant_num_per_cell.size(); ++i) {
            bs_ant_num_per_cell[i]
                = std::to_string(this->cfg_->bs_sdr_ids().at(i).size()
                    * (int)this->cfg_->bs_channel().length());
        }
        write_attribute(mainGroup, "BS_ANT_NUM_PER_CELL", bs_ant_num_per_cell);

        // Number of symbols in a frame
        write_attribute(
            mainGroup, "BS_FRAME_LEN", this->cfg_->symbols_per_frame());

        // Number of uplink symbols per frame
        write_attribute(
            mainGroup, "UL_SYMS", (int)this->cfg_->ul_syms_per_frame());

        // Reciprocal Calibration Mode
        write_attribute(mainGroup, "RECIPROCAL_CALIB",
            this->cfg_->reciprocal_calib() ? 1 : 0);

        // ******* Clients ******** //
        // Freq. Domain Pilot symbols
        std::vector<double> split_vec_pilot(
            2 * this->cfg_->pilot_sym().at(0).size());
        for (size_t i = 0; i < this->cfg_->pilot_sym().at(0).size(); i++) {
            split_vec_pilot[2 * i + 0] = this->cfg_->pilot_sym().at(0).at(i);
            split_vec_pilot[2 * i + 1] = this->cfg_->pilot_sym().at(1).at(i);
        }
        write_attribute(mainGroup, "OFDM_PILOT", split_vec_pilot);

        // Number of Pilots
        write_attribute(
            mainGroup, "PILOT_NUM", (int)this->cfg_->pilot_syms_per_frame());

        // Number of Client Antennas
        write_attribute(
            mainGroup, "CL_NUM", (int)this->cfg_->num_cl_antennas());

        // Data modulation
        write_attribute(mainGroup, "CL_MODULATION", this->cfg_->data_mod());

        if (this->cfg_->client_present() == true) {
            // Client antenna polarization
            write_attribute(
                mainGroup, "CL_CH_PER_RADIO", (int)this->cfg_->cl_sdr_ch());

            // Client AGC enable flag
            write_attribute(
                mainGroup, "CL_AGC_EN", this->cfg_->cl_agc_en() ? 1 : 0);

            // RX Gain RF channel A
            write_attribute(
                mainGroup, "CL_RX_GAIN_A", this->cfg_->cl_rxgain_vec().at(0));

            // TX Gain RF channel A
            write_attribute(
                mainGroup, "CL_TX_GAIN_A", this->cfg_->cl_txgain_vec().at(0));

            // RX Gain RF channel B
            write_attribute(
                mainGroup, "CL_RX_GAIN_B", this->cfg_->cl_rxgain_vec().at(1));

            // TX Gain RF channel B
            write_attribute(
                mainGroup, "CL_TX_GAIN_B", this->cfg_->cl_txgain_vec().at(1));

            // Client frame schedule (vec of strings)
            write_attribute(
                mainGroup, "CL_FRAME_SCHED", this->cfg_->cl_frames());

            // Set of client SDR IDs (vec of strings)
            write_attribute(mainGroup, "CL_SDR_ID", this->cfg_->cl_sdr_ids());
        }

        if (this->cfg_->ul_data_sym_present()) {
            // Data subcarriers
            if (this->cfg_->data_ind().size() > 0)
                write_attribute(
                    mainGroup, "OFDM_DATA_SC", this->cfg_->data_ind());

            // Pilot subcarriers (indexes)
            if (this->cfg_->pilot_sc().at(0).size() > 0)
                write_attribute(
                    mainGroup, "OFDM_PILOT_SC", this->cfg_->pilot_sc().at(0));
            if (this->cfg_->pilot_sc().at(1).size() > 0)
                write_attribute(mainGroup, "OFDM_PILOT_SC_VALS",
                    this->cfg_->pilot_sc().at(1));

            // Freq. Domain Data Symbols
            for (size_t i = 0; i < this->cfg_->txdata_freq_dom().size(); i++) {
                std::string var
                    = std::string("OFDM_DATA_CL") + std::to_string(i);
                write_attribute(mainGroup, var.c_str(),
                    this->cfg_->txdata_freq_dom().at(i));
            }

            // Time Domain Data Symbols
            for (size_t i = 0; i < this->cfg_->txdata_time_dom().size(); i++) {
                std::string var
                    = std::string("OFDM_DATA_TIME_CL") + std::to_string(i);
                write_attribute(mainGroup, var.c_str(),
                    this->cfg_->txdata_time_dom().at(i));
            }
        }
        // ********************* //

        this->pilot_prop_.close();
        if (this->cfg_->ul_syms_per_frame() > 0) {
            H5::DataSpace data_dataspace(DS_DIM, dims_data, max_dims_data);
            this->data_prop_.setChunk(DS_DIM, cdims);
            this->file_->createDataSet("/Data/UplinkData",
                H5::PredType::STD_I16BE, data_dataspace, this->data_prop_);
            this->data_prop_.close();
        }
        //status = H5Gclose(group_id);
        //if (status < 0 ) return status;
        this->file_->close();
    }
    // catch failure caused by the H5File operations
    catch (H5::FileIException& error) {
        error.printErrorStack();
        return -1;
    }

    // catch failure caused by the DataSet operations
    catch (H5::DataSetIException& error) {
        error.printErrorStack();
        return -1;
    }

    // catch failure caused by the DataSpace operations
    catch (H5::DataSpaceIException& error) {
        error.printErrorStack();
        return -1;
    }
    this->max_frame_number_ = MAX_FRAME_INC;
    return 0; // successfully terminated
}

void Recorder::openHDF5()
{
    MLPD_TRACE("Open HDF5 file\n");
    this->file_->openFile(this->hdf5_name_, H5F_ACC_RDWR);
    // Get Dataset for pilot and check the shape of it
    this->pilot_dataset_
        = new H5::DataSet(this->file_->openDataSet("/Data/Pilot_Samples"));

    // Get the dataset's dataspace and creation property list.
    H5::DataSpace pilot_filespace(this->pilot_dataset_->getSpace());
    this->pilot_prop_.copy(this->pilot_dataset_->getCreatePlist());

    // Get information to obtain memory dataspace.
    // DataspaceIndex dims_pilot = {
    //    this->frame_number_pilot_, this->cfg_->num_cells(),
    //    this->cfg_->pilot_syms_per_frame(), this->cfg_->getNumAntennas(), IQ
    //};
    // herr_t status_n = pilot_filespace.getSimpleExtentDims(dims_pilot);

#if DEBUG_PRINT
    hsize_t IQ = 2 * this->cfg_->samps_per_symbol();
    int cndims_pilot = 0;
    int ndims = pilot_filespace.getSimpleExtentNdims();
    DataspaceIndex dims_pilot = { this->frame_number_pilot_,
        this->cfg_->num_cells(), this->cfg_->pilot_syms_per_frame(),
        this->cfg_->getMaxNumAntennas(), IQ };
    if (H5D_CHUNKED == this->pilot_prop_.getLayout())
        cndims_pilot = this->pilot_prop_.getChunk(ndims, dims_pilot);
    using std::cout;
    cout << "dim pilot chunk = " << cndims_pilot << std::endl;
    cout << "New Pilot Dataset Dimension: [";
    for (auto i = 0; i < kDsSim - 1; ++i)
        cout << dims_pilot[i] << ",";
    cout << dims_pilot[kDsSim - 1] << "]" << std::endl;
#endif
    pilot_filespace.close();
    // Get Dataset for DATA (If Enabled) and check the shape of it
    if (this->cfg_->ul_syms_per_frame() > 0) {
        this->data_dataset_
            = new H5::DataSet(this->file_->openDataSet("/Data/UplinkData"));

        H5::DataSpace data_filespace(this->data_dataset_->getSpace());
        this->data_prop_.copy(this->data_dataset_->getCreatePlist());

#if DEBUG_PRINT
        int ndims = data_filespace.getSimpleExtentNdims();
        // status_n = data_filespace.getSimpleExtentDims(dims_data);
        int cndims_data = 0;
        DataspaceIndex cdims_data
            = { 1, 1, 1, 1, IQ }; // data chunk size, TODO: optimize size
        if (H5D_CHUNKED == this->data_prop_.getLayout())
            cndims_data = this->data_prop_.getChunk(ndims, cdims_data);
        cout << "dim data chunk = " << cndims_data << std::endl;
        DataspaceIndex dims_data = { this->frame_number_data_,
            this->cfg_->num_cells(), this->cfg_->ul_syms_per_frame(),
            this->cfg_->getMaxNumAntennas(), IQ };
        cout << "New Data Dataset Dimension " << ndims << ",";
        for (auto i = 0; i < kDsSim - 1; ++i)
            cout << dims_pilot[i] << ",";
        cout << dims_pilot[kDsSim - 1] << std::endl;
#endif
        data_filespace.close();
    }
}

void Recorder::closeHDF5()
{
    MLPD_TRACE("Close HD5F file\n");
    unsigned frame_number = this->max_frame_number_;
    hsize_t IQ = 2 * this->cfg_->samps_per_symbol();

    // Resize Pilot Dataset
    this->frame_number_pilot_ = frame_number;
    DataspaceIndex dims_pilot = { this->frame_number_pilot_,
        this->cfg_->num_cells(), this->cfg_->pilot_syms_per_frame(),
        this->cfg_->getMaxNumAntennas(), IQ };
    this->pilot_dataset_->extend(dims_pilot);
    this->pilot_prop_.close();
    this->pilot_dataset_->close();
    delete this->pilot_dataset_;
    this->pilot_dataset_ = nullptr;

    // Resize Data Dataset (If Needed)
    if (this->cfg_->ul_syms_per_frame() > 0) {
        this->frame_number_data_ = frame_number;
        DataspaceIndex dims_data = { this->frame_number_data_,
            this->cfg_->num_cells(), this->cfg_->ul_syms_per_frame(),
            this->cfg_->getMaxNumAntennas(), IQ };
        this->data_dataset_->extend(dims_data);
        this->data_prop_.close();
        this->data_dataset_->close();
        delete this->data_dataset_;
        this->data_dataset_ = nullptr;
    }

    this->file_->close();
    MLPD_INFO("Saving HD5F: %d frames saved\n", frame_number);
}

Recorder::~Recorder() { this->gc(); }

void Recorder::do_it()
{
    MLPD_TRACE("Recorder work thread\n");
    if (this->cfg_->core_alloc() && pin_to_core(0) != 0) {
        perror("pinning main thread to core 0 failed");
        exit(0);
    }

    if (this->cfg_->client_present()) {
        auto client_threads = this->receiver_->startClientThreads();
    }

    if (this->cfg_->rx_thread_num() > 0) {
        if (initHDF5(this->cfg_->trace_file()) < 0)
            exit(1);
        openHDF5();

        // create socket buffer and socket threads
        auto recv_thread
            = this->receiver_->startRecvThreads(this->rx_buffer_, 1);
    } else
        this->receiver_->go(); // only beamsweeping

    moodycamel::ProducerToken ptok(this->task_queue_);
    moodycamel::ConsumerToken ctok(this->message_queue_);

    Event_data events_list[KDequeueBulkSize];
    int ret = 0;
    while (this->cfg_->running() && !SignalHandler::gotExitSignal()) {
        // get a bulk of events
        ret = this->message_queue_.try_dequeue_bulk(
            ctok, events_list, KDequeueBulkSize);
        //if (ret > 0)
        //{
        //    MLPD_TRACE("Message(s) received: %d\n", ret );
        //}
        // handle each event
        for (int bulk_count = 0; bulk_count < ret; bulk_count++) {
            Event_data& event = events_list[bulk_count];

            // if EVENT_RX_SYMBOL, do crop
            if (event.event_type == EVENT_RX_SYMBOL) {
                int offset = event.data;
                Event_data do_record_task;
                do_record_task.event_type = TASK_RECORD;
                do_record_task.data = offset;
                if (!this->task_queue_.try_enqueue(ptok, do_record_task)) {
                    std::cerr << "Queue limit has reached! try to increase "
                                 "queue size."
                              << std::endl;
                    if (!this->task_queue_.enqueue(ptok, do_record_task)) {
                        std::cerr << "Record task enqueue failed" << std::endl;
                        exit(0);
                    }
                }
            }
        }
    }
    this->cfg_->running(false);
    this->receiver_.reset();
    if ((this->cfg_->bs_present() == true)
        && (this->cfg_->rx_thread_num() > 0)) {
        closeHDF5();
    }
    if (this->cfg_->rx_thread_num() > 0) {
        finishHDF5();
    }
}

void* Recorder::taskThread_launch(void* in_context)
{
    EventHandlerContext* context = (EventHandlerContext*)in_context;
    Recorder* recorder = context->obj_ptr;
    recorder->taskThread(context);
    return nullptr;
}

void Recorder::taskThread(EventHandlerContext* context)
{
    int tid = context->id;
    delete context;
    MLPD_TRACE("Task thread: %d started\n", tid);

    Event_data event;
    bool ret = false;
    while (this->cfg_->running() == true) {
        ret = this->task_queue_.try_dequeue(event);
        // do different tasks according to task type
        if ((ret == true) && (event.event_type == TASK_RECORD)) {
            record(tid, event.data);
        }
    }
}

herr_t Recorder::record(int, int offset)
{
    herr_t ret = 0;
    size_t buffer_id = offset / rx_thread_buff_size_;
    size_t buffer_offset = offset - (buffer_id * rx_thread_buff_size_);

    // read info
    size_t package_length
        = sizeof(Package) + this->cfg_->getPackageDataLength();
    char* cur_ptr_buffer = this->rx_buffer_[buffer_id].buffer.data()
        + (buffer_offset * package_length);
    Package* pkg = reinterpret_cast<Package*>(cur_ptr_buffer);

    //Generates a ton of messages
    //MLPD_TRACE( "Tid: %d -- Record chunk size: %zu, buffer id: %d, buffer offset: %d, length %zu, offset %d\n",
    //            tid, rx_thread_buff_size_, buffer_id, buffer_offset, package_length, offset );

#if DEBUG_PRINT
    printf("record            frame %d, symbol %d, cell %d, ant %d samples: %d "
           "%d %d %d %d %d %d %d ....\n",
        pkg->frame_id, pkg->symbol_id, pkg->cell_id, pkg->ant_id, pkg->data[1],
        pkg->data[2], pkg->data[3], pkg->data[4], pkg->data[5], pkg->data[6],
        pkg->data[7], pkg->data[8]);
#endif
    hsize_t IQ = 2 * this->cfg_->samps_per_symbol();
    if ((this->cfg_->max_frame()) != 0
        && (pkg->frame_id > this->cfg_->max_frame())) {
        closeHDF5();
        MLPD_TRACE("Closing file due to frame id %d : %zu max\n", pkg->frame_id,
            this->cfg_->max_frame());
    } else {
        try {
            H5::Exception::dontPrint();
            // Update the max frame number.
            // Note that the 'frameid' might be out of order.
            if (pkg->frame_id > this->max_frame_number_) {
                // Open the hdf5 file if we haven't.
                closeHDF5();
                openHDF5();
                this->max_frame_number_
                    = this->max_frame_number_ + MAX_FRAME_INC;
            }
            DataspaceIndex hdfoffset
                = { pkg->frame_id, pkg->cell_id, 0, pkg->ant_id, 0 };
            if (this->cfg_->reciprocal_calib()
                || this->cfg_->isPilot(pkg->frame_id, pkg->symbol_id)) {
                //assert(this->pilot_dataset_ >= 0);
                // Are we going to extend the dataset?
                if (pkg->frame_id >= this->frame_number_pilot_) {
                    this->frame_number_pilot_ += kConfigPilotExtentStep;
                    if (this->cfg_->max_frame() != 0) {
                        this->frame_number_pilot_
                            = std::min(this->frame_number_pilot_,
                                this->cfg_->max_frame() + 1);
                    }
                    DataspaceIndex dims_pilot
                        = { this->frame_number_pilot_, this->cfg_->num_cells(),
                              this->cfg_->pilot_syms_per_frame(),
                              this->cfg_->getMaxNumAntennas(), IQ };
                    this->pilot_dataset_->extend(dims_pilot);
#if DEBUG_PRINT
                    std::cout
                        << "FrameId " << pkg->frame_id << ", (Pilot) Extent to "
                        << this->frame_number_pilot_ << " Frames" << std::endl;
#endif
                }
                hdfoffset[DS_SYMS_PER_FRAME]
                    = this->cfg_->getClientId(pkg->frame_id, pkg->symbol_id);

                // Select a hyperslab in extended portion of the dataset
                H5::DataSpace pilot_filespace(this->pilot_dataset_->getSpace());
                DataspaceIndex count = { 1, 1, 1, 1, IQ };
                pilot_filespace.selectHyperslab(
                    H5S_SELECT_SET, count, hdfoffset);
                // define memory space
                H5::DataSpace pilot_memspace(DS_DIM, count, NULL);
                this->pilot_dataset_->write(pkg->data,
                    H5::PredType::NATIVE_INT16, pilot_memspace,
                    pilot_filespace);
                pilot_filespace.close();
            } else if (this->cfg_->isData(pkg->frame_id, pkg->symbol_id)) {
                //assert(this->data_dataset_ >= 0);
                // Are we going to extend the dataset?
                if (pkg->frame_id >= this->frame_number_data_) {
                    this->frame_number_data_ += kConfigDataExtentStep;
                    if (this->cfg_->max_frame() != 0)
                        this->frame_number_data_
                            = std::min(this->frame_number_data_,
                                this->cfg_->max_frame() + 1);
                    DataspaceIndex dims_data
                        = { this->frame_number_data_, this->cfg_->num_cells(),
                              this->cfg_->ul_syms_per_frame(),
                              this->cfg_->getMaxNumAntennas(), IQ };
                    this->data_dataset_->extend(dims_data);
#if DEBUG_PRINT
                    std::cout
                        << "FrameId " << pkg->frame_id << ", (Data) Extent to "
                        << this->frame_number_data_ << " Frames" << std::endl;
#endif
                }
                hdfoffset[DS_SYMS_PER_FRAME]
                    = this->cfg_->getUlSFIndex(pkg->frame_id, pkg->symbol_id);
                // Select a hyperslab in extended portion of the dataset
                H5::DataSpace data_filespace(this->data_dataset_->getSpace());
                DataspaceIndex count = { 1, 1, 1, 1, IQ };
                data_filespace.selectHyperslab(
                    H5S_SELECT_SET, count, hdfoffset);
                // define memory space
                H5::DataSpace data_memspace(DS_DIM, count, NULL);
                this->data_dataset_->write(pkg->data,
                    H5::PredType::NATIVE_INT16, data_memspace, data_filespace);
            }
        }
        // catch failure caused by the H5File operations
        catch (H5::FileIException& error) {
            error.printErrorStack();
            ret = -1;
            throw;
        }
        // catch failure caused by the DataSet operations
        catch (H5::DataSetIException& error) {
            error.printErrorStack();

            MLPD_WARN("DataSet: Failed to record pilots from frame: %d , UE %d "
                      "antenna %d IQ %llu\n",
                pkg->frame_id,
                this->cfg_->getClientId(pkg->frame_id, pkg->symbol_id),
                pkg->ant_id, IQ);

            DataspaceIndex dims_pilot = { this->frame_number_pilot_,
                this->cfg_->num_cells(), this->cfg_->pilot_syms_per_frame(),
                this->cfg_->getMaxNumAntennas(), IQ };
            int ndims = this->data_dataset_->getSpace().getSimpleExtentNdims();

            std::stringstream ss;
            ss.str(std::string());
            ss << "Dataset Dimension is: " << ndims;
            for (auto i = 0; i < DS_DIM - 1; ++i) {
                ss << dims_pilot[i] << ",";
            }
            ss << dims_pilot[DS_DIM - 1];
            MLPD_TRACE("%s", ss.str().c_str());
            ret = -1;
            throw;
        }
        // catch failure caused by the DataSpace operations
        catch (H5::DataSpaceIException& error) {
            error.printErrorStack();
            ret = -1;
            throw;
        }
    } /* End else */

    int bit = 1 << (buffer_offset % sizeof(std::atomic_int));
    int offs = (buffer_offset / sizeof(std::atomic_int));
    std::atomic_fetch_and(
        &this->rx_buffer_[buffer_id].pkg_buf_inuse[offs], ~bit); // now empty
    return ret;
}

int Recorder::getRecordedFrameNum() { return this->max_frame_number_; }

extern "C" {
Recorder* Recorder_new(Config* in_cfg)
{
    Recorder* rec = new Recorder(in_cfg);
    return rec;
}

void Recorder_start(Recorder* rec) { rec->do_it(); }
int Recorder_getRecordedFrameNum(Recorder* rec)
{
    return rec->getRecordedFrameNum();
}
const char* Recorder_getTraceFileName(Recorder* rec)
{
    return rec->getTraceFileName().c_str();
}
}
