/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu

----------------------------------------------------------------------
 Record received frames from massive-mimo base station in HDF5 format
---------------------------------------------------------------------
*/

#include "include/recorder.h"
#include "include/macros.h"
#include "include/signalHandler.hpp"
#include "include/utils.h"

// buffer length of each rx thread
const int Recorder::SAMPLE_BUFFER_FRAME_NUM = 80;
// buffer length of recording part
const int Recorder::TASK_BUFFER_FRAME_NUM = 60;
// dequeue bulk size, used to reduce the overhead of dequeue in main thread
const int Recorder::dequeue_bulk_size = 5;
// pilot dataset size increment
const int Recorder::config_pilot_extent_step = 400;
// data dataset size increment
const int Recorder::config_data_extent_step = 400;

Recorder::Recorder(Config* cfg)
{
    this->cfg = cfg;
    size_t buffer_chunk_size = SAMPLE_BUFFER_FRAME_NUM * cfg->symbolsPerFrame * cfg->getNumAntennas();
    std::cout << "buffer_chunk_size " << buffer_chunk_size << std::endl;
    task_queue_ = moodycamel::ConcurrentQueue<Event_data>(buffer_chunk_size * 36);
    message_queue_ = moodycamel::ConcurrentQueue<Event_data>(buffer_chunk_size * 36);
    int rx_thread_num = cfg->rx_thread_num;
    int task_thread_num = cfg->task_thread_num;

    if (rx_thread_num > 0) {
        // initialize rx buffers
        rx_buffer_ = new SampleBuffer[rx_thread_num];
        int intsize = sizeof(std::atomic_int);
        int arraysize = (buffer_chunk_size + intsize - 1) / intsize;
        for (int i = 0; i < rx_thread_num; i++) {
            rx_buffer_[i].buffer.resize(buffer_chunk_size * cfg->getPackageLength());
            rx_buffer_[i].pkg_buf_inuse = new std::atomic_int[arraysize];
            std::fill_n(rx_buffer_[i].pkg_buf_inuse, arraysize, 0);
        }
    }

    receiver_.reset(new Receiver(rx_thread_num, cfg, &message_queue_));

    if (task_thread_num > 0) {
        // task threads
        // task_ptok.resize(task_thread_num);
        pthread_attr_t detached_attr;
        pthread_attr_init(&detached_attr);
        pthread_attr_setdetachstate(&detached_attr, PTHREAD_CREATE_DETACHED);

        for (int i = 0; i < task_thread_num; i++) {
            EventHandlerContext* context = new EventHandlerContext;
            pthread_t task_thread;
            context->obj_ptr = this;
            context->id = i;
            if (pthread_create(&task_thread, &detached_attr, Recorder::taskThread_launch, context) != 0) {
                perror("task thread create failed");
                exit(0);
            }
        }
    }
}

void Recorder::finishHDF5()
{
    // delete group;
    delete file;
}

static void write_attribute(Group& g, const char name[], double val)
{
    hsize_t dims[] = { 1 };
    DataSpace attr_ds = DataSpace(1, dims);
    Attribute att = g.createAttribute(name, PredType::NATIVE_DOUBLE, attr_ds);
    att.write(PredType::NATIVE_DOUBLE, &val);
}

static void write_attribute(Group& g, const char name[], const std::vector<double>& val)
{
    size_t size = val.size();
    hsize_t dims[] = { size };
    DataSpace attr_ds = DataSpace(1, dims);
    Attribute att = g.createAttribute(name, PredType::NATIVE_DOUBLE, attr_ds);
    att.write(PredType::NATIVE_DOUBLE, &val[0]);
}

static void write_attribute(Group& g, const char name[], const std::vector<std::complex<float>>& val)
{
    size_t size = val.size();
    hsize_t dims[] = { 2 * size };
    DataSpace attr_ds = DataSpace(1, dims);
    Attribute att = g.createAttribute(name, PredType::NATIVE_DOUBLE, attr_ds);
    double val_pair[2 * size];
    for (size_t j = 0; j < size; j++) {
        val_pair[2 * j + 0] = std::real(val[j]);
        val_pair[2 * j + 1] = std::imag(val[j]);
    }
    att.write(PredType::NATIVE_DOUBLE, &val_pair[0]);
}

static void write_attribute(Group& g, const char name[], int val)
{
    hsize_t dims[] = { 1 };
    DataSpace attr_ds = DataSpace(1, dims);
    Attribute att = g.createAttribute(name, PredType::STD_I32BE, attr_ds);
    att.write(PredType::NATIVE_INT, &val);
}

static void write_attribute(Group& g, const char name[], const std::vector<int>& val)
{
    size_t size = val.size();
    hsize_t dims[] = { size };
    DataSpace attr_ds = DataSpace(1, dims);
    Attribute att = g.createAttribute(name, PredType::STD_I32BE, attr_ds);
    att.write(PredType::NATIVE_INT, &val[0]);
}

static void write_attribute(Group& g, const char name[], const std::string& val)
{
    hsize_t dims[] = { 1 };
    DataSpace attr_ds = DataSpace(1, dims);
    StrType strdatatype(PredType::C_S1, H5T_VARIABLE); // of variable length characters
    Attribute att = g.createAttribute(name, strdatatype, attr_ds);
    att.write(strdatatype, val);
}

static void write_attribute(Group& g, const char name[], const std::vector<std::string>& val)
{
    if (val.empty())
        return;
    size_t size = val.size();
    StrType strdatatype(PredType::C_S1, H5T_VARIABLE); // of variable length characters
    hsize_t dims[] = { size };
    DataSpace attr_ds = DataSpace(1, dims);
    Attribute att = g.createAttribute(name, strdatatype, attr_ds);
    const char* cStrArray[size];

    for (size_t i = 0; i < size; ++i)
        cStrArray[i] = val[i].c_str();
    att.write(strdatatype, cStrArray);
}

herr_t Recorder::initHDF5(const std::string& hdf5)
{
    hdf5name = hdf5;
    hdf5group = "/"; // make sure it starts with '/'
    std::cout << "Set HDF5 File to " << hdf5name << " and group to " << hdf5group << std::endl;

    // dataset dimension
    hsize_t IQ = 2 * cfg->sampsPerSymbol;
    hsize_t cdims[5] = { 1, 1, 1, 1, IQ }; // pilot chunk size, TODO: optimize size
    frame_number_pilot = MAX_FRAME_INC; //cfg->maxFrame;
    hsize_t dims_pilot[] = {
        frame_number_pilot, cfg->nCells,
        cfg->pilotSymsPerFrame, cfg->getNumAntennas(), IQ
    };
    hsize_t max_dims_pilot[5] = { H5S_UNLIMITED, cfg->nCells, cfg->pilotSymsPerFrame, cfg->getNumAntennas(), IQ };

    frame_number_data = MAX_FRAME_INC; //cfg->maxFrame;
    hsize_t dims_data[] = {
        frame_number_data, cfg->nCells,
        cfg->ulSymsPerFrame, cfg->getNumAntennas(), IQ
    };
    hsize_t max_dims_data[5] = { H5S_UNLIMITED, cfg->nCells, cfg->ulSymsPerFrame, cfg->getNumAntennas(), IQ };

    try {
        Exception::dontPrint();

        file = new H5File(hdf5name, H5F_ACC_TRUNC);
        //group = new Group(file->createGroup("/Data"));
        auto mainGroup = file->createGroup("/Data");
        pilot_prop.setChunk(5, cdims);

        DataSpace pilot_dataspace(5, dims_pilot, max_dims_pilot);
        file->createDataSet("/Data/Pilot_Samples", PredType::STD_I16BE,
            pilot_dataspace, pilot_prop);
        pilot_dataspace.close();

        // ******* COMMON ******** //
        // TX/RX Frequency
        write_attribute(mainGroup, "FREQ", cfg->freq);

        // BW
        write_attribute(mainGroup, "RATE", cfg->rate);

        // Number of samples on each symbol (excluding prefix/postfix)
        write_attribute(mainGroup, "SYMBOL_LEN_NO_PAD",
            cfg->sampsPerSymbol - cfg->prefix - cfg->postfix);

        // Number of samples for prefix (padding)
        write_attribute(mainGroup, "PREFIX_LEN", cfg->prefix);

        // Number of samples for postfix (padding)
        write_attribute(mainGroup, "POSTFIX_LEN", cfg->postfix);

        // Number of samples on each symbol including prefix and postfix
        write_attribute(mainGroup, "SYMBOL_LEN", cfg->sampsPerSymbol);

        // Size of FFT
        write_attribute(mainGroup, "FFT_SIZE", cfg->fftSize);

        // Length of cyclic prefix
        write_attribute(mainGroup, "CP_LEN", cfg->cpSize);

        // Beacon sequence type (string)
        write_attribute(mainGroup, "BEACON_SEQ_TYPE", cfg->beacon_seq);

        // Pilot sequence type (string)
        write_attribute(mainGroup, "PILOT_SEQ_TYPE", cfg->pilot_seq);

        // ******* Base Station ******** //

        // Hub IDs (vec of strings)
        write_attribute(mainGroup, "BS_HUB_ID", cfg->hub_ids);

        // BS SDR IDs
        // *** first, how many boards in each cell? ***
        std::vector<std::string> bs_sdr_num_per_cell(cfg->bs_sdr_ids.size());
        for (size_t i = 0; i < bs_sdr_num_per_cell.size(); ++i) {
            bs_sdr_num_per_cell[i] = std::to_string(cfg->bs_sdr_ids[i].size());
        }
        write_attribute(mainGroup, "BS_SDR_NUM_PER_CELL", bs_sdr_num_per_cell);

        // *** second, reshape matrix into vector ***
        std::vector<std::string> bs_sdr_id;
        for (auto&& v : cfg->bs_sdr_ids) {
            bs_sdr_id.insert(bs_sdr_id.end(), v.begin(), v.end());
        }
        write_attribute(mainGroup, "BS_SDR_ID", bs_sdr_id);

        // Number of Base Station Cells
        write_attribute(mainGroup, "BS_NUM_CELLS", (int)cfg->nCells);

        // How many RF channels per Iris board are enabled ("single" or "dual")
        write_attribute(mainGroup, "BS_CH_PER_RADIO", (int)cfg->bsChannel.length());

        // Frame schedule (vec of strings for now, this should change to matrix when we go to multi-cell)
        write_attribute(mainGroup, "BS_FRAME_SCHED", cfg->frames);

        // RX Gain RF channel A
        write_attribute(mainGroup, "BS_RX_GAIN_A", cfg->rxgain[0]);

        // TX Gain RF channel A
        write_attribute(mainGroup, "BS_TX_GAIN_A", cfg->txgain[0]);

        // RX Gain RF channel B
        write_attribute(mainGroup, "BS_RX_GAIN_B", cfg->rxgain[1]);

        // TX Gain RF channel B
        write_attribute(mainGroup, "BS_TX_GAIN_B", cfg->txgain[1]);

        // Beamsweep (true or false)
        write_attribute(mainGroup, "BS_BEAMSWEEP", cfg->beamsweep ? 1 : 0);

        // Beacon Antenna
        write_attribute(mainGroup, "BS_BEACON_ANT", (int)cfg->beacon_ant);

        // Number of antennas on Base Station
        write_attribute(mainGroup, "BS_NUM_ANT", (int)cfg->getNumAntennas());

        // Number of symbols in a frame
        write_attribute(mainGroup, "BS_FRAME_LEN", cfg->symbolsPerFrame);

        // Number of uplink symbols per frame
        write_attribute(mainGroup, "UL_SYMS", (int)cfg->ulSymsPerFrame);

        // ******* Clients ******** //
        // Data subcarriers
        write_attribute(mainGroup, "OFDM_DATA_SC", cfg->data_ind);

        // Pilot subcarriers (indexes)
        write_attribute(mainGroup, "OFDM_PILOT_SC", cfg->pilot_sc[0]);
        write_attribute(mainGroup, "OFDM_PILOT_SC_VALS", cfg->pilot_sc[1]);

        // Freq. Domain Data Symbols - OBCH
        for (size_t i = 0; i < cfg->txdata_freq_dom.size(); i++) {
            std::string var = std::string("OFDM_DATA_CL") + std::to_string(i);
            write_attribute(mainGroup, var.c_str(), cfg->txdata_freq_dom[i]);
        }

        // Time Domain Data Symbols
        for (size_t i = 0; i < cfg->txdata_time_dom.size(); i++) {
            std::string var = std::string("OFDM_DATA_TIME_CL") + std::to_string(i);
            write_attribute(mainGroup, var.c_str(), cfg->txdata_time_dom[i]);
        }

        // Freq. Domain Pilot symbols
        std::vector<double> split_vec_pilot(2 * cfg->pilot_double[0].size());
        for (size_t i = 0; i < cfg->pilot_double[0].size(); i++) {
            split_vec_pilot[2 * i + 0] = cfg->pilot_double[0][i];
            split_vec_pilot[2 * i + 1] = cfg->pilot_double[1][i];
        }
        write_attribute(mainGroup, "OFDM_PILOT", split_vec_pilot);

        // Number of Clients
        write_attribute(mainGroup, "PILOT_NUM", (int)cfg->pilotSymsPerFrame);

        // Number of Clients
        write_attribute(mainGroup, "CL_NUM", (int)cfg->nClSdrs);

        if (cfg->clPresent) {
            // Client antenna polarization
            write_attribute(mainGroup, "CL_CH_PER_RADIO", cfg->clSdrCh);

            // Client AGC enable flag
            write_attribute(mainGroup, "CL_AGC_EN", cfg->clAgcEn ? 1 : 0);

            // RX Gain RF channel A
            write_attribute(mainGroup, "CL_RX_GAIN_A", cfg->clRxgain_vec[0]);

            // TX Gain RF channel A
            write_attribute(mainGroup, "CL_TX_GAIN_A", cfg->clTxgain_vec[0]);

            // RX Gain RF channel B
            write_attribute(mainGroup, "CL_RX_GAIN_B", cfg->clRxgain_vec[1]);

            // TX Gain RF channel B
            write_attribute(mainGroup, "CL_TX_GAIN_B", cfg->clTxgain_vec[1]);

            // Client frame schedule (vec of strings)
            write_attribute(mainGroup, "CL_FRAME_SCHED", cfg->clFrames);

            // Set of client SDR IDs (vec of strings)
            write_attribute(mainGroup, "CL_SDR_ID", cfg->cl_sdr_ids);

            // Data modulation
            write_attribute(mainGroup, "CL_MODULATION", cfg->clDataMod);
        }
        // ********************* //

        pilot_prop.close();
        if (cfg->ulSymsPerFrame > 0) {
            DataSpace data_dataspace(5, dims_data, max_dims_data);
            data_prop.setChunk(5, cdims);
            file->createDataSet("/Data/UplinkData",
                PredType::STD_I16BE, data_dataspace, data_prop);
            data_prop.close();
        }
        //status = H5Gclose(group_id);
        //if (status < 0 ) return status;
        file->close();
    }
    // catch failure caused by the H5File operations
    catch (FileIException error) {
        error.printError();
        return -1;
    }

    // catch failure caused by the DataSet operations
    catch (DataSetIException error) {
        error.printError();
        return -1;
    }

    // catch failure caused by the DataSpace operations
    catch (DataSpaceIException error) {
        error.printError();
        return -1;
    }
    maxFrameNumber = MAX_FRAME_INC;
    return 0; // successfully terminated
}

void Recorder::openHDF5()
{
    file->openFile(hdf5name, H5F_ACC_RDWR);
    // Get Dataset for pilot and check the shape of it
    pilot_dataset = new DataSet(file->openDataSet("/Data/Pilot_Samples"));

    // Get the dataset's dataspace and creation property list.
    DataSpace pilot_filespace(pilot_dataset->getSpace());
    pilot_prop = pilot_dataset->getCreatePlist();

    // Get information to obtain memory dataspace.
    // hsize_t dims_pilot[] = {
    //    frame_number_pilot, cfg->nCells,
    //    cfg->pilotSymsPerFrame, cfg->getNumAntennas(), IQ
    //};
    // herr_t status_n = pilot_filespace.getSimpleExtentDims(dims_pilot);

#if DEBUG_PRINT
    hsize_t IQ = 2 * cfg->sampsPerSymbol;
    int cndims_pilot = 0;
    int ndims = pilot_filespace.getSimpleExtentNdims();
    if (H5D_CHUNKED == pilot_prop.getLayout())
        cndims_pilot = pilot_prop.getChunk(ndims, cdims_pilot);
    using std::cout;
    cout << "dim pilot chunk = " << cndims_pilot << std::endl;
    hsize_t dims_pilot[] = {
        frame_number_pilot, cfg->nCells,
        cfg->pilotSymsPerFrame, cfg->getNumAntennas(), IQ
    };
    cout << "New Pilot Dataset Dimension: [";
    cout << dims_pilot[0] << "," << dims_pilot[1] << ",";
    cout << dims_pilot[2] << "," << dims_pilot[3] << ",";
    cout << IQ << "]" << std::endl;
#endif
    pilot_filespace.close();
    // Get Dataset for DATA (If Enabled) and check the shape of it
    if (cfg->ulSymsPerFrame > 0) {
        data_dataset = new DataSet(file->openDataSet("/Data/UplinkData"));

        DataSpace data_filespace(data_dataset->getSpace());
        data_prop = data_dataset->getCreatePlist();

#if DEBUG_PRINT
        int ndims = data_filespace.getSimpleExtentNdims();
        // status_n = data_filespace.getSimpleExtentDims(dims_data);
        int cndims_data = 0;
        hsize_t cdims_data[5] = { 1, 1, 1, 1, IQ }; // data chunk size, TODO: optimize size
        if (H5D_CHUNKED == data_prop.getLayout())
            cndims_data = data_prop.getChunk(ndims, cdims_data);
        cout << "dim data chunk = " << cndims_data << std::endl;
        ;
        hsize_t dims_data[] = {
            frame_number_data, cfg->nCells,
            cfg->ulSymsPerFrame, cfg->getNumAntennas(), IQ
        };
        cout << "New Data Dataset Dimension " << ndims << "," << dims_data[0] << ",";
        cout << dims_data[1] << "," << dims_data[2] << "," << dims_data[3] << "," << IQ << std::endl;
#endif
        data_filespace.close();
    }
}

void Recorder::closeHDF5()
{
    unsigned frameNumber = maxFrameNumber;
    hsize_t IQ = 2 * cfg->sampsPerSymbol;

    // Resize Pilot Dataset
    frame_number_pilot = frameNumber;
    hsize_t dims_pilot[] = {
        frame_number_pilot, cfg->nCells,
        cfg->pilotSymsPerFrame, cfg->getNumAntennas(), IQ
    };
    pilot_dataset->extend(dims_pilot);
    pilot_prop.close();
    pilot_dataset->close();
    delete pilot_dataset;

    // Resize Data Dataset (If Needed)
    if (cfg->ulSymsPerFrame > 0) {
        frame_number_data = frameNumber;
        hsize_t dims_data[] = {
            frame_number_data, cfg->nCells,
            cfg->ulSymsPerFrame, cfg->getNumAntennas(), IQ
        };
        data_dataset->extend(dims_data);
        data_prop.close();
        data_dataset->close();
    }

    file->close();

    std::cout << "Saving HDF5, " << frameNumber << " frames saved." << std::endl;
}

Recorder::~Recorder()
{
    for (size_t i = 0; i < cfg->rx_thread_num; i++)
        delete[] rx_buffer_[i].pkg_buf_inuse;
    delete[] rx_buffer_;
}

void Recorder::do_it()
{
    if (cfg->core_alloc && pin_to_core(0) != 0) {
        perror("pinning main thread to core 0 failed");
        exit(0);
    }

    if (cfg->clPresent) {
        auto client_threads = receiver_->startClientThreads();
    }

    if (cfg->rx_thread_num > 0) {
        if (initHDF5(cfg->trace_file) < 0)
            exit(1);
        openHDF5();

        // create socket buffer and socket threads
        auto recv_thread = receiver_->startRecvThreads(rx_buffer_, 1);
    } else
        receiver_->go(); // only beamsweeping

    moodycamel::ProducerToken ptok(task_queue_);
    moodycamel::ConsumerToken ctok(message_queue_);

    Event_data events_list[dequeue_bulk_size];
    int ret = 0;
    while (cfg->running && !SignalHandler::gotExitSignal()) {
        // get a bulk of events
        ret = message_queue_.try_dequeue_bulk(ctok, events_list, dequeue_bulk_size);
        // handle each event
        for (int bulk_count = 0; bulk_count < ret; bulk_count++) {
            Event_data& event = events_list[bulk_count];

            // if EVENT_RX_SYMBOL, do crop
            if (event.event_type == EVENT_RX_SYMBOL) {
                int offset = event.data;
                Event_data do_record_task;
                do_record_task.event_type = TASK_RECORD;
                do_record_task.data = offset;
                if (!task_queue_.try_enqueue(ptok, do_record_task)) {
                    printf("queue limit has reached! try to increase queue size.\n");
                    if (!task_queue_.enqueue(ptok, do_record_task)) {
                        printf("record task enqueue failed\n");
                        exit(0);
                    }
                }
            }
        }
    }
    cfg->running = false;
    receiver_.reset();
    if (cfg->bsPresent && cfg->rx_thread_num > 0)
        closeHDF5();
    if (cfg->rx_thread_num > 0)
        finishHDF5();
}

void* Recorder::taskThread_launch(void* in_context)
{
    EventHandlerContext* context = (EventHandlerContext*)in_context;
    Recorder* recorder = context->obj_ptr;
    recorder->taskThread(context);
    return 0;
}

void Recorder::taskThread(EventHandlerContext* context)
{
    int tid = context->id;
    delete context;
    printf("task thread %d starts\n", tid);

    // task_ptok[tid].reset(new moodycamel::ProducerToken(message_queue_));

    Event_data event;
    bool ret = false;
    while (cfg->running) {
        ret = task_queue_.try_dequeue(event);
        if (!ret)
            continue;

        // do different tasks according to task type
        if (event.event_type == TASK_RECORD) {
            record(tid, event.data);
        }
    }
}

// do Crop
herr_t Recorder::record(int, int offset)
{
    size_t buffer_chunk_size = SAMPLE_BUFFER_FRAME_NUM * cfg->symbolsPerFrame * cfg->getNumAntennas();
    int buffer_id = offset / buffer_chunk_size;
    offset = offset - buffer_id * buffer_chunk_size;
    // read info
    char* cur_ptr_buffer = rx_buffer_[buffer_id].buffer.data() + offset * cfg->getPackageLength();
    struct Package* pkg = (struct Package*)cur_ptr_buffer;
#if DEBUG_PRINT
    printf("record            frame %d, symbol %d, cell %d, ant %d samples: %d %d %d %d %d %d %d %d ....\n",
        pkg->frame_id, pkg->symbol_id, pkg->cell_id, pkg->ant_id,
        pkg->data[1], pkg->data[2], pkg->data[3], pkg->data[4],
        pkg->data[5], pkg->data[6], pkg->data[7], pkg->data[8]);
#endif
    hsize_t IQ = 2 * cfg->sampsPerSymbol;
    if (cfg->max_frame != 0 && pkg->frame_id > cfg->max_frame) {
        closeHDF5();
        goto clean_exit;
    }
    try {
        Exception::dontPrint();
        // Update the max frame number.
        // Note that the 'frameid' might be out of order.
        if (pkg->frame_id > maxFrameNumber) {
            // Open the hdf5 file if we haven't.
            closeHDF5();
            openHDF5();
            maxFrameNumber = maxFrameNumber + MAX_FRAME_INC;
        }

        hsize_t hdfoffset[5];
        hdfoffset[0] = pkg->frame_id;
        hdfoffset[1] = 0; // will change later after we use cell_id
        hdfoffset[3] = pkg->ant_id;
        hdfoffset[4] = 0;

        if (cfg->isPilot(pkg->frame_id, pkg->symbol_id)) {
            //assert(pilot_dataset >= 0);
            // Are we going to extend the dataset?
            if (pkg->frame_id >= frame_number_pilot) {
                frame_number_pilot += config_pilot_extent_step;
                if (cfg->max_frame != 0)
                    frame_number_pilot = std::min(frame_number_pilot, cfg->max_frame + 1);
                hsize_t dims_pilot[] = {
                    frame_number_pilot, cfg->nCells,
                    cfg->pilotSymsPerFrame, cfg->getNumAntennas(), IQ
                };
                pilot_dataset->extend(dims_pilot);
#if DEBUG_PRINT
                std::cout << "FrameId " << pkg->frame_id << ", (Pilot) Extent to " << frame_number_pilot << " Frames" << std::endl;
#endif
            }

            hdfoffset[2] = cfg->getClientId(pkg->frame_id, pkg->symbol_id);

            // Select a hyperslab in extended portion of the dataset
            DataSpace pilot_filespace(pilot_dataset->getSpace());
            hsize_t count[] = { 1, 1, 1, 1, IQ };
            pilot_filespace.selectHyperslab(H5S_SELECT_SET, count, hdfoffset);

            // define memory space
            DataSpace pilot_memspace(5, count, NULL);
            pilot_dataset->write(pkg->data, PredType::NATIVE_INT16,
                pilot_memspace, pilot_filespace);
            pilot_filespace.close();
        } else if (cfg->isData(pkg->frame_id, pkg->symbol_id)) {

            //assert(data_dataset >= 0);
            // Are we going to extend the dataset?
            if (pkg->frame_id >= frame_number_data) {
                frame_number_data += config_data_extent_step;
                if (cfg->max_frame != 0)
                    frame_number_data = std::min(frame_number_data, cfg->max_frame + 1);
                hsize_t dims_data[] = {
                    frame_number_data, cfg->nCells,
                    cfg->ulSymsPerFrame, cfg->getNumAntennas(), IQ
                };
                data_dataset->extend(dims_data);
#if DEBUG_PRINT
                std::cout << "FrameId " << pkg->frame_id << ", (Data) Extent to " << frame_number_data << " Frames" << std::endl;
#endif
            }

            hdfoffset[2] = cfg->getUlSFIndex(pkg->frame_id, pkg->symbol_id);

            // Select a hyperslab in extended portion of the dataset
            DataSpace data_filespace(data_dataset->getSpace());
            hsize_t count[] = { 1, 1, 1, 1, IQ };
            data_filespace.selectHyperslab(H5S_SELECT_SET, count, hdfoffset);

            // define memory space
            DataSpace data_memspace(5, count, NULL);
            data_dataset->write(pkg->data, PredType::NATIVE_INT16, data_memspace, data_filespace);
        }
    }
    // catch failure caused by the H5File operations
    catch (FileIException error) {
        error.printError();
        return -1;
    }

    // catch failure caused by the DataSet operations
    catch (DataSetIException error) {
        error.printError();
        std::cout << "DataSet: Failed to record pilots from frame " << pkg->frame_id << " , UE " << cfg->getClientId(pkg->frame_id, pkg->symbol_id) << " antenna " << pkg->ant_id << " IQ " << IQ << std::endl;
        hsize_t dims_pilot[] = {
            frame_number_pilot, cfg->nCells,
            cfg->pilotSymsPerFrame, cfg->getNumAntennas(), IQ
        };
        int ndims = data_dataset->getSpace().getSimpleExtentNdims();
        std::cout << "Dataset Dimension is " << ndims << "," << dims_pilot[0] << "," << dims_pilot[1] << "," << dims_pilot[2] << "," << dims_pilot[3] << "," << IQ << std::endl;
        return -1;
    }

    // catch failure caused by the DataSpace operations
    catch (DataSpaceIException error) {
        error.printError();
        return -1;
    }

clean_exit:

    // after finish
    int bit = 1 << offset % sizeof(std::atomic_int);
    int offs = offset / sizeof(std::atomic_int);
    std::atomic_fetch_and(&rx_buffer_[buffer_id].pkg_buf_inuse[offs], ~bit); // now empty
    return 0;
}
