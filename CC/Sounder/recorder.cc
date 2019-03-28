/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu

----------------------------------------------------------------------
 Record received frames from massive-mimo base station in HDF5 format
---------------------------------------------------------------------
*/

#include "include/recorder.h"

Recorder::Recorder(Config *cfg)
{
    this->cfg = cfg;
    unsigned nCores = std::thread::hardware_concurrency();
    printf("number of CPU cores %d\n", nCores);
    rx_thread_num = nCores >= 2*SOCKET_THREAD_NUM and cfg->nBsSdrs[0] >= SOCKET_THREAD_NUM ? SOCKET_THREAD_NUM : 1; // TODO: read number of cores and assign accordingly 
    printf("allocating %d cores to receive threads ... \n", rx_thread_num);
    task_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * cfg->symbolsPerFrame * cfg->getNumAntennas() * 36);
    message_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * cfg->symbolsPerFrame * cfg->getNumAntennas() * 36);
    
    // initialize socket buffer
    for(int i = 0; i < rx_thread_num; i++)
    {
        socket_buffer_[i].buffer.resize(cfg->getPackageLength() * cfg->getNumAntennas() * cfg->symbolsPerFrame * SOCKET_BUFFER_FRAME_NUM);
        socket_buffer_[i].buffer_status.resize(cfg->symbolsPerFrame * SOCKET_BUFFER_FRAME_NUM * cfg->getNumAntennas());
    }

    receiver_.reset(new Receiver(rx_thread_num, cfg, &message_queue_));

    // create task thread
    for(int i = 0; i < TASK_THREAD_NUM; i++)
    {
        context[i].obj_ptr = this;
        context[i].id = i;
        if(pthread_create( &task_threads[i], NULL, Recorder::taskThread, &context[i]) != 0)
        {
            perror("task thread create failed");
            exit(0);
        }
    }
}

herr_t Recorder::initHDF5(std::string hdf5)
{
    char tmpstr[1024];
    
    hdf5name = hdf5; 
    hdf5group = "/"; // make sure it starts with '/' 
    std::cout << "Set HDF5 File to " << hdf5name << " and group to " << hdf5group << std::endl;	

    // dataset dimension    
    dims_pilot[0] = MAX_FRAME_INC; //cfg->maxFrame; 
    dims_pilot[1] = cfg->nCells; 
    dims_pilot[2] = cfg->nClSdrs;
    dims_pilot[3] = cfg->getNumAntennas();
    dims_pilot[4] = 2 * cfg->sampsPerSymbol; // IQ
    hsize_t cdims[5] = {1, 1, 1, 1, 2 * cfg->sampsPerSymbol}; // pilot chunk size, TODO: optimize size
    hsize_t max_dims_pilot[5] = {H5S_UNLIMITED, cfg->nCells, cfg->nClSdrs, cfg->getNumAntennas(), 2 * cfg->sampsPerSymbol};

    dims_data[0] = MAX_FRAME_INC; //cfg->maxFrame; 
    dims_data[1] = cfg->nCells; 
    dims_data[2] = cfg->ulSymsPerFrame;
    dims_data[3] = cfg->getNumAntennas();
    dims_data[4] = 2 * cfg->sampsPerSymbol; // IQ
    hsize_t cdims_data[5] = {1, 1, 1, 1, 2 * cfg->sampsPerSymbol}; // data chunk size, TODO: optimize size
    hsize_t max_dims_data[5] = {H5S_UNLIMITED, cfg->nCells, cfg->ulSymsPerFrame, cfg->getNumAntennas(), 2 * cfg->sampsPerSymbol};

    // Used to create variable strings
    std::ostringstream oss;

    try
    {
	Exception::dontPrint();

        file = new H5File(hdf5name, H5F_ACC_TRUNC);
        //group = new Group(file->createGroup("/Data"));
        auto mainGroup = file->createGroup("/Data");
        DataSpace *pilot_dataspace = new DataSpace(5, dims_pilot, max_dims_pilot);
        pilot_prop.setChunk(5, cdims);

        DataSet *pilot_dataset = new DataSet(file->createDataSet("/Data/Pilot_Samples", 
                                     PredType::STD_I16BE, *pilot_dataspace, pilot_prop) );

	// Attribute dataspace
        hsize_t dims[1] = {1};
        DataSpace attr_ds = DataSpace (1, dims);
        hsize_t dimsVec[1];
	DataSpace attr_vec_ds;

	// Create new string datatype for attribute
        StrType strdatatype(PredType::C_S1, H5T_VARIABLE); // of variable length characters

	// For scalars
	int attr_data;
	double attr_data_double;

	// Create str/vec variables and attribute object
	std::string attr_data_str;
	std::vector<std::string> att_vector;
	Attribute att;
	std::vector<std::vector<std::string>> att_matrix;
	std::vector<const char *> cStrArray;
	std::vector<int> attr_data_ind;
	std::vector<std::vector<int>> attr_pilot_sc;
	std::vector<std::vector<std::complex<float>>> attr_txdata_freq_dom;
	std::vector<std::vector<double>> attr_pilot_double;

	// ******* COMMON ******** //
	// TX/RX Frequency
        attr_data_double = cfg->freq;
	att = mainGroup.createAttribute("FREQ", PredType::NATIVE_DOUBLE, attr_ds);
        att.write(PredType::NATIVE_DOUBLE, &attr_data_double);

	// BW 
        attr_data_double = cfg->rate;
        att = mainGroup.createAttribute("RATE", PredType::NATIVE_DOUBLE, attr_ds);
        att.write(PredType::NATIVE_DOUBLE, &attr_data_double);

	// Number of samples on each symbol (excluding prefix/postfix)
        attr_data = cfg->sampsPerSymbol - cfg->prefix - cfg->postfix;
        att = mainGroup.createAttribute("SYMBOL_LEN_NO_PAD", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data);

	// Number of samples for prefix (padding)
        attr_data = cfg->prefix;
        att = mainGroup.createAttribute("PREFIX_LEN", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data);

	// Number of samples for postfix (padding)
        attr_data = cfg->postfix;
        att = mainGroup.createAttribute("POSTFIX_LEN", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data);

	// Number of samples on each symbol including prefix and postfix
        attr_data = cfg->sampsPerSymbol;
        att = mainGroup.createAttribute("SYMBOL_LEN", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data);

	// Size of FFT
        attr_data = cfg->fftSize;
        att = mainGroup.createAttribute("FFT_SIZE", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data);

	// Length of cyclic prefix
        attr_data = cfg->cpSize;
        att = mainGroup.createAttribute("CP_LEN", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data);

	// Beacon sequence type (string)
        //attr_data_str = cfg->beacon_seq;
        att = mainGroup.createAttribute("BEACON_SEQ_TYPE", strdatatype, attr_ds);
        att.write(strdatatype, cfg->beacon_seq);

	// Pilot sequence type (string)
	//attr_data_str = cfg->pilot_seq;
	att = mainGroup.createAttribute("PILOT_SEQ_TYPE", strdatatype, attr_ds);
	att.write(strdatatype, cfg->pilot_seq); 

	// ******* Base Station ******** //

	// Hub IDs (vec of strings)
	cStrArray.clear();
	att_vector = cfg->hub_ids;
	dimsVec[0] = att_vector.size();
	attr_vec_ds = DataSpace (1, dimsVec);
	att = mainGroup.createAttribute("BS_HUB_ID", strdatatype, attr_vec_ds);
	//Convert the vector into a C string array - Because the input function ::write requires that.
	for(int index = 0; index < att_vector.size(); ++index) { cStrArray.push_back(att_vector[index].c_str()); }
	if(!(cfg->hub_ids).empty()) { att.write(strdatatype, (void*)&cStrArray[0]); }

	// BS SDR IDs 
	// *** first, how many boards in each cell? ***
	cStrArray.clear();
        att_vector.clear();
	std::vector<std::string> att_vectorTmp;
	att_matrix = cfg->bs_sdr_ids; //test: {{"AAA", "BBB"}, {"CCC", "DDD"}, {"EEE", "FFF"}}
	for(int index = 0; index < att_matrix.size(); ++index) { 
	    std::string irisPerCell = std::to_string(att_matrix[index].size());
	    att_vectorTmp.push_back(irisPerCell); 
	}
	dimsVec[0] = att_vectorTmp.size();
	attr_vec_ds = DataSpace (1, dimsVec);
	att = mainGroup.createAttribute("BS_SDR_NUM_PER_CELL", strdatatype, attr_vec_ds);
	//Convert the vector into a C string array - Because the input function ::write requires that.
	for(int index = 0; index < att_vectorTmp.size(); ++index) { cStrArray.push_back(att_vectorTmp[index].c_str()); }
	if(!att_vectorTmp.empty()) { att.write(strdatatype, (void*)&cStrArray[0]); }
	// *** second, reshape matrix into vector ***
	for(auto && v : att_matrix){ att_vector.insert(att_vector.end(), v.begin(), v.end()); }
        cStrArray.clear();
	dimsVec[0] = att_vector.size();
	attr_vec_ds = DataSpace (1, dimsVec);
	att = mainGroup.createAttribute("BS_SDR_ID", strdatatype, attr_vec_ds);
	//Convert the vector into a C string array - Because the input function ::write requires that.
	for(int index = 0; index < att_vector.size(); ++index) { cStrArray.push_back(att_vector[index].c_str()); }
	if(!att_vector.empty()) { att.write(strdatatype, (void*)&cStrArray[0]); }

	// Number of Base Station Cells
        attr_data = cfg->nCells; 
        att = mainGroup.createAttribute("BS_NUM_CELLS", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data); 

	// How many RF channels per Iris board are enabled ("single" or "dual")
        attr_data = cfg->bsSdrCh;
        att = mainGroup.createAttribute("BS_CH_PER_RADIO", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data);

	// Frame schedule (vec of strings for now, this should change to matrix when we go to multi-cell)
	cStrArray.clear();
	att_vector = cfg->frames;
	dimsVec[0] = att_vector.size();
        attr_vec_ds = DataSpace (1, dimsVec);
        att = mainGroup.createAttribute("BS_FRAME_SCHED", strdatatype, attr_vec_ds);
	//Convert the vector into a C string array - Because the input function ::write requires that.
	for(int index = 0; index < att_vector.size(); ++index) { cStrArray.push_back(att_vector[index].c_str()); }
        if(!(cfg->frames).empty()) { att.write(strdatatype, (void*)&cStrArray[0]); }

	// RX Gain RF channel A
        attr_data_double = cfg->rxgainA;
        att = mainGroup.createAttribute("BS_RX_GAIN_A", PredType::NATIVE_DOUBLE, attr_ds);
        att.write(PredType::NATIVE_DOUBLE, &attr_data_double);

	// TX Gain RF channel A
        attr_data_double = cfg->txgainA;
        att = mainGroup.createAttribute("BS_TX_GAIN_A", PredType::NATIVE_DOUBLE, attr_ds);
        att.write(PredType::NATIVE_DOUBLE, &attr_data_double);

	// RX Gain RF channel B
        attr_data_double = cfg->rxgainB;
        att = mainGroup.createAttribute("BS_RX_GAIN_B", PredType::NATIVE_DOUBLE, attr_ds);
        att.write(PredType::NATIVE_DOUBLE, &attr_data_double);

	// TX Gain RF channel B
        attr_data_double = cfg->txgainB;
        att = mainGroup.createAttribute("BS_TX_GAIN_B", PredType::NATIVE_DOUBLE, attr_ds);
        att.write(PredType::NATIVE_DOUBLE, &attr_data_double);

	// Beamsweep (true or false)
        attr_data = cfg->beamsweep? 1 : 0;
        att = mainGroup.createAttribute("BS_BEAMSWEEP", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data);

	// Beacon Antenna
        attr_data = cfg->beacon_ant;
        att = mainGroup.createAttribute("BS_BEACON_ANT", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data);

	// Number of antennas on Base Station
        attr_data = cfg->getNumAntennas();
        att = mainGroup.createAttribute("BS_NUM_ANT", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data); 

	// Number of symbols in a frame
        attr_data = cfg->symbolsPerFrame;
        att = mainGroup.createAttribute("BS_FRAME_LEN", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data); 

	// ******* Clients ******** //
	// Data subcarriers
	attr_data_ind = cfg->data_ind;
	dimsVec[0] = attr_data_ind.size();
        attr_vec_ds = DataSpace (1, dimsVec);
        att = mainGroup.createAttribute("OFDM_DATA_SC", PredType::STD_I32BE, attr_vec_ds);
	att.write(PredType::NATIVE_INT, &attr_data_ind[0]);

	// Pilot subcarriers (indexes)
	attr_pilot_sc = cfg->pilot_sc;
	dimsVec[0] = attr_pilot_sc[0].size();
        attr_vec_ds = DataSpace (1, dimsVec);
        att = mainGroup.createAttribute("OFDM_PILOT_SC", PredType::STD_I32BE, attr_vec_ds);
	att.write(PredType::NATIVE_INT, &attr_pilot_sc[0][0]);
	// Pilot subcarriers (values)
	dimsVec[0] = attr_pilot_sc[1].size();
        attr_vec_ds = DataSpace (1, dimsVec);
        att = mainGroup.createAttribute("OFDM_PILOT_SC_VALS", PredType::STD_I32BE, attr_vec_ds);
	att.write(PredType::NATIVE_INT, &attr_pilot_sc[1][0]);

	// Freq. Domain Data Symbols
	attr_txdata_freq_dom = cfg->txdata_freq_dom;
	for (int i = 0; i < attr_txdata_freq_dom.size(); i++){
	    oss.str("");
	    oss.clear();
	    oss << " OFDM_DATA_CL" << i;
	    std::string var = oss.str();

	    std::vector<double> re_im_split_vec;  // re-write complex vector. type not supported by hdf5
	    for (int j = 0; j < attr_txdata_freq_dom[i].size(); j++){
	        double re_val = std::real(attr_txdata_freq_dom[i][j]);
	        double im_val = std::imag(attr_txdata_freq_dom[i][j]);
            re_im_split_vec.push_back(re_val);
            re_im_split_vec.push_back(im_val);
	    }
	    dimsVec[0] = re_im_split_vec.size();  // real and imaginary parts
        attr_vec_ds = DataSpace (1, dimsVec);
        att = mainGroup.createAttribute(var, PredType::NATIVE_DOUBLE, attr_vec_ds);
	    att.write(PredType::NATIVE_DOUBLE, &re_im_split_vec[i][0]);
	    re_im_split_vec.clear();
	}

	// Freq. Domain Pilot symbols (real and imaginary parts)
	attr_pilot_double = cfg->pilot_double;
	dimsVec[0] = attr_pilot_double[0].size();
        attr_vec_ds = DataSpace (1, dimsVec);
        att = mainGroup.createAttribute("OFDM_PILOT_RE", PredType::NATIVE_DOUBLE, attr_vec_ds);
	att.write(PredType::NATIVE_DOUBLE, &attr_pilot_double[0][0]);
	// Pilot subcarriers (values)
	dimsVec[0] = attr_pilot_double[1].size();
        attr_vec_ds = DataSpace (1, dimsVec);
        att = mainGroup.createAttribute("OFDM_PILOT_IM", PredType::NATIVE_DOUBLE, attr_vec_ds);
	att.write(PredType::NATIVE_DOUBLE, &attr_pilot_double[1][0]);

	// Number of Clients
        attr_data = cfg->nClSdrs;
        att = mainGroup.createAttribute("CL_NUM", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data); 

	// Client antenna polarization
        attr_data = cfg->clSdrCh;
        att = mainGroup.createAttribute("CL_CH_PER_RADIO", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data); 

	// Client AGC enable flag
        attr_data = cfg->clAgcEn;
        att = mainGroup.createAttribute("CL_AGC_EN", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data); 

	// RX Gain RF channel A
        attr_data_double = cfg->clRxgainA;
        att = mainGroup.createAttribute("CL_RX_GAIN_A", PredType::NATIVE_DOUBLE, attr_ds);
        att.write(PredType::NATIVE_DOUBLE, &attr_data_double);

	// TX Gain RF channel A
        attr_data_double = cfg->clTxgainA;
        att = mainGroup.createAttribute("CL_TX_GAIN_A", PredType::NATIVE_DOUBLE, attr_ds);
        att.write(PredType::NATIVE_DOUBLE, &attr_data_double);

	// RX Gain RF channel B
        attr_data_double = cfg->clRxgainB;
        att = mainGroup.createAttribute("CL_RX_GAIN_B", PredType::NATIVE_DOUBLE, attr_ds);
        att.write(PredType::NATIVE_DOUBLE, &attr_data_double);

	// TX Gain RF channel B
        attr_data_double = cfg->clTxgainB;
        att = mainGroup.createAttribute("CL_TX_GAIN_B", PredType::NATIVE_DOUBLE, attr_ds);
        att.write(PredType::NATIVE_DOUBLE, &attr_data_double);

	// Client frame schedule (vec of strings)
	cStrArray.clear();
	att_vector = cfg->clFrames;
	dimsVec[0] = att_vector.size();
        attr_vec_ds = DataSpace (1, dimsVec);
        att = mainGroup.createAttribute("CL_FRAME_SCHED", strdatatype, attr_vec_ds);
	//Convert the vector into a C string array - Because the input function ::write requires that.
	for(int index = 0; index < att_vector.size(); ++index) { cStrArray.push_back(att_vector[index].c_str()); }
        if(!(cfg->clFrames).empty()) { att.write(strdatatype, (void*)&cStrArray[0]); }

	// Set of client SDR IDs (vec of strings)
	cStrArray.clear();
	att_vector = cfg->cl_sdr_ids;
	dimsVec[0] = att_vector.size();
        attr_vec_ds = DataSpace (1, dimsVec);
        att = mainGroup.createAttribute("CL_SDR_ID", strdatatype, attr_vec_ds);
	//Convert the vector into a C string array - Because the input function ::write requires that.
	for(int index = 0; index < att_vector.size(); ++index) { cStrArray.push_back(att_vector[index].c_str()); }
        if(!(cfg->cl_sdr_ids).empty()) { att.write(strdatatype, (void*)&cStrArray[0]); }

	// Data modulation
        att = mainGroup.createAttribute("CL_MODULATION", strdatatype, attr_ds);
        att.write(strdatatype, cfg->clDataMod); 

	// Number of uplink symbols per frame
	attr_data = cfg->ulSymsPerFrame;
        att = mainGroup.createAttribute("UL_SYMS", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data);

	// ********************* //

        pilot_prop.close();
        pilot_dataspace->close();
        pilot_dataset->close();
        if(cfg->ulSymsPerFrame > 0)
        {
            DataSpace *data_dataspace = new DataSpace(5, dims_data, max_dims_data);
            data_prop.setChunk(5, cdims);

            DataSet *data_dataset = new DataSet(file->createDataSet("/Data/UplinkData", 
                                         PredType::STD_I16BE, *data_dataspace, data_prop) );
 
            data_prop.close();
            data_dataspace->close();
            data_dataset->close();
            config_dump_data = 1;
            delete data_dataspace;
            data_dataset->close();
        }
        else
        {
            config_dump_data_size = 0;
            config_dump_data = 0;	
        }
        //status = H5Gclose(group_id);
        //if (status < 0 ) return status;
        delete pilot_dataspace;
        pilot_dataset->close();
        file->close();
    }
    // catch failure caused by the H5File operations
    catch(FileIException error)
    {
	error.printError();
	return -1;
    }

    // catch failure caused by the DataSet operations
    catch(DataSetIException error)
    {
	error.printError();
	return -1;
    }

    // catch failure caused by the DataSpace operations
    catch(DataSpaceIException error)
    {
	error.printError();
	return -1;
    }
    maxFrameNumber = MAX_FRAME_INC;
    return 0;  // successfully terminated
}

void Recorder::openHDF5()
{
    file->openFile(hdf5name, H5F_ACC_RDWR);
    // Get Dataset for pilot and check the shape of it
    pilot_dataset = new DataSet(file->openDataSet("/Data/Pilot_Samples"));
 
    // Get the dataset's dataspace and creation property list.
    pilot_filespace = new DataSpace(pilot_dataset->getSpace());
    pilot_prop = pilot_dataset->getCreatePlist();

    // Get information to obtain memory dataspace.
    ndims = pilot_filespace->getSimpleExtentNdims();
    herr_t status_n = pilot_filespace->getSimpleExtentDims(dims_pilot);

    int cndims_pilot;
    if (H5D_CHUNKED == pilot_prop.getLayout())
        cndims_pilot = pilot_prop.getChunk(ndims, cdims_pilot);
    cout << "dim pilot chunk = " << cndims_pilot << endl;
    cout << "New Pilot Dataset Dimension " << ndims << "," << dims_pilot[0] << "," << dims_pilot[1] << "," << dims_pilot[2] << "," << dims_pilot[3] << "," << dims_pilot[4] << endl;
    pilot_filespace->close();
    // Get Dataset for DATA (If Enabled) and check the shape of it
    if((config_dump_data == 1))
    {
        data_dataset = new DataSet(file->openDataSet("/Data/UplinkData"));

        data_filespace = new DataSpace(data_dataset->getSpace());
        data_prop = data_dataset->getCreatePlist();
        ndims = data_filespace->getSimpleExtentNdims();
        status_n = data_filespace->getSimpleExtentDims(dims_data);

        int cndims_data;
        if (H5D_CHUNKED == data_prop.getLayout())
            cndims_data = data_prop.getChunk(ndims, cdims_data);
        cout << "dim data chunk = " << cndims_data << endl;;
        cout << "New Data Dataset Dimension " << ndims << "," << dims_data[0] << "," << dims_data[1] << "," << dims_data[2] << ","<< dims_data[3] << "," << dims_data[4] << endl;
        data_filespace->close();
    }
}

void Recorder::closeHDF5()
{
    int frameNumber = maxFrameNumber;

    // Resize Pilot Dataset
    dims_pilot[0] = frameNumber;
    pilot_dataset->extend(dims_pilot);
    pilot_prop.close();
    pilot_filespace->close();
    pilot_dataset->close();

    // Resize Data Dataset (If Needed)
    if(config_dump_data == 1)
    {
        dims_data[0] = frameNumber;
        data_dataset->extend(dims_data);
        data_prop.close();
        data_filespace->close();
        data_dataset->close();
    }


    file->close();

    cout << "Closing HDF5, " << frameNumber + 1 << " frames saved." << endl;
}

Recorder::~Recorder()
{
    this->closeHDF5();
}

void Recorder::start()
{
    // if ENABLE_CPU_ATTACH, attach main thread to core 0
#ifdef ENABLE_CPU_ATTACH
    if(pin_to_core(0) != 0)
    {
        perror("pinning main thread to core 0 failed");
        exit(0);
    }
#endif
    // creare socket buffer and socket threads
    void* socket_buffer_ptrs[rx_thread_num];
    int* socket_buffer_status_ptrs[rx_thread_num];
    for(int i = 0; i < rx_thread_num; i++)
    {
        socket_buffer_ptrs[i] = socket_buffer_[i].buffer.data();
        socket_buffer_status_ptrs[i] = socket_buffer_[i].buffer_status.data();
    }
    std::vector<pthread_t> recv_thread = receiver_->startRecv(socket_buffer_ptrs, 
        socket_buffer_status_ptrs, socket_buffer_[0].buffer_status.size(), socket_buffer_[0].buffer.size(), 1);
    moodycamel::ProducerToken ptok(task_queue_);
    moodycamel::ConsumerToken ctok(message_queue_);

    // counter for print log
    int miss_count = 0;
    int total_count = 0;

    Event_data events_list[dequeue_bulk_size];
    int ret = 0;
    while(true)
    {
        // get a bulk of events
        ret = message_queue_.try_dequeue_bulk(ctok, events_list, dequeue_bulk_size);
        total_count++;
        if(total_count == 1e7)
        {
            total_count = 0;
            miss_count = 0;
        }
        if(ret == 0)
        {
            miss_count++;
            continue;
        }
        // handle each event
        for(int bulk_count = 0; bulk_count < ret; bulk_count ++)
        {
            Event_data& event = events_list[bulk_count];

            // if EVENT_RX_SYMBOL, do crop
            if(event.event_type == EVENT_RX_SYMBOL)
            {
                int offset = event.data;
                Event_data do_crop_task;
                do_crop_task.event_type = TASK_CROP;
                do_crop_task.data = offset;
                if ( !task_queue_.try_enqueue(ptok, do_crop_task ) ) {
                    printf("need more memory\n");
                    if ( !task_queue_.enqueue(ptok, do_crop_task ) ) {
                        printf("crop task enqueue failed\n");
                        exit(0);
                    }
                }

                
            }
        }
    }
}

void* Recorder::taskThread(void* context)
{
    
    Recorder* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    moodycamel::ConcurrentQueue<Event_data>* task_queue_ = &(obj_ptr->task_queue_);
    int tid = ((EventHandlerContext *)context)->id;
    printf("task thread %d starts\n", tid);
    
    // attach task threads to specific cores
#ifdef ENABLE_CPU_ATTACH
    int offset_id = obj_ptr->rx_thread_num + 1;
    int tar_core_id = tid + offset_id;
    if(tar_core_id >= 18)
        tar_core_id = (tar_core_id - 18) + 36;
    if(pin_to_core(tar_core_id) != 0)
    {
        printf("stitch thread %d to core %d failed\n", tid, tar_core_id);
        exit(0);
    }
#endif

    obj_ptr->task_ptok[tid].reset(new moodycamel::ProducerToken(obj_ptr->message_queue_));

    int total_count = 0;
    int miss_count = 0;
    Event_data event;
    bool ret = false;
    while(true)
    {
        ret = task_queue_->try_dequeue(event);
        if(tid == 0)
            total_count++;
        if(tid == 0 && total_count == 1e6)
        {
            // print the task queue miss rate if required
            //printf("thread 0 task dequeue miss rate %f, queue length %d\n", (float)miss_count / total_count, task_queue_->size_approx());
            total_count = 0;
            miss_count = 0;
        }
        if(!ret)
        {
            if(tid == 0)
                miss_count++;
            continue;
        }

        // do different tasks according to task type
        if(event.event_type == TASK_CROP)
        {   
            obj_ptr->record(tid, event.data);
        }
    }
}

// do Crop
herr_t Recorder::record(int tid, int offset)
{
    int buffer_frame_num = cfg->symbolsPerFrame * SOCKET_BUFFER_FRAME_NUM * cfg->getNumAntennas();
    int buffer_id = offset / buffer_frame_num;
    offset = offset - buffer_id * buffer_frame_num;
    // read info
    char* cur_ptr_buffer = socket_buffer_[buffer_id].buffer.data() + offset * cfg->getPackageLength();
    int ant_id, frame_id, symbol_id, cell_id;
    frame_id = *((int *)cur_ptr_buffer);
    symbol_id = *((int *)cur_ptr_buffer + 1);
    cell_id = *((int *)cur_ptr_buffer + 2);
    ant_id = *((int *)cur_ptr_buffer + 3);
#if DEBUG_PRINT
    printf("record thread %d process frame_id %d, symbol_id %d, cell_id %d, ant_id %d\n", tid, frame_id, symbol_id, cell_id, ant_id);
            printf("record samples: %d %d %d %d %d %d %d %d ....\n",*((short *)cur_ptr_buffer+9), 
							   *((short *)cur_ptr_buffer+10),
                                                           *((short *)cur_ptr_buffer+11),
                                                           *((short *)cur_ptr_buffer+12),
                                                           *((short *)cur_ptr_buffer+13),
                                                           *((short *)cur_ptr_buffer+14),
                                                           *((short *)cur_ptr_buffer+15),
                                                           *((short *)cur_ptr_buffer+16)); 
#endif
    short* cur_ptr_buffer_short = (short*)(cur_ptr_buffer + sizeof(int) * 4);
    try
    {
	Exception::dontPrint();

        // Update the max frame number.
        // Note that the 'frameid' might be out of order.
        if(frame_id > maxFrameNumber)
        {
            // Open the hdf5 file if we haven't.
            closeHDF5();
            openHDF5();
            maxFrameNumber = maxFrameNumber + MAX_FRAME_INC;
        }
        hsize_t count[5];
        hsize_t hdfoffset[5];

        hdfoffset[0] = frame_id;
        hdfoffset[1] = 0;  // will change later after we use cell_id
        hdfoffset[2] = cfg->getClientId(frame_id, symbol_id);
        hdfoffset[3] = ant_id;
        hdfoffset[4] = 0;

        count[0] = 1; // frame
        count[1] = 1; // cell
        count[2] = 1; // ue num (subframe) 
        count[3] = 1; // ant 
        count[4] = dims_pilot[4]; // data size 

        if(cfg->isPilot(frame_id, symbol_id))
        {   
            assert(pilot_dataset >= 0);
            // Are we going to extend the dataset?
            if(frame_id >= dims_pilot[0])
            {
                dims_pilot[0] = dims_pilot[0] + config_pilot_extent_step; // 400 is a threshold.
                pilot_dataset->extend(dims_pilot);
                std::cout << "FrameId " << frame_id << ", (Pilot) Extent to " << dims_pilot[0] << " Frames" << std::endl;
            }
            // Select a hyperslab in extended portion of the dataset
            pilot_filespace = new DataSpace(pilot_dataset->getSpace());
            pilot_filespace->selectHyperslab(H5S_SELECT_SET, count, hdfoffset);

            // define memory space
            DataSpace *pilot_memspace = new DataSpace(5, count, NULL);
            pilot_dataset->write(cur_ptr_buffer_short, PredType::NATIVE_INT16, *pilot_memspace, *pilot_filespace);
            pilot_filespace->close();
            delete pilot_memspace;
        }
        else if(cfg->isData(frame_id, symbol_id)) 
        {
            
            assert(data_dataset >= 0);
            // Are we going to extend the dataset?
            if(frame_id >= dims_data[0])
            {
                dims_data[0] = dims_data[0] + config_data_extent_step;
                data_dataset->extend(dims_data);
                printf("(Data) Extent to %d Frames\n",dims_data[0]);
            }

            hdfoffset[2] = cfg->getUlSFIndex(frame_id, symbol_id);
            count[0] = 1;
            count[1] = 1;
            count[2] = 1;
            count[3] = 1;
            count[4] = dims_data[4];

            // Select a hyperslab in extended portion of the dataset
            data_filespace = new DataSpace(data_dataset->getSpace());
            data_filespace->selectHyperslab(H5S_SELECT_SET, count, hdfoffset);

            // define memory space
            DataSpace *data_memspace = new DataSpace(5, count, NULL);
            data_dataset->write(cur_ptr_buffer_short, PredType::NATIVE_INT16, *data_memspace, *data_filespace);
            delete data_memspace;
        }
    } 
    // catch failure caused by the H5File operations
    catch(FileIException error)
    {
	error.printError();
	return -1;
    }

    // catch failure caused by the DataSet operations
    catch(DataSetIException error)
    {
	error.printError();
        std::cout << "DataSet: Failed to record pilots from frame " << frame_id << " , UE " << cfg->getClientId(frame_id, symbol_id) << " antenna " << ant_id << " dims_pilot[4] " << dims_pilot[4] << std::endl;
        std::cout << "Dataset Dimension is " << ndims << "," << dims_pilot[0] << "," << dims_pilot[1] << "," << dims_pilot[2] << "," << dims_pilot[3] << "," << dims_pilot[4] << endl;
	return -1;
    }

    // catch failure caused by the DataSpace operations
    catch(DataSpaceIException error)
    {
	error.printError();
	return -1;
    }

    // after finish
    socket_buffer_[buffer_id].buffer_status[offset] = 0; // now empty
    return 0;
}

