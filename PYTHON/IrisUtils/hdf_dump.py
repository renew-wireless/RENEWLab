#!/usr/bin/python3
"""
 hdfDump.py

 Plotting from HDF5 file
 Script to analyze recorded hdf5 file from channel sounding (see Sounder/).
 Usage format is:
    ./hdfDump.py <hdf5_file_name> <frame_to_plot (optional, default=100)>

 Example:
    ./hdfDump.py ../Sounder/logs/test-hdf5.py 150


 Author(s): Oscar Bejarano: obejarano@rice.edu
            Rahman Doost-Mohamamdy: doost@rice.edu

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
import numpy as np
import h5py
import matplotlib.pyplot as plt
import collections
import time
from channel_analysis import *
 # add frame_start for plot indexing!
def frame_sanity(match_filt, k_lts, n_lts, st_frame = 0, frame_to_plot = 0, plt_ant=0, cp=16):
    """ 
    Creates a map of the frames per antenna. 3 categories: Good frames, bad frames, probably partial frames.
    Good frames are those where all k_lts peaks are present and spaced n_lts samples apart.
    Bad frames are those with random peaks. 
    Potentially partial frames are those with some peaks at the right positions.
    This is a random event. Some frames may have accidentally some peaks at the right places.
    First the largest peak is detected, peaks at +1/-1 (probably due to multipath) and +CP/-CP samples are cleared out.
    Then, the positions of the largest k_lts peaks are determined.
    Finally, the function checks if these k_lts peaks are at the correct n_lts offstes.  
    Disclaimer: This function is good only for a high SNR scenario!
    """
    
    debug = False
    dtct_eal_tx = True                  # Detect early transmission: further processing if this is desired
    n_frame = match_filt.shape[0]       # no. of captured frames
    n_cell = match_filt.shape[1]        # no. of cells
    n_ue = match_filt.shape[2]          # no. of UEs 
    n_ant = match_filt.shape[3]         # no. of BS antennas
    n_corr = match_filt.shape[4]        # no. of corr. samples
    
    if debug:
        print("frame_sanity(): n_frame = {}, n_cell = {}, n_ue = {}, n_ant = {}, n_corr = {}, k_lts = {}".format(
        n_frame, n_cell, n_ue, n_ant, n_corr, k_lts) )
    

    # clean up the matched filter of extra peaks:
    mf_amax = np.argmax(match_filt, axis = -1)
    base_arr = np.arange(0,k_lts*n_lts, n_lts)
    for i in range(n_frame):
        for j in range(n_cell):
            for k in range(n_ue):
                for l in range(n_ant):
                    mfa = mf_amax[i,j,k,l] 
                   # NB: addition: try to detect early packets: TEST it!
                    if dtct_eal_tx:
                            for ik in range(k_lts):
                                mf_prev = match_filt[i,j,k,l, (mfa - n_lts) if (mfa - n_lts) >= 0 else 0] 
                                if 1 - np.abs(match_filt[i,j,k,l, mfa] -  mf_prev)/match_filt[i,j,k,l, mfa] >= 0.89:
                                    mfa = (mfa - n_lts) if (mfa - n_lts) >= 0 else 0
                                else:
                                    break
                    # NB: addition: Clean everything right of the largest peak.
                    match_filt[i,j,k,l, mfa+1:] = 0         # we don't care about the peaks after the largest.
                    # misleading peaks seem to apear at +- argmax and argmax -1/+1/+2 CP and 29-30
                    for m in range(base_arr.shape[0]):                        
                        adj_idx1 = (mfa - 1) - base_arr[m]
                        adj_idx2 = (mfa + 1) - base_arr[m]
                        cp_idx1 = (mfa + cp) - base_arr[m]
                        cp_idx2 = (mfa + 1  + cp) - base_arr[m]
                        cp_idx3 = (mfa + -1  + cp) - base_arr[m]
                        idx_30 = (mfa + 30) - base_arr[m]
                        idx_29 = (mfa + 29) - base_arr[m]
                        if adj_idx1 >= 0 and adj_idx2 >=0 and adj_idx2 < n_corr:
                            match_filt[i,j,k,l, adj_idx1 ] = 0
                            match_filt[i,j,k,l, adj_idx2 ] = 0
                        if (cp_idx1 >=0) and (cp_idx1 < n_corr) and (cp_idx2 < n_corr) and (cp_idx3 < n_corr):
                            match_filt[i,j,k,l, cp_idx1 ] = 0
                            match_filt[i,j,k,l, cp_idx2 ] = 0
                            match_filt[i,j,k,l, cp_idx3 ] = 0
                        if (idx_30 >=0) and (idx_30 < n_corr) and (idx_29 >=0) and (idx_29 < n_corr):
                            match_filt[i,j,k,l,idx_30] = 0
                            match_filt[i,j,k,l,idx_29] = 0
                            
    # get the k_lts largest peaks and their position
    k_max = np.sort(match_filt, axis = -1)[:,:,:,:, -k_lts:]
    k_amax =np.argsort(match_filt, axis = -1)[:,:,:,:, -k_lts:]
    # If the frame is good, the largerst peak is at the last place of k_amax
    lst_pk_idx = np.expand_dims(k_amax[:,:,:,:,-1], axis = 4)
    lst_pk_idx = np.tile(lst_pk_idx, (1,1,1,1,base_arr.shape[0]))
    # create an array with indices n_lts apart from each other relative to lst_pk_idx 
    pk_idx = lst_pk_idx - np.tile(base_arr[::-1], (n_frame, n_cell, n_ue, n_ant,1))
    #subtract. In case of a good frame their should only be zeros in every postion
    idx_diff = k_amax - pk_idx
    frame_map = (idx_diff ==0).astype(np.int)
    # count the 0 and non-zero elements and reshape to n_frame-by-n_ant
    frame_map = np.sum(frame_map, axis =-1)
    #NB:
    zetas = frame_map*n_lts
    f_st = mf_amax - zetas
    print("f_st = {}".format(f_st[frame_to_plot - st_frame,0,0,:]))    
    if debug: 
        print("frame_sanity(): Shape of k_max.shape = {}, k_amax.shape = {}, lst_pk_idx.shape = {}".format(
                k_max.shape, k_amax.shape, lst_pk_idx.shape) )
        print("frame_sanity(): k_amax = {}".format(k_amax))
        print("frame_sanity(): frame_map.shape = {}\n".format(frame_map.shape))
        print(k_amax[frame_to_plot - st_frame,0,0,plt_ant,:])  
        print(idx_diff[frame_to_plot - st_frame,0,0,plt_ant,:])    
    
    frame_map[frame_map == 1] = -1
    frame_map[frame_map >= (k_lts -1)] = 1
    frame_map[frame_map > 1] = 0
    if debug:
        print("frame_sanity(): frame_map = \n{}".format(frame_map)) 
        print(frame_to_plot - st_frame)
    
    #print results:
    n_rf = frame_map.size
    n_gf = frame_map[frame_map == 1].size
    n_bf = frame_map[frame_map == -1].size
    n_pr = frame_map[frame_map == 0].size
    print("===================== frame_sanity(): frame status: ============")
    print("Out of total {} received frames: \nGood frames: {}\nBad frames: {}\nProbably Partially received or corrupt: {}".format(
            n_rf, n_gf, n_bf, n_pr,))
    print("===================== ============================= ============")
    
    return match_filt, frame_map, f_st

class hdfDump:

    def __init__(self, filename, n_frames_to_inspect=0, n_fr_insp_st = 0):
        self.h5file = None
        self.filename = filename
        self.h5struct = []
        self.data = []
        self.metadata = {}
        self.samples = {}
        self.n_frm_st = n_fr_insp_st                                # index of last frame
        self.n_frm_end = self.n_frm_st + n_frames_to_inspect    # index of last frame in the range of n_frames_to_inspect

    def get_hdf5(self):
        """
        Get the most recent log file, open it if necessary.
        """
        if (not self.h5file) or (self.filename != self.h5file.filename):
            # if it's closed, e.g. for the C version, open it
            print('Opening %s...' % self.filename)
            try:
                self.h5file = h5py.File(self.filename, 'r')
            except OSError:
                print("File not found. Terminating program now")
                sys.exit(0)
        # return self.h5file

    def parse_hdf5(self):
        """
        Parse file to retrieve metadata and data.
        HDF5 file has been written in DataRecorder.cpp (in Sounder folder)

        Output:
            Data (hierarchy):
                -Path
                -Pilot_Samples
                    --Samples
                -UplinkData
                    --Samples
                -Attributes
                        {FREQ, RATE, SYMBOL_LEN_NO_PAD, PREFIX_LEN, POSTFIX_LEN, SYMBOL_LEN, FFT_SIZE, CP_LEN,
                        BEACON_SEQ_TYPE, PILOT_SEQ_TYPE, BS_HUB_ID, BS_SDR_NUM_PER_CELL, BS_SDR_ID, BS_NUM_CELLS,
                        BS_CH_PER_RADIO, BS_FRAME_SCHED, BS_RX_GAIN_A, BS_TX_GAIN_A, BS_RX_GAIN_B, BS_TX_GAIN_B,
                        BS_BEAMSWEEP, BS_BEACON_ANT, BS_NUM_ANT, BS_FRAME_LEN, CL_NUM, CL_CH_PER_RADIO, CL_AGC_EN,
                        CL_RX_GAIN_A, CL_TX_GAIN_A, CL_RX_GAIN_B, CL_TX_GAIN_B, CL_FRAME_SCHED, CL_SDR_ID,
                        CL_MODULATION, UL_SYMS}

        Dimensions of input sample data (as shown in DataRecorder.cpp in Sounder):
            - Pilots
                dims_pilot[0] = maxFrame
                dims_pilot[1] = number of cells
                dims_pilot[2] = number of UEs
                dims_pilot[3] = number of antennas (at BS)
                dims_pilot[4] = samples per symbol * 2 (IQ)

            - Uplink Data
                dims_data[0] = maxFrame
                dims_data[1] = number of cells
                dims_data[2] = uplink symbols per frame
                dims_data[3] = number of antennas (at BS)
                dims_data[4] = samples per symbol * 2 (IQ)
        """
        g = self.h5file
        prefix = ''
        self.data = collections.defaultdict(lambda: collections.defaultdict(dict))
        for key in g.keys():
            item = g[key]
            path = '{}/{}'.format(prefix, key)
            keys = [i for i in item.keys()]
            if isinstance(item[keys[0]], h5py.Dataset):  # test for dataset
                # Path
                self.data['path'] = path
                # Attributes
                for attribute in item.attrs.keys():
                    # Store attributes
                    self.data['Attributes'][attribute] = item.attrs[attribute]
                # Pilot and UplinkData Samples
                for k in keys:
                    if not isinstance(item[k], h5py.Group):
                        # dataset = np.array(item[k].value)  # dataset.value has been deprecated. dataset[()] instead
                        dtst_ptr = item[(k)]
                        n_frm = np.abs(self.n_frm_end - self.n_frm_st)
                    
                        # check if the number fof requested frames and, upper and lower bounds make sense
                        # also check if end_frame > strt_frame:
                        if (n_frm > 0 and self.n_frm_st >=0 and (self.n_frm_end >= 0 and self.n_frm_end > self.n_frm_st) ):
                            dataset = np.array(dtst_ptr[self.n_frm_st:self.n_frm_end,...])
                        else:
                            #if previous if Flase, do as usual:
                            print("WARNING: No frames_to_inspect given and/or boundries don't make sense. Will process the whole dataset.") 
                            dataset = np.array(dtst_ptr)

                        if type(dataset) is np.ndarray:
                            if dataset.size != 0:
                                if type(dataset[0]) is np.bytes_:
                                    dataset = [a.decode('ascii') for a in dataset]

                        # Store samples
                        self.data[k]['Samples'] = dataset

                    # for attribute in item[k].attrs.keys():
                        # # Store attributes
                        # self.data[k]['Attributes'][attribute] = item[k].attrs[attribute]

            else:
                raise Exception("No datasets found")

        return self.data

    def get_attributes(self):
        # Retrieve attributes, translate into python dictionary
        data = self.data

        # Check if attributes are there
        client_id = data['Attributes']['CL_SDR_ID'].astype(str)
        if client_id.size == 0:
            cl_present = False
            print('Client information not present. It is likely the client was run separately')
        else:
            cl_present = True

        bs_id = data['Attributes']['BS_SDR_ID'].astype(str)
        if bs_id.size == 0:
            raise Exception('Base Station information not present')

        # Data cleanup
        # In OFDM_DATA_CLx and OFDM_PILOT, we have stored both real and imaginary in same vector
        # (i.e., RE1,IM1,RE2,IM2...REm,IM,)
        # Pilots
        pilot_vec = data['Attributes']['OFDM_PILOT']
        # some_list[start:stop:step]
        I = pilot_vec[0::2]
        Q = pilot_vec[1::2]
        pilot_complex = I + Q * 1j

        if cl_present:
            # Time-domain OFDM data
            num_cl = np.squeeze(data['Attributes']['CL_NUM'])
            ofdm_data_time = []  # np.zeros((num_cl, 320)).astype(complex)
            for clIdx in range(num_cl):
                this_str = 'OFDM_DATA_TIME_CL' + str(clIdx)
                data_per_cl = np.squeeze(data['Attributes'][this_str])
                # some_list[start:stop:step]
                if np.any(data_per_cl):
                    # If data present
                    I = np.double(data_per_cl[0::2])
                    Q = np.double(data_per_cl[1::2])
                    IQ = I + Q * 1j
                    ofdm_data_time.append(IQ)

            # Frequency-domain OFDM data
            ofdm_data = []  # np.zeros((num_cl, 320)).astype(complex)
            for clIdx in range(num_cl):
                this_str = 'OFDM_DATA_CL' + str(clIdx)
                data_per_cl = np.squeeze(data['Attributes'][this_str])
                # some_list[start:stop:step]
                if np.any(data_per_cl):
                    # If data present
                    I = np.double(data_per_cl[0::2])
                    Q = np.double(data_per_cl[1::2])
                    IQ = I + Q * 1j
                    ofdm_data.append(IQ)

            # Populate dictionary
            self.metadata = {
                'CLIENT_PRESENT': cl_present,
                'FREQ': np.squeeze(data['Attributes']['FREQ']),
                'RATE': np.squeeze(data['Attributes']['RATE']),
                'SYM_LEN_NO_PAD': np.squeeze(data['Attributes']['SYMBOL_LEN_NO_PAD']),
                'PREFIX_LEN': np.squeeze(data['Attributes']['PREFIX_LEN']),
                'POSTFIX_LEN': np.squeeze(data['Attributes']['POSTFIX_LEN']),
                'SYM_LEN': np.squeeze(data['Attributes']['SYMBOL_LEN']),
                'FFT_SIZE': np.squeeze(data['Attributes']['FFT_SIZE']),
                'CP_LEN': np.squeeze(data['Attributes']['CP_LEN']),
                'BEACON_SEQ': np.squeeze(data['Attributes']['BEACON_SEQ_TYPE']).astype(str),
                'PILOT_SEQ': np.squeeze(data['Attributes']['PILOT_SEQ_TYPE']).astype(str),
                'BS_HUB_ID': np.squeeze(data['Attributes']['BS_HUB_ID']).astype(str),
                'BS_SDR_NUM_PER_CELL': np.squeeze(data['Attributes']['BS_SDR_NUM_PER_CELL']).astype(int),
                'BS_SDR_ID': np.squeeze(data['Attributes']['BS_SDR_ID']).astype(str),
                'BS_NUM_CELLS': np.squeeze(data['Attributes']['BS_NUM_CELLS']),
                'BS_CH_PER_RADIO': np.squeeze(data['Attributes']['BS_CH_PER_RADIO']),
                'BS_FRAME_SCHED': np.squeeze(data['Attributes']['BS_FRAME_SCHED']).astype(str),
                'BS_RX_GAIN_A': np.squeeze(data['Attributes']['BS_RX_GAIN_A']),
                'BS_TX_GAIN_A': np.squeeze(data['Attributes']['BS_TX_GAIN_A']),
                'BS_RX_GAIN_B': np.squeeze(data['Attributes']['BS_RX_GAIN_B']),
                'BS_TX_GAIN_B': np.squeeze(data['Attributes']['BS_TX_GAIN_B']),
                'BS_BEAMSWEEP': np.squeeze(data['Attributes']['BS_BEAMSWEEP']),
                'BS_BEACON_ANT': np.squeeze(data['Attributes']['BS_BEACON_ANT']),
                'BS_NUM_ANT': np.squeeze(data['Attributes']['BS_NUM_ANT']),
                'BS_FRAME_LEN': np.squeeze(data['Attributes']['BS_FRAME_LEN']),
                'NUM_CLIENTS': np.squeeze(data['Attributes']['CL_NUM']),
                'CL_CH_PER_RADIO': np.squeeze(data['Attributes']['CL_CH_PER_RADIO']),
                'CL_AGC_EN': np.squeeze(data['Attributes']['CL_AGC_EN']),
                'CL_RX_GAIN_A': np.squeeze(data['Attributes']['CL_RX_GAIN_A']),
                'CL_TX_GAIN_A': np.squeeze(data['Attributes']['CL_TX_GAIN_A']),
                'CL_RX_GAIN_B': np.squeeze(data['Attributes']['CL_RX_GAIN_B']),
                'CL_TX_GAIN_B': np.squeeze(data['Attributes']['CL_TX_GAIN_B']),
                'CL_FRAME_SCHED': np.squeeze(data['Attributes']['CL_FRAME_SCHED']).astype(str),
                'CL_SDR_ID': np.squeeze(data['Attributes']['CL_SDR_ID']).astype(str),
                'CL_MODULATION': np.squeeze(data['Attributes']['CL_MODULATION']).astype(str),
                'UL_SYMS': np.squeeze(data['Attributes']['UL_SYMS']),
                'OFDM_DATA_SC': np.squeeze(data['Attributes']['OFDM_DATA_SC']),
                'OFDM_PILOT_SC': np.squeeze(data['Attributes']['OFDM_PILOT_SC']),
                'OFDM_PILOT_SC_VALS': np.squeeze(data['Attributes']['OFDM_PILOT_SC_VALS']),
                'OFDM_PILOT_TIME': pilot_complex,
                'OFDM_DATA': ofdm_data,
                'OFDM_DATA_TIME': ofdm_data_time,
            }

        else:
            # Client not present
            # Populate dictionary
            self.metadata = {
                'CLIENT_PRESENT': cl_present,
                'FREQ': np.squeeze(data['Attributes']['FREQ']),
                'RATE': np.squeeze(data['Attributes']['RATE']),
                'SYM_LEN_NO_PAD': np.squeeze(data['Attributes']['SYMBOL_LEN_NO_PAD']),
                'PREFIX_LEN': np.squeeze(data['Attributes']['PREFIX_LEN']),
                'POSTFIX_LEN': np.squeeze(data['Attributes']['POSTFIX_LEN']),
                'SYM_LEN': np.squeeze(data['Attributes']['SYMBOL_LEN']),
                'FFT_SIZE': np.squeeze(data['Attributes']['FFT_SIZE']),
                'CP_LEN': np.squeeze(data['Attributes']['CP_LEN']),
                'BEACON_SEQ': np.squeeze(data['Attributes']['BEACON_SEQ_TYPE']).astype(str),
                'PILOT_SEQ': np.squeeze(data['Attributes']['PILOT_SEQ_TYPE']).astype(str),
                'BS_HUB_ID': np.squeeze(data['Attributes']['BS_HUB_ID']).astype(str),
                'BS_SDR_NUM_PER_CELL': np.squeeze(data['Attributes']['BS_SDR_NUM_PER_CELL']).astype(int),
                'BS_SDR_ID': np.squeeze(data['Attributes']['BS_SDR_ID']).astype(str),
                'BS_NUM_CELLS': np.squeeze(data['Attributes']['BS_NUM_CELLS']),
                'BS_CH_PER_RADIO': np.squeeze(data['Attributes']['BS_CH_PER_RADIO']),
                'BS_FRAME_SCHED': np.squeeze(data['Attributes']['BS_FRAME_SCHED']).astype(str),
                'BS_RX_GAIN_A': np.squeeze(data['Attributes']['BS_RX_GAIN_A']),
                'BS_TX_GAIN_A': np.squeeze(data['Attributes']['BS_TX_GAIN_A']),
                'BS_RX_GAIN_B': np.squeeze(data['Attributes']['BS_RX_GAIN_B']),
                'BS_TX_GAIN_B': np.squeeze(data['Attributes']['BS_TX_GAIN_B']),
                'BS_BEAMSWEEP': np.squeeze(data['Attributes']['BS_BEAMSWEEP']),
                'BS_BEACON_ANT': np.squeeze(data['Attributes']['BS_BEACON_ANT']),
                'BS_NUM_ANT': np.squeeze(data['Attributes']['BS_NUM_ANT']),
                'BS_FRAME_LEN': np.squeeze(data['Attributes']['BS_FRAME_LEN']),
                'NUM_CLIENTS': np.squeeze(data['Attributes']['CL_NUM']),
                'CL_MODULATION': np.squeeze(data['Attributes']['CL_MODULATION']).astype(str),
                'UL_SYMS': np.squeeze(data['Attributes']['UL_SYMS']),
                'OFDM_DATA_SC': np.squeeze(data['Attributes']['OFDM_DATA_SC']),
                'OFDM_PILOT_SC': np.squeeze(data['Attributes']['OFDM_PILOT_SC']),
                'OFDM_PILOT_SC_VALS': np.squeeze(data['Attributes']['OFDM_PILOT_SC_VALS']),
                'OFDM_PILOT_TIME': pilot_complex,
            }

    def get_samples(self, data_types_avail):
        # Retrieve samples, translate into python dictionary
        samples_pilots = []
        samples_ulData = []
        for idx, ftype in enumerate(data_types_avail):
            if ftype == "PILOTS":
                samples = self.data['Pilot_Samples']['Samples']
                samples_pilots = samples

            elif ftype == "UL_DATA":
                samples = self.data['UplinkData']['Samples']
                samples_ulData = samples

        self.samples = {'PILOT_SAMPS': samples_pilots,
                        'UL_SAMPS': samples_ulData,
                        }

    def verify_hdf5(self, default_frame=100, ant_i =0):
        """
        Plot data in file to verify contents.

        Input:
            default_frame: Index of frame to be plotted. Default to frame #100
        """
        plt.close("all")

        data = self.data

        # Check which data we have available
        data_types_avail = []
        pilots_avail = bool(data['Pilot_Samples'])
        ul_data_avail = bool(data['UplinkData'])
        offset = int(np.squeeze(data['Attributes']['PREFIX_LEN']))

        if pilots_avail:
            data_types_avail.append("PILOTS")
            print("PILOT Data Available")
        if ul_data_avail:
            data_types_avail.append("UL_DATA")
            print("Uplink Data Available")

        # Empty structure
        if not data_types_avail:
            raise Exception(' **** No pilots or uplink data found **** ')

        # Retrieve attributes
        freq = np.squeeze(data['Attributes']['FREQ'])
        rate = np.squeeze(data['Attributes']['RATE'])
        symbol_length = np.squeeze(data['Attributes']['SYMBOL_LEN'])
        num_cl = np.squeeze(data['Attributes']['CL_NUM'])
        cp = np.squeeze(data['Attributes']['CP_LEN'])
        prefix_len = np.squeeze(data['Attributes']['PREFIX_LEN'])
        postfix_len = np.squeeze(data['Attributes']['POSTFIX_LEN'])
        z_padding = prefix_len + postfix_len

        print(" symbol_length = {}, cp = {}, prefix_len = {}, postfix_len = {}, z_padding = {}".format(symbol_length, cp, prefix_len, postfix_len, z_padding))
       
        print("********     verify_hdf5(): Calling csi_from_pilots and frame_sanity    *********")
        samples_P = data['Pilot_Samples']['Samples']
        n_ue = num_cl
        frm_plt = min(default_frame, samples_P.shape[0] + self.n_frm_st)
        csi_from_pilots_start = time.time()
        csi_mat, match_filt, sub_fr_strt, cmpx_pilots, k_lts, n_lts = csi_from_pilots(
                samples_P, z_padding, frm_st_idx = self.n_frm_st, frame_to_plot = frm_plt, ref_ant =ant_i)
        csi_from_pilots_end = time.time()
        
        frame_sanity_start = time.time()
        match_filt_clr, frame_map, f_st = frame_sanity(match_filt, k_lts, n_lts, self.n_frm_st, frm_plt, plt_ant = ant_i)
        frame_sanity_end = time.time()
       
        print(">>>> csi_from_pilots time: %f \n" % ( csi_from_pilots_end - csi_from_pilots_start) )
        print(">>>> frame_sanity time: %f \n" % ( frame_sanity_end - frame_sanity_start) )
        
        # PLOTTER
        # Plot pilots or data or both
        fig, axes = plt.subplots(nrows=6, ncols=len(data_types_avail), squeeze=False)
        for idx, ftype in enumerate(data_types_avail):
            if ftype == "PILOTS":
                axes[0, idx].set_title('PILOTS - Cell 0')
                samples = data['Pilot_Samples']['Samples']
                num_cl_tmp = num_cl  # number of UEs to plot data for

            elif ftype == "UL_DATA":

                axes[0, idx].set_title('UPLINK DATA - Cell 0')
                samples = data['UplinkData']['Samples']
                num_cl_tmp = 1  # number of UEs to plot data for
            

            # Compute CSI from IQ samples
            # Samps: #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Samples
            # CSI:   #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Subcarrier
            # For correlation use a fft size of 64
            print("*verify_hdf5(): Calling samps2csi with fft_size = 64, offset = {}, bound = cp = 0 *".format(offset))
            csi, samps = samps2csi(samples, num_cl_tmp, symbol_length, fft_size=64, offset=offset, bound=0, cp=0)
            
           
            # Correlation (Debug plot useful for checking sync)
            amps = np.mean(np.abs(samps[:, 0, 0, 0, 0, :]), axis=1)
            pilot_frames = [i for i in range(len(amps)) if amps[i] > 0.001]
            #if len(pilot_frames) > 0: 
            ref_frame = pilot_frames[len(pilot_frames) // 2]
            #else: return 
            cellCSI = csi[:, 0, :, :, :, :]     # First cell
            userCSI = np.mean(cellCSI[:, :, :, :, :], 2)
            #print("====================== OLD STATS: ========================")
            corr_total, sig_sc = calCorr(userCSI, np.transpose(np.conj(userCSI[ref_frame, :, :, :]), (1, 0, 2) ) )
            #best_frames = [i for i in pilot_frames if corr_total[i, 0] > 0.99]
            #good_frames = [i for i in pilot_frames if corr_total[i, 0] > 0.95]
            #bad_frames = [i for i in pilot_frames if corr_total[i, 0] > 0.9 and corr_total[i, 0] <= 0.94]
            #worst_frames = [i for i in pilot_frames if corr_total[i, 0] < 0.9]
            #print("Good frames len: %d" % len(pilot_frames))
            #print("Amplitude of reference frame %d is %f" % (ref_frame, amps[ref_frame]))
            #print("num of best frames %d" % len(best_frames))
            #print("num of good frames %d" % len(good_frames))
            #print("num of bad frames   %d" % len(bad_frames))
            #print("num of worst frames   %d" % len(worst_frames))
            #print("===========================================================")
            
            # Compute CSI from IQ samples
            # Samps: #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Samples
            # CSI:   #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Subcarrier
            # For looking at the whole picture, use a fft size of whole symbol_length as fft window (for visualization),
            # and no offset
            print("*verify_hdf5():Calling samps2csi *AGAIN*(?) with fft_size = symbol_length, no offset*")
            csi, samps = samps2csi(samples, num_cl_tmp, symbol_length, fft_size=symbol_length, offset=0, bound=0, cp=0)

            # Verify default_frame does not exceed max number of collected frames
            frame_to_plot = min(default_frame - self.n_frm_st, samps.shape[0])
            ant_plt = ant_i
            # Plotter
            # Samps Dimensions: (Frame, Cell, User, Pilot Rep, Antenna, Sample)
            axes[0, idx].set_ylabel('Frame %d ant 0 (Re)' %(frame_to_plot + self.n_frm_st))
            axes[0, idx].plot(np.real(samps[frame_to_plot, 0, 0, 0, ant_plt, :]))

            axes[1, idx].set_ylabel('Frame %d ant 1 (Re)' %(frame_to_plot + self.n_frm_st))
            axes[1, idx].plot(np.real(samps[frame_to_plot, 0, 0, 0, ant_plt, :]))

            axes[2, idx].set_ylabel('All Frames ant 0 (Re)')
            axes[2, idx].plot(np.real(samps[:, 0, 0, 0, 0, :]).flatten())

            axes[3, idx].set_ylabel('All Frames ant 1 (Re)')
            axes[3, idx].plot(np.real(samps[:, 0, 0, 0, 1, :]).flatten())

            axes[4, idx].set_ylabel('Amplitude')
            for i in range(samps.shape[4]):
                axes[4, idx].plot(np.mean(np.abs(samps[:, 0, 0, 0, i, :]), axis=1).flatten())
            axes[4, idx].set_xlabel('Sample')

            axes[5, idx].set_ylabel('Correlation with Frame %d' % ref_frame)
            axes[5, idx].set_ylim([0, 1.1])
            axes[5, idx].set_title('Cell %d offset %d' % (0, offset))
            for u in range(num_cl_tmp):
                axes[5, idx].plot(corr_total[pilot_frames, u])
            axes[5, idx].set_xlabel('Frame')
        #plt.show()
        
        return csi_mat, match_filt_clr, frame_map, sub_fr_strt, cmpx_pilots, f_st
    
if __name__ == '__main__':
    # Tested with inputs: ./data_in/Argos-2019-3-11-11-45-17_1x8x2.hdf5 300  (for two users)
    #                     ./data_in/Argos-2019-3-30-12-20-50_1x8x1.hdf5 300  (for one user)
    scrpt_strt = time.time()
    if len(sys.argv) >1:
        if sys.argv[1] == "-h":
            print('>>> format: ./hdfPlot.py <filename> <frame_to_plot (optional, default=100)> <ref_antenna (optional, default=0)> <n_frames_to_inspect (optional, default=0)> <start_of_frames (optional, default=0)><<<')
            sys.exit(0)

        if len(sys.argv) > 6:
            print('Too many arguments! >>> format: ./hdfPlot.py <filename> <frame_to_plot (optional, default=100)> <ref_antenna (optional, default=0)> <n_frames_to_inspect (optional, default=0)> <start_of_frames (optional, default=0)> <<<')
            sys.exit(0)

        filename = sys.argv[1]
        frame_to_plot = None
        ref_ant = None
        if len(sys.argv) == 3:
            frame_to_plot = int(sys.argv[2])
            n_frames_to_inspect = 0
            n_f_st = 0
            if n_frames_to_inspect == 0:
                print("WARNING: No frames_to_inspect given. Will process the whole dataset.") 
        if len(sys.argv) == 4:
            frame_to_plot = int(sys.argv[2])
            ref_ant = int(sys.argv[3])
            n_frames_to_inspect = 0
            n_f_st = 0
            if frame_to_plot < 0:
                frame_to_plot = 0
                print("WARNING: Gave negative value for frame _to_plot, set it to {}".format(frame_to_plot))
            if ref_ant < 0:
                ref_ant = 0
                print("WARNING: Gave negative value for ref_ant, set it to {}.".format(ref_ant))
            if n_frames_to_inspect == 0:
                print("WARNING: No frames_to_inspect given. Will process the whole dataset.") 
                
        
        if len(sys.argv) == 5:
            frame_to_plot = int(sys.argv[2])
            ref_ant = int(sys.argv[3])
            n_frames_to_inspect = int(sys.argv[4])
            n_f_st = 0
            if frame_to_plot < 0:
                frame_to_plot = 0
                print("WARNING: Gave negative value for frame _to_plot, set it to {}.".format(frame_to_plot)) 
            if ref_ant < 0:
                ref_ant = 0
                print("WARNING: Gave negative value for ref_ant, set it to {}.".format(ref_ant))
            if n_frames_to_inspect <= 0:
                n_frames_to_inspect = 1
                print("WARNING: Gave negative/zero value for n_frames_to_inspect, set it to {}.".format(n_frames_to_inspect))               
                
            if frame_to_plot > n_frames_to_inspect:
                print("WARNING: Attempted to inspect a frame at an index larger than the no. of requested frames: frame_to_plot:{} >  n_frames_to_inspect:{}. ".format(
                        frame_to_plot, n_frames_to_inspect))
                print("Setting the frame to inspect to 0")
                frame_to_plot = 0
            
                
        if len(sys.argv) == 6:
            frame_to_plot = int(sys.argv[2])
            ref_ant = int(sys.argv[3])
            n_frames_to_inspect = int(sys.argv[4])
            n_f_st = int(sys.argv[5])

            if frame_to_plot < 0:
                frame_to_plot = 0
                print("WARNING: Gave negative value for frame _to_plot, set it to {}.".format(frame_to_plot))
            if ref_ant < 0:
                ref_ant = 0
                print("WARNING: Gave negative value for ref_ant, set it to {}.".format(ref_ant))
            if n_frames_to_inspect <= 0:
                n_frames_to_inspect = 1
                print("WARNING: Gave negative/zero value for n_frames_to_inspect, set it to {}.".format(n_frames_to_inspect))               
            if n_f_st < 0:
                n_f_st = 0
                print("WARNING: Gave negative value for start_of_frames, set it to {}.".format(n_f_st))
                               
            if (frame_to_plot > n_f_st + n_frames_to_inspect) or (frame_to_plot < n_f_st) :
                print("WARNING: Attempted to inspect a frame at an index larger than the no. of requested frames +  or at an index smaller than the required start of the frames: frame_to_plot:{} > n_frames_to_inspect:{} or frame_to_plot:{} <  n_f_st:{}. ".format(
                        frame_to_plot, n_frames_to_inspect, frame_to_plot, n_f_st))
                print("Setting the frame to inspect/plot to {}".format(n_f_st))
                frame_to_plot = n_f_st
            
        print(">> frame to plot = {}, ref. ant = {}, no. of frames to inspect = {}, starting frame = {} <<".format(frame_to_plot,ref_ant,n_frames_to_inspect, n_f_st))
        # Instantiate
        hdf5 = hdfDump(filename, n_frames_to_inspect, n_f_st)
        hdf5.get_hdf5()
        hdf5.parse_hdf5()

        # Check which data we have available
        data_types_avail = []
        pilots_avail = bool(hdf5.data['Pilot_Samples'])
        ul_data_avail = bool(hdf5.data['UplinkData'])

        if pilots_avail:
            data_types_avail.append("PILOTS")
            print("PILOT Data Available")
        if ul_data_avail:
            data_types_avail.append("UL_DATA")
            print("Uplink Data Available")

        # Empty structure
        if not data_types_avail:
            raise Exception(' **** No pilots or uplink data found **** ')

        hdf5.get_attributes()
        hdf5.get_samples(data_types_avail)

        raw_data = hdf5.data
        metadata = hdf5.metadata
        samples = hdf5.samples
      
        if frame_to_plot is not None and ref_ant is not None:
            csi_mat, match_filt_clr, frame_map, sub_fr_strt, cmpx_pilots, f_st = hdf5.verify_hdf5(frame_to_plot, ref_ant)
            
        elif frame_to_plot is not None and ref_ant is None:
            csi_mat, match_filt_clr, frame_map, sub_fr_strt, cmpx_pilots, f_st = hdf5.verify_hdf5(frame_to_plot)
            ref_ant = 0
        else:
            csi_mat, match_filt_clr, frame_map, sub_fr_strt, cmpx_pilots, f_st = hdf5.verify_hdf5()
            frame_to_plot = 0
            
    
    else:
        raise Exception("format: ./hdfPlot.py <filename>")
    
    scrpt_end = time.time()
    
    #plots:
    
    print("Plotting the results:\n")
    n_cell = match_filt_clr.shape[1]
    n_ue = match_filt_clr.shape[2]
   
    # plot a frame:
    fig, axes = plt.subplots(nrows=n_cell, ncols=n_ue, squeeze=False)
    fig.suptitle('MF Frame # {} Antenna # {}'.format(frame_to_plot, ref_ant)) 
    for n_c in range(n_cell):
        for n_u in range(n_ue):
            axes[n_c, n_u].stem(match_filt_clr[frame_to_plot - hdf5.n_frm_st, n_c,n_u,ref_ant,:])
            axes[n_c, n_u].set_xlabel('Samples')
            axes[n_c, n_u].set_title('Cell {} UE {}'.format(n_c, n_u))
            axes[n_c, n_u].grid(True)
    #plt.show()
    
    # plot frame_map:
    n_cell = frame_map.shape[1]
    n_ue = frame_map.shape[2]
    n_ant = frame_map.shape[3]
    
    # For some damm reason, if one of the subplots has all of the frames in the same state (good/bad/partial)
    # it chooses a random color to paint the whole subplot!
    # Below is some sort of remedy (will fail if SISO!):
    for n_c in range(n_cell):
        for n_u in range(n_ue):
            f_map = frame_map[:,n_c,n_u,:]
            n_gf = f_map[f_map == 1].size
            n_bf = f_map[f_map == -1].size
            n_pr = f_map[f_map == 0].size
            if n_gf == 0:
                frame_map[-1,n_c,n_u,-1] = 1
                print("No good frames! Colored the last frame of the last antenna Good for cell {} and UE {} to keep plotter happy!".format(n_c,n_u))
            if n_pr == 0:
                frame_map[0,n_c,n_u,-1] = 0
                print("No partial frames! Colored frame 0 of the last antenna for cell {} and UE {} Partial to keep plotter happy!".format(n_c,n_u))
            if n_bf == 0:
                frame_map[-1,n_c,n_u,0] = -1
                print("No bad frames! Colored the last frame of antenna 0 Bad for cell {} and UE {} to keep plotter happy!".format(n_c,n_u))
      
    fig, axes = plt.subplots(nrows=n_ue, ncols=n_cell, squeeze=False)
    c = []
    fig.suptitle('Frame Map')
    for n_c in range(n_cell):
        for n_u in range(n_ue):
            c.append( axes[n_u, n_c].imshow(frame_map[:,n_c,n_u,:].T, cmap=plt.cm.get_cmap('Blues', 3), interpolation='none',
                  extent=[hdf5.n_frm_st,hdf5.n_frm_end, n_ant,0],  aspect="auto") )
            axes[n_u, n_c].set_title('Cell {} UE {}'.format(n_c, n_u))
            axes[n_u, n_c].set_ylabel('Antenna #')
            axes[n_u, n_c].set_xlabel('Frame #')
            # Minor ticks
            axes[n_u, n_c].set_xticks(np.arange(hdf5.n_frm_st, hdf5.n_frm_end, 1), minor=True);
            axes[n_u, n_c].set_yticks(np.arange(0, n_ant, 1), minor=True);
            # Gridlines based on minor ticks
            axes[n_u, n_c].grid(which='minor', color='0.75', linestyle='-', linewidth=0.1)
    
            
    cbar = plt.colorbar(c[-1], ax=axes.ravel().tolist(), ticks=[-1, 0, 1], orientation = 'horizontal')       
    cbar.ax.set_xticklabels(['Bad Frame', 'Probably partial/corrupt', 'Good Frame'])  
    #plt.show()
    
    #plot F starts for each antenna
    sub_fr_strt = f_st
    n_frame = sub_fr_strt.shape[0]      # no. of captured frames
    n_cell = sub_fr_strt.shape[1]       # no. of cells
    n_ue = sub_fr_strt.shape[2]         # no. of UEs 
    n_ant = sub_fr_strt.shape[3]        # no. of BS antennas 
    sf_strts = np.reshape(sub_fr_strt, (n_frame*n_cell*n_ue,n_ant))
    
    fig, axes = plt.subplots(nrows=n_ue, ncols=n_cell, squeeze=False)
    fig.suptitle('Frames\' starting indices per antenna')
    for n_c in range(n_cell):
        for n_u in range(n_ue):
            sf_strts = sub_fr_strt[:,n_c,n_u,:]
            x_pl = np.arange(sf_strts.shape[0]) + hdf5.n_frm_st
            for j in range(n_ant):
                axes[n_u, n_c].plot(x_pl,sf_strts[:,j].flatten(), label = 'Antenna: {}'.format(j) )
            axes[n_u, n_c].legend(loc='lower right', ncol=8, frameon=False)
            axes[n_u, n_c].set_xlabel('Frame no.')
            axes[n_u, n_c].set_ylabel('Starting index')
            axes[n_u, n_c].grid(True)
    plt.show()
    
    print("** \tWARNING: If you attempt to plot a different frame after running this script, remember to subtract the frame_start you gave! **")
    print(">> \tE.g.: frame no. 1763 and frame_start = 1500 --> plot(match_filter_clr[<frame 1736 - 1500>, <cell>, <ue>, ref_antenna,:])\n")
    print(">>>> Script Duration: time: %f \n" % ( scrpt_end - scrpt_strt) )
