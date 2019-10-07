#!/usr/bin/python3
"""
 plot_hdf5.py

 Plotting from HDF5 file
 Script to analyze recorded hdf5 file from channel sounding (see Sounder/).
 Usage format is:
    ./plot_hdf5.py <hdf5_file_name>

 Example:
    ./plot_hdf5.py ../Sounder/logs/test-hdf5.py


 Author(s): 
             C. Nicolas Barati: nicobarati@rice.edu
             Oscar Bejarano: obejarano@rice.edu
             Rahman Doost-Mohammady: doost@rice.edu

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
from optparse import OptionParser
from channel_analysis import *
import hdf5_lib
from hdf5_lib import *

def verify_hdf5(hdf5, default_frame=100, ant_i =0, n_frm_st=0, do_corr=False):
    """
    Plot data in file to verify contents.

    Input:
        default_frame: Index of frame to be plotted. Default to frame #100
    """
    plt.close("all")
    data = hdf5.data
    metadata = hdf5.metadata
    pilot_samples = hdf5.pilot_samples
    uplink_samples = hdf5.uplink_samples

    # Check which data we have available
    data_types_avail = []
    pilots_avail = bool(data['Pilot_Samples'])
    ul_data_avail = bool(data['UplinkData'])

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
    freq = np.squeeze(metadata['FREQ'])
    rate = np.squeeze(metadata['RATE'])
    symbol_length = np.squeeze(metadata['SYMBOL_LEN'])
    num_cl = np.squeeze(metadata['CL_NUM'])
    cp = np.squeeze(metadata['CP_LEN'])
    prefix_len = np.squeeze(metadata['PREFIX_LEN'])
    postfix_len = np.squeeze(metadata['POSTFIX_LEN'])
    z_padding = prefix_len + postfix_len
    offset = int(np.squeeze(metadata['PREFIX_LEN']))

    print(" symbol_length = {}, cp = {}, prefix_len = {}, postfix_len = {}, z_padding = {}".format(symbol_length, cp, prefix_len, postfix_len, z_padding))
   
    print("********     verify_hdf5(): Calling csi_from_pilots and frame_sanity    *********")
    samples_P = data['Pilot_Samples']['Samples']
    n_ue = num_cl
    frm_plt = min(default_frame, samples_P.shape[0] + n_frm_st)
    
    if do_corr:
        csi_from_pilots_start = time.time()
        csi_mat, match_filt, sub_fr_strt, cmpx_pilots, k_lts, n_lts = hdf5.csi_from_pilots(
            samples_P, z_padding, frm_st_idx = n_frm_st, frame_to_plot = frm_plt, ref_ant =ant_i)
        csi_from_pilots_end = time.time()
    
        frame_sanity_start = time.time()
        match_filt_clr, frame_map, f_st = hdf5.frame_sanity(match_filt, k_lts, n_lts, n_frm_st, frm_plt, plt_ant = ant_i)
        frame_sanity_end = time.time()
   
        print(">>>> csi_from_pilots time: %f \n" % ( csi_from_pilots_end - csi_from_pilots_start) )
        print(">>>> frame_sanity time: %f \n" % ( frame_sanity_end - frame_sanity_start) )
    
    # PLOTTER
    # Plot pilots or data or both
    fig, axes = plt.subplots(nrows=6, ncols=len(data_types_avail), squeeze=False)
    for idx, ftype in enumerate(data_types_avail):
        if ftype == "PILOTS":
            axes[0, idx].set_title('PILOTS - Cell 0')
            samples = pilot_samples 
            num_cl_tmp = num_cl  # number of UEs to plot data for

        elif ftype == "UL_DATA":

            axes[0, idx].set_title('UPLINK DATA - Cell 0')
            samples = uplink_samples
            num_cl_tmp = samples.shape[2]  # number of UEs to plot data for

        # Compute CSI from IQ samples
        # Samps: #Frames, #Cell, #Users, #Antennas, #Samples
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
        corr_total, sig_sc = calCorr(userCSI, np.transpose(np.conj(userCSI[ref_frame, :, :, :]), (1, 0, 2) ) )
        
        # Compute CSI from IQ samples
        # Samps: #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Samples
        # CSI:   #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Subcarrier
        # For looking at the whole picture, use a fft size of whole symbol_length as fft window (for visualization),
        # and no offset
        print("*verify_hdf5():Calling samps2csi *AGAIN*(?) with fft_size = symbol_length, no offset*")
        csi, samps = samps2csi(samples, num_cl_tmp, symbol_length, fft_size=symbol_length, offset=0, bound=0, cp=0)

        # Verify default_frame does not exceed max number of collected frames
        frame_to_plot = min(default_frame - n_frm_st, samps.shape[0])
        ant_plt = ant_i
        # Plotter
        # Samps Dimensions: (Frame, Cell, User, Pilot Rep, Antenna, Sample)
        axes[0, idx].set_ylabel('Frame %d ant %d (Re)' %( (frame_to_plot + n_frm_st), ant_plt) )
        axes[0, idx].plot(np.real(samps[frame_to_plot, 0, 0, 0, ant_plt, :]))

        axes[1, idx].set_ylabel('Frame %d ant %d (Im)' %( (frame_to_plot + n_frm_st), ant_plt) )
        axes[1, idx].plot(np.imag(samps[frame_to_plot, 0, 0, 0, ant_plt, :]))

        axes[2, idx].set_ylabel('All Frames ant %d (Re)' %ant_plt )
        axes[2, idx].plot(np.real(samps[:, 0, 0, 0, ant_plt, :]).flatten())

        axes[3, idx].set_ylabel('All Frames ant %d (Im)' %ant_plt)
        axes[3, idx].plot(np.imag(samps[:, 0, 0, 0, ant_plt, :]).flatten())

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
    
    if do_corr:
        return csi_mat, match_filt_clr, frame_map, sub_fr_strt, cmpx_pilots, f_st
    else:
        plt.show()
        return 0

def main():
    # Tested with inputs: ./data_in/Argos-2019-3-11-11-45-17_1x8x2.hdf5 300  (for two users)
    #                     ./data_in/Argos-2019-3-30-12-20-50_1x8x1.hdf5 300  (for one user) 
    parser = OptionParser()
    parser.add_option("--do_corr", action="store_true", dest="do_corr", help="Run script without analysis", default= False)
    parser.add_option("--frame_to_plot", type="int", dest="frame_to_plot", help="Frame number to plot", default=0)
    parser.add_option("--ref_ant", type="int", dest="ref_ant", help="Reference antenna", default=0)
    parser.add_option("--n_frames_to_inspect", type="int", dest="n_frames_to_inspect", help="Number of frames to inspect", default=2000)
    parser.add_option("--fr_strt", type="int", dest="fr_strt", help="Starting frame. Must have set n_frames_to_inspect first and make sure fr_strt is within boundaries ", default=0)
    (options, args) = parser.parse_args()
    
    do_corr = options.do_corr 
    n_frames_to_inspect = options.n_frames_to_inspect
    ref_ant = options.ref_ant
    frame_to_plot = options.frame_to_plot
    fr_strt = options.fr_strt
    
    
    filename = sys.argv[1]
     
    scrpt_strt = time.time()
   
   

    if n_frames_to_inspect == 0:
        print("WARNING: No frames_to_inspect given. Will process the whole dataset.") 
       
    if frame_to_plot > n_frames_to_inspect:
        print("WARNING: Attempted to inspect a frame at an index larger than the no. of requested frames: frame_to_plot:{} >  n_frames_to_inspect:{}. ".format(
                frame_to_plot, n_frames_to_inspect))
        print("Setting the frame to inspect to 0")
        frame_to_plot = 0
 
    if (frame_to_plot > fr_strt + n_frames_to_inspect) or (frame_to_plot < fr_strt) :
        print("WARNING: Attempted to inspect a frame at an index larger than the no. of requested frames +  or at an index smaller than the required start of the frames: frame_to_plot:{} > n_frames_to_inspect:{} or frame_to_plot:{} <  fr_strt:{}. ".format(
                frame_to_plot, n_frames_to_inspect, frame_to_plot, fr_strt))
        print("Setting the frame to inspect/plot to {}".format(fr_strt))
        frame_to_plot = fr_strt
            
    print(">> frame to plot = {}, ref. ant = {}, no. of frames to inspect = {}, starting frame = {} <<".format(frame_to_plot,ref_ant,n_frames_to_inspect, fr_strt))
        
    # Instantiate
    hdf5 = hdf5_lib(filename, n_frames_to_inspect, fr_strt)
    hdf5.open_hdf5()
    hdf5.get_data()
    hdf5.get_metadata()

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

    if not do_corr:
        x = verify_hdf5(hdf5, frame_to_plot, ref_ant, fr_strt)
        sys.exit(0)
    else:
        csi_mat, match_filt_clr, frame_map, sub_fr_strt, cmpx_pilots, f_st = verify_hdf5(hdf5, frame_to_plot, ref_ant, fr_strt, do_corr)
    
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
            axes[n_u, n_c].set_xticks(np.arange(hdf5.n_frm_st, hdf5.n_frm_end, 1), minor=True)
            axes[n_u, n_c].set_yticks(np.arange(0, n_ant, 1), minor=True)
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

if __name__ == '__main__':
    main()

