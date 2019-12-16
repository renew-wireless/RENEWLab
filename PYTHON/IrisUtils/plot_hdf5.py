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
             Clayton Shepard: cws@rice.edu
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

def verify_hdf5(hdf5, default_frame=100, ant_i =0, user_i=0, n_frm_st=0, thresh=0.001, deep_inspect=False, sub_sample=1):
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
    pilots_avail = len(pilot_samples) > 0
    ul_data_avail = len(uplink_samples) > 0

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
    symbol_length = int(metadata['SYMBOL_LEN'])
    num_pilots = int(metadata['PILOT_NUM'])
    num_cl = int(metadata['CL_NUM'])
    cp = int(metadata['CP_LEN'])
    prefix_len = int(metadata['PREFIX_LEN'])
    postfix_len = int(metadata['POSTFIX_LEN'])
    z_padding = prefix_len + postfix_len
    offset = int(prefix_len)

    print(" symbol_length = {}, cp = {}, prefix_len = {}, postfix_len = {}, z_padding = {}".format(symbol_length, cp, prefix_len, postfix_len, z_padding))

    print("********     verify_hdf5(): Calling csi_from_pilots and frame_sanity    *********")
    n_ue = num_cl
    frm_plt = min(default_frame, pilot_samples.shape[0] + n_frm_st)

    if deep_inspect:
        csi_from_pilots_start = time.time()
        csi_mat, match_filt, sub_fr_strt, cmpx_pilots, k_lts, n_lts = csi_from_pilots(
            pilot_samples, z_padding, frm_st_idx=n_frm_st, frame_to_plot=frm_plt, ref_ant=ant_i)
        csi_from_pilots_end = time.time()

        frame_sanity_start = time.time()
        match_filt_clr, frame_map, f_st = hdf5_lib.frame_sanity(match_filt, k_lts, n_lts, n_frm_st, frm_plt, plt_ant=ant_i)
        frame_sanity_end = time.time()

        print(">>>> csi_from_pilots time: %f \n" % ( csi_from_pilots_end - csi_from_pilots_start) )
        print(">>>> frame_sanity time: %f \n" % ( frame_sanity_end - frame_sanity_start) )

    # PLOTTER
    # Plot pilots or data or both
    fig, axes = plt.subplots(nrows=6, ncols=len(data_types_avail), squeeze=False, figsize=(10, 8))
    for idx, ftype in enumerate(data_types_avail):
        if ftype == "PILOTS":
            axes[0, idx].set_title('PILOTS - Cell 0')
            samples = pilot_samples 
            num_cl_tmp = num_pilots  # number of UEs to plot data for

        elif ftype == "UL_DATA":

            axes[0, idx].set_title('UPLINK DATA - Cell 0')
            samples = uplink_samples
            num_cl_tmp = samples.shape[2]  # number of UEs to plot data for

        # Compute CSI from IQ samples
        # Samps: #Frames, #Cell, #Users, #Antennas, #Samples
        # CSI:   #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Subcarrier
        # For correlation use a fft size of 64
        print("*verify_hdf5(): Calling samps2csi with fft_size = 64, offset = {}, bound = cp = 0 *".format(offset))
        csi, samps = hdf5_lib.samps2csi(samples, num_cl_tmp, symbol_length, fft_size=64, offset=offset, bound=0, cp=0, sub=sub_sample)

        # Correlation (Debug plot useful for checking sync)
        amps = np.mean(np.abs(samps[:, 0, user_i, 0, ant_i, :]), axis=1)
        pilot_frames = [i for i in range(len(amps)) if amps[i] > thresh]
        if len(pilot_frames) > 0: 
            corr_ref_frame = pilot_frames[len(pilot_frames) // 2]
        else:
            print("no valid frames where found. Decision threshold for amplitude was %f" % thresh)
            return 
        cellCSI = csi[:, 0, :, :, :, :]     # First cell
        userCSI = np.mean(cellCSI[:, :, :, :, :], 2)
        corr_total, sig_sc = calCorr(userCSI, np.transpose(np.conj(userCSI[corr_ref_frame, :, :, :]), (1, 0, 2) ) )

        # Compute CSI from IQ samples
        # Samps: #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Samples
        # CSI:   #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Subcarrier
        # For looking at the whole picture, use a fft size of whole symbol_length as fft window (for visualization),
        # and no offset
        print("*verify_hdf5():Calling samps2csi *AGAIN*(?) with fft_size = symbol_length, no offset*")
        csi, samps = hdf5_lib.samps2csi(samples, num_cl_tmp, symbol_length, fft_size=symbol_length, offset=0, bound=0, cp=0, sub=sub_sample)

        # Verify default_frame does not exceed max number of collected frames
        ref_frame = min(default_frame - n_frm_st, samps.shape[0])
        ant_plt = ant_i
        user_plt = user_i
        # Plotter
        # Samps Dimensions: (Frame, Cell, User, Pilot Rep, Antenna, Sample)
        axes[0, idx].set_ylabel('Frame %d ant %d (Re)' %( (ref_frame + n_frm_st), ant_plt) )
        axes[0, idx].plot(np.real(samps[ref_frame, 0, user_plt, 0, ant_plt, :]))

        axes[1, idx].set_ylabel('Frame %d ant %d (Im)' %( (ref_frame + n_frm_st), ant_plt) )
        axes[1, idx].plot(np.imag(samps[ref_frame, 0, user_plt, 0, ant_plt, :]))

        axes[2, idx].set_ylabel('All Frames ant %d (Re)' %ant_plt )
        axes[2, idx].plot(np.real(samps[:, 0, user_plt, 0, ant_plt, :]).flatten())

        axes[3, idx].set_ylabel('All Frames ant %d (Im)' %ant_plt)
        axes[3, idx].plot(np.imag(samps[:, 0, user_plt, 0, ant_plt, :]).flatten())

        axes[4, idx].set_ylabel('Amplitude')
        for i in range(samps.shape[4]):
            axes[4, idx].plot(np.mean(np.abs(samps[:, 0, user_plt, 0, i, :]), axis=1).flatten())
        axes[4, idx].set_xlabel('Sample')

        axes[5, idx].set_ylabel('Correlation with Frame %d' % corr_ref_frame)
        axes[5, idx].set_ylim([0, 1.1])
        axes[5, idx].set_title('Cell %d offset %d' % (0, offset))
        for u in range(num_cl_tmp):
            axes[5, idx].plot(corr_total[pilot_frames, u], label="user %d"%u)
        axes[5, idx].legend(loc='lower right', frameon=False)
        axes[5, idx].set_xlabel('Frame')

    if deep_inspect:

        #plots:

        print("Plotting the results:\n")
        n_cell = match_filt_clr.shape[1]
        n_ue = match_filt_clr.shape[2]

        # plot a frame:
        fig, axes = plt.subplots(nrows=n_cell, ncols=n_ue, squeeze=False)
        fig.suptitle('MF Frame # {} Antenna # {}'.format(ref_frame, ant_i))
        for n_c in range(n_cell):
            for n_u in range(n_ue):
                axes[n_c, n_u].stem(match_filt_clr[ref_frame - hdf5.n_frm_st, n_c, n_u, ant_i, :])
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
    else:
        plt.show()

def analyze_hdf5(hdf5, frame=10, cell=0, zoom=0, pl=0):
    '''
    Calculates and plots achievable rates from hdf5 traces

    '''

    metadata = hdf5.metadata
    pilot_samples = hdf5.pilot_samples
    symbol_length = int(metadata['SYMBOL_LEN'])
    rate = float(metadata['RATE'])
    symbol_num = int(metadata['BS_FRAME_LEN'])
    timestep = symbol_length*symbol_num/rate
    num_cl = int(metadata['CL_NUM'])
    num_pilots = int(metadata['PILOT_NUM'])
    cp = int(metadata['CP_LEN'])
    prefix_len = int(metadata['PREFIX_LEN'])
    postfix_len = int(metadata['POSTFIX_LEN'])
    offset = prefix_len

    # compute CSI for each user and get a nice numpy array
    # Returns csi with Frame, User, LTS (there are 2), BS ant, Subcarrier
    #also, iq samples nicely chunked out, same dims, but subcarrier is sample.
    csi, _ = hdf5_lib.samps2csi(pilot_samples, num_pilots, symbol_length, offset=offset)
    csi = csi[:, cell, :, :, :, :]
    # zoom in too look at behavior around peak (and reduce processing time)
    if zoom > 0:
        csi = csi[frame-zoom:frame+zoom, :, :, :, :]
        # recenter the plots (otherwise it errors)
        frame = zoom
    noise = csi[:, -1, :, :, :]  # noise is last set of data.
    # don't include noise, average over both LTSs
    userCSI = np.mean(csi[:, :num_cl, :, :, :], 2)

    # compute beamweights based on the specified frame.
    conjbws = np.transpose(
        np.conj(userCSI[frame, :, :, :]), (1, 0, 2))
    zfbws = np.empty(
        (userCSI.shape[2], userCSI.shape[1], userCSI.shape[3]), dtype='complex64')
    for sc in range(userCSI.shape[3]):
        zfbws[:, :, sc] = np.linalg.pinv(
            userCSI[frame, :, :, sc])

    downlink = True
    # calculate capacity based on these weights
    # these return total capacity, per-user capacity, per-user/per-subcarrier capacity,..
    #    SINR, single-user capacity(no inter-user interference), and SNR

    # conjcap_total,conjcap_u,conjcap_sc,conjSINR,conjcap_su_sc,conjcap_su_u,conjSNR
    conj = calCapacity(userCSI, noise, conjbws, downlink=downlink)
    # zfcap_total,zfcap_u,zfcap_sc,zfSINR,zfcap_su_sc,zfcap_su_u,zfSNR
    zf = calCapacity(userCSI, noise, zfbws, downlink=downlink)

    _, demmel = calDemmel(userCSI)

    # plot stuff
    subf_conj = conj[-2]
    subf_zf = zf[-2]
    mubf_conj = conj[1]
    mubf_zf = zf[1]
    fig1, axes1 = plt.subplots(nrows=2, ncols=2, squeeze=False, figsize=(10, 8))
    for j in range(2):
        axes1[0, 0].plot(np.arange(0, csi.shape[0]*timestep, timestep)[:csi.shape[0]], mubf_conj[:,j], label = 'Conj User: {}'.format(j) )
    for j in range(2):
        axes1[0, 1].plot(np.arange(0, csi.shape[0]*timestep, timestep)[:csi.shape[0]], mubf_zf[:,j], label = 'ZF User: {}'.format(j) )
    axes1[0,0].legend(loc='upper right', ncol=1, frameon=False)
    axes1[0,0].set_xlabel('Time (s)', fontsize=18)
    axes1[0,0].set_ylabel('MUBF User Achievable Rate (bps/Hz)', fontsize=18)
    axes1[0,1].legend(loc='upper right', ncol=1, frameon=False)
    axes1[0,1].set_xlabel('Time (s)')
    for j in range(2):
        axes1[1, 0].plot(np.arange(0, csi.shape[0]*timestep, timestep)[:csi.shape[0]], subf_conj[:,j], label = 'Conj User: {}'.format(j) )
    for j in range(2):
        axes1[1, 1].plot(np.arange(0, csi.shape[0]*timestep, timestep)[:csi.shape[0]], subf_zf[:,j], label = 'ZF User: {}'.format(j) )
    axes1[1,0].legend(loc='upper right', ncol=1, frameon=False)
    axes1[1,0].set_xlabel('Time (s)', fontsize=18)
    axes1[1,0].set_ylabel('SUBF User Achievable Rate (bps/Hz)', fontsize=18)
    axes1[1,1].legend(loc='upper right', ncol=1, frameon=False)
    axes1[1,1].set_xlabel('Time (s)')
    #axes1[1].set_ylabel('Per User Achievable Rate (bps/Hz)')


    # demmel number
    plt.figure(1000*pl+3, figsize=(10, 8))
    plt.plot(
            np.arange(0, csi.shape[0]*timestep, timestep)[:csi.shape[0]], demmel[:, 7])
    # plt.ylim([0,2])
    plt.xlabel('Time (s)')
    plt.ylabel('Demmel condition number, Subcarrier 7')
    plt.show()
    pl += 1

    del csi  # free the memory

def main():
    # Tested with inputs: ./data_in/Argos-2019-3-11-11-45-17_1x8x2.hdf5 300  (for two users)
    #                     ./data_in/Argos-2019-3-30-12-20-50_1x8x1.hdf5 300  (for one user) 
    parser = OptionParser()
    parser.add_option("--deep-inspect", action="store_true", dest="deep_inspect", help="Run script without analysis", default= False)
    parser.add_option("--ref-frame", type="int", dest="ref_frame", help="Frame number to plot", default=0)
    parser.add_option("--ref-ant", type="int", dest="ref_ant", help="Reference antenna", default=0)
    parser.add_option("--ref-user", type="int", dest="ref_user", help="Reference User", default=0)
    parser.add_option("--n-frames", type="int", dest="n_frames_to_inspect", help="Number of frames to inspect", default=2000)
    parser.add_option("--sub-sample", type="int", dest="sub_sample", help="Sub sample rate", default=1)
    parser.add_option("--thresh", type="float", dest="thresh", help="Ampiltude Threshold for valid frames", default=0.001)
    parser.add_option("--frame-start", type="int", dest="fr_strt", help="Starting frame. Must have set n_frames_to_inspect first and make sure fr_strt is within boundaries ", default=0)
    parser.add_option("--verify-trace", action="store_true", dest="verify", help="Run script without analysis", default= True)
    parser.add_option("--analyze-trace", action="store_true", dest="analyze", help="Run script without analysis", default= False)
    (options, args) = parser.parse_args()

    deep_inspect = options.deep_inspect
    n_frames_to_inspect = options.n_frames_to_inspect
    ref_ant = options.ref_ant
    ref_frame = options.ref_frame
    ref_user = options.ref_user
    thresh = options.thresh
    fr_strt = options.fr_strt
    verify = options.verify
    analyze = options.analyze
    sub_sample = options.sub_sample

    filename = sys.argv[1]
    scrpt_strt = time.time()

    if n_frames_to_inspect == 0:
        print("WARNING: No frames_to_inspect given. Will process the whole dataset.") 

    if ref_frame > n_frames_to_inspect:
        print("WARNING: Attempted to inspect a frame at an index larger than the no. of requested frames: ref_frame:{} >  n_frames_to_inspect:{}. ".format(
                ref_frame, n_frames_to_inspect))
        print("Setting the frame to inspect to 0")
        ref_frame = 0
 
    if (ref_frame > fr_strt + n_frames_to_inspect) or (ref_frame < fr_strt) :
        print("WARNING: Attempted to inspect a frame at an index larger than the no. of requested frames +  or at an index smaller than the required start of the frames: ref_frame:{} > n_frames_to_inspect:{} or ref_frame:{} <  fr_strt:{}. ".format(
                ref_frame, n_frames_to_inspect, ref_frame, fr_strt))
        print("Setting the frame to inspect/plot to {}".format(fr_strt))
        ref_frame = fr_strt

    print(">> frame to plot = {}, ref. ant = {}, ref. user = {}, no. of frames to inspect = {}, starting frame = {} <<".format(ref_frame, ref_ant, ref_user, n_frames_to_inspect, fr_strt))

    # Instantiate
    hdf5 = hdf5_lib(filename, n_frames_to_inspect, fr_strt)

    if verify:
        verify_hdf5(hdf5, ref_frame, ref_ant, ref_user, fr_strt, thresh, deep_inspect,sub_sample)
    if analyze:
        analyze_hdf5(hdf5)
    scrpt_end = time.time()
    print(">>>> Script Duration: time: %f \n" % ( scrpt_end - scrpt_strt) )


if __name__ == '__main__':
    main()

