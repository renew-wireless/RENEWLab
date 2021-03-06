#!/usr/bin/python3
"""
 plot_hdf5.py

 Plotting from HDF5 file
 Script to analyze recorded hdf5 file from channel sounding (see Sounder/).
 Usage format is:
    ./plot_hdf5.py <hdf5_file_name>

 Example:
    ./plot_hdf5.py ../Sounder/logs/test-hdf5.py


---------------------------------------------------------------------
 Copyright © 2018-2020. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
import numpy as np
import h5py
import matplotlib.pyplot as plt
import collections
import time
from find_lts import *
from optparse import OptionParser
from channel_analysis import *
import hdf5_lib
from hdf5_lib import *
import matplotlib
#matplotlib.use("Agg")

def verify_hdf5(hdf5, frame_i=100, cell_i=0, ofdm_sym_i=0, ant_i =0,
                user_i=0, ul_sf_i=0, subcarrier_i=10, offset=-1,
                dn_calib_offset=0, up_calib_offset=0, thresh=0.001,
                deep_inspect=False, corr_thresh=0.00, exclude_bs_nodes=[]):
    """Plot data in the hdf5 file to verify contents.

    Args:
        hdf5: An hdf5_lib object.
        frame_i: The index of the frame to be plotted.
        cell_i: The index of the hub where base station is connected.
        ofdm_sym_i: The index of the reference ofdm symbol in a pilot.
        ant_i: The index of the reference base station antenna.
        user_i: The index of the reference user.
    """
    plt.close("all")

    # Retrieve attributes
    n_frm_end = hdf5.n_frm_end
    n_frm_st = hdf5.n_frm_st
    metadata = hdf5.metadata
    symbol_length = int(metadata['SYMBOL_LEN'])
    num_pilots = int(metadata['PILOT_NUM'])
    num_cl = int(metadata['CL_NUM'])
    prefix_len = int(metadata['PREFIX_LEN'])
    postfix_len = int(metadata['POSTFIX_LEN'])
    z_padding = prefix_len + postfix_len
    if offset < 0: # if no offset is given use prefix from HDF5
        offset = int(prefix_len)
    fft_size = int(metadata['FFT_SIZE'])
    cp = int(metadata['CP_LEN'])
    rate = int(metadata['RATE'])
    pilot_type = metadata['PILOT_SEQ_TYPE'].astype(str)[0]
    nonzero_sc_size = metadata['DATA_SUBCARRIER_NUM']
    ofdm_pilot = np.array(metadata['OFDM_PILOT'])
    reciprocal_calib = np.array(metadata['RECIPROCAL_CALIB'])
    symbol_length_no_pad = symbol_length - z_padding
    num_pilots_per_sym = ((symbol_length_no_pad) // len(ofdm_pilot))
    n_ue = num_cl

    all_bs_nodes = set(range(hdf5.pilot_samples.shape[3]))
    plot_bs_nodes = list(all_bs_nodes - set(exclude_bs_nodes))
    pilot_samples = hdf5.pilot_samples[:, :, :, plot_bs_nodes, :]
    ul_data_avail = len(hdf5.uplink_samples) > 0
    if ul_data_avail:
        uplink_samples = hdf5.uplink_samples[:, :, :, plot_bs_nodes, :]
    noise_avail = len(hdf5.noise_samples) > 0
    if noise_avail:
        noise_samples = hdf5.noise_samples[:, :, :, plot_bs_nodes, :]

    frm_plt = min(frame_i, pilot_samples.shape[0] + n_frm_st)

    # Verify frame_i does not exceed max number of collected frames
    ref_frame = min(frame_i - n_frm_st, pilot_samples.shape[0])

    print("symbol_length = {}, offset = {}, cp = {}, prefix_len = {}, postfix_len = {}, z_padding = {}, pilot_rep = {}".format(symbol_length, offset, cp, prefix_len, postfix_len, z_padding, num_pilots_per_sym))

    # pilot_samples dimensions:
    # ( #frames, #cells, #pilot subframes or cl ant sending pilots, #bs nodes or # bs ant, #samps per frame * 2 for IQ )
    num_cl_tmp = num_pilots  # number of UEs to plot data for
    num_frames = pilot_samples.shape[0]
    num_cells = pilot_samples.shape[1]
    num_bs_ants = pilot_samples.shape[3]

    samps_mat = np.reshape(
            pilot_samples, (num_frames, num_cells, num_cl_tmp, num_bs_ants, symbol_length, 2))
    samps = (samps_mat[:, :, :, :, :, 0] +
            samps_mat[:, :, :, :, :, 1]*1j)*2**-15

    # Correlation (Debug plot useful for checking sync)
    good_ants = []
    insp_ants = [] # antennas to be inspected
    if ant_i > num_bs_ants - 1:
        insp_ants = range(samps.shape[3])
    else:
        insp_ants = [ant_i]
    for i in insp_ants:
        amps = np.mean(np.abs(samps[:, 0, user_i, i, :]), axis=1)
        pilot_frames = [i for i in range(len(amps)) if amps[i] > thresh]
        if len(pilot_frames) > 0:
            good_ants = good_ants + [i]
        else:
            print("no valid frames where found in antenna %d. Decision threshold (average pilot amplitude) was %f" % (i, thresh))
    if len(good_ants) == 0:
        print("no valid frames found in data belonging to user %d. Exitting ..." % user_i)
        return 

    # Compute CSI from IQ samples
    # Samps: #Frames, #Cell, #Users, #Antennas, #Samples
    # CSI:   #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Subcarrier
    # For correlation use a fft size of 64
    print("*verify_hdf5(): Calling samps2csi with fft_size = {}, offset = {}, bound = {}, cp = {} *".format(fft_size, offset, z_padding, cp))
    csi, _ = hdf5_lib.samps2csi(pilot_samples, num_cl_tmp, symbol_length, fft_size=fft_size, offset=offset, bound=z_padding,
                                cp=cp, sub=1, pilot_type=pilot_type, nonzero_sc_size=nonzero_sc_size)

    cellCSI = csi[:, cell_i, :, :, :, :]
    if corr_thresh > 0.0: 
        bad_nodes = find_bad_nodes(cellCSI, corr_thresh=corr_thresh,
                                   user=user_i)
        if bad_nodes:
            print(">>> Warning! List of bad nodes (1-based): {bad_nodes}".
                  format(bad_nodes=bad_nodes))
        else:
            print(">>> All Iris nodes are good!")

    if ofdm_sym_i >= num_pilots_per_sym:  # if out of range index, do average
        userCSI = np.mean(cellCSI[:, :, :, :, :], 2)
    else:
        userCSI = cellCSI[:, :, ofdm_sym_i, :, :]
    corr_total, sig_sc = calCorr(userCSI, np.transpose(np.conj(userCSI[ref_frame, :, :, :]), (1, 0, 2) ) )

    # Plotter
    # Plot pilots
    for i in insp_ants:
        fig1, axes1 = plt.subplots(nrows=2, ncols=1, squeeze=False, figsize=(10, 8))
        axes1[0, 0].set_title('Pilots IQ - Cell %d - Antenna %d - User %d'%(cell_i, i, user_i))
        # Samps Dimensions: (Frame, Cell, User, Pilot Rep, Antenna, Sample)
        axes1[0, 0].set_ylabel('Frame %d (IQ)' %( (ref_frame + n_frm_st)) )
        axes1[0, 0].plot(np.real(samps[ref_frame, cell_i, user_i, i, :]))
        axes1[0, 0].plot(np.imag(samps[ref_frame, cell_i, user_i, i, :]))

        axes1[1, 0].set_ylabel('All Frames (I)')
        axes1[1, 0].plot(np.real(samps[:, cell_i, user_i, i, :]).flatten())
        axes1[1, 0].plot(np.imag(samps[:, cell_i, user_i, i, :]).flatten())

    fig2, axes2 = plt.subplots(nrows=3, ncols=1, squeeze=False, figsize=(10, 8))
    axes2[0, 0].set_title('Pilot CSI Stats Across Frames- Cell %d - User %d - Subcarrier %d' % (cell_i, user_i, subcarrier_i))
    axes2[0, 0].set_ylabel('Magnitude')
    for i in range(csi.shape[4]):
        axes2[0, 0].plot(np.abs(userCSI[:, user_i, i, subcarrier_i]).flatten(), label="ant %d" % plot_bs_nodes[i])
    axes2[0, 0].legend(loc='lower right', frameon=False)
    axes2[0, 0].set_xlabel('Frame')

    axes2[1, 0].set_ylabel('Phase')
    for i in range(csi.shape[4]):
        axes2[1, 0].plot(np.angle(userCSI[:, user_i, i, subcarrier_i]).flatten(), label="ant %d" % plot_bs_nodes[i])
    axes2[1, 0].legend(loc='lower right', frameon=False)
    axes2[1, 0].set_ylim(-np.pi, np.pi)
    axes2[1, 0].set_xlabel('Frame')

    axes2[2, 0].set_ylabel('Correlation with Frame %d' % ref_frame)
    axes2[2, 0].set_ylim([0, 1.1])
    axes2[2, 0].set_title('Cell %d offset %d' % (0, offset))
    for u in range(num_cl_tmp):
        axes2[2, 0].plot(corr_total[pilot_frames, u], label="user %d"%u)
    axes2[2, 0].legend(loc='lower right', frameon=False)
    axes2[2, 0].set_xlabel('Frame')

    if reciprocal_calib:
        # frame, downlink(0)-uplink(1), antennas, subcarrier
        csi_u = csi
        csi_d = csi
        if up_calib_offset != offset:
            csi_u,_ = hdf5_lib.samps2csi(pilot_samples, num_cl_tmp, symbol_length, fft_size=fft_size, offset=up_calib_offset,
                                         bound=z_padding, cp=cp, sub=1, pilot_type=pilot_type, nonzero_sc_size=nonzero_sc_size)
        if dn_calib_offset != offset:
            csi_d,_ = hdf5_lib.samps2csi(pilot_samples, num_cl_tmp, symbol_length, fft_size=fft_size, offset=dn_calib_offset,
                                        bound=z_padding, cp=cp, sub=1, pilot_type=pilot_type, nonzero_sc_size=nonzero_sc_size)
        calib_corrected_csi = np.zeros(csi_d.shape, dtype='complex64')
        calib_corrected_csi[:, :, 0, :, :, :] = csi_d[:, :, 0, :, :, :]
        calib_corrected_csi[:, :, 1, :, :, :] = csi_u[:, :, 1, :, :, :]
        calib_corrected_csi_cell0 = calib_corrected_csi[:, 0, :, :, :, :]
        # TODO: add option for averaging across repeated pilots vs individual pilots
        #calibCSI = np.mean(calib_corrected_csi_cell0, 2) # average several csi copies
        calibCSI = calib_corrected_csi_cell0[:, :, 0, :, :] # take first csi
        calib_mat = np.divide(calibCSI[:, 0, :, :], calibCSI[:, 1, :, :])

        fig3, axes3 = plt.subplots(nrows=4, ncols=1, squeeze=False, figsize=(10, 8))
        axes3[0, 0].set_title('Reciprocity Calibration Factor Across Frames - Cell 0 - Subcarrier %d' % subcarrier_i)

        axes3[0, 0].set_ylabel('Magtinute (ant %d)' % (ant_i))
        axes3[0, 0].plot(np.abs(calib_mat[:, ant_i, subcarrier_i]).flatten(), label='')
        axes3[0, 0].set_xlabel('Frame')
        axes3[0, 0].legend(frameon=False)

        axes3[1, 0].set_ylabel('Phase (ant %d)' % (ant_i))
        axes3[1, 0].plot(np.angle(calib_mat[:, ant_i, subcarrier_i]).flatten())
        axes3[1, 0].set_ylim(-np.pi, np.pi)
        axes3[1, 0].set_xlabel('Frame')
        axes3[1, 0].legend(frameon=False)
        axes3[1, 0].grid()

        axes3[2, 0].set_ylabel('Magnitude')
        for i in range(calib_mat.shape[1]):
            axes3[2, 0].plot(np.abs(calib_mat[:, i, subcarrier_i]).flatten(), label="ant %d" % plot_bs_nodes[i])
        axes3[2, 0].set_xlabel('Frame')
        axes3[2, 0].legend(loc='lower right', frameon=False)

        axes3[3, 0].set_ylabel('Phase')
        for i in range(calib_mat.shape[1]):
            axes3[3, 0].plot(np.angle(calib_mat[:, i, subcarrier_i]).flatten(), label="ant %d" % plot_bs_nodes[i])
        axes3[3, 0].set_xlabel('Frame')
        axes3[3, 0].set_ylim(-np.pi, np.pi)
        axes3[3, 0].legend(loc='lower right', frameon=False)
        axes3[3, 0].grid()

        fig4, axes4 = plt.subplots(nrows=4, ncols=1, squeeze=False, figsize=(10, 8))
        axes4[0, 0].set_title('Reciprocity Calibration Factor Across Subcarriers - Cell 0 - Frame %d' % ref_frame)
        axes4[0, 0].set_ylabel('Magnitude ant %d' % (ant_i))
        axes4[0, 0].plot(np.abs(calib_mat[ref_frame, ant_i, :]).flatten())
        axes4[0, 0].set_xlabel('Subcarrier')

        axes4[1, 0].set_ylabel('Phase ant %d' % (ant_i))
        axes4[1, 0].plot(np.angle(calib_mat[ref_frame, ant_i, :]).flatten())
        axes4[1, 0].set_ylim(-np.pi, np.pi)
        axes4[1, 0].set_xlabel('Subcarrier')

        axes4[2, 0].set_ylabel('Magnitude')
        for i in range(calib_mat.shape[1]):
            axes4[2, 0].plot(np.abs(calib_mat[ref_frame, i, :]).flatten(), label="ant %d" % plot_bs_nodes[i])
        axes4[2, 0].set_xlabel('Subcarrier')
        axes4[2, 0].legend(loc='lower right', frameon=False)

        axes4[3, 0].set_ylabel('Phase')
        for i in range(calib_mat.shape[1]):
            axes4[3, 0].plot(np.angle(calib_mat[ref_frame, i, :]).flatten(), label="ant %d" % plot_bs_nodes[i])
        axes4[3, 0].set_xlabel('Subcarrier')
        axes4[3, 0].set_ylim(-np.pi, np.pi)
        axes4[3, 0].legend(loc='lower right', frameon=False)
        plt.show()

    else:
        # Plot UL data symbols
        if ul_data_avail > 0:
            fig4, axes4 = plt.subplots(nrows=2, ncols=1, squeeze=False, figsize=(10, 8))
            num_cl_tmp = uplink_samples.shape[2]  # number of UEs to plot data for

            # UL Samps: #Frames, #Cell, #Users, #Uplink Symbol, #Antennas, #Samples
            # For looking at the whole picture, use a fft size of whole symbol_length as fft window (for visualization),
            # and no offset
            #print("*verify_hdf5():Calling samps2csi *AGAIN*(?) with fft_size = symbol_length, no offset*")
            #_, uplink_samps = hdf5_lib.samps2csi(uplink_samples, num_cl_tmp, symbol_length, fft_size=symbol_length, offset=0, bound=0, cp=0, sub=sub_sample)
            samps_mat = np.reshape(
                    uplink_samples, (uplink_samples.shape[0], uplink_samples.shape[1], num_cl_tmp, uplink_samples.shape[3], symbol_length, 2))
            uplink_samps = (samps_mat[:, :, :, :, :, 0] +
                    samps_mat[:, :, :, :, :, 1]*1j)*2**-15


            # Samps Dimensions: (Frame, Cell, User, Pilot Rep, Antenna, Sample)
            axes4[0, 0].set_title('Uplink Data IQ - Cell %d - Antenna %d - Symbol %d' % (cell_i, ant_i, ul_sf_i))
            axes4[0, 0].set_ylabel('Frame %d (IQ)' % ref_frame)
            axes4[0, 0].plot(np.real(uplink_samps[ref_frame, cell_i, ul_sf_i, ant_i, :]))
            axes4[0, 0].plot(np.imag(uplink_samps[ref_frame, cell_i, ul_sf_i, ant_i, :]))

            axes4[1, 0].set_ylabel('All Frames (IQ)')
            axes4[1, 0].plot(np.real(uplink_samps[:, cell_i, ul_sf_i, ant_i, :]).flatten())
            axes4[1, 0].plot(np.imag(uplink_samps[:, cell_i, ul_sf_i, ant_i, :]).flatten())


        if deep_inspect:
            filter_pilots_start = time.time()
            match_filt, k_lts, n_lts, cmpx_pilots, lts_seq_orig = hdf5_lib.filter_pilots(pilot_samples, z_padding, fft_size = fft_size, cp = cp)
            filter_pilots_end = time.time()

            frame_sanity_start = time.time()
            match_filt_clr, frame_map, f_st, peak_map = hdf5_lib.frame_sanity(match_filt, k_lts, n_lts, n_frm_st, frm_plt, plt_ant=ant_i, cp = cp)
            frame_sanity_end = time.time()
            print(">>>> filter_pilots time: %f \n" % ( filter_pilots_end - filter_pilots_start) )
            print(">>>> frame_sanity time: %f \n" % ( frame_sanity_end - frame_sanity_start) )

            # Find LTS peaks across frame
            snr_start = time.time()
            n_frame = pilot_samples.shape[0]
            n_cell = pilot_samples.shape[1]
            n_ue = pilot_samples.shape[2]
            n_ant = pilot_samples.shape[3]
            seq_found = np.zeros((n_frame, n_cell, n_ue, n_ant))

            td_pwr_dbm_noise = np.empty_like(pilot_samples[:, :, :, :, 0], dtype=float)
            td_pwr_dbm_signal = np.empty_like(pilot_samples[:, :, :, :, 0], dtype=float)
            snr = np.empty_like(pilot_samples[:, :, :, :, 0], dtype=float)

            for frameIdx in range(n_frame):    # Frame
                for cellIdx in range(n_cell):  # Cell
                    for ueIdx in range(n_ue):  # UE
                        for bsAntIdx in range(n_ant):  # BS ANT

                            I = pilot_samples[frameIdx, cellIdx, ueIdx, bsAntIdx, 0:symbol_length * 2:2] / 2 ** 15
                            Q = pilot_samples[frameIdx, cellIdx, ueIdx, bsAntIdx, 1:symbol_length * 2:2] / 2 ** 15
                            IQ = I + (Q * 1j)
                            tx_pilot, lts_pks, lts_corr, pilot_thresh, best_pk = pilot_finder(IQ, pilot_type, flip=True,
                                                                                              pilot_seq=ofdm_pilot)
                            # Find percentage of LTS peaks within a symbol
                            # (e.g., in a 4096-sample pilot symbol, we expect 64, 64-long sequences... assuming no CP)
                            # seq_found[frameIdx, cellIdx, ueIdx, bsAntIdx] = 100 * (lts_pks.size / num_pilots_per_sym)
                            seq_found[frameIdx, cellIdx, ueIdx, bsAntIdx] = 100 * (peak_map[frameIdx, cellIdx, ueIdx, bsAntIdx] / num_pilots_per_sym)  # use matched filter analysis output

                            # Compute Power of Time Domain Signal
                            rms = np.sqrt(np.mean(IQ * np.conj(IQ)))
                            td_pwr_lin = np.real(rms) ** 2
                            td_pwr_dbm_s = 10 * np.log10(td_pwr_lin / 1e-3)
                            td_pwr_dbm_signal[frameIdx, cellIdx, ueIdx, bsAntIdx] = td_pwr_dbm_s

                            # Compute SNR
                            # Noise
                            if noise_avail:
                                # noise_samples
                                In = noise_samples[frameIdx, cellIdx, 0, bsAntIdx, 0:symbol_length * 2:2] / 2 ** 15
                                Qn = noise_samples[frameIdx, cellIdx, 0, bsAntIdx, 1:symbol_length * 2:2] / 2 ** 15
                                IQn = In + (Qn * 1j)
                                # sio.savemat('test_pwr.mat', {'pilot_t': IQn})

                                # Compute Noise Power (Time Domain)
                                rms = np.sqrt(np.mean(IQn * np.conj(IQn)))
                                td_pwr_lin = np.real(rms) ** 2
                                td_pwr_dbm_n = 10 * np.log10(td_pwr_lin / 1e-3)
                                td_pwr_dbm_noise[frameIdx, cellIdx, ueIdx, bsAntIdx] = td_pwr_dbm_n
                                # SNR
                                snr[frameIdx, cellIdx, ueIdx, bsAntIdx] = td_pwr_dbm_s - td_pwr_dbm_n

                            dbg2 = False
                            if dbg2:
                                fig = plt.figure(1234)
                                ax1 = fig.add_subplot(2, 1, 1)
                                ax1.plot(np.abs(IQ))
                                ax2 = fig.add_subplot(2, 1, 2)
                                ax2.stem(np.abs(lts_corr))
                                ax2.scatter(np.linspace(0.0, len(lts_corr), num=1000), pilot_thresh * np.ones(1000), color='r')
                                plt.show()

            snr_end = time.time()
            print(">>>> compute_snr time: %f \n" % (snr_end - snr_start))

            # Plots:
            print("Plotting the results:\n")
            n_cell = match_filt_clr.shape[1]
            n_ue = match_filt_clr.shape[2]

            # plot a frame:
            fig, axes = plt.subplots(nrows=n_cell, ncols=n_ue, squeeze=False)
            fig.suptitle('MF Frame # {} Antenna # {}'.format(ref_frame, ant_i))
            for n_c in range(n_cell):
                for n_u in range(n_ue):
                    axes[n_c, n_u].stem(match_filt_clr[ref_frame - n_frm_st, n_c, n_u, ant_i, :])
                    axes[n_c, n_u].set_xlabel('Samples')
                    axes[n_c, n_u].set_title('Cell {} UE {}'.format(n_c, n_u))
                    axes[n_c, n_u].grid(True)
 
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

            #plot F starts for each antenna
            sub_fr_strt = f_st
            n_frame = sub_fr_strt.shape[0]      # no. of captured frames
            n_cell = sub_fr_strt.shape[1]       # no. of cells
            n_ue = sub_fr_strt.shape[2]         # no. of UEs
            n_ant = sub_fr_strt.shape[3]        # no. of BS antennas
            sf_strts = np.reshape(sub_fr_strt, (n_frame*n_cell*n_ue,n_ant))

            fig, axes = plt.subplots(nrows=n_ue, ncols=n_cell, squeeze=False)
            fig.suptitle('Frames\' starting indices per antenna')
            #plot channel analysis

            show_plot(cmpx_pilots, lts_seq_orig, match_filt, user_i, ant_i, ref_frame, n_frm_st)


            for n_c in range(n_cell):
                for n_u in range(n_ue):
                    sf_strts = sub_fr_strt[:,n_c,n_u,:]
                    x_pl = np.arange(sf_strts.shape[0]) + n_frm_st
                    for j in range(n_ant):
                        axes[n_u, n_c].plot(x_pl,sf_strts[:,j].flatten(), label = 'Antenna: {}'.format(j) )
                    axes[n_u, n_c].legend(loc='lower right', ncol=8, frameon=False)
                    axes[n_u, n_c].set_xlabel('Frame no.')
                    axes[n_u, n_c].set_ylabel('Starting index')
                    axes[n_u, n_c].grid(True)

            # PILOT MAP
            fig, axes = plt.subplots(nrows=n_ue, ncols=n_cell, squeeze=False)
            c = []
            fig.suptitle('Pilot Map (Percentage of Detected Pilots Per Symbol) - NOTE: Might exceed 100% due to threshold')
            for n_c in range(n_cell):
                for n_u in range(n_ue):
                    c.append(axes[n_u, n_c].imshow(seq_found[:, n_c, n_u, :].T, vmin=0, vmax=100, cmap='Blues',
                                                   interpolation='nearest',
                                                   extent=[n_frm_st, n_frm_end, n_ant, 0],
                                                   aspect="auto"))
                    axes[n_u, n_c].set_title('Cell {} UE {}'.format(n_c, n_u))
                    axes[n_u, n_c].set_ylabel('Antenna #')
                    axes[n_u, n_c].set_xlabel('Frame #')
                    axes[n_u, n_c].set_xticks(np.arange(n_frm_st, n_frm_end, 1), minor=True)
                    axes[n_u, n_c].set_yticks(np.arange(0, n_ant, 1), minor=True)
                    axes[n_u, n_c].grid(which='minor', color='0.75', linestyle='-', linewidth=0.05)
            cbar = plt.colorbar(c[-1], ax=axes.ravel().tolist(), ticks=np.linspace(0, 100, 11), orientation='horizontal')
            cbar.ax.set_xticklabels(['0%', '10%', '20%', '30%', '40%', '50%', '60%', '70%', '80%', '90%', '100%'])

            fig, axes = plt.subplots(nrows=n_ue, ncols=n_cell, squeeze=False)
            c = []
            fig.suptitle('Frame Map')
            for n_c in range(n_cell):
                for n_u in range(n_ue):
                    c.append( axes[n_u, n_c].imshow(frame_map[:,n_c,n_u,:].T, cmap=plt.cm.get_cmap('Blues', 3), interpolation='none',
                          extent=[n_frm_st,n_frm_end, n_ant,0],  aspect="auto") )
                    axes[n_u, n_c].set_title('Cell {} UE {}'.format(n_c, n_u))
                    axes[n_u, n_c].set_ylabel('Antenna #')
                    axes[n_u, n_c].set_xlabel('Frame #')
                    # Minor ticks
                    axes[n_u, n_c].set_xticks(np.arange(n_frm_st, n_frm_end, 1), minor=True)
                    axes[n_u, n_c].set_yticks(np.arange(0, n_ant, 1), minor=True)
                    # Gridlines based on minor ticks
                    axes[n_u, n_c].grid(which='minor', color='0.75', linestyle='-', linewidth=0.1)

            cbar = plt.colorbar(c[-1], ax=axes.ravel().tolist(), ticks=[-1, 0, 1], orientation = 'horizontal')
            cbar.ax.set_xticklabels(['Bad Frame', 'Probably partial/corrupt', 'Good Frame'])
            ##plt.show()

            #############
            #  SNR MAP  #
            #############
            if noise_avail:
                fig, axes = plt.subplots(nrows=n_ue, ncols=n_cell, squeeze=False)
                c = []
                fig.suptitle('SNR Map')
                for n_c in range(n_cell):
                    for n_u in range(n_ue):
                        c.append(
                            axes[n_u, n_c].imshow(snr[:, n_c, n_u, :].T, vmin=np.min(snr), vmax=np.max(snr), cmap='Blues',
                                                  interpolation='nearest',
                                                  extent=[n_frm_st, n_frm_end, n_ant, 0],
                                                  aspect="auto"))
                        axes[n_u, n_c].set_title('Cell {} UE {}'.format(n_c, n_u))
                        axes[n_u, n_c].set_ylabel('Antenna #')
                        axes[n_u, n_c].set_xlabel('Frame #')
                        axes[n_u, n_c].set_xticks(np.arange(n_frm_st, n_frm_end, 1), minor=True)
                        axes[n_u, n_c].set_yticks(np.arange(0, n_ant, 1), minor=True)
                        axes[n_u, n_c].grid(which='minor', color='0.75', linestyle='-', linewidth=0.05)
                cbar = plt.colorbar(c[-1], ax=axes.ravel().tolist(), ticks=np.linspace(0, np.max(snr), 10),
                                    orientation='horizontal')

            plt.show()
        else:
            plt.show()


def analyze_hdf5(hdf5, frame_i=10, cell_i=0, subcarrier_i=7, offset=-1, zoom=0, pl=0):
    '''
    Calculates and plots achievable rates from hdf5 traces

    '''

    metadata = hdf5.metadata
    pilot_samples = hdf5.pilot_samples
    noise_avail = len(hdf5.noise_samples) > 0
    # TODO: noise can be estimated from null subcarriers
    if noise_avail:
        noise_samples = hdf5.noise_samples
    else:
        print('Trace-based Estimation of performance requires presense of noise samples!')
        print('Noise data not present. Exitting...')
        return
    symbol_length = int(metadata['SYMBOL_LEN'])
    rate = float(metadata['RATE'])
    symbol_num = int(metadata['BS_FRAME_LEN'])
    timestep = symbol_length*symbol_num/rate
    num_cl = int(metadata['CL_NUM'])
    num_pilots = int(metadata['PILOT_NUM'])
    prefix_len = int(metadata['PREFIX_LEN'])
    postfix_len = int(metadata['POSTFIX_LEN'])
    z_padding = prefix_len + postfix_len
    if offset < 0: # if no offset is given use prefix from HDF5
        offset = int(prefix_len)
    fft_size = int(metadata['FFT_SIZE'])
    cp = int(metadata['CP_LEN'])
    pilot_type = metadata['PILOT_SEQ_TYPE'].astype(str)[0]
    nonzero_sc_size = metadata['DATA_SUBCARRIER_NUM']

    num_noise_syms = noise_samples.shape[2]
    n_frame = pilot_samples.shape[0]
    n_cell = pilot_samples.shape[1]
    n_ue = pilot_samples.shape[2]
    n_ant = pilot_samples.shape[3]

    # compute CSI for each user and get a nice numpy array
    # Returns csi with Frame, Cell, User, pilot repetitions, BS ant, Subcarrier
    # also, iq samples nicely chunked out, same dims, but subcarrier is sample.
    num_cl_tmp = num_pilots
    csi,_ = hdf5_lib.samps2csi(pilot_samples, num_cl_tmp, symbol_length, fft_size=fft_size,
                                offset=offset, bound=z_padding, cp=cp, sub=1,
                                pilot_type=pilot_type, nonzero_sc_size=nonzero_sc_size)
    csi = csi[:, cell_i, :, :, :, :]

    noise,_ = hdf5_lib.samps2csi(noise_samples, num_noise_syms, symbol_length, fft_size=fft_size,
                                offset=offset, bound=z_padding, cp=cp, sub=1,
                                pilot_type=pilot_type, nonzero_sc_size=nonzero_sc_size)
    noise = noise[:, cell_i, :, :, :, :]

    # zoom in too look at behavior around peak (and reduce processing time)
    if zoom > 0:
        csi = csi[frame_i-zoom:frame_i+zoom, :, :, :, :]
        noise = noise[frame_i-zoom:frame_i+zoom, :, :, :, :]
        # recenter the plots (otherwise it errors)
        frame = zoom
    # don't include noise, average over all pilot repetitions
    userCSI = np.mean(csi, 2)
    noise = np.mean(noise, 2)

    # compute beamweights based on the specified frame.
    conjbws = np.transpose(
        np.conj(userCSI[frame_i, :, :, :]), (1, 0, 2))
    zfbws = np.empty(
        (userCSI.shape[2], userCSI.shape[1], userCSI.shape[3]), dtype='complex64')
    for sc in range(userCSI.shape[3]):
        zfbws[:, :, sc] = np.linalg.pinv(
            userCSI[frame_i, :, :, sc])

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
    axes1[0, 0].set_title('Subcarrier-Mean Spectral Efficiency Using Beamforming Weights at Frame %d'%frame_i)
    for j in range(num_cl_tmp):
        axes1[0, 0].plot(np.arange(0, csi.shape[0]*timestep, timestep)[:csi.shape[0]], mubf_conj[:,j], label = 'Conj User: {}'.format(j) )
    for j in range(num_cl_tmp):
        axes1[0, 1].plot(np.arange(0, csi.shape[0]*timestep, timestep)[:csi.shape[0]], mubf_zf[:,j], label = 'ZF User: {}'.format(j) )
    axes1[0,0].legend(loc='upper right', ncol=1, frameon=False)
    axes1[0,0].set_xlabel('Time (s)', fontsize=14)
    axes1[0,0].set_ylabel('MUBF %dx%d (bps/Hz)'%(n_ant, n_ue), fontsize=14)
    axes1[0,1].legend(loc='upper right', ncol=1, frameon=False)
    axes1[0,1].set_xlabel('Time (s)', fontsize=14)
    for j in range(num_cl_tmp):
        axes1[1, 0].plot(np.arange(0, csi.shape[0]*timestep, timestep)[:csi.shape[0]], subf_conj[:,j], label = 'Conj User: {}'.format(j) )
    for j in range(num_cl_tmp):
        axes1[1, 1].plot(np.arange(0, csi.shape[0]*timestep, timestep)[:csi.shape[0]], subf_zf[:,j], label = 'ZF User: {}'.format(j) )
    axes1[1,0].legend(loc='upper right', ncol=1, frameon=False)
    axes1[1,0].set_xlabel('Time (s)', fontsize=14)
    axes1[1,0].set_ylabel('SUBF %dx1 (bps/Hz)'%n_ant, fontsize=14)
    axes1[1,1].legend(loc='upper right', ncol=1, frameon=False)
    axes1[1,1].set_xlabel('Time (s)', fontsize=14)


    # demmel number
    plt.figure(pl+2, figsize=(10, 8))
    plt.plot(np.arange(0, csi.shape[0]*timestep, timestep)[:csi.shape[0]], demmel[:, subcarrier_i])
    plt.xlabel('Time (s)', fontsize=14)
    plt.ylabel('Condition Number', fontsize=14)
    plt.title('CSI Matrix Demmel condition number across time, Subcarrier %d'%subcarrier_i)
    #pl += 1

    # SNR 
    #snr_linear = np.mean(zf[-1], axis = -1)
    #snr_dB = 10 * np.log10(snr_linear)
    #plt.figure(pl+2, figsize=(10, 8))
    #for i in range(num_cl_tmp):
    #    plt.plot(np.arange(0, csi.shape[0]*timestep, timestep)[:csi.shape[0]], snr_dB[:, i], label = 'User: {}'.format(i))
    ## plt.ylim([0,2])
    #plt.xlabel('Time (s)', fontsize=14)
    #plt.ylabel('ZF SNR (dB)', fontsize=14)
    #plt.title('ZF SNR Across Frames')
    #plt.legend()
    plt.show()

    del csi  # free the memory
    del noise


def compute_legacy(hdf5):
    '''
    Parse and plot data from legacy files
    '''

    print("starting legacy function")
    starttime = time.time()
    show_plots = True
    zoom = 0  # samples to zoom in around frame (to look at local behavior), 0 to disable
    pl = 0

    frame = 10  # frame to compute beamweights from
    conjdata = []
    zfdata = []
    # print("main checkpoint1 time expended %f" % (starttime - time.time()))
    for h5log in [hdf5]:  # , env, mobile]:
        # read parameters for this measurement data
        samps_per_user = h5log.attrs['samples_per_user']
        num_users = h5log.attrs['num_mob_ant']
        timestep = h5log.attrs['frame_length'] / 20e6
        noise_meas_en = h5log.attrs.get('measured_noise', 1)

        # compute CSI for each user and get a nice numpy array
        csi, iq = hdf5_lib.samps2csi(h5log['Pilot_Samples'], num_users + noise_meas_en, samps_per_user,
                                     legacy=True)  # Returns csi with Frame, User, LTS (there are 2), BS ant, Subcarrier  #also, iq samples nicely chunked out, same dims, but subcarrier is sample.
        if zoom > 0:  # zoom in too look at behavior around peak (and reduce processing time)
            csi = csi[frame - zoom:frame + zoom, :, :, :, :]
            frame = zoom  # recenter the plots (otherwise it errors)
        noise = csi[:, -1, :, :, :]  # noise is last set of data.
        userCSI = np.mean(csi[:, :num_users, :, :, :], 2)  # don't include noise, average over both LTSs

        # example lts find:
        user = 0
        # so, this is pretty ugly, but we want all the samples (not just those chunked from samps2csi), so we not only convert ints to the complex floats, but also have to figure out where to chunk the user from.
        lts_iq = h5log['Pilot_Samples'][frame, 0, user * samps_per_user:(user + 1) * samps_per_user, 0] * 1. + \
                 h5log['Pilot_Samples'][frame, 0, user * samps_per_user:(user + 1) * samps_per_user, 1] * 1j
        lts_iq /= 2 ** 15
        offset = lts.findLTS(
            lts_iq)  # Andrew wrote this, but I don't really like the way he did the convolve method...  works well enough for high SNRs.
        offset = offset[0] + 32
        print("LTS offset for user %d, frame %d: %d" % (user, frame, offset))

        # compute beamweights based on the specified frame.
        conjbws = np.transpose(np.conj(userCSI[frame, :, :, :]), (1, 0, 2))
        zfbws = np.empty((userCSI.shape[2], userCSI.shape[1], userCSI.shape[3]), dtype='complex64')
        for sc in range(userCSI.shape[3]):
            zfbws[:, :, sc] = np.linalg.pinv(userCSI[frame, :, :, sc])

        downlink = True
        # calculate capacity based on these weights
        # these return total capacity, per-user capacity, per-user/per-subcarrier capacity, SINR, single-user capacity(no inter-user interference), and SNR
        conj = calCapacity(userCSI, noise, conjbws,
                           downlink=downlink)  # conjcap_total,conjcap_u,conjcap_sc,conjSINR,conjcap_su_sc,conjcap_su_u,conjSNR
        zf = calCapacity(userCSI, noise, zfbws,
                         downlink=downlink)  # zfcap_total,zfcap_u,zfcap_sc,zfSINR,zfcap_su_sc,zfcap_su_u,zfSNR
        # print("main checkpoint2 time expended %f" % (starttime - time.time()))

        # plot stuff
        if show_plots:
            # Multiuser Conjugate
            plt.figure(1000 * pl, figsize=(50, 10))
            plt.plot(np.arange(0, csi.shape[0] * timestep, timestep)[:csi.shape[0]], conj[1])
            # plt.ylim([0,2])
            plt.xlabel('Time (s)')
            plt.ylabel('Per User Capacity Conj (bps/Hz)')
            plt.show(block=False)
            # Multiuser Zeroforcing
            plt.figure(1000 * pl + 1, figsize=(50, 10))
            plt.plot(np.arange(0, csi.shape[0] * timestep, timestep)[:csi.shape[0]], zf[1])
            # plt.ylim([0,2])
            plt.xlabel('Time (s)')
            plt.ylabel('Per User Capacity ZF (bps/Hz)')
            plt.show(block=False)
            # Single user (but show all users)
            plt.figure(1000 * pl + 2, figsize=(50, 10))
            plt.plot(np.arange(0, csi.shape[0] * timestep, timestep)[:csi.shape[0]], conj[-2])
            # plt.ylim([0,2])
            plt.xlabel('Time (s)')
            plt.ylabel('SUBF Capacity Conj (bps/Hz)')
            plt.show(block=False)
            pl += 1
        # print("main checkpoint3 time expended %f" % (starttime - time.time()))
        # save for exporting to matlab (prettier plots)
        conjdata.append(conj)
        zfdata.append(zf)
        # print("main checkpoint4 time expended %f" % (starttime - time.time()))

        del csi, iq  # free the memory

    endtime = time.time()
    print("Total time: %f" % (endtime - starttime))


def show_plot(cmpx_pilots, lts_seq_orig, match_filt, ref_user, ref_ant, ref_frame, frm_st_idx):
    '''
    Plot channel analysis
    '''

    # WZC: fix the hardcode issue
    frame_to_plot = ref_frame
    frm_st_idx = frm_st_idx
    ref_ant = ref_ant
    ref_user = ref_user
    test_mf = False
    debug = False

    fig = plt.figure()
    ax1 = fig.add_subplot(3, 1, 1)
    ax1.grid(True)
    ax1.set_title(
        'channel_analysis:csi_from_pilots(): Re of Rx pilot - ref frame {} and ref ant. {} (UE {})'.format(
            frame_to_plot, ref_ant, ref_user))

    if debug:
        print("cmpx_pilots.shape = {}".format(cmpx_pilots.shape))

    ax1.plot(
        np.real(cmpx_pilots[frame_to_plot - frm_st_idx, 0, ref_user, ref_ant, :]))

    z_pre = np.zeros(82, dtype='complex64')
    z_post = np.zeros(68, dtype='complex64')
    lts_t_rep = lts_seq_orig

    if debug:

        lts_t_rep_tst = np.append(z_pre, lts_t_rep)
        lts_t_rep_tst = np.append(lts_t_rep_tst, z_post)

        if test_mf:
            w = np.random.normal(0, 0.1 / 2, len(lts_t_rep_tst)) + \
                1j * np.random.normal(0, 0.1 / 2, len(lts_t_rep_tst))
            lts_t_rep_tst = lts_t_rep_tst + w
            cmpx_pilots = np.tile(
                lts_t_rep_tst, (n_frame, cmpx_pilots.shape[1], cmpx_pilots.shape[2], cmpx_pilots.shape[3], 1))
            print("if test_mf: Shape of lts_t_rep_tst: {} , cmpx_pilots.shape = {}".format(
                lts_t_rep_tst.shape, cmpx_pilots.shape))

        loc_sec = lts_t_rep_tst
    else:
        loc_sec = np.append(z_pre, lts_t_rep)
        loc_sec = np.append(loc_sec, z_post)
    ax2 = fig.add_subplot(3, 1, 2)
    ax2.grid(True)
    ax2.set_title(
        'channel_analysis:csi_from_pilots(): Local LTS sequence zero padded')
    ax2.plot(loc_sec)

    ax3 = fig.add_subplot(3, 1, 3)
    ax3.grid(True)
    ax3.set_title(
        'channel_analysis:csi_from_pilots(): MF (uncleared peaks) - ref frame {} and ref ant. {} (UE {})'.format(
            frame_to_plot, ref_ant, ref_user))
    ax3.stem(match_filt[frame_to_plot - frm_st_idx, 0, ref_user, ref_ant, :])
    ax3.set_xlabel('Samples')


def pilot_finder(samples, pilot_type, flip=False, pilot_seq=[]):
    """
    Find pilots from clients to each of the base station antennas

    Input:
        samples    - Raw samples from pilots and data.
                     Dimensions: vector [1 x num samples]
        pilot_type - Type of TX pilot (e.g., 802.11 LTS)
        flip       - Needed for finding LTS function

    Output:
        pilot     - Received pilot (from multiple clients)
        tx_pilot  - Transmitted pilot (same pilot sent by all clients)
    """

    if pilot_type.find('lts') != -1:
        # LTS-based pilot
        lts_thresh = 0.8
        best_pk, lts_pks, lts_corr = find_lts(samples, thresh=lts_thresh, flip=flip, lts_seq=pilot_seq)

        # full lts contains 2.5 64-sample-LTS sequences, we need only one symbol
        lts, lts_f = generate_training_seq(preamble_type='lts', cp=32, upsample=1)

        if not (pilot_seq.size == 0):
            # pilot provided, overwrite the one returned above
            lts = pilot_seq

        lts_syms_len = len(lts)
        pilot_thresh = lts_thresh * np.max(lts_corr)
        # We'll need the transmitted version of the pilot (for channel estimation, for example)
        tx_pilot = [lts, lts_f]
        lts_start = 0

        # Check if LTS found
        if not best_pk:
            # print("SISO_OFDM: No LTS Found! Continue...")
            pilot = np.array([])
            return tx_pilot, lts_pks, lts_corr, pilot_thresh, best_pk
    else:
        raise Exception("Only LTS Pilots supported at the moment")

    return tx_pilot, lts_pks, lts_corr, pilot_thresh, best_pk


def main():
    # Tested with inputs: ./data_in/Argos-2019-3-11-11-45-17_1x8x2.hdf5 300  (for two users)
    #                     ./data_in/Argos-2019-3-30-12-20-50_1x8x1.hdf5 300  (for one user) 
    parser = OptionParser()
    parser.add_option("--show-metadata", action="store_true", dest="show_metadata", help="Displays hdf5 metadata", default=False)
    parser.add_option("--deep-inspect", action="store_true", dest="deep_inspect", help="Run script without analysis", default=False)
    parser.add_option("--ref-frame", type="int", dest="ref_frame", help="Frame number to plot", default=1000)
    parser.add_option("--ref-ul-subframe", type="int", dest="ref_ul_subframe", help="Frame number to plot", default=0)
    parser.add_option("--ref-cell", type="int", dest="ref_cell", help="Cell number to plot", default=0)
    parser.add_option("--legacy", action="store_true", dest="legacy", help="Parse and plot legacy hdf5 file", default=False)
    parser.add_option("--ref-ant", type="int", dest="ref_ant", help="Reference antenna", default=0)
    parser.add_option("--exclude-bs-ants", type="string", dest="exclude_bs_nodes", help="Bs antennas to be excluded in plotting", default="")
    parser.add_option("--ref-ofdm-sym", type="int", dest="ref_ofdm_sym", help="Reference ofdm symbol within a pilot", default=0)
    parser.add_option("--ref-user", type="int", dest="ref_user", help="Reference User", default=0)
    parser.add_option("--ref-subcarrier", type="int", dest="ref_subcarrier", help="Reference subcarrier", default=0)
    parser.add_option("--signal-offset", type="int", dest="signal_offset", help="signal offset from the start of the time-domain symbols", default=-1)
    parser.add_option("--downlink-calib-offset", type="int", dest="downlink_calib_offset", help="signal offset from the start of the time-domain symbols in downlink reciprocal calibration", default=288)
    parser.add_option("--uplink-calib-offset", type="int", dest="uplink_calib_offset", help="signal offset from the start of the time-domain symbols in uplink reciprocal calibration", default=168)
    parser.add_option("--n-frames", type="int", dest="n_frames_to_inspect", help="Number of frames to inspect", default=2000)
    parser.add_option("--sub-sample", type="int", dest="sub_sample", help="Sub sample rate", default=1)
    parser.add_option("--thresh", type="float", dest="thresh", help="Ampiltude Threshold for valid frames", default=0.001)
    parser.add_option("--frame-start", type="int", dest="fr_strt", help="Starting frame. Must have set n_frames_to_inspect first and make sure fr_strt is within boundaries ", default=0)
    parser.add_option("--verify-trace", action="store_true", dest="verify", help="Run script without analysis", default=True)
    parser.add_option("--analyze-trace", action="store_true", dest="analyze", help="Run script without analysis", default=False)
    parser.add_option("--corr-thresh", type="float", dest="corr_thresh",
                      help="Correlation threshold to exclude bad nodes",
                      default=0.00)
    (options, args) = parser.parse_args()

    show_metadata = options.show_metadata
    deep_inspect = options.deep_inspect
    n_frames_to_inspect = options.n_frames_to_inspect
    ref_frame = options.ref_frame
    ref_cell = options.ref_cell
    ref_ofdm_sym = options.ref_ofdm_sym
    ref_ant = options.ref_ant
    ref_user = options.ref_user
    ref_subcarrier = options.ref_subcarrier
    ref_ul_subframe = options.ref_ul_subframe
    signal_offset = options.signal_offset
    downlink_calib_offset = options.downlink_calib_offset
    uplink_calib_offset = options.uplink_calib_offset
    thresh = options.thresh
    fr_strt = options.fr_strt
    verify = options.verify
    analyze = options.analyze
    sub_sample = options.sub_sample
    legacy = options.legacy
    corr_thresh = options.corr_thresh
    exclude_bs_nodes_str = options.exclude_bs_nodes
    exclude_bs_nodes = []
    if len(exclude_bs_nodes_str) > 0:
        exclude_ant_ids = exclude_bs_nodes_str.split(',')
        exclude_bs_nodes = [int(i) for i in exclude_ant_ids]

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
    if legacy:
        # TODO: Needs to be thoroughly tested!
        # filename = 'ArgosCSI-96x8-2016-11-03-03-03-45_5GHz_static.hdf5'
        hdf5 = h5py.File(str(filename), 'r')
        compute_legacy(hdf5)
    else:
        if show_metadata:
            hdf5 = hdf5_lib(filename)
            print(hdf5.metadata)
            pilot_samples = hdf5.pilot_samples
            uplink_samples = hdf5.uplink_samples
            noise_avail = len(noise_samples) > 0

            # Check which data we have available
            pilots_avail = len(pilot_samples) > 0
            ul_data_avail = len(uplink_samples) > 0
            if pilots_avail:
                print("HDF5 pilot data size:")
                print(pilot_samples.shape)
            if ul_data_avail:
                print("HDF5 uplink data size:")
                print(uplink_samples.shape)

        else:
            hdf5 = hdf5_lib(filename, n_frames_to_inspect, fr_strt, sub_sample)
            data = hdf5.data
            pilot_samples = hdf5.pilot_samples
            uplink_samples = hdf5.uplink_samples
            noise_samples = hdf5.noise_samples

            # Check which data we have available
            pilots_avail = len(pilot_samples) > 0
            ul_data_avail = len(uplink_samples) > 0
            noise_avail = len(noise_samples) > 0

            if pilots_avail:
                print("Found Pilots!")
                if ul_data_avail:
                    print("Found Uplink Data")
                if noise_avail:
                    print("Found Noise Samples!")
            else:
                if not ul_data_avail:
                    raise Exception(' **** No pilots or uplink data found **** ')

            if verify:
                verify_hdf5(hdf5, ref_frame, ref_cell, ref_ofdm_sym, ref_ant,
                            ref_user, ref_ul_subframe, ref_subcarrier,
                            signal_offset, downlink_calib_offset,
                            uplink_calib_offset, thresh, deep_inspect,
                            corr_thresh, exclude_bs_nodes)
            if analyze:
                analyze_hdf5(hdf5, ref_frame, ref_cell, ref_subcarrier, signal_offset)
    scrpt_end = time.time()
    print(">>>> Script Duration: time: %f \n" % ( scrpt_end - scrpt_strt) )


if __name__ == '__main__':
    main()

