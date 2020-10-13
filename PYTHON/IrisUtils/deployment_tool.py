#!/usr/bin/python3
"""
 deployment_tool.py

 Script used to parse and analyze data collected during measurement campaigns.
 It supports multi-cell mode.
 Datasets are publicly available in our website
 http://renew-wireless.org/

 NOTE: A major assumption here is that the noise dataset used to compute
 SNR used the same parameters (same metadata)

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
import numpy as np
import scipy.io as sio
import h5py
import matplotlib.pyplot as plt
import collections
import time
import math
from find_lts import *
from optparse import OptionParser
from channel_analysis import *
from scipy import signal
import hdf5_lib
from hdf5_lib import *
from bandpower import *
from fft_power import *
import matplotlib
import matplotlib.ticker as ticker
matplotlib.use("TkAgg")


def datasets(n_frames_to_inspect, fr_strt):
    """
    TOPOLOGY
    cell 3 | RF3E000179 - USTAR
    cell 4 | RF3E000119 – EBC (David)
    cell 0 | RF3E000145 - ACC
    cell 1 | RF3E000166 – Honors
    cell 2 | RF3E000030 – MEB
    client | RF3E000157 – Client (Kirk)

    IMPORTANT: The node at EBC couldn't be connected to the network so we collected data separately
    """

    path1 = '/media/Storage/argos/Documents/POWDER_datasets/ACC_HONORS_MEB_USTAR/'
    path2 = '/media/Storage/argos/Documents/POWDER_datasets/EBC/'

    # Data from ACC, HONORS, MEB, USTAR nodes
    files1 = ['trace-2020-9-23-11-37-58_4x2x1_NOISE.hdf5',
    # 'trace-2020-9-23-11-40-25_4x2x1_NOISE2.hdf5',
    # 'trace-2020-9-23-11-54-13_4x2x1_LOC1_v0.hdf5',
    # 'trace-2020-9-23-11-55-14_4x2x1_LOC1_v1.hdf5',
    'trace-2020-9-23-11-56-23_4x2x1_LOC1_v2.hdf5',
    # 'trace-2020-9-23-12-14-8_4x2x1_LOC2_v0.hdf5',
    'trace-2020-9-23-12-15-13_4x2x1_LOC2_v1.hdf5',
    'trace-2020-9-23-12-26-56_4x2x1_LOC3.hdf5',
    'trace-2020-9-23-12-43-34_4x2x1_LOC4.hdf5',
    'trace-2020-9-23-12-54-58_4x2x1_LOC5.hdf5',
    'trace-2020-9-23-13-25-6_4x2x1_LOC6.hdf5',
    # 'trace-2020-9-23-13-14-14_4x2x1_LOC7_v0.hdf5',
    'trace-2020-9-23-13-15-18_4x2x1_LOC7_v1.hdf5',
    # 'trace-2020-9-23-13-34-49_4x2x1_LOC8_v0.hdf5',
    'trace-2020-9-23-13-36-34_4x2x1_LOC8_v1.hdf5',
    'trace-2020-9-23-13-4-6_4x2x1_LOC9.hdf5',
    # 'trace-2020-9-23-13-47-33_4x2x1_LOC10_v0.hdf5',
    'trace-2020-9-23-13-48-35_4x2x1_LOC10_v1.hdf5',
    'trace-2020-9-23-13-57-8_4x2x1_LOC11.hdf5',
    'trace-2020-9-23-14-29-43_4x2x1_LOC13.hdf5',
    # 'trace-2020-9-23-14-46-44_4x2x1_LOC14_v0.hdf5',
    'trace-2020-9-23-14-47-53_4x2x1_LOC14_v1.hdf5',
    'trace-2020-9-23-14-57-51_4x2x1_LOC15.hdf5',
    # 'trace-2020-9-23-15-21-24_4x2x1_LOC3_3.6.hdf5',
    # 'trace-2020-9-23-15-6-49_4x2x1_LOC15_3.6_NOISE.hdf5',
    # 'trace-2020-9-23-15-8-40_4x2x1_LOC15_3.6.hdf5',
    ]

    # Data from EBC node
    files2 = ['trace-2020-9-23-11-37-49_1x2x1_NOISE.hdf5',
    'trace-2020-9-23-11-54-40_1x2x1_LOC1.hdf5',
    'trace-2020-9-23-12-14-53_1x2x1_LOC2.hdf5',
    'trace-2020-9-23-12-27-15_1x2x1_LOC3.hdf5',
    'trace-2020-9-23-12-44-2_1x2x1_LOC4.hdf5',
    'trace-2020-9-23-12-55-18_1x2x1_LOC5.hdf5',
    'trace-2020-9-23-13-25-1_1x2x1_LOC6.hdf5',
    'trace-2020-9-23-13-14-34_1x2x1_LOC7.hdf5',
    'trace-2020-9-23-13-34-41_1x2x1_LOC8.hdf5',
    'trace-2020-9-23-13-4-29_1x2x1_LOC9.hdf5',
    'trace-2020-9-23-13-47-42_1x2x1_LOC10.hdf5',
    'trace-2020-9-23-13-57-13_1x2x1_LOC11.hdf5',
    'trace-2020-9-23-14-29-39_1x2x1_LOC13.hdf5',
    'trace-2020-9-23-14-46-31_1x2x1_LOC14.hdf5',
    'trace-2020-9-23-14-57-52_1x2x1_LOC15.hdf5',
    # 'trace-2020-9-23-15-21-26_1x2x1_LOC3_3.6.hdf5',
    # 'trace-2020-9-23-15-7-2_1x2x1-NOISE-3.6.hdf5',
    # 'trace-2020-9-23-15-8-21_1x2x1_LOC15_3.6.hdf5',
    ]

    signal_file1 = files1[1::]
    noise_file1 = files1[0]
    signal_file2 = files2[1::]
    noise_file2 = files2[0]
    curr_file_n1 = path1 + noise_file1
    curr_file_n2 = path2 + noise_file2

    hdf5_signal = []
    hdf5_noise = []
    for fidx in range(len(signal_file1)):

        curr_file_s1 = path1 + signal_file1[fidx]
        curr_file_s2 = path2 + signal_file2[fidx]

        # Instantiate
        hdf5_signal1 = hdf5_lib(curr_file_s1, n_frames_to_inspect, fr_strt)
        hdf5_signal2 = hdf5_lib(curr_file_s2, n_frames_to_inspect, fr_strt)
        try:
            hdf5_noise1 = hdf5_lib(curr_file_n1, n_frames_to_inspect, fr_strt)
            hdf5_noise2 = hdf5_lib(curr_file_n2, n_frames_to_inspect, fr_strt)
        except Exception as ex:
            msgstr = "An exception of type {0} occurred. Arguments:\n{1!r}"
            message = msgstr.format(type(ex).__name__, ex.args)
            print(message)
            hdf5_noise = []

        # Some datasets in files1 have fewer frames than expected...
        num_frames_s = hdf5_signal1.pilot_samples.shape[0]
        num_frames_n = hdf5_noise1.pilot_samples.shape[0]

        hdf5_signal1.pilot_samples = np.append(hdf5_signal1.pilot_samples,
                                               hdf5_signal2.pilot_samples[0:num_frames_s, :, :, :, :], axis=1)
        hdf5_noise1.pilot_samples = np.append(hdf5_noise1.pilot_samples,
                                              hdf5_noise2.pilot_samples[0:num_frames_n, :, :, :, :], axis=1)

        hdf5_signal.append(hdf5_signal1)
        hdf5_noise.append(hdf5_noise1)

    return hdf5_signal, hdf5_noise


def parse_signal_hdf5(hdf5_signal, hdf5_noise, default_frame=100, ant_i=0, user_i=0, thresh=0.001, sub_sample=1,
                      plot_level=0):
    """

    """
    plt.close("all")
    hdf5 = hdf5_signal
    hdf5n = hdf5_noise
    metadata = hdf5.metadata
    samples = hdf5.pilot_samples

    if hdf5n:
        noisef_present = True
        metadata_n = hdf5n.metadata
        samples_n = hdf5n.pilot_samples

    # Check which data we have available
    data_types_avail = []
    pilots_avail = len(samples) > 0

    if pilots_avail:
        data_types_avail.append("PILOTS")
        print("PILOT Data Available")

    # Empty structure
    if not data_types_avail:
        raise Exception(' **** No pilot data found **** ')

    # Retrieve attributes
    symbol_length = int(metadata['SYMBOL_LEN'])
    num_pilots = int(metadata['PILOT_NUM'])
    num_cl = int(metadata['CL_NUM'])
    cp = int(metadata['CP_LEN'])
    prefix_len = int(metadata['PREFIX_LEN'])
    postfix_len = int(metadata['POSTFIX_LEN'])
    z_padding = prefix_len + postfix_len
    offset = int(prefix_len)
    fft_size = int(metadata['FFT_SIZE'])
    pilot_type = metadata['PILOT_SEQ_TYPE'].astype(str)[0]
    ofdm_pilot = np.array(metadata['OFDM_PILOT'])
    rate = int(metadata['RATE'])
    num_cl_tmp = num_pilots  # number of UEs to plot data for

    print(" symbol_length = {}, cp = {}, prefix_len = {}, postfix_len = {}, z_padding = {}".
          format(symbol_length, cp, prefix_len,  postfix_len, z_padding))

    # Filter pilots and check frame sanity
    print("********     Calling filter_pilots and frame_sanity    *********")
    frm_plt = min(default_frame, samples.shape[0] + hdf5.n_frm_st)
    match_filt, k_lts, n_lts, cmpx_pilots, lts_seq_orig = hdf5_lib.filter_pilots(samples, z_padding,
                                                                                 fft_size=fft_size, cp=cp)
    match_filt_clr, frame_map, f_st, peak_map = hdf5_lib.frame_sanity(match_filt, k_lts, n_lts, hdf5.n_frm_st, frm_plt,
                                                                      plt_ant=ant_i, cp=cp)

    # Find LTS peaks across frame
    n_frame = samples.shape[0]
    n_cell = samples.shape[1]
    n_ue = samples.shape[2]
    n_ant = samples.shape[3]
    seq_found = np.zeros((n_frame, n_cell, n_ue, n_ant))
    num_pilots_per_sym = ((symbol_length - z_padding) // len(ofdm_pilot))

    # Retrieve frequency-domain LTS sequence
    _, lts_freq = generate_training_seq(
        preamble_type='lts', seq_length=[], cp=32, upsample=1, reps=[])

    td_pwr_dbm_noise = np.empty_like(samples[:, :, :, :, 0], dtype=float)
    td_pwr_dbm_signal = np.empty_like(samples[:, :, :, :, 0], dtype=float)
    fd_pwr_dbm_noise = np.empty_like(samples[:, :, :, :, 0], dtype=float)
    fd_pwr_dbm_signal = np.empty_like(samples[:, :, :, :, 0], dtype=float)
    snr = np.empty_like(samples[:, :, :, :, 0], dtype=float)
    snr_f = np.empty_like(samples[:, :, :, :, 0], dtype=float)
    for frameIdx in range(n_frame):  # Frame
        for cellIdx in range(n_cell):  # Cell
            for ueIdx in range(n_ue):  # UE
                for bsAntIdx in range(n_ant):  # BS ANT

                    # Compute SNR
                    # Noise
                    if noisef_present:
                        In = samples_n[frameIdx, cellIdx, ueIdx, bsAntIdx, 0:symbol_length * 2:2] / 2 ** 15
                        Qn = samples_n[frameIdx, cellIdx, ueIdx, bsAntIdx, 1:symbol_length * 2:2] / 2 ** 15
                        IQn = In + (Qn * 1j)
                        #sio.savemat('test_pwr.mat', {'pilot_t': IQn})

                        # Compute Noise Power (Freq Domain)
                        fd_pwr_dbm_n = []
                        for pidx in range(len(IQn)//num_pilots_per_sym):
                            this_IQn = IQn[pidx*num_pilots_per_sym: (pidx+1)*num_pilots_per_sym]
                            f, Pxx_den = signal.periodogram(this_IQn, fs=rate, window='hamming', nfft=fft_size,
                                                            return_onesided=False, scaling='density')
                            lastRectWidth = 0  # Don't include last point of PSD data.
                            width = np.append(np.diff(f), lastRectWidth)
                            # fd_pwr_dbm_n.append(10 * np.log10(np.dot(width, Pxx_den) / 1e-3))
                            fd_pwr_dbm_n.append(np.dot(width, Pxx_den))

                        fd_pwr_dbm_n = [x for x in fd_pwr_dbm_n if x > 0]           # Remove negative entries
                        fd_pwr_dbm_n = 10 * np.log10(np.mean(fd_pwr_dbm_n) / 1e-3)
                        if math.isnan(fd_pwr_dbm_n):
                            print("WOOPS, NaN found (noise)!")

                        fd_pwr_dbm_noise[frameIdx, cellIdx, ueIdx, bsAntIdx] = fd_pwr_dbm_n

                        # Compute Noise Power (Time Domain)
                        rms = np.sqrt(np.mean(IQn * np.conj(IQn)))
                        td_pwr_lin = np.real(rms) ** 2
                        td_pwr_dbm_n = 10 * np.log10(td_pwr_lin / 1e-3)
                        td_pwr_dbm_noise[frameIdx, cellIdx, ueIdx, bsAntIdx] = td_pwr_dbm_n

                    # Signal
                    I = samples[frameIdx, cellIdx, ueIdx, bsAntIdx, 0:symbol_length * 2:2] / 2 ** 15
                    Q = samples[frameIdx, cellIdx, ueIdx, bsAntIdx, 1:symbol_length * 2:2] / 2 ** 15
                    IQ = I + (Q * 1j)

                    # Compute Power of Time Domain Signal
                    rms = np.sqrt(np.mean(IQ * np.conj(IQ)))
                    td_pwr_lin = np.real(rms) ** 2
                    td_pwr_dbm_s = 10 * np.log10(td_pwr_lin / 1e-3)
                    td_pwr_dbm_signal[frameIdx, cellIdx, ueIdx, bsAntIdx] = td_pwr_dbm_s

                    # Compute power in frequency domain signal
                    fd_pwr_dbm_s = []
                    for pidx in range(len(IQ) // num_pilots_per_sym):
                        this_IQ = IQ[pidx * num_pilots_per_sym: (pidx + 1) * num_pilots_per_sym]
                        f, Pxx_den = signal.periodogram(this_IQ, fs=rate, window='hamming', nfft=fft_size,
                                                        return_onesided=False, scaling='density')
                        lastRectWidth = 0  # Don't include last point of PSD data.
                        width = np.append(np.diff(f), lastRectWidth)
                        # fd_pwr_dbm_s.append(10 * np.log10(np.dot(width, Pxx_den) / 1e-3))
                        fd_pwr_dbm_s.append(np.dot(width, Pxx_den))

                    fd_pwr_dbm_s = [x for x in fd_pwr_dbm_s if x > 0]  # Remove negative entries
                    fd_pwr_dbm_s = 10 * np.log10(np.mean(fd_pwr_dbm_s) / 1e-3)

                    if math.isnan(fd_pwr_dbm_s):
                        print("WOOPS, NaN found (signal)!")

                    fd_pwr_dbm_signal[frameIdx, cellIdx, ueIdx, bsAntIdx] = fd_pwr_dbm_s

                    # SNR
                    snr[frameIdx, cellIdx, ueIdx, bsAntIdx] = td_pwr_dbm_s - td_pwr_dbm_n
                    snr_f[frameIdx, cellIdx, ueIdx, bsAntIdx] = fd_pwr_dbm_s - fd_pwr_dbm_n

                    # PILOTS
                    tx_pilot, lts_pks, lts_corr, pilot_thresh, best_pk = pilot_finder(IQ, pilot_type, flip=True,
                                                                                      pilot_seq=ofdm_pilot)

                    ######################################
                    # Compute EVM
                    # If more peaks...
                    dbg2 = False
                    if dbg2:
                        fig = plt.figure(1234 + cellIdx)
                        ax1 = fig.add_subplot(2, 1, 1)
                        ax1.plot(np.abs(IQ))
                        ax2 = fig.add_subplot(2, 1, 2)
                        ax2.stem(np.abs(lts_corr))
                        ax2.scatter(np.linspace(0.0, len(lts_corr), num=1000), pilot_thresh * np.ones(1000),
                                    color='r')
                        plt.show()
                        """

                            
    
                        for x in np.nditer(lts_pks):
                            this_pilot = (IQ[x - len(ofdm_pilot): x])
                            this_pilot_fd = np.fft.fft(this_pilot, fft_size)
                            fd_pwr_lin = np.sum(this_pilot_fd**2)
                            fd_pwr_db = 10 * np.log10(fd_pwr_lin)
                        """
                    ######################################

                    # Find percentage of LTS peaks within a symbol
                    # (e.g., in a 4096-sample pilot symbol, we expect 64, 64-long sequences... assuming no CP)
                    # seq_found[frameIdx, cellIdx, ueIdx, bsAntIdx] = 100 * (lts_pks.size / num_pilots_per_sym)  # use simple lts detection
                    seq_found[frameIdx, cellIdx, ueIdx, bsAntIdx] = 100 * (peak_map[frameIdx, cellIdx, ueIdx, bsAntIdx] / num_pilots_per_sym)  # use matched filter analysis output

    # Compute CSI from IQ samples
    # Samps: #Frames, #Cell, #Users, #Antennas, #Samples
    # CSI:   #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Subcarrier
    # For correlation use a fft size of 64
    print(
        "*verify_hdf5(): Calling samps2csi with fft_size = {}, offset = {}, bound = cp = 0 *".format(fft_size, offset))
    csi, samps = hdf5_lib.samps2csi(samples, num_cl_tmp, symbol_length, fft_size=fft_size, offset=offset, bound=0,
                                    cp=cp, sub=sub_sample)

    # Correlation (Debug plot useful for checking sync)
    amps = np.mean(np.abs(samps[:, 0, user_i, 0, ant_i, :]), axis=1)
    pilot_frames = [i for i in range(len(amps)) if amps[i] > thresh]
    if len(pilot_frames) > 0:
        corr_ref_frame = pilot_frames[len(pilot_frames) // 2]
    else:
        print("no valid frames where found. Decision threshold for amplitude was %f" % thresh)
        return
    cellCSI = csi[:, 0, :, :, :, :]  # First cell
    userCSI = np.mean(cellCSI[:, :, :, :, :], 2)
    corr_total, sig_sc = calCorr(userCSI, np.transpose(np.conj(userCSI[corr_ref_frame, :, :, :]), (1, 0, 2)))

    # Compute CSI from IQ samples
    # Samps: #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Samples
    # CSI:   #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Subcarrier
    # For looking at the whole picture, use a fft size of whole symbol_length as fft window (for visualization),
    # and no offset
    print("*verify_hdf5():Calling samps2csi *AGAIN*(?) with fft_size = symbol_length, no offset*")
    csi, samps = hdf5_lib.samps2csi(samples, num_cl_tmp, symbol_length, fft_size=symbol_length, offset=0, bound=0, cp=0,
                                    sub=sub_sample)

    # For some damm reason, if one of the subplots has all of the frames in the same state (good/bad/partial)
    # it chooses a random color to paint the whole subplot!
    # Below is some sort of remedy
    for n_c in range(n_cell):
        for n_u in range(n_ue):
            f_map = frame_map[:, n_c, n_u, :]
            n_gf = f_map[f_map == 1].size
            n_bf = f_map[f_map == -1].size
            n_pr = f_map[f_map == 0].size
            if n_gf == 0:
                # frame_map[-1, n_c, n_u, -1] = 1
                print("No good frames for cell {} and UE {}. Color last frame to keep plotter happy".format(n_c, n_u))
            if n_pr == 0:
                # frame_map[0, n_c, n_u, -1] = 0
                print("No partial frames for cell {} and UE {}.Color last frame to keep plotter happy".format(n_c, n_u))
            if n_bf == 0:
                # frame_map[-1, n_c, n_u, 0] = -1
                print("No bad frames for cell {} and UE {}.Color last frame to keep plotter happy".format(n_c, n_u))

    # plot channel analysis
    if plot_level > 0:
        plotter_per_dataset(samps, default_frame, ant_i, user_i, offset, pilot_frames, match_filt_clr, hdf5, frame_map,
                            seq_found, corr_total, snr)

    print("** WARNING: If you attempt to plot a different frame after running this script, remember to subtract the \n"
          " frame_start you gave! e.g., frame no. 1763 and frame_start = 1500 --> "
          "plot(match_filter_clr[<frame 1736 - 1500>, <cell>, <ue>, ref_antenna,:])**")

    return frame_map, snr, snr_f, seq_found, fd_pwr_dbm_signal, fd_pwr_dbm_noise, td_pwr_dbm_signal, td_pwr_dbm_noise


def plotter_per_dataset(samps, default_frame, ant_i, user_i, offset, pilot_frames, match_filt_clr, hdf5, frame_map,
                        seq_found, corr_total, snr):
    """
        Plot channel analysis
    """

    print("PLOT NOW... \n")

    plt_pilots_all = True
    plt_pilots_one = False
    plt_matched_flt = True
    plt_frame_map = False
    plt_pilot_map = True
    plt_snr_map = True

    ref_frame = min(default_frame - hdf5.n_frm_st, samps.shape[0])  # Verify default_frame doesn't exceed # of collected
    ant_plt = ant_i
    user_plt = user_i
    n_cell = match_filt_clr.shape[1]
    n_ue = match_filt_clr.shape[2]
    n_ant = match_filt_clr.shape[3]
    corr_ref_frame = pilot_frames[len(pilot_frames) // 2]

    #####################
    # PLOT PILOTS (ALL) #
    #####################
    if plt_pilots_all:
        # Samps Dimensions: (Frame, Cell, User, Pilot Rep, Antenna, Sample)
        fig, axes = plt.subplots(nrows=4, ncols=n_cell, squeeze=False, figsize=(10, 8))
        for idx in range(0, n_cell):
            axes[0, idx].set_title('ALL PILOTS - Cell %d' % idx)
            axes[0, idx].set_ylabel('All Frames ant %d from user %d (Re)' % (ant_plt, user_plt))
            axes[0, idx].plot(np.real(samps[:, idx, user_plt, 0, ant_plt, :]).flatten())

            axes[1, idx].set_ylabel('All Frames ant %d from user %d (Im)' % (ant_plt, user_plt))
            axes[1, idx].plot(np.imag(samps[:, idx, user_plt, 0, ant_plt, :]).flatten())

            axes[2, idx].set_ylabel('Amplitude')
            for i in range(samps.shape[4]):
                axes[2, idx].plot(np.mean(np.abs(samps[:, idx, user_plt, 0, i, :]), axis=1).flatten())
            axes[2, idx].set_xlabel('Sample')

            axes[3, idx].set_ylabel('Correlation with Frame %d' % corr_ref_frame)
            axes[3, idx].set_ylim([0, 1.1])
            axes[3, idx].set_title('Cell %d offset %d' % (0, offset))
            for u in range(n_ue):
                axes[3, idx].plot(corr_total[pilot_frames, u], label="user %d" % u)
            axes[3, idx].legend(loc='lower right', frameon=False)
            axes[3, idx].set_xlabel('Frame')

    ###########################
    # PLOT PILOTS (ONE FRAME) #
    ###########################
    if plt_pilots_one:
        # Samps Dimensions: (Frame, Cell, User, Pilot Rep, Antenna, Sample)
        fig, axes = plt.subplots(nrows=2, ncols=n_cell, squeeze=False, figsize=(10, 8))
        for idx in range(0, n_cell):
            axes[0, idx].set_title('PILOTS Real (FRAME %d) - Cell %d user' % ((ref_frame + hdf5.n_frm_st), idx))
            axes[0, idx].set_ylabel('Frame %d ant %d cell %d user %d (Re)' % ((ref_frame + hdf5.n_frm_st), ant_plt,
                                                                              idx, user_plt))
            axes[0, idx].plot(np.real(samps[ref_frame, idx, user_plt, 0, ant_plt, :]))

            axes[1, idx].set_title('PILOTS Imag (FRAME %d) - Cell %d user' % ((ref_frame + hdf5.n_frm_st), idx))
            axes[1, idx].set_ylabel('Frame %d ant %d cell %d user %d (Im)' % ((ref_frame + hdf5.n_frm_st), ant_plt,
                                                                              idx, user_plt))
            axes[1, idx].plot(np.imag(samps[ref_frame, idx, user_plt, 0, ant_plt, :]))

    #########################
    # MATCHED FILTER OUTPUT #
    #########################
    if plt_matched_flt:
        fig, axes = plt.subplots(nrows=n_cell, ncols=n_ue, squeeze=False)
        fig.suptitle('Matched Filter Frame # {} Antenna # {}'.format(ref_frame, ant_i))
        for n_c in range(n_cell):
            for n_u in range(n_ue):
                axes[n_c, n_u].stem(match_filt_clr[ref_frame - hdf5.n_frm_st, n_c, n_u, ant_i, :])
                axes[n_c, n_u].set_xlabel('Samples')
                axes[n_c, n_u].set_title('Cell {} UE {}'.format(n_c, n_u))
                axes[n_c, n_u].grid(True)

    #############
    # FRAME MAP #
    #############
    if plt_frame_map:
        fig, axes = plt.subplots(nrows=n_ue, ncols=n_cell, squeeze=False)
        c = []
        fig.suptitle('Frame Map')
        for n_c in range(n_cell):
            for n_u in range(n_ue):
                c.append(axes[n_u, n_c].imshow(frame_map[:, n_c, n_u, :].T, cmap=plt.cm.get_cmap('Blues', 3),
                                               interpolation='none',
                                               extent=[hdf5.n_frm_st, hdf5.n_frm_end, n_ant, 0], aspect="auto"))
                axes[n_u, n_c].set_title('Cell {} UE {}'.format(n_c, n_u))
                axes[n_u, n_c].set_ylabel('Antenna #')
                axes[n_u, n_c].set_xlabel('Frame #')
                # Minor ticks
                axes[n_u, n_c].set_xticks(np.arange(hdf5.n_frm_st, hdf5.n_frm_end, 1), minor=True)
                axes[n_u, n_c].set_yticks(np.arange(0, n_ant, 1), minor=True)
                # Gridlines based on minor ticks
                axes[n_u, n_c].grid(which='minor', color='0.75', linestyle='-', linewidth=0.1)

        cbar = plt.colorbar(c[-1], ax=axes.ravel().tolist(), ticks=[-1, 0, 1], orientation='horizontal')
        cbar.ax.set_xticklabels(['Bad Frame', 'Probably partial/corrupt', 'Good Frame'])

    #############
    # PILOT MAP #
    #############
    if plt_pilot_map:
        fig, axes = plt.subplots(nrows=n_ue, ncols=n_cell, squeeze=False)
        c = []
        fig.suptitle('Pilot Map (Percentage of Detected Pilots Per Symbol) - NOTE: Might exceed 100% due to threshold')
        for n_c in range(n_cell):
            for n_u in range(n_ue):
                c.append(axes[n_u, n_c].imshow(seq_found[:, n_c, n_u, :].T, vmin=0, vmax=100, cmap='Blues',
                                               interpolation='nearest',
                                               extent=[hdf5.n_frm_st, hdf5.n_frm_end, n_ant, 0],
                                               aspect="auto"))
                axes[n_u, n_c].set_title('Cell {} UE {}'.format(n_c, n_u))
                axes[n_u, n_c].set_ylabel('Antenna #')
                axes[n_u, n_c].set_xlabel('Frame #')
                axes[n_u, n_c].set_xticks(np.arange(hdf5.n_frm_st, hdf5.n_frm_end, 1), minor=True)
                axes[n_u, n_c].set_yticks(np.arange(0, n_ant, 1), minor=True)
                axes[n_u, n_c].grid(which='minor', color='0.75', linestyle='-', linewidth=0.05)
        cbar = plt.colorbar(c[-1], ax=axes.ravel().tolist(), ticks=np.linspace(0, 100, 11), orientation='horizontal')
        cbar.ax.set_xticklabels(['0%', '10%', '20%', '30%', '40%', '50%', '60%', '70%', '80%', '90%', '100%'])

    #############
    #  SNR MAP  #
    #############
    if plt_snr_map:
        fig, axes = plt.subplots(nrows=n_ue, ncols=n_cell, squeeze=False)
        c = []
        fig.suptitle('SNR Map')
        for n_c in range(n_cell):
            for n_u in range(n_ue):
                c.append(axes[n_u, n_c].imshow(snr[:, n_c, n_u, :].T, vmin=np.min(snr), vmax=np.max(snr), cmap='Blues',
                                               interpolation='nearest',
                                               extent=[hdf5.n_frm_st, hdf5.n_frm_end, n_ant, 0],
                                               aspect="auto"))
                axes[n_u, n_c].set_title('Cell {} UE {}'.format(n_c, n_u))
                axes[n_u, n_c].set_ylabel('Antenna #')
                axes[n_u, n_c].set_xlabel('Frame #')
                axes[n_u, n_c].set_xticks(np.arange(hdf5.n_frm_st, hdf5.n_frm_end, 1), minor=True)
                axes[n_u, n_c].set_yticks(np.arange(0, n_ant, 1), minor=True)
                axes[n_u, n_c].grid(which='minor', color='0.75', linestyle='-', linewidth=0.05)
        cbar = plt.colorbar(c[-1], ax=axes.ravel().tolist(), ticks=np.linspace(0, np.max(snr), 10),
                            orientation='horizontal')
        # cbar.ax.set_xticklabels(['0%', '10%', '20%', '30%', '40%', '50%', '60%', '70%', '80%', '90%', '100%'])

    # SHOW FIGURES
    plt.show()


def plotter_final(frame_map, snr, snrf, seq_found, n_frames_to_inspect):
    """
        Plotter

        Only supports one client (needs to be extended if we collect measurements with more clients)
    """

    print("FINAL PLOT, NOW... \n")

    # params
    n_loc = len(frame_map)
    n_cell = frame_map[0].shape[1]
    n_ue = frame_map[0].shape[2]
    n_ant = frame_map[0].shape[3]

    # Reshape for plot
    plot_snr_map = np.empty([n_loc, n_cell, n_ant])
    plot_snrf_map = np.empty([n_loc, n_cell, n_ant])
    plot_pilot_map = np.empty([n_loc, n_cell, n_ant])
    plot_goodp_map = np.empty([n_loc, n_cell, n_ant])
    plot_badp_map = np.empty([n_loc, n_cell, n_ant])
    plot_partialp_map = np.empty([n_loc, n_cell, n_ant])
    for dsidx in range(n_loc):
        # Frame G/B/P
        for cidx in range(n_cell):
            for aidx in range(n_ant):
                # FRAME MAP
                this_frame_map = frame_map[dsidx][:, cidx, 0, aidx]
                plot_goodp_map[dsidx, cidx, aidx] = this_frame_map[this_frame_map == 1].size
                plot_badp_map[dsidx, cidx, aidx] = this_frame_map[this_frame_map == -1].size
                plot_partialp_map[dsidx, cidx, aidx] = this_frame_map[this_frame_map == 0].size
                # SNR
                plot_snr_map[dsidx, cidx, aidx] = np.nanmean(snr[dsidx][:, cidx, 0, aidx], axis=0)
                plot_snrf_map[dsidx, cidx, aidx] = np.nanmean(snrf[dsidx][:, cidx, 0, aidx], axis=0)

                tmp1 = np.nanmean(snr[dsidx][:, cidx, 0, aidx], axis=0)
                tmp2 = np.nanmean(snrf[dsidx][:, cidx, 0, aidx], axis=0)
                if np.isnan(tmp1).any() or np.isnan(tmp2).any():
                    print("Found Nan!!")

                # NUM PILOTS
                plot_pilot_map[dsidx, cidx, aidx] = np.mean(seq_found[dsidx][:, cidx, 0, aidx], axis=0)

    ############
    # PLOT SNR #
    ############
    fig, axes = plt.subplots(nrows=1, ncols=n_ant, squeeze=False)
    c = []
    fig.suptitle('SNR')
    for n_a in range(n_ant):
        c.append(axes[0, n_a].imshow(plot_snr_map[:, :, n_a].T, vmin=np.min(plot_snr_map), vmax=np.max(plot_snr_map),
                                     cmap='Blues',
                                       interpolation='none',
                                       extent=[0, n_loc, n_cell, 0], aspect="auto"))
        axes[0, n_a].set_ylabel('Cell #')
        axes[0, n_a].set_xlabel('Location Index')
        # Minor ticks
        axes[0, n_a].xaxis.set_major_locator(ticker.FixedLocator(np.arange(0, n_loc, 1)))
        axes[0, n_a].xaxis.set_major_formatter(ticker.FixedFormatter(['1', '2', '3', '4', '5', '6', '7', '8', '9', '10',
                                                                      '11', '13', '14', '15']))
        axes[0, n_a].yaxis.set_major_locator(ticker.FixedLocator(np.arange(0, n_cell, 1)))
        axes[0, n_a].yaxis.set_major_formatter(ticker.FixedFormatter(['ACC', 'Honors', 'MEB', 'USTAR', 'EBC']))
        axes[0, n_a].tick_params(axis="x", labelsize=7)
        axes[0, n_a].tick_params(axis="y", labelsize=7)
        # Gridlines based on minor ticks
        axes[0, n_a].grid(which='minor', color='0.75', linestyle='-', linewidth=0.1)

        for i in range(n_loc):
            for j in range(n_cell):
                curr_val = str(round(plot_snr_map[i, j, n_a], 2))
                # print("LOC: {} CELL: {} SNR: {}".format(i, j, curr_val))
                text = axes[0, n_a].text(i, j, curr_val, ha="left", va="top", color="r", fontsize=4)

    cbar = plt.colorbar(c[-1], ax=axes.ravel().tolist(), ticks=np.linspace(np.min(plot_snr_map), np.max(plot_snr_map),
                                                                           20), orientation='horizontal')
    cbar.ax.tick_params(labelsize=5)
    cbar.set_label('SNR (dB)')
    plt.savefig('snrt.pdf')

    ############
    # PLOT SNRf #
    ############
    fig, axes = plt.subplots(nrows=1, ncols=n_ant, squeeze=False)
    c = []
    fig.suptitle('SNR')
    for n_a in range(n_ant):
        c.append(axes[0, n_a].imshow(plot_snrf_map[:, :, n_a].T, vmin=np.min(plot_snrf_map), vmax=np.max(plot_snrf_map),
                                     cmap='Blues',
                                       interpolation='none',
                                       extent=[0, n_loc, n_cell, 0], aspect="auto"))
        if n_a == 0:
            axes[0, n_a].set_ylabel('Cell #')
        axes[0, n_a].set_xlabel('Location Index')
        # Minor ticks
        axes[0, n_a].xaxis.set_major_locator(ticker.FixedLocator(np.arange(0, n_loc, 1)))
        axes[0, n_a].xaxis.set_major_formatter(ticker.FixedFormatter(['1', '2', '3', '4', '5', '6', '7', '8', '9', '10',
                                                                      '11', '13', '14', '15']))
        axes[0, n_a].yaxis.set_major_locator(ticker.FixedLocator(np.arange(0, n_cell, 1)))
        axes[0, n_a].yaxis.set_major_formatter(ticker.FixedFormatter(['ACC', 'Honors', 'MEB', 'USTAR', 'EBC']))
        axes[0, n_a].tick_params(axis="x", labelsize=7)
        axes[0, n_a].tick_params(axis="y", labelsize=7)
        # Gridlines based on minor ticks
        axes[0, n_a].grid(which='minor', color='0.75', linestyle='-', linewidth=0.1)

        for i in range(n_loc):
            for j in range(n_cell):
                curr_val = str(round(plot_snrf_map[i, j, n_a], 2))
                # print("LOC: {} CELL: {} SNR: {}".format(i, j, curr_val))
                text = axes[0, n_a].text(i, j, curr_val, ha="left", va="top", color="r", fontsize=4)

    cbar = plt.colorbar(c[-1], ax=axes.ravel().tolist(), ticks=np.linspace(np.min(plot_snrf_map), np.max(plot_snrf_map),
                                                                           20), orientation='horizontal')
    cbar.ax.tick_params(labelsize=5)
    cbar.set_label('SNR (dB)')
    plt.savefig('snrf.pdf')
    plt.savefig('snrf.png')

    #########################
    # PLOT NUMBER OF PILOTS #
    #########################
    fig, axes = plt.subplots(nrows=1, ncols=n_ant, squeeze=False)
    c = []
    fig.suptitle('PERCENT OF PILOTS')
    for n_a in range(n_ant):
        c.append(axes[0, n_a].imshow(plot_pilot_map[:, :, n_a].T, vmin=0, vmax=np.max(plot_pilot_map), cmap='Blues',
                                       interpolation='none',
                                       extent=[0, n_loc, n_cell, 0], aspect="auto"))
        axes[0, n_a].set_ylabel('Cell #')
        axes[0, n_a].set_xlabel('Location Index')
        # Minor ticks
        axes[0, n_a].xaxis.set_major_locator(ticker.FixedLocator(np.arange(0, n_loc, 1)))
        axes[0, n_a].xaxis.set_major_formatter(ticker.FixedFormatter(['1', '2', '3', '4', '5', '6', '7', '8', '9', '10',
                                                                      '11', '13', '14', '15']))
        axes[0, n_a].yaxis.set_major_locator(ticker.FixedLocator(np.arange(0, n_cell, 1)))
        axes[0, n_a].yaxis.set_major_formatter(ticker.FixedFormatter(['ACC', 'Honors', 'MEB', 'USTAR', 'EBC']))
        # Gridlines based on minor ticks
        axes[0, n_a].grid(which='minor', color='0.75', linestyle='-', linewidth=0.1)

        for i in range(n_loc):
            for j in range(n_cell):
                curr_val = str(round(plot_pilot_map[i, j, n_a], 2))
                text = axes[0, n_a].text(i, j, curr_val, ha="left", va="top", color="r", fontsize=8)

    cbar = plt.colorbar(c[-1], ax=axes.ravel().tolist(), ticks=np.linspace(0, np.max(plot_pilot_map)), orientation='horizontal')
    cbar.set_label('Percent of Detected Pilots (%)')
    plt.savefig('pilots.pdf')

    ##########################################
    # PLOT NUMBER OF GOOD/BAD/PARTIAL FRAMES #
    ##########################################
    fig, axes = plt.subplots(nrows=n_ant, ncols=3, squeeze=False)
    c = []
    fig.suptitle('# OF GOOD vs BAD vs PARTIAL')
    for n_p in range(3):
        for n_a in range(n_ant):
            # TODO: lazy, redo this...
            if n_p == 0:
                c.append(axes[n_a, n_p].imshow(plot_goodp_map[:, :, n_a].T, vmin=0, vmax=n_frames_to_inspect,
                                               cmap='Blues',
                                               interpolation='none',
                                               extent=[0, n_loc, n_cell, 0], aspect="auto"))
                axes[n_a, n_p].set_title('GOOD, ANT {}'.format(n_a))
                for i in range(n_loc):
                    for j in range(n_cell):
                        text = axes[n_a, n_p].text(i, j, plot_goodp_map[i, j, n_a], ha="left", va="top", color="r",
                                                   fontsize=12)
            elif n_p == 1:
                c.append(axes[n_a, n_p].imshow(plot_badp_map[:, :, n_a].T, vmin=0, vmax=n_frames_to_inspect,
                                               cmap='Blues',
                                               interpolation='none',
                                               extent=[0, n_loc, n_cell, 0], aspect="auto"))
                axes[n_a, n_p].set_title('BAD, ANT {}'.format(n_a))
                for i in range(n_loc):
                    for j in range(n_cell):
                        text = axes[n_a, n_p].text(i, j, plot_badp_map[i, j, n_a], ha="left", va="top", color="r",
                                                   fontsize=12)
            elif n_p == 2:
                c.append(axes[n_a, n_p].imshow(plot_partialp_map[:, :, n_a].T, vmin=0, vmax=n_frames_to_inspect,
                                               cmap='Blues',
                                               interpolation='none',
                                               extent=[0, n_loc, n_cell, 0], aspect="auto"))
                axes[n_a, n_p].set_title('PARTIAL, ANT {}'.format(n_a))
                for i in range(n_loc):
                    for j in range(n_cell):
                        text = axes[n_a, n_p].text(i, j, plot_partialp_map[i, j, n_a], ha="left", va="top", color="r",
                                                   fontsize=12)

            axes[n_a, n_p].set_ylabel('Cell #')
            axes[n_a, n_p].set_xlabel('Location Index')
            # Minor ticks
            axes[n_a, n_p].xaxis.set_major_locator(ticker.FixedLocator(np.arange(0, n_loc, 1)))
            axes[n_a, n_p].xaxis.set_major_formatter(ticker.FixedFormatter(['1', '2', '3', '4', '5', '6', '7', '8', '9',
                                                                            '10', '11', '13', '14', '15']))
            axes[n_a, n_p].yaxis.set_major_locator(ticker.FixedLocator(np.arange(0, n_cell, 1)))
            axes[n_a, n_p].yaxis.set_major_formatter(ticker.FixedFormatter(['ACC', 'Honors', 'MEB', 'USTAR', 'EBC']))
            # Gridlines based on minor ticks
            axes[n_a, n_p].grid(which='minor', color='0.75', linestyle='-', linewidth=0.1)
    cbar = plt.colorbar(c[-1], ax=axes.ravel().tolist(), ticks=np.linspace(0, n_frames_to_inspect),
                        orientation='horizontal')
    cbar.set_label('Number of Good/Bad/Partial Frames')

    plt.show()


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

    parser = OptionParser()
    parser.add_option("--ref-frame",    type="int",          dest="ref_frame",           help="Frame number to plot", default=0)
    parser.add_option("--ref-ant",      type="int",          dest="ref_ant",             help="Reference antenna", default=0)
    parser.add_option("--ref-user",     type="int",          dest="ref_user",            help="Reference User", default=0)
    parser.add_option("--n-frames",     type="int",          dest="n_frames_to_inspect", help="Number of frames to inspect. This number must be smaller than the number of collected frames", default=2000)
    parser.add_option("--sub-sample",   type="int",          dest="sub_sample",          help="Sub sample rate", default=1)
    parser.add_option("--thresh",       type="float",        dest="thresh",              help="Ampiltude Threshold for valid frames", default=0.001)
    parser.add_option("--frame-start",  type="int",          dest="fr_strt",             help="Starting frame. Must have set n_frames_to_inspect first and make sure fr_strt is within boundaries ", default=0)
    parser.add_option("--plot-level",   type="int",          dest="plot_level",          help="Level 0: only plot overall results. Level 1: plot results for each individual dataset plus overall results", default=0)  # TODO add more?
    (options, args) = parser.parse_args()

    n_frames_to_inspect = options.n_frames_to_inspect
    ref_ant = options.ref_ant
    ref_frame = options.ref_frame
    ref_user = options.ref_user
    thresh = options.thresh
    fr_strt = options.fr_strt
    sub_sample = options.sub_sample
    plot_level = options.plot_level

    scrpt_strt = time.time()
    if n_frames_to_inspect == 0:
        print("WARNING: No frames_to_inspect given. Will process the whole dataset.")

    if ref_frame > n_frames_to_inspect:
        print("WARNING: Attempted to inspect a frame at an index larger than the no. of requested frames: "
              "ref_frame:{} >  n_frames_to_inspect:{}. ".format(
                ref_frame, n_frames_to_inspect))
        print("Setting the frame to inspect to 0")
        ref_frame = 0

    if (ref_frame > fr_strt + n_frames_to_inspect) or (ref_frame < fr_strt) :
        print("WARNING: Attempted to inspect a frame at an index larger than the no. of requested frames +  "
              "or at an index smaller than the required start of the frames: ref_frame:{} > n_frames_to_inspect:{} or "
              "ref_frame:{} <  fr_strt:{}. ".format(
                ref_frame, n_frames_to_inspect, ref_frame, fr_strt))
        print("Setting the frame to inspect/plot to {}".format(fr_strt))
        ref_frame = fr_strt
    print(">> frame to plot = {}, ref. ant = {}, ref. user = {}, no. of frames to inspect = {}, "
          "starting frame = {} <<".format(ref_frame, ref_ant, ref_user, n_frames_to_inspect, fr_strt))

    run_full = True
    if run_full:
        # Run full analysis, not just plotter
        # Load datasets
        hdf5_signal, hdf5_noise = datasets(n_frames_to_inspect, fr_strt)

        # Main Loop
        frame_map = []
        snr = []
        snr_f = []
        seq_found = []
        fd_dbm_s = []
        fd_dbm_n = []
        td_dbm_s = []
        td_dbm_n = []
        for dsidx in range(len(hdf5_signal)):
            print("Running Dataset {} out of {}".format(dsidx, len(hdf5_signal)))
            #iframe_map, isnr, isnr_f, iseq_found = parse_signal_hdf5(hdf5_signal[dsidx], hdf5_noise[dsidx], ref_frame, ref_ant,
            #                                                 ref_user, thresh, sub_sample, plot_level)
            iframe_map, isnr, isnr_f, iseq_found, ifd_pwr_dbm_s, ifd_pwr_dbm_n, itd_pwr_dbm_s, itd_pwr_dbm_n = \
                parse_signal_hdf5(hdf5_signal[dsidx], hdf5_noise[dsidx], ref_frame, ref_ant, ref_user, thresh, sub_sample, plot_level)
            frame_map.append(iframe_map)
            snr.append(isnr)
            snr_f.append(isnr_f)
            seq_found.append(iseq_found)
            fd_dbm_s.append(ifd_pwr_dbm_s)
            fd_dbm_n.append(ifd_pwr_dbm_n)
            td_dbm_s.append(itd_pwr_dbm_s)
            td_dbm_n.append(itd_pwr_dbm_n)
        #sio.savemat('mcampaign_data_snrf.mat', {'frame_map': frame_map, 'snr': snr, 'snr_f': snr_f,
        #                                        'seq_found': seq_found,
        #                                        'n_frames_to_inspect': n_frames_to_inspect,
        #                                        'fd_dbm_s': fd_dbm_s,
        #                                        'fd_dbm_n': fd_dbm_n,
        #                                        'td_dbm_s': td_dbm_s,
        #                                        'td_dbm_n': td_dbm_n})
    else:
        # Only run plotter with data saved from last analysis
        # mat_contents = sio.loadmat('mcampaign_data.mat')
        mat_contents = sio.loadmat('mcampaign_data_snrf.mat')
        frame_map = mat_contents['frame_map']
        snr = mat_contents['snr']
        snr_f = mat_contents['snr_f']
        seq_found = mat_contents['seq_found']
        n_frames_to_inspect = mat_contents['n_frames_to_inspect']

    plotter_final(frame_map, snr, snr_f, seq_found, n_frames_to_inspect)

    scrpt_end = time.time()
    print(">>>> Script Duration: time: %f \n" % (scrpt_end - scrpt_strt))


if __name__ == '__main__':
    main()

