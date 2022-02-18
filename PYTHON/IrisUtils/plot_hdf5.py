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
 Copyright © 2018-2022. Rice University.
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
import matplotlib
from plot_lib import *
#matplotlib.use("Agg")

def verify_hdf5(hdf5, frame_i=100, cell_i=0, ofdm_sym_i=0, ant_i =0,
                user_i=0, ul_slot_i=0, dl_slot_i=0, subcarrier_i=10, offset=-1,
                dn_calib_offset=0, up_calib_offset=0, thresh=0.001,
                deep_inspect=False, corr_thresh=0.00, exclude_bs_nodes=[],
                demodulate=False, analyze=False):
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
    if 'SYMBOL_LEN' in metadata: # to support older datasets
        samps_per_slot = int(metadata['SYMBOL_LEN'])
    elif 'SLOT_SAMP_LEN' in metadata:
        samps_per_slot = int(metadata['SLOT_SAMP_LEN'])
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
    nonzero_sc_size = fft_size
    if 'DATA_SUBCARRIER_NUM' in metadata:
        nonzero_sc_size = metadata['DATA_SUBCARRIER_NUM']
    ofdm_pilot = np.array(metadata['OFDM_PILOT'])
    if "OFDM_PILOT_F" in metadata.keys():
        ofdm_pilot_f = np.array(metadata['OFDM_PILOT_F'])
    else:
        if pilot_type == 'zadoff-chu':
            _, pilot_f = generate_training_seq(preamble_type='zadoff-chu', seq_length=nonzero_sc_size, cp=cp, upsample=1, reps=1)
        else:
            _, pilot_f = generate_training_seq(preamble_type='lts', cp=32, upsample=1)
        ofdm_pilot_f = pilot_f
    reciprocal_calib = np.array(metadata['RECIPROCAL_CALIB'])
    ofdm_len = fft_size + cp
    symbol_per_slot = (samps_per_slot - z_padding) // ofdm_len
    if 'UL_SYMS' in metadata:
        ul_slot_num = int(metadata['UL_SYMS'])
    elif 'UL_SLOTS' in metadata:
        ul_slot_num = int(metadata['UL_SLOTS'])
    num_bs_ant = int(metadata['BS_ANT_NUM_PER_CELL'][cell_i])
    all_bs_nodes = set(range(num_bs_ant))
    plot_bs_nodes = list(all_bs_nodes - set(exclude_bs_nodes))
    n_ue = num_cl

    pilot_data_avail = len(hdf5.pilot_samples) > 0
    ul_data_avail = len(hdf5.uplink_samples) > 0
    noise_avail = len(hdf5.noise_samples) > 0
    dl_data_avail = len(hdf5.downlink_samples) > 0

    # Plot DL data symbols
    if dl_data_avail > 0:
        # DL Samps: #Frames, #Cell, #Uplink Symbol, #Antennas, #Samples
        downlink_samples = hdf5.downlink_samples
        frm_plt = min(frame_i, downlink_samples.shape[0] + n_frm_st)
        # Verify frame_i does not exceed max number of collected frames
        ref_frame = min(frame_i - n_frm_st, downlink_samples.shape[0])
        samps_mat = np.reshape(
                downlink_samples, (downlink_samples.shape[0], downlink_samples.shape[1], downlink_samples.shape[2], downlink_samples.shape[3], samps_per_slot, 2))
        dl_samps = (samps_mat[:, :, :, :, :, 0] +
                samps_mat[:, :, :, :, :, 1]*1j)*2**-15

        plot_iq_samps(dl_samps, n_frm_st, ref_frame, [cell_i], [dl_slot_i], [user_i], data_str="Downlink Data")

    if pilot_data_avail:
        pilot_samples = hdf5.pilot_samples[:, :, :, plot_bs_nodes, :]

        frm_plt = min(frame_i, pilot_samples.shape[0] + n_frm_st)
        # Verify frame_i does not exceed max number of collected frames
        ref_frame = min(frame_i - n_frm_st, pilot_samples.shape[0])

        print("samps_per_slot = {}, offset = {}, cp = {}, prefix_len = {}, postfix_len = {}, z_padding = {}, pilot_rep = {}".format(samps_per_slot, offset, cp, prefix_len, postfix_len, z_padding, symbol_per_slot))

        # pilot_samples dimensions:
        # ( #frames, #cells, #pilot subframes or cl ant sending pilots, #bs nodes or # bs ant, #samps per frame * 2 for IQ )
        num_frames = pilot_samples.shape[0]
        num_cells = pilot_samples.shape[1]
        num_cl_tmp = pilot_samples.shape[2]
        num_ants = pilot_samples.shape[3]

        samps_mat = np.reshape(
                pilot_samples, (num_frames, num_cells, num_cl_tmp, num_ants, samps_per_slot, 2))
        samps = (samps_mat[:, :, :, :, :, 0] +
                samps_mat[:, :, :, :, :, 1]*1j)*2**-15

        # Correlation (Debug plot useful for checking sync)
        good_ants = []
        insp_ants = [] # antennas to be inspected
        if ant_i > num_bs_ant - 1:
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

        # Plotter
        # Plot pilots
        if not reciprocal_calib:
            # Compute CSI from IQ samples
            # Samps: #Frames, #Cell, #Users, #Antennas, #Samples
            # CSI:   #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Subcarrier
            # For correlation use a fft size of 64
            print("*verify_hdf5(): Calling samps2csi with fft_size = {}, offset = {}, bound = {}, cp = {} *".format(fft_size, offset, z_padding, cp))
            csi, _ = hdf5_lib.samps2csi(pilot_samples, num_pilots, samps_per_slot, fft_size=fft_size,
                                            offset=offset, bound=z_padding, cp=cp, pilot_f=ofdm_pilot_f)

            cellCSI = csi[:, cell_i, :, :, :, :]
            if corr_thresh > 0.0: 
                bad_nodes = find_bad_nodes(cellCSI, corr_thresh=corr_thresh,
                                           user=user_i)
                if bad_nodes:
                    print(">>> Warning! List of bad nodes (1-based): {bad_nodes}".
                          format(bad_nodes=bad_nodes))
                else:
                    print(">>> All Iris nodes are good!")

            if ofdm_sym_i >= symbol_per_slot:  # if out of range index, do average
                userCSI = np.mean(cellCSI[:, :, :, :, :], 2)
            else:
                userCSI = cellCSI[:, :, ofdm_sym_i, :, :]
            corr_total, sig_sc = calCorr(userCSI, np.transpose(np.conj(userCSI[ref_frame, :, :, :]), (1, 0, 2) ) )

            for i in insp_ants:
                plot_iq_samps(samps, n_frm_st, ref_frame, [cell_i], [user_i], [ant_i])
            plot_csi(userCSI, corr_total, plot_bs_nodes, pilot_frames, ref_frame, cell_i, user_i, subcarrier_i, offset)
        else:
            calib_plot_bs_nodes = list(set(plot_bs_nodes) - set([num_bs_ant - 1]))
            if plot_bs_nodes[-1] == num_bs_ant - 1:
                calib_plot_bs_nodes = plot_bs_nodes[:-1]
                calib_pilot_samples = pilot_samples[:, :, :, :-1, :]
            else:
                calib_plot_bs_nodes = plot_bs_nodes
                calib_pilot_samples = pilot_samples
            print(calib_plot_bs_nodes)
            for i in insp_ants:
                plot_iq_samps(samps, n_frm_st, ref_frame, [cell_i], [0, 1, 2 + user_i], [ant_i])

            # frame, downlink(0)-uplink(1), antennas, subcarrier
            csi_u,_ = hdf5_lib.samps2csi(calib_pilot_samples[:, :, 1:2, :, :], 1, samps_per_slot, fft_size=fft_size,
                                          offset=up_calib_offset, bound=z_padding, cp=cp, pilot_f=ofdm_pilot_f)
            csi_d,_ = hdf5_lib.samps2csi(calib_pilot_samples[:, :, 0:1, :, :], 1, samps_per_slot, fft_size=fft_size,
                                          offset=dn_calib_offset, bound=z_padding, cp=cp, pilot_f=ofdm_pilot_f)
            calib_mat = np.divide(csi_d[:, cell_i, 0, ofdm_sym_i, :, :], csi_u[:, cell_i, 0, ofdm_sym_i, :, :])
            try:
                ant_id = calib_plot_bs_nodes.index(ant_i)
            except:
                ant_id = calib_plot_bs_nodes[0]
            plot_calib(calib_mat, range(len(calib_plot_bs_nodes)), ref_frame, ant_id, subcarrier_i)
            if num_cl > 0:
                csi, _ = hdf5_lib.samps2csi(calib_pilot_samples[:, :, 2:, :, :], num_cl, samps_per_slot, fft_size=fft_size,
                                                offset=offset, bound=z_padding, cp=cp, pilot_f=ofdm_pilot_f)
                uplink_csi = csi[:, cell_i, :, ofdm_sym_i, :, :]
                corr_total, sig_sc = calCorr(uplink_csi, np.transpose(np.conj(uplink_csi[ref_frame, :, :, :]), (1, 0, 2) ) )
                plot_csi(uplink_csi, corr_total, calib_plot_bs_nodes, pilot_frames, ref_frame, cell_i, user_i, subcarrier_i, offset)
                # imp: Frames, #Antennas, #Users, Subcarrier
                implicit_dl_csi = np.empty((uplink_csi.shape[0], uplink_csi.shape[2], uplink_csi.shape[1], uplink_csi.shape[3]), dtype='complex64')
                for i in range(uplink_csi.shape[0]):
                    for j in range(uplink_csi.shape[3]):
                        implicit_dl_csi[i, :, :, j] = np.transpose(uplink_csi[i, :, :, j] * calib_mat[i, :, j])
                # downlink_samples: #Frames, #Cell, #Bs Antenna, #Users, #Samples
                # CSI:              #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Subcarrier
                dl_offset = 288 # make a param
                dl_csi, _ = hdf5_lib.samps2csi(downlink_samples[:, :, calib_plot_bs_nodes, :, :], len(calib_plot_bs_nodes), samps_per_slot, fft_size=fft_size,
                                                offset=dl_offset, bound=z_padding, cp=cp, pilot_f=ofdm_pilot_f)
                # exp: Frames, #Antennas, #Users, Subcarrier
                explicit_dl_csi = np.transpose(dl_csi[:, cell_i, :, ofdm_sym_i, :, :], (0, 2, 1, 3))
                corr_dl, _ = calCorr(explicit_dl_csi, np.transpose(np.conj(explicit_dl_csi[ref_frame, :, :, :]), (1, 0, 2) ) )
                plot_csi(explicit_dl_csi, corr_dl, range(len(calib_plot_bs_nodes)), range(explicit_dl_csi.shape[0]), ref_frame, cell_i, user_i, subcarrier_i, dl_offset, "Downlink")

                dl_csi_err_mag = np.abs(explicit_dl_csi - implicit_dl_csi)
                dl_csi_err_phs = np.angle(explicit_dl_csi - implicit_dl_csi)
                fig3, axes3 = plt.subplots(nrows=2, ncols=1, squeeze=False, figsize=(10, 8))
                axes3[0, 0].set_title('Implicit vs Explicit DL CSI - Cell 0 - Subcarrier %d' % subcarrier_i)

                axes3[0, 0].set_ylabel('magtinute (ant %d)' % (ant_i))
                axes3[0, 0].plot(dl_csi_err_mag[:, ant_id, user_i, subcarrier_i].flatten(), label='')
                axes3[0, 0].set_xlabel('frame')
                axes3[0, 0].legend(frameon=False)

                axes3[1, 0].set_ylabel('Phase (ant %d)' % (ant_i))
                axes3[1, 0].plot(dl_csi_err_phs[:, ant_id, user_i, subcarrier_i].flatten(), label='')
                axes3[1, 0].set_xlabel('frame')
                axes3[1, 0].set_ylim(-np.pi, np.pi)
                axes3[1, 0].legend(frameon=False)

    if analyze:
        analyze_hdf5(hdf5, frame_i, cell_i, subcarrier_i, offset)

    # Plot UL data symbols
    if ul_data_avail > 0:
        # UL Samps: #Frames, #Cell, #Uplink Symbol, #Antennas, #Samples
        uplink_samples = hdf5.uplink_samples[:, :, :, plot_bs_nodes, :]
        ref_frame = min(frame_i - n_frm_st, uplink_samples.shape[0])
        samps_mat = np.reshape(
                uplink_samples, (uplink_samples.shape[0], uplink_samples.shape[1], uplink_samples.shape[2], uplink_samples.shape[3], samps_per_slot, 2))
        ul_samps = (samps_mat[:, :, :, :, :, 0] +
                samps_mat[:, :, :, :, :, 1]*1j)*2**-15

        plot_iq_samps(ul_samps, n_frm_st, ref_frame, [cell_i], [ul_slot_i], [ant_i], data_str="Uplink Data")

        if demodulate:
            equalized_symbols, tx_symbols, slot_evm, slot_evm_snr = hdf5_lib.demodulate(ul_samps[:, cell_i, :, :, :], userCSI, metadata, hdf5.dirpath, offset, ul_slot_i)
            plot_constellation_stats(slot_evm, slot_evm_snr, equalized_symbols, tx_symbols, ref_frame, cell_i, ul_slot_i)


    if deep_inspect:
        filter_pilots_start = time.time()
        match_filt, seq_num, seq_len, cmpx_pilots, seq_orig = hdf5_lib.filter_pilots(samps, z_padding, fft_size, cp, pilot_type, nonzero_sc_size)
        filter_pilots_end = time.time()

        frame_sanity_start = time.time()
        match_filt_clr, frame_map, f_st, peak_map = hdf5_lib.frame_sanity(match_filt, seq_num, seq_len, n_frm_st, frame_to_plot=frame_i, plt_ant=ant_i, cp = cp)
        frame_sanity_end = time.time()
        print(">>>> filter_pilots time: %f \n" % ( filter_pilots_end - filter_pilots_start) )
        print(">>>> frame_sanity time: %f \n" % ( frame_sanity_end - frame_sanity_start) )

        snr_start = time.time()
        snr, seq_found, cfo = hdf5_lib.measure_snr(pilot_samples, hdf5.noise_samples, peak_map, pilot_type, ofdm_pilot, ofdm_len, z_padding, rate, plot_bs_nodes)
        snr_end = time.time()
        print(">>>> compute_snr time: %f \n" % (snr_end - snr_start))

        # Plots:
        print("Plotting the results:\n")

        # plot a frame:
        plot_match_filter(match_filt, ref_frame, n_frm_st, ant_i)
        #plot channel analysis
        show_plot(cmpx_pilots, seq_orig, match_filt, user_i, ant_i, ref_frame, n_frm_st)

        plot_start_frame(f_st, n_frm_st)
        #plot_cfo(cfo, n_frm_st)
        plot_pilot_mat(seq_found, n_frm_st, n_frm_end)

        #############
        #  SNR MAP  #
        #############
        if noise_avail:
            plot_snr_map(snr, n_frm_st, n_frm_end, num_bs_ant)

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
    samps_per_slot = int(metadata['SLOT_SAMP_LEN'])
    rate = float(metadata['RATE'])
    slot_num = int(metadata['BS_FRAME_LEN'])
    timestep = samps_per_slot*slot_num/rate
    num_cl = int(metadata['CL_NUM'])
    num_pilots = int(metadata['PILOT_NUM'])
    prefix_len = int(metadata['PREFIX_LEN'])
    postfix_len = int(metadata['POSTFIX_LEN'])
    z_padding = prefix_len + postfix_len
    if offset < 0: # if no offset is given use prefix from HDF5
        offset = int(prefix_len)
    fft_size = int(metadata['FFT_SIZE'])
    cp = int(metadata['CP_LEN'])
    ofdm_pilot_f = np.array(metadata['OFDM_PILOT_F'])
    sched = metadata['BS_FRAME_SCHED'].astype(str)[0]

    num_noise_slots = noise_samples.shape[2]
    n_frame = pilot_samples.shape[0]
    n_cell = pilot_samples.shape[1]
    n_ue = pilot_samples.shape[2]
    n_ant = pilot_samples.shape[3]

    # compute CSI for each user and get a nice numpy array
    # Returns csi with Frame, Cell, User, pilot repetitions, BS ant, Subcarrier
    # also, iq samples nicely chunked out, same dims, but subcarrier is sample.
    num_cl_tmp = num_pilots
    csi,_ = hdf5_lib.samps2csi(pilot_samples, num_cl_tmp, samps_per_slot, fft_size=fft_size,
                                offset=offset, bound=z_padding, cp=cp, pilot_f=ofdm_pilot_f)
    csi = csi[:, cell_i, :, :, :, :]

    noise,_ = hdf5_lib.samps2csi(noise_samples, num_noise_slots, samps_per_slot, fft_size=fft_size,
                                offset=offset, bound=z_padding, cp=cp, pilot_f=ofdm_pilot_f)
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

    subf_conj = conj[-2]
    subf_zf = zf[-2]
    mubf_conj = conj[1]
    mubf_zf = zf[1]

    # plot stuff
    time_vector = np.arange(0, csi.shape[0]*timestep, timestep)[:csi.shape[0]]
    plot_spectral_efficiency(subf_conj, subf_zf, mubf_conj, mubf_zf, time_vector, num_cl_tmp, n_ant, n_ue, frame_i) 
    #plot_demmel_snr(demmel, timestamp, subcarrier_i)

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


def main():
    # Tested with inputs: ./data_in/Argos-2019-3-11-11-45-17_1x8x2.hdf5 300  (for two users)
    #                     ./data_in/Argos-2019-3-30-12-20-50_1x8x1.hdf5 300  (for one user) 
    parser = OptionParser()
    parser.add_option("--show-metadata", action="store_true", dest="show_metadata", help="Displays hdf5 metadata", default=False)
    parser.add_option("--deep-inspect", action="store_true", dest="deep_inspect", help="Run script without analysis", default=False)
    parser.add_option("--demodulate", action="store_true", dest="demodulate", help="Demodulate uplink data", default=False)
    parser.add_option("--ref-frame", type="int", dest="ref_frame", help="Frame number to plot", default=1000)
    parser.add_option("--ref-ul-slot", type="int", dest="ref_ul_slot", help="UL slot number to plot", default=0)
    parser.add_option("--ref-dl-slot", type="int", dest="ref_dl_slot", help="DL slot number to plot", default=0)
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
    parser.add_option("--n-frames", type="int", dest="n_frames", help="Number of frames to inspect", default=2000)
    parser.add_option("--sub-sample", type="int", dest="sub_sample", help="Sub sample rate", default=1)
    parser.add_option("--thresh", type="float", dest="thresh", help="Ampiltude Threshold for valid frames", default=0.001)
    parser.add_option("--frame-start", type="int", dest="fr_strt", help="Starting frame. Must have set n_frames first and make sure fr_strt is within boundaries ", default=0)
    parser.add_option("--verify-trace", action="store_true", dest="verify", help="Run script without analysis", default=True)
    parser.add_option("--analyze-trace", action="store_true", dest="analyze", help="Run script without analysis", default=False)
    parser.add_option("--corr-thresh", type="float", dest="corr_thresh",
                      help="Correlation threshold to exclude bad nodes",
                      default=0.00)
    (options, args) = parser.parse_args()

    show_metadata = options.show_metadata
    deep_inspect = options.deep_inspect
    demodulate = options.demodulate
    n_frames = options.n_frames
    ref_frame = options.ref_frame
    ref_cell = options.ref_cell
    ref_ofdm_sym = options.ref_ofdm_sym
    ref_ant = options.ref_ant
    ref_user = options.ref_user
    ref_subcarrier = options.ref_subcarrier
    ref_ul_slot = options.ref_ul_slot
    ref_dl_slot = options.ref_dl_slot
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

    if n_frames == 0:
        print("WARNING: No frames_to_inspect given. Will process the whole dataset.") 

    if ref_frame > n_frames:
        print("WARNING: Attempted to inspect a frame at an index larger than the no. of requested frames: ref_frame:{} >  n_frames:{}. ".format(
                ref_frame, n_frames))
        print("Setting the frame to inspect to 0")
        ref_frame = 0
 
    if (ref_frame > fr_strt + n_frames) or (ref_frame < fr_strt) :
        print("WARNING: Attempted to inspect a frame at an index larger than the no. of requested frames +  or at an index smaller than the required start of the frames: ref_frame:{} > n_frames:{} or ref_frame:{} <  fr_strt:{}. ".format(
                ref_frame, n_frames, ref_frame, fr_strt))
        print("Setting the frame to inspect/plot to {}".format(fr_strt))
        ref_frame = fr_strt

    print(">> frame to plot = {}, ref. ant = {}, ref. user = {}, no. of frames to inspect = {}, starting frame = {} <<".format(ref_frame, ref_ant, ref_user, n_frames, fr_strt))

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
            downlink_samples = hdf5.downlink_samples
            noise_avail = len(hdf5.noise_samples) > 0

            # Check which data we have available
            pilots_avail = len(pilot_samples) > 0
            ul_data_avail = len(uplink_samples) > 0
            dl_data_avail = len(downlink_samples) > 0
            if pilots_avail:
                print("HDF5 pilot data size:")
                print(pilot_samples.shape)
            if ul_data_avail:
                print("HDF5 uplink data size:")
                print(uplink_samples.shape)
            if dl_data_avail:
                print("HDF5 downlink data size:")
                print(downlink_samples.shape)

        else:
            hdf5 = hdf5_lib(filename, n_frames, fr_strt, sub_sample)
            data = hdf5.data
            pilot_samples = hdf5.pilot_samples
            uplink_samples = hdf5.uplink_samples
            noise_samples = hdf5.noise_samples
            downlink_samples = hdf5.downlink_samples

            # Check which data we have available
            pilots_avail = len(pilot_samples) > 0
            ul_data_avail = len(uplink_samples) > 0
            noise_avail = len(noise_samples) > 0
            dl_data_avail = len(downlink_samples) > 0

            if pilots_avail:
                print("Found pilot data with shape:")
                print(pilot_samples.shape)
                if ul_data_avail:
                    print("Found uplink data with shape:")
                    print(uplink_samples.shape)
                if noise_avail:
                    print("Found Noise Samples!")
                if dl_data_avail:
                    print("Found downlink data with shape:")
                    print(downlink_samples.shape)
            else:
                if dl_data_avail:
                    print("Found Downlink Data")
                else:
                    if not ul_data_avail:
                        raise Exception(' **** No pilots or uplink data found **** ')
            if not ul_data_avail and demodulate:
                demodulate = False
                print("Uplink data is not available, ignoring demodulate option...")

            if verify:
                verify_hdf5(hdf5, ref_frame, ref_cell, ref_ofdm_sym, ref_ant,
                            ref_user, ref_ul_slot, ref_dl_slot, ref_subcarrier,
                            signal_offset, downlink_calib_offset,
                            uplink_calib_offset, thresh, deep_inspect,
                            corr_thresh, exclude_bs_nodes, demodulate, analyze)
            #if analyze:
            #    analyze_hdf5(hdf5, ref_frame, ref_cell, ref_subcarrier, signal_offset)
    scrpt_end = time.time()
    print(">>>> Script Duration: time: %f \n" % ( scrpt_end - scrpt_strt) )


if __name__ == '__main__':
    main()

