"""
 channel_analysis.py

 CSI analysis API file

 Author(s): Clay Shepard: cws@rice.edu
            Rahman Doost-Mohamamdy: doost@rice.edu
            Oscar Bejarano: obejarano@rice.edu

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
import struct
import numpy as np
import os
import math
import time
import datetime
from scipy import signal
import matplotlib.pyplot as plt
from generate_sequence import *

#                       all data, n_UE,      pilots/frame
#csi, samps = samps2csi(samples, num_cl_tmp, symbol_length, fft_size=64, offset=offset, bound=0, cp=0)


def samps2csi(samps, num_users, samps_per_user=224, fft_size=64, offset=0, bound=94, cp=0):
    """Convert an Argos HDF5 log file with raw IQ in to CSI.
    Asumes 802.11 style LTS used for trace collection.

    Args:
        samps: The h5py or numpy array containing the raw IQ samples,
            dims = [Frame, Cell, User, Antenna, Sample].
        num_users: Number of users used in trace collection. (Last 'user' is noise.)
        samps_per_user: Number of samples allocated to each user in each frame.
 
    Returns:
        csi: Complex numpy array with [Frame, Cell, User, Pilot Rep, Antenna, Subcarrier]
        iq: Complex numpy array of raw IQ samples [Frame, Cell, User, Pilot Rep, Antenna, samples]
 
    Example:
        h5log = h5py.File(filename,'r')
        csi,iq = samps2csi(h5log['Pilot_Samples'], h5log.attrs['num_mob_ant']+1, h5log.attrs['samples_per_user'])
    """
    debug = False
    chunkstart = time.time()
    usersamps = np.reshape(
        samps, (samps.shape[0], samps.shape[1], num_users, samps.shape[3], samps_per_user, 2))
    # What is this? It is eiter 1 or 2: 2 LTSs??
    pilot_rep = min([(samps_per_user-bound)//(fft_size+cp), 2])
    iq = np.empty((samps.shape[0], samps.shape[1], num_users,
                   samps.shape[3], pilot_rep, fft_size), dtype='complex64')
    if debug:
        print("chunkstart = {}, usersamps.shape = {}, samps.shape = {}, samps_per_user = {}, nbat= {}, iq.shape = {}".format(
            chunkstart, usersamps.shape, samps.shape, samps_per_user, nbat, iq.shape))
    for i in range(pilot_rep):  # 2 first symbols (assumed LTS) seperate estimates
        iq[:, :, :, :, i, :] = (usersamps[:, :, :, :, offset + cp + i*fft_size:offset+cp+(i+1)*fft_size, 0] +
                                usersamps[:, :, :, :, offset + cp + i*fft_size:offset+cp+(i+1)*fft_size, 1]*1j)*2**-15

    iq = iq.swapaxes(3, 4)
    if debug:
        print("iq.shape after axes swapping: {}".format(iq.shape))

    fftstart = time.time()
    csi = np.empty(iq.shape, dtype='complex64')
    if fft_size == 64:
        # Retrieve frequency-domain LTS sequence
        _, lts_freq = generate_training_seq(
            preamble_type='lts', seq_length=[], cp=32, upsample=1, reps=[])
        pre_csi = np.fft.fftshift(np.fft.fft(iq, fft_size, 5), 5)
        csi = np.fft.fftshift(np.fft.fft(iq, fft_size, 5), 5) * lts_freq
        if debug:
            print("csi.shape:{} lts_freq.shape: {}, pre_csi.shape = {}".format(
                csi.shape, lts_freq.shape, pre_csi.shape))
        endtime = time.time()
        if debug:
            print("chunk time: %f fft time: %f" %
                  (fftstart - chunkstart, endtime - fftstart))
        # remove zero subcarriers
        csi = np.delete(csi, [0, 1, 2, 3, 4, 5, 32, 59, 60, 61, 62, 63], 5)
    return csi, iq


def samps2csi_large(samps, num_users, samps_per_user=224, offset=47, chunk_size=1000):
    """Wrapper function for samps2csi_main for to speed up large logs by leveraging data-locality. Chunk_size may need to be adjusted based on your computer."""

    if samps.shape[0] > chunk_size:
                # rather than memmap let's just increase swap... should be just as fast.
                #csi = np.memmap(os.path.join(_here,'temp1.mymemmap'), dtype='complex64', mode='w+', shape=(samps.shape[0], num_users, 2, samps.shape[1],52))
                #iq = np.memmap(os.path.join(_here,'temp2.mymemmap'), dtype='complex64', mode='w+', shape=(samps.shape[0], num_users, 2, samps.shape[1],64))
        csi = np.empty(
            (samps.shape[0], num_users, 2, samps.shape[1], 52), dtype='complex64')
        iq = np.empty(
            (samps.shape[0], num_users, 2, samps.shape[1], 64), dtype='complex64')
        chunk_num = samps.shape[0]//chunk_size
        for i in range(chunk_num):
            csi[i*chunk_size:i*chunk_size+chunk_size], iq[i*chunk_size:i*chunk_size+chunk_size] = samps2csi(
                samps[i*chunk_size:(i*chunk_size+chunk_size), :, :, :], num_users, samps_per_user=samps_per_user)
        csi[chunk_num*chunk_size:], iq[chunk_num*chunk_size:] = samps2csi(
            samps[chunk_num*chunk_size:, :, :, :], num_users, samps_per_user=samps_per_user)
    else:
        csi, iq = samps2csi(
            samps, num_users, samps_per_user=samps_per_user, offset=offset)
    return csi, iq


def calCond(userCSI):
    """Calculate the standard matrix condition number.

    Args:
            userCSI: Complex numpy array with [Frame, User, BS Ant, Subcarrier]

    Returns:
            condNumber_ave: The average condition number across all users and subcarriers.
            condNumber: Numpy array of condition number [Frame, Subcarrier]. 
    """
    condNumber = np.empty(
        (userCSI.shape[0], userCSI.shape[3]), dtype='float32')
    for sc in range(userCSI.shape[3]):
        condNumber[:, sc] = np.linalg.cond(
            userCSI[:, :, :, sc])
    condNumber_ave = np.average(condNumber)
    return condNumber_ave, condNumber


def calDemmel(userCSI):
    """Calculate the Demmel condition number.

    Args:
            userCSI: Complex numpy array with [Frame, User, BS Ant, Subcarrier]

    Returns:
            demmelNumber_ave: The average condition number across all users and subcarriers.
            demmelNumber: Numpy array of condition number [Frame, Subcarrier].
    """
    demmelNumber = np.empty(
        (userCSI.shape[0], userCSI.shape[3]), dtype='float32')
    for sc in range(userCSI.shape[3]):

        # covariance matrix
        cov = np.matmul(userCSI[:, :, :, sc], np.transpose(
            userCSI[:, :, :, sc], [0, 2, 1]).conj())
        eigenvalues = np.abs(np.linalg.eigvals(cov))
        demmelNumber[:, sc] = np.sum(
            eigenvalues, axis=1)/np.min(eigenvalues, axis=1)
    demmelNumber_ave = np.average(demmelNumber)
    return demmelNumber_ave, demmelNumber


def calCapacity(userCSI, noise, beamweights, downlink=False):
    """Calculate the capacity of a trace with static beamweights.

    Apply a set of beamweights to a set of wideband user channels and calculate the shannon capacity of the resulting channel for every Frame.

    Note that if the beamweights are calculated with a frame from the trace, that frame will have unrealistic capacity since it will correlate noise as signal.

    Args:
            userCSI: Complex numpy array with [Frame, User, BS Ant, Subcarrier]
            noise: Complex numpy array with [Frame, BS Ant, Subcarrier]
            beamweights: Set of beamweights to apply to userCSI [BS Ant, User, Subcarrier]
            downlink: (Boolean) Compute downlink capacity if True, else Uplink

    Returns:
            cap_total: Total capacity across all users averaged over subarriers in bps/hz [Frame]
            cap_u: Capacity per user across averaged over subcarriers in bps/hz [Frame, User]
            cap_sc: Capacity per user and subcarrier in bps/hz [Frame, User, Subcarrier]
            SINR: Signtal to interference and noise ratio for each frame user and subcarrier [Frame, User, Subcarrier]
            cap_su_sc: Single user (no interference) capacity per subcarrier in bps/hz  [Frame, User, Subcarrier]
            cap_su_u: Single user (no interference) capacity averaged over subcarriers in bps/hz [Frame, User]
            SNR: Signtal to noise ratio for each frame user and subcarrier [Frame, User, Subcarrier]
    """
    noise_bs_sc = np.mean(np.mean(np.abs(noise), 0),
                          0)  # average over time and the two ltss
    sig_intf = np.empty(
        (userCSI.shape[0], userCSI.shape[1], userCSI.shape[1], userCSI.shape[3]), dtype='float32')
    noise_sc_u = np.empty(
        (userCSI.shape[1], userCSI.shape[3]), dtype='float32')
    for sc in range(userCSI.shape[3]):
        # TODO: can we get rid of the for loop?
        sig_intf[:, :, :, sc] = np.square(
            np.abs(np.dot(userCSI[:, :, :, sc], beamweights[:, :, sc])))
        # noise is uncorrelated, and all we have is average power here (Evan wants to do it per frame, but I think that's a bad idea)
        noise_sc_u[:, sc] = np.dot(
            np.square(noise_bs_sc[:, sc]), np.square(np.abs(beamweights[:, :, sc])))

    # noise_sc_u *= 4 #fudge factor since our noise doesn't include a lot of noise sources

    sig_sc = np.diagonal(sig_intf, axis1=1, axis2=2)
    sig_sc = np.swapaxes(sig_sc, 1, 2)
    # remove noise from signal power (only matters in low snr really...)
    sig_sc = sig_sc - noise_sc_u
    sig_sc[sig_sc < 0] = 0  # can't have negative power (prevent errors)
    intf_sc = np.sum(sig_intf, axis=1+int(downlink)) - sig_sc
    SINR = sig_sc/(noise_sc_u+intf_sc)

    cap_sc = np.log2(1+SINR)
    cap_u = np.mean(cap_sc, axis=2)
    cap_total = np.sum(cap_u, axis=1)

    SNR = sig_sc/noise_sc_u
    cap_su_sc = np.log2(1+SNR)
    cap_su_u = np.mean(cap_su_sc, axis=2)

    return cap_total, cap_u, cap_sc, SINR, cap_su_sc, cap_su_u, SNR


def calContCapacity(csi, conj=True, downlink=False, offset=1):
    """Calculate the capacity of a trace with continuous beamforming.

    For every frame in a trace, calculate beamweights (either conjugate or ZF),
    apply them to a set of wideband user channels either from the same frame or some constant offset (delay),
    then calculate the shannon capacity of the resulting channel.

    The main difference in uplink/downlink is the source of interference (and power allocation).
    In uplink the intended user's interference is a result of every other user's signal passed through that user's beamweights.
    In downlink the inteded user's interference is a result of every other user's signal passed through their beamweights (applied to the intended user's channel).

    Note that every user has a full 802.11 LTS, which is a repitition of the same symbol.
    This method uses the first half of the LTS to make beamweights, then applies them to the second half.
    Otherwise, noise is correlated, resulting in inaccurate results.

    Args:
            csi: Full complex numpy array with separate LTSs and noise [Frame, User, BS Ant, Subcarrier] (noise is last user)
            conj: (Boolean) If True use conjugate beamforming, else use zeroforcing beamforming.
            downlink: (Boolean) Compute downlink capacity if True, else Uplink
            offset: Number of frames to delay beamweight application.

    Returns:
            cap_total: Total capacity across all users averaged over subarriers in bps/hz [Frame]
            cap_u: Capacity per user across averaged over subcarriers in bps/hz [Frame, User]
            cap_sc: Capacity per user and subcarrier in bps/hz [Frame, User, Subcarrier]
            SINR: Signtal to interference and noise ratio for each frame user and subcarrier [Frame, User, Subcarrier]
            cap_su_sc: Single user (no interference) capacity per subcarrier in bps/hz  [Frame, User, Subcarrier]
            cap_su_u: Single user (no interference) capacity averaged over subcarriers in bps/hz [Frame, User]
            SNR: Signtal to noise ratio for each frame user and subcarrier [Frame, User, Subcarrier]
    """
    csi_sw = np.transpose(
        csi, (0, 4, 1, 3, 2))  # hack to avoid for loop (matmul requires last two axes to be matrix) #frame, sc, user, bsant, lts
    # noise is last set of data. #frame, sc, bsant, lts
    noise = csi_sw[:, :, -1, :, :]
    # don't include noise, use first LTS for CSI #frame, sc, user, bsant, lts
    userCSI_sw = csi_sw[:, :, :-1, :, 0]

    # average over time and the two ltss
    noise_sc_bs = np.mean(np.mean(np.abs(noise), 3), 0)

    if conj:
        '''Calculate weights as conjugate.'''
        beamweights = np.transpose(
            np.conj(csi_sw[:, :, :-1, :, 1]), (0, 1, 3, 2))
    else:
        '''Calculate weights using zeroforcing.'''
        beamweights = np.empty(
            (userCSI_sw.shape[0], userCSI_sw.shape[1], userCSI_sw.shape[3], userCSI_sw.shape[2]), dtype='complex64')
        for frame in range(userCSI_sw.shape[0]):
            for sc in range(userCSI_sw.shape[1]):
                # * np.linalg.norm(csi[frame,:4,0,:,sc]) #either this, or the noise power has to be scaled back accordingly
                beamweights[frame, sc, :, :] = np.linalg.pinv(
                    csi_sw[frame, sc, :-1, :, 1])
    if offset > 0:
        # delay offset samples
        beamweights = np.roll(beamweights, offset, axis=0)

    sig_intf = np.square(
        np.abs(np.matmul(userCSI_sw[offset:, :, :, :], beamweights[offset:, :, :, :])))

    noise_sc_u = np.transpose(np.sum(np.square(
        noise_sc_bs)*np.square(np.abs(np.transpose(beamweights, (0, 3, 1, 2)))), 3), (0, 2, 1))
    noise_sc_u = noise_sc_u[offset:]
    # noise_sc_u *= 4 #fudge factor since our noise doesn't include a lot of noise sources.  this should probably be justified/measured or removed

    sig_sc = np.diagonal(sig_intf, axis1=2, axis2=3)
    # remove noise from signal power (only matters in low snr really...)
    sig_sc = sig_sc - noise_sc_u
    sig_sc[sig_sc < 0] = 0  # can't have negative power (prevent errors)
    # lazy hack -- just sum then subtract the intended signal.
    intf_sc = np.sum(sig_intf, axis=2+int(downlink)) - sig_sc
    SINR = sig_sc/(noise_sc_u+intf_sc)

    cap_sc = np.log2(1+SINR)
    cap_u = np.mean(cap_sc, axis=1)
    cap_total = np.sum(cap_u, axis=1)

    SNR = sig_sc/noise_sc_u
    cap_su_sc = np.log2(1+SNR)
    cap_su_u = np.mean(cap_su_sc, axis=1)

    return cap_total, cap_u, cap_sc, SINR, cap_su_sc, cap_su_u, SNR


def calExpectedCapacity(csi, user=0, max_delay=100, conj=True, downlink=False):
    """Calculate the expected capacity for beamweights calculated with delayed stale CSI.


    Args:
            csi: Full complex numpy array with separate LTSs and noise [Frame, User, BS Ant, Subcarrier] (noise is last user)
            user: Index of user to compute for (note that other users still affect capacity due to their interference)
            max_delay: Maximum delay (in frames) to delay the beamweight computation.
            conj: (Boolean) If True use conjugate beamforming, else use zeroforcing beamforming.
            downlink: (Boolean) Compute downlink capacity if True, else Uplink

    Returns:
            cap: Average capacity across all frames for a given delay (in frames) in bps/hz [Delay]
    """
    cap = []
    for d in range(max_delay):
        # print([d,time.time()])
        delayed = calContCapacity(
            csi, conj=conj, downlink=downlink, offset=d)
        cap.append(np.mean(delayed[1][:, user]))

    return cap


def calCorr(userCSI, corr_vec):
    """
    Calculate the instantaneous correlation with a given correlation vector
    """
    sig_intf = np.empty(
        (userCSI.shape[0], userCSI.shape[1], userCSI.shape[1], userCSI.shape[3]), dtype='float32')

    for sc in range(userCSI.shape[3]):
        sig_intf[:, :, :, sc] = np.abs(np.dot(userCSI[:, :, :, sc], corr_vec[:, :, sc])) / np.dot(
            np.abs(userCSI[:, :, :, sc]), np.abs(corr_vec[:, :, sc]))

    # gets correlation of subcarriers for each user across bs antennas
    sig_sc = np.diagonal(sig_intf, axis1=1, axis2=2)
    sig_sc = np.swapaxes(sig_sc, 1, 2)
    corr_total = np.mean(sig_sc, axis=2)  # averaging corr across users

    return corr_total, sig_sc


def demult(csi, data, method='zf'):
    # TODO include cell dimension for both csi and data and symbol num for data
    """csi: Frame, User, Pilot Rep, Antenna, Subcarrier"""
    """data: Frame, Antenna, Subcarrier"""
    # Compute beamweights based on the specified frame.
    userCSI = np.mean(csi, 2)  # average over both LTSs
    sig_intf = np.empty(
        (userCSI.shape[0], userCSI.shape[1], userCSI.shape[3]), dtype='complex64')
    for frame in range(csi.shape[0]):
        for sc in range(userCSI.shape[3]):
            if method == 'zf':
                sig_intf[frame, :, sc] = np.dot(
                    data[frame, :, sc], np.linalg.pinv(userCSI[frame, :, :, sc]))
            else:
                sig_intf[frame, :, sc] = np.dot(data[frame, :, sc], np.transpose(
                    np.conj(userCSI[frame, :, :, sc]), (1, 0)))
    return sig_intf


def csi_from_pilots(pilots_dump, z_padding=150, fft_size=64, cp=16, frm_st_idx=0, frame_to_plot=0, ref_ant=0):
    """ 
    Finds the end of the pilots' frames, finds all the lts indices relative to that.
    Divides the data with lts sequences, calculates csi per lts, csi per frame, csi total.  
    """
    print("********************* csi_from_pilots(): *********************")

    # Reviewing options and vars:
    show_plot = True
    debug = False
    test_mf = False
    write_to_file = True
    legacy = False

    # dimensions of pilots_dump
    n_frame = pilots_dump.shape[0]      # no. of captured frames
    n_cell = pilots_dump.shape[1]       # no. of cells
    n_ue = pilots_dump.shape[2]         # no. of UEs
    n_ant = pilots_dump.shape[3]        # no. of BS antennas
    n_iq = pilots_dump.shape[4]         # no. of IQ samples per frame

    if debug:
        print("input : z_padding = {}, fft_size={}, cp={}, frm_st_idx = {}, frame_to_plot = {}, ref_ant={}".format(
            z_padding, fft_size, cp, frm_st_idx, frame_to_plot, ref_ant))
        print("n_frame = {}, n_cell = {}, n_ue = {}, n_ant = {}, n_iq = {}".format(
            n_frame, n_cell, n_ue, n_ant, n_iq))

    if ((n_iq % 2) != 0):
        print("Size of iq samples:".format(n_iq))
        raise Exception(
            ' **** The length of iq samples per frames HAS to be an even number! **** ')

    n_cmpx = n_iq // 2  # no. of complex samples
    # no. of complex samples in a P subframe without pre- and post- fixes
    n_csamp = n_cmpx - z_padding
    if legacy:
        # even indices: real part of iq      --> ATTENTION: I and Q are flipped at RX for some weird reason! So, even starts from 1!
        idx_e = np.arange(1, n_iq, 2)
        # odd  indices: imaginary part of iq --> ATTENTION: I and Q are flipped at RX for some weird reason! So, odd starts from 0!
        idx_o = np.arange(0, n_iq, 2)
    else:
        idx_e = np.arange(0, n_iq, 2)       # even indices: real part of iq
        # odd  indices: imaginary part of iq
        idx_o = np.arange(1, n_iq, 2)

    # make a new data structure where the iq samples become complex numbers
    cmpx_pilots = (pilots_dump[:, :, :, :, idx_e] +
                   1j*pilots_dump[:, :, :, :, idx_o])*2**-15

    # take a time-domain lts sequence, concatenate more copies, flip, conjugate
    lts_t, lts_f = generate_training_seq(preamble_type='lts', seq_length=[
    ], cp=32, upsample=1, reps=[])    # TD LTS sequences (x2.5), FD LTS sequences
    # last 80 samps (assume 16 cp)
    lts_tmp = lts_t[-80:]
    n_lts = len(lts_tmp)
    # no. of LTS sequences in a pilot SF
    k_lts = n_csamp // n_lts
    # concatenate k LTS's to filter/correlate below
    lts_seq = np.tile(lts_tmp, k_lts)
    lts_seq = lts_seq[::-1]                         # flip
    # conjugate the local LTS sequence
    lts_seq_conj = np.conjugate(lts_seq)
    # length of the local LTS seq.
    l_lts_fc = len(lts_seq_conj)

    if debug:
        print("cmpx_pilots.shape = {}, lts_t.shape = {}".format(
            cmpx_pilots.shape, lts_t.shape))
        #print("idx_e= {}, idx_o= {}".format(idx_e, idx_o))
        print("n_cmpx = {}, n_csamp = {}, n_lts = {}, k_lts = {}, lts_seq_conj.shape = {}".format(
            n_cmpx, n_csamp, n_lts, k_lts, lts_seq_conj.shape))

    # debug/ testing
    if debug:
        z_pre = np.zeros(82, dtype='complex64')
        z_post = np.zeros(68, dtype='complex64')
        lts_t_rep = np.tile(lts_tmp, k_lts)
        lts_t_rep_tst = np.append(z_pre, lts_t_rep)
        lts_t_rep_tst = np.append(lts_t_rep_tst, z_post)

        if test_mf:
            w = np.random.normal(0, 0.1/2, len(lts_t_rep_tst)) + \
                1j*np.random.normal(0, 0.1/2, len(lts_t_rep_tst))
            lts_t_rep_tst = lts_t_rep_tst + w
            cmpx_pilots = np.tile(
                lts_t_rep_tst, (n_frame, cmpx_pilots.shape[1], cmpx_pilots.shape[2], cmpx_pilots.shape[3], 1))
            print("if test_mf: Shape of lts_t_rep_tst: {} , cmpx_pilots.shape = {}".format(
                lts_t_rep_tst.shape, cmpx_pilots.shape))

    # normalized matched filter
    a = 1
    unos = np.ones(l_lts_fc)
    v0 = signal.lfilter(lts_seq_conj, a, cmpx_pilots, axis=4)
    v1 = signal.lfilter(unos, a, (abs(cmpx_pilots)**2), axis=4)
    m_filt = (np.abs(v0)**2)/v1

    # clean up nan samples: replace nan with -1
    nan_indices = np.argwhere(np.isnan(m_filt))
    m_filt[np.isnan(m_filt)] = -0.5  # the only negative value in m_filt

    if write_to_file:
        # write the nan_indices into a file
        np.savetxt("nan_indices.txt", nan_indices, fmt='%i')

    if debug:
        print("Shape of truncated complex pilots: {} , l_lts_fc = {}, v0.shape = {}, v1.shape = {}, m_filt.shape = {}".
              format(cmpx_pilots.shape, l_lts_fc, v0.shape, v1.shape, m_filt.shape))

    rho_max = np.amax(m_filt, axis=4)         # maximum peak per SF per antenna
    rho_min = np.amin(m_filt, axis=4)        # minimum peak per SF per antenna
    ipos = np.argmax(m_filt, axis=4)          # positons of the max peaks
    sf_start = ipos - l_lts_fc + 1             # start of every received SF
    # get rid of negative indices in case of an incorrect peak
    sf_start = np.where(sf_start < 0, 0, sf_start)

    # get the pilot samples from the cmpx_pilots array and reshape for k_lts LTS pilots:
    pilots_rx_t = np.empty(
        [n_frame, n_cell, n_ue, n_ant, k_lts * n_lts], dtype='complex64')

    indexing_start = time.time()
    for i in range(n_frame):
        for j in range(n_cell):
            for k in range(n_ue):
                for l in range(n_ant):
                    pilots_rx_t[i, j, k, l, :] = cmpx_pilots[i, j, k, l,
                                                             sf_start[i, j, k, l]:  sf_start[i, j, k, l] + (k_lts * n_lts)]
    indexing_end = time.time()

    # *************** This fancy indexing is slower than the for loop! **************
#    aaa= np.reshape(cmpx_pilots, (n_frame*n_cell* n_ue * n_ant, n_cmpx))
#    idxx = np.expand_dims(sf_start.flatten(), axis=1)
#    idxx = np.tile(idxx, (1,k_lts*n_lts))
#    idxx = idxx + np.arange(k_lts*n_lts)
#    indexing_start2 = time.time()
#    m,n = aaa.shape
#    #bb = aaa[np.arange(aaa.shape[0])[:,None],idxx]
#    bb = np.take(aaa,idxx + n*np.arange(m)[:,None])
#    indexing_end2 = time.time()
#    cc = np.reshape(bb,(n_frame,n_cell, n_ue , n_ant, k_lts * n_lts) )
#    if debug:
#       print("Shape of: aaa  = {}, bb: {}, cc: {}, flattened sf_start: {}\n".format(aaa.shape, bb.shape, cc.shape, sf_start.flatten().shape))
#       print("Indexing time 2: %f \n" % ( indexing_end2 -indexing_start2) )

    if debug:
        print("Shape of: pilots_rx_t before truncation: {}\n".format(
            pilots_rx_t.shape))

    pilots_rx_t = np.reshape(
        pilots_rx_t, (n_frame, n_cell, n_ue, n_ant, k_lts, n_lts))
    pilots_rx_t = np.delete(pilots_rx_t, range(fft_size, n_lts), 5)

    if debug:
        print("Indexing time: %f \n" % (indexing_end - indexing_start))
        print("Shape of: pilots_rx_t = {}\n".format(pilots_rx_t.shape))
        print("Shape of: rho_max = {}, rho_min = {}, ipos = {}, sf_start = {}".format(
            rho_max.shape, rho_min.shape, ipos.shape, sf_start.shape))

    # take fft and get the raw CSI matrix (no averaging)
    # align SCs based on how they were Tx-ec
    lts_f_shft = np.fft.fftshift(lts_f)
    pilots_rx_f = np.fft.fft(pilots_rx_t, fft_size, 5)      # take FFT
    # find the zero SCs corresponding to lts_f_shft
    zero_sc = np.where(lts_f_shft == 0)[0]
    # remove zero subcarriers
    lts_f_nzsc = np.delete(lts_f_shft, zero_sc)
    # remove zero subcarriers
    pilots_rx_f = np.delete(pilots_rx_f, zero_sc, 5)
    # take channel estimate by dividing with the non-zero elements of lts_f_shft
    csi = pilots_rx_f / lts_f_nzsc
    # unecessary step: just to make it in accordance to lts_f as returned by generate_training_seq()
    csi = np.fft.fftshift(csi, 5)

    if debug:
        print(">>>> number of NaN indices = {} NaN indices =\n{}".format(
            nan_indices.shape, nan_indices))
        print("Shape of: csi = {}\n".format(csi.shape))

    # plot something to see if it worked!
    if show_plot:
        fig = plt.figure()
        ax1 = fig.add_subplot(3, 1, 1)
        ax1.grid(True)
        ax1.set_title(
            'channel_analysis:csi_from_pilots(): Re of Rx pilot - ref frame {} and ref ant. {} (UE 0)'.format(frame_to_plot, ref_ant))
        if debug:
            print("cmpx_pilots.shape = {}".format(cmpx_pilots.shape))

        ax1.plot(
            np.real(cmpx_pilots[frame_to_plot - frm_st_idx, 0, 0, ref_ant, :]))

        if debug:
            loc_sec = lts_t_rep_tst
        else:
            z_pre = np.zeros(82, dtype='complex64')
            z_post = np.zeros(68, dtype='complex64')
            lts_t_rep = np.tile(lts_tmp, k_lts)
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
            'channel_analysis:csi_from_pilots(): MF (uncleared peaks) - ref frame {} and ref ant. {} (UE 0)'.format(frame_to_plot, ref_ant))
        ax3.stem(m_filt[frame_to_plot - frm_st_idx, 0, 0, ref_ant, :])
        ax3.set_xlabel('Samples')
        # plt.show()

    print("********************* ******************** *********************\n")
    return csi, m_filt, sf_start, cmpx_pilots, k_lts, n_lts



# ********************* Example Code *********************


if __name__ == '__main__':
    starttime = time.time()
    show_plots = True
    # samples to zoom in around frame (to look at local behavior), 0 to disable
    zoom = 0
    pl = 0
    static = h5py.File('logs/ArgosCSI-76x2-2017-02-07-18-25-47.hdf5', 'r')
    env = h5py.File('logs/ArgosCSI-76x2-2017-02-07-18-25-47.hdf5', 'r')
    mobile = h5py.File('logs/ArgosCSI-76x2-2017-02-07-18-25-47.hdf5', 'r')

    frame = 10  # frame to compute beamweights from
    conjdata = []
    zfdata = []

    for h5log in [static, env, mobile]:
        # read parameters for this measurement data
        samps_per_user = h5log.attrs['samples_per_user']
        num_users = h5log.attrs['num_mob_ant']
        timestep = h5log.attrs['frame_length']/20e6
        noise_meas_en = h5log.attrs.get('measured_noise', 1)

        # compute CSI for each user and get a nice numpy array
        # Returns csi with Frame, User, LTS (there are 2), BS ant, Subcarrier  #also, iq samples nicely chunked out, same dims, but subcarrier is sample.
        csi, iq = samps2csi(
            h5log['Pilot_Samples'], num_users+noise_meas_en, samps_per_user)
        # zoom in too look at behavior around peak (and reduce processing time)
        if zoom > 0:
            csi = csi[frame-zoom:frame+zoom, :, :, :, :]
            # recenter the plots (otherwise it errors)
            frame = zoom
        noise = csi[:, -1, :, :, :]  # noise is last set of data.
        # don't include noise, average over both LTSs
        userCSI = np.mean(csi[:, :num_users, :, :, :], 2)

        # example lts find:
        user = 0
        # so, this is pretty ugly, but we want all the samples (not just those chunked from samps2csi), so we not only convert ints to the complex floats, but also have to figure out where to chunk the user from.
        lts_iq = h5log['Pilot_Samples'][frame, 0, user*samps_per_user:(
            user+1)*samps_per_user, 0]*1.+h5log['Pilot_Samples'][frame, 0, user*samps_per_user:(user+1)*samps_per_user, 1]*1j
        lts_iq /= 2**15
        # Andrew wrote this, but I don't really like the way he did the convolve method...  works well enough for high SNRs.
        offset = lts.findLTS(lts_iq)+32
        print("LTS offset for user %d, frame %d: %d" %
              (user, frame, offset))

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
        # these return total capacity, per-user capacity, per-user/per-subcarrier capacity, SINR, single-user capacity(no inter-user interference), and SNR
        # conjcap_total,conjcap_u,conjcap_sc,conjSINR,conjcap_su_sc,conjcap_su_u,conjSNR
        conj = calCapacity(userCSI, noise, conjbws, downlink=downlink)
        # zfcap_total,zfcap_u,zfcap_sc,zfSINR,zfcap_su_sc,zfcap_su_u,zfSNR
        zf = calCapacity(userCSI, noise, zfbws, downlink=downlink)

        # plot stuff
        if show_plots:
            # Multiuser Conjugate
            plt.figure(1000*pl, figsize=(50, 10))
            plt.plot(
                np.arange(0, csi.shape[0]*timestep, timestep)[:csi.shape[0]], conj[1])
            # plt.ylim([0,2])
            plt.xlabel('Time (s)')
            plt.ylabel('Per User Capacity Conj (bps/Hz)')
            plt.show()

            # Multiuser Zeroforcing
            plt.figure(1000*pl+1, figsize=(50, 10))
            plt.plot(
                np.arange(0, csi.shape[0]*timestep, timestep)[:csi.shape[0]], zf[1])
            # plt.ylim([0,2])
            plt.xlabel('Time (s)')
            plt.ylabel('Per User Capacity ZF (bps/Hz)')
            plt.show()

            # Single user (but show all users)
            plt.figure(1000*pl+2, figsize=(50, 10))
            plt.plot(
                np.arange(0, csi.shape[0]*timestep, timestep)[:csi.shape[0]], conj[-2])
            # plt.ylim([0,2])
            plt.xlabel('Time (s)')
            plt.ylabel('SUBF Capacity Conj (bps/Hz)')
            plt.show()
            pl += 1

        # save for exporting to matlab (prettier plots)
        conjdata.append(conj)
        zfdata.append(zf)

        del csi, iq  # free the memory

    endtime = time.time()
    print("Total time: %f" % (endtime-starttime))
    '''
        import scipy.io
        data = dict(timestep=timestep)
        data.update(dict(static_zf_cap_total=zfdata[0][0], static_zf_cap_u=zfdata[0][1],static_conj_cap_total=conjdata[0][0], static_conj_cap_u=conjdata[0][1], static_conj_cap_su_u=conjdata[0][-2]))
        data.update(dict(env_zf_cap_total=zfdata[1][0], env_zf_cap_u=zfdata[1][1], env_conj_cap_total=conjdata[1][0], env_conj_cap_u=conjdata[1][1], env_conj_cap_su_u=conjdata[1][-2]))
        data.update(dict(mobile_zf_cap_total=zfdata[2][0], mobile_zf_cap_u=zfdata[2][1],mobile_conj_cap_total=conjdata[2][0], mobile_conj_cap_u=conjdata[2][1], mobile_conj_cap_su_u=conjdata[2][-2]))
        #data = dict(timestep=timestep, static_zf_cap_total=zfdata[0][0], static_zf_cap_u=zfdata[0][1],static_conj_cap_total=conjdata[0][0], static_conj_cap_u=conjdata[0][1], env_zf_cap_total=zfdata[1][0], env_zf_cap_u=zfdata[1][1],env_conj_cap_total=conjdata[1][0], env_conj_cap_u=conjdata[1][1], mobile_zf_cap_total=zfdata[2][0], mobile_zf_cap_u=zfdata[2][1],mobile_conj_cap_total=conjdata[2][0], mobile_conj_cap_u=conjdata[2][1], static_conj_cap_su_u=conjdata[0][-2], env_conj_cap_su_u=conjdata[1][-2], mobile_conj_cap_su_u=conjdata[2][-2])
        scipy.io.savemat('logs/capacity-frame_%d.mat' % frame, data)
        '''

'''
%example matlab script for loading the saved file
load capacity-frame_500
%timestep = 0.035
plot(0:timestep:timestep*(length(env_conj_cap_u)-1),env_conj_cap_u)
plot(0:timestep:timestep*(length(mobile_conj_cap_u)-1),mobile_conj_cap_u)
xlim([0,600])
ylim([0,5])
ylabel('Per User Capacity Conj (bps/Hz)')
xlabel('Time (s)')

figure(4)
plot(0:timestep:timestep*(length(mobile_conj_cap_u)-1),mobile_conj_cap_u(:,2))
xlim([0,120])
ylim([0,5])
xlabel('Time (s)')
ylabel('User Capacity Conjugate (bps/Hz)')
print -clipboard -dmeta %windows only
'''

#import os
#import glob
'''
        #Example for simply converting raw IQ to CSI.
        import glob
        logdir = "logs/uhf_wb_traces_vito/"
        filenames = glob.glob(logdir+"*.hdf5")
        #filenames = ('ChannelTracesVitosLand/ArgosCSI-8x5-2015-12-19-00-00-29_good_uhf_mobile_2directionalpolarized_1staticmobile_2mobile',
        #                       'ChannelTracesVitosLand/ArgosCSI-8x4-2015-12-18-22-34-02_good_static_uhf_vito_alldirectional',
        #                       'ChannelTracesVitosLand/ArgosCSI-8x4-2015-12-18-22-53-16_good_uhf_envmobility_vito.hdf5',)

        for filename in filenames:
                print(filename)
                log2csi_hdf5(filename)
'''

