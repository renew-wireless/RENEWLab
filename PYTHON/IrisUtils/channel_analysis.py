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

