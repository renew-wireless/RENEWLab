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
from generate_sequence import *


def samps2csi(samps, num_users,samps_per_user=224, fft_size=64, offset=0, bound=94, cp=0):
    """Input samps dims: Frame, Cell, Antenna, User, Sample"""
    """Returns iq with Frame, Cell, User, Pilot Rep, Antenna, Sample"""
    """Returns csi with Frame, Cell, User, Pilot Rep, Antenna, Subcarrier"""
    chunkstart = time.time()
    usersamps = np.reshape(samps, (samps.shape[0], samps.shape[1], num_users, samps.shape[3], samps_per_user, 2))
    nbat = min([(samps_per_user-bound)//(fft_size+cp),2])
    iq = np.empty((samps.shape[0],samps.shape[1],num_users,samps.shape[3],nbat,fft_size),dtype='complex64')
    for i in range(nbat):  # 2 seperate estimates
        iq[:, :, :, :, i, :] = (usersamps[:, :, :, :, offset + cp + i*fft_size:offset+cp+(i+1)*fft_size, 0] +
                                usersamps[:, :, :, :, offset+cp+i*fft_size:offset+cp+(i+1)*fft_size, 1]*1j)*2**-15
    iq = iq.swapaxes(3, 4)
    fftstart = time.time()
    csi = np.empty(iq.shape, dtype='complex64')
    if fft_size == 64:
        # Retrieve frequency-domain LTS sequence
        _, lts_freq = generate_training_seq(preamble_type='lts', seq_length=[], cp=32, upsample=1, reps=[])
        csi = np.fft.fftshift(np.fft.fft(iq, fft_size, 5), 5) * lts_freq
        endtime = time.time()
        print("chunk time: %f fft time: %f" % (fftstart - chunkstart, endtime -fftstart) )
        csi = np.delete(csi, [0, 1, 2, 3, 4, 5, 32, 59, 60, 61, 62, 63], 5)  # remove zero subcarriers
    return csi, iq


def calCorr(userCSI, corr_vec):
    """
    Calculate the instantaneous correlation with a given correlation vector
    """
    sig_intf = np.empty((userCSI.shape[0], userCSI.shape[1], userCSI.shape[1], userCSI.shape[3]), dtype='float32')

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
    sig_intf = np.empty((userCSI.shape[0], userCSI.shape[1], userCSI.shape[3]), dtype='complex64')
    for frame in range(csi.shape[0]):
        for sc in range(userCSI.shape[3]):
            if method == 'zf':
                sig_intf[frame, :, sc] = np.dot(data[frame, :, sc], np.linalg.pinv(userCSI[frame, :, :, sc]))
            else:
                sig_intf[frame, :, sc] = np.dot(data[frame, :, sc], np.transpose(np.conj(userCSI[frame, :, :, sc]), (1, 0)))
    return sig_intf 

