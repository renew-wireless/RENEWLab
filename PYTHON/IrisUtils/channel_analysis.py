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
def samps2csi(samps, num_users,samps_per_user=224, fft_size=64, offset=0, bound=94, cp=0):
    """Input samps dims: Frame, Cell, Antenna, User, Sample"""
    """Returns iq with Frame, Cell, User, Pilot Rep, Antenna, Sample"""
    """Returns csi with Frame, Cell, User, Pilot Rep, Antenna, Subcarrier"""
    debug = False
    chunkstart = time.time()
    usersamps = np.reshape(samps, (samps.shape[0], samps.shape[1], num_users, samps.shape[3], samps_per_user, 2))
    nbat = min([(samps_per_user-bound)//(fft_size+cp),2]) #What is this? It is eiter 1 or 2: 2 LTSs??
    iq = np.empty((samps.shape[0],samps.shape[1],num_users,samps.shape[3],nbat,fft_size),dtype='complex64')
    if debug:
        print("chunkstart = {}, usersamps.shape = {}, samps.shape = {}, samps_per_user = {}, nbat= {}, iq.shape = {}".format(chunkstart, usersamps.shape, samps.shape, samps_per_user, nbat, iq.shape))
    for i in range(nbat):  # 2 first symbols (assumed LTS) seperate estimates
        iq[:, :, :, :, i, :] = (usersamps[:, :, :, :, offset + cp + i*fft_size:offset+cp+(i+1)*fft_size, 0] +
                                usersamps[:, :, :, :, offset + cp + i*fft_size:offset+cp+(i+1)*fft_size, 1]*1j)*2**-15

    iq = iq.swapaxes(3, 4) 
    if debug:
        print("iq.shape after axes swapping: {}".format(iq.shape))

    fftstart = time.time()
    csi = np.empty(iq.shape, dtype='complex64')
    if fft_size == 64:
        # Retrieve frequency-domain LTS sequence
        _, lts_freq = generate_training_seq(preamble_type='lts', seq_length=[], cp=32, upsample=1, reps=[])
        pre_csi = np.fft.fftshift(np.fft.fft(iq, fft_size, 5), 5)
        csi = np.fft.fftshift(np.fft.fft(iq, fft_size, 5), 5) * lts_freq
        if debug:
            print("csi.shape:{} lts_freq.shape: {}, pre_csi.shape = {}".format(csi.shape, lts_freq.shape, pre_csi.shape))
        endtime = time.time()
        if debug:
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

def csi_from_pilots(pilots_dump, z_padding = 150, fft_size=64, cp=16, frm_st_idx = 0, frame_to_plot = 0, ref_ant=0):
    """ 
    Finds the end of the pilots' frames, finds all the lts indices relative to that.
    Divides the data with lts sequences, calculates csi per lts, csi per frame, csi total.  
    """
    print("********************* csi_from_pilots(): *********************")
    
    # Reviewing options and vars: 
    show_plot = True
    debug  = False
    test_mf = False
    write_to_file = True
    
    # dimensions of pilots_dump
    n_frame = pilots_dump.shape[0]      # no. of captured frames
    n_cell = pilots_dump.shape[1]       # no. of cells
    n_ue = pilots_dump.shape[2]         # no. of UEs 
    n_ant = pilots_dump.shape[3]        # no. of BS antennas
    n_iq = pilots_dump.shape[4]         # no. of IQ samples per frame
    
    if debug:
        print("input : z_padding = {}, fft_size={}, cp={}, frm_st_idx = {}, frame_to_plot = {}, ref_ant={}".format(z_padding, fft_size, cp, frm_st_idx, frame_to_plot, ref_ant))
        print("n_frame = {}, n_cell = {}, n_ue = {}, n_ant = {}, n_iq = {}".format(
        n_frame, n_cell, n_ue, n_ant, n_iq) )

    if ((n_iq % 2) != 0 ):
        print("Size of iq samples:".format(n_iq))
        raise Exception(' **** The length of iq samples per frames HAS to be an even number! **** ')

    n_cmpx =  n_iq // 2                 # no. of complex samples
    n_csamp = n_cmpx - z_padding        # no. of complex samples in a P subframe without pre- and post- fixes
    idx_e = np.arange(0, n_iq, 2)       # even indices: real part of iq
    idx_o = np.arange(1, n_iq, 2)       # odd indices: imaginary part of iq 

    # make a new data structure where the iq samples become complex numbers
    cmpx_pilots = (pilots_dump[:,:,:,:,idx_e] + 1j*pilots_dump[:,:,:,:,idx_o])*2**-15

    # take a time-domain lts sequence, concatenate more copies, flip, conjugate
    lts_t, lts_f = generate_training_seq(preamble_type='lts', seq_length=[], cp=32, upsample=1, reps=[])    # TD LTS sequences (x2.5), FD LTS sequences
    lts_tmp = lts_t[-80:]                            # last 80 samps (assume 16 cp)
    n_lts = len(lts_tmp)              
    k_lts = n_csamp // n_lts                         # no. of LTS sequences in a pilot SF
    lts_seq = lts_tmp
    lts_seq = np.tile(lts_tmp, k_lts)                # concatenate k LTS's to filter/correlate below             
    #lts_seq = lts_seq[::-1]                         # flip
    lts_seq_conj = np.conjugate(lts_seq)             # conjugate the local LTS sequence
    l_lts_fc = len(lts_seq_conj)                     # length of the local LTS seq.

    if debug:
        print("cmpx_pilots.shape = {}, lts_t.shape = {}".format(cmpx_pilots.shape, lts_t.shape))
        #print("idx_e= {}, idx_o= {}".format(idx_e, idx_o))
        print("n_cmpx = {}, n_csamp = {}, n_lts = {}, k_lts = {}, lts_seq_conj.shape = {}".format(
            n_cmpx, n_csamp, n_lts, k_lts, lts_seq_conj.shape))
       
    
    # debug/ testing
    if debug:
        z_pre = np.zeros(82, dtype='complex64')
        z_post = np.zeros(68, dtype='complex64')
        lts_t_rep = np.tile(lts_tmp, k_lts)
        lts_t_rep_tst = np.append(z_pre,lts_t_rep)
        lts_t_rep_tst = np.append(lts_t_rep_tst, z_post)
        
        if test_mf:
            w = np.random.normal(0, 0.1/2, len(lts_t_rep_tst)) + 1j*np.random.normal(0, 0.1/2, len(lts_t_rep_tst))
            lts_t_rep_tst = lts_t_rep_tst + w
            cmpx_pilots = np.tile(lts_t_rep_tst,(n_frame,cmpx_pilots.shape[1],cmpx_pilots.shape[2],cmpx_pilots.shape[3],1))
            print("if test_mf: Shape of lts_t_rep_tst: {} , cmpx_pilots.shape = {}".format(lts_t_rep_tst.shape, cmpx_pilots.shape))
            

    # normalized matched filter 
    a = 1;
    unos = np.ones(l_lts_fc)
    v0 = signal.lfilter(lts_seq_conj, a, cmpx_pilots, axis = 4)
    v1 = signal.lfilter(unos, a, (abs(cmpx_pilots)**2), axis = 4)     
    m_filt = (np.abs(v0)**2)/v1

    # clean up nan samples: replace nan with -1 
    nan_indices = np.argwhere(np.isnan(m_filt))    
    m_filt[np.isnan(m_filt)] = -0.5 # the only negative value in m_filt
    
    if write_to_file:
        # write the nan_indices into a file
        np.savetxt("nan_indices.txt", nan_indices, fmt='%i')
        
    if debug:
        print("Shape of truncated complex pilots: {} , l_lts_fc = {}, v0.shape = {}, v1.shape = {}, m_filt.shape = {}".
            format(cmpx_pilots.shape, l_lts_fc, v0.shape, v1.shape, m_filt.shape))
       
    rho_max = np.amax(m_filt, axis = 4)         # maximum peak per SF per antenna
    rho_min = np.amin(m_filt, axis  = 4)        # minimum peak per SF per antenna
    ipos = np.argmax(m_filt, axis = 4)          # positons of the max peaks
    sf_start = ipos - l_lts_fc + 1              # start of every received SF 
    sf_start = np.where(sf_start < 0, 0, sf_start)  #get rid of negative indices in case of an incorrect peak
         
    # get the pilot samples from the cmpx_pilots array and reshape for k_lts LTS pilots:
    pilots_rx_t = np.empty([n_frame, n_cell, n_ue, n_ant, k_lts * n_lts], dtype='complex64' )
    
    indexing_start = time.time()
    for i in range(n_frame):
        for j in range(n_cell):
            for k in range(n_ue):
                for l in range(n_ant):
                   pilots_rx_t[i,j,k,l,:] = cmpx_pilots[i,j,k,l, sf_start[i,j,k,l]:  sf_start[i,j,k,l] + (k_lts * n_lts)]
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
        print("Shape of: pilots_rx_t before truncation: {}\n".format(pilots_rx_t.shape))
   
    pilots_rx_t = np.reshape(pilots_rx_t, (n_frame, n_cell, n_ue, n_ant, k_lts, n_lts))
    pilots_rx_t = np.delete(pilots_rx_t, range(fft_size,n_lts),5)
      
    if debug:
        print("Indexing time: %f \n" % ( indexing_end -indexing_start) )      
        print("Shape of: pilots_rx_t = {}\n".format(pilots_rx_t.shape))
        print("Shape of: rho_max = {}, rho_min = {}, ipos = {}, sf_start = {}".format(
        rho_max.shape, rho_min.shape, ipos.shape, sf_start.shape))
        
    
    # take fft and get the raw CSI matrix (no averaging)
    lts_f_shft = np.fft.fftshift(lts_f)                     # align SCs based on how they were Tx-ec 
    pilots_rx_f = np.fft.fft(pilots_rx_t, fft_size, 5)      # take FFT
    zero_sc = np.where(lts_f_shft == 0)[0]                  # find the zero SCs corresponding to lts_f_shft
    lts_f_nzsc = np.delete(lts_f_shft, zero_sc)             # remove zero subcarriers
    pilots_rx_f = np.delete(pilots_rx_f, zero_sc, 5)        # remove zero subcarriers
    csi = pilots_rx_f / lts_f_nzsc                          # take channel estimate by dividing with the non-zero elements of lts_f_shft
    csi = np.fft.fftshift(csi, 5)                           # unecessary step: just to make it in accordance to lts_f as returned by generate_training_seq() 

    if debug:
        print(">>>> number of NaN indices = {} NaN indices =\n{}".format(
                nan_indices.shape, nan_indices))
        print("Shape of: csi = {}\n".format(csi.shape))
    
    # plot something to see if it worked!
    if show_plot:
        fig = plt.figure()
        ax1 = fig.add_subplot(3, 1, 1)
        ax1.grid(True)
        ax1.set_title('channel_analysis:csi_from_pilots(): Re of Rx pilot - ref frame {} and ref ant. {}'.format(frame_to_plot, ref_ant))
        if debug:
            print("cmpx_pilots.shape = {}".format(cmpx_pilots.shape))
            
        ax1.plot(np.real(cmpx_pilots[frame_to_plot - frm_st_idx,0,0,ref_ant,:]))
        
        if debug:
            loc_sec = lts_t_rep_tst
        else:
            z_pre = np.zeros(82, dtype='complex64')
            z_post = np.zeros(68, dtype='complex64')
            lts_t_rep = np.tile(lts_tmp, k_lts)
            loc_sec = np.append(z_pre,lts_t_rep)
            loc_sec = np.append(loc_sec, z_post)
        ax2 = fig.add_subplot(3, 1, 2)
        ax2.grid(True)
        ax2.set_title('channel_analysis:csi_from_pilots(): Local LTS sequence zero padded')
        ax2.plot(loc_sec)

        ax3 = fig.add_subplot(3, 1, 3)
        ax3.grid(True)
        ax3.set_title('channel_analysis:csi_from_pilots(): MF (uncleared peaks) - ref frame {} and ref ant. {}'.format(frame_to_plot, ref_ant))
        ax3.stem(m_filt[frame_to_plot - frm_st_idx, 0,0,ref_ant,:])
        plt.show()

    print("********************* ******************** *********************\n")
    return csi, m_filt, sf_start, k_lts, n_lts




