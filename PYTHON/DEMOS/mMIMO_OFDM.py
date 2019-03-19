#!/usr/bin/python
"""
 mMIMO_OFDM.py

    !!!!!!!!!!!!! FIXME: IN PROGRESS. NOT WORKING AT THE MOMENT !!!!!!!!!!!!!!

    Extension of SISO_OFDM.py
    One data stream per RX Iris board. Supports one antenna per RX device

   Generates, transmits, and receives and OFDM signal.
   The user can select one of the following modulation schemes BPSK/QPSK/16QAM/64QAM
   It requires two Iris boards (chained or unchained). The TX board will transmit
   the signal from RF chain A and the RX board will receive it at RF chain A as well
   (script can be extended to support both chains).
   The script can be run in two modes:
       - SIM (simulation using an AWGN channel)
       - OTA (over-the-air transmission)

   Figure "SISO_OFDM_output.png" inside the "figures" folder shows an the output
   generated for a 16-QAM OTA transmission

   Usage example: python3 SISO_OFDM.py --mode="SIM"

   Based on the wl_example_siso_ofdm.m script developed for the WARP platform:
   http://warpproject.org/trac/wiki/WARPLab/Examples/OFDM

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 ---------------------------------------------------------------------
"""

import sys
sys.path.append('../IrisUtils/')
sys.path.append('../IrisUtils/data_in/')

import SoapySDR
import numpy as np
import numpy.matlib
import time
import datetime
import os
import json
import math
import signal
import threading
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import collections
import logging
import csv
import scipy.io as spio
import pickle
from SoapySDR import *              # SOAPY_SDR_ constants
from optparse import OptionParser
from matplotlib import animation
from data_recorder import *
from find_lts import *
from digital_rssi import *
from bandpower import *
from file_rdwr import *
from ofdmtxrx import *
from type_conv import *
from print_sensor import *


#########################################
#            Global Parameters          #
#########################################
running = True
pkt_count = 0
FIG_LEN = 2**13     # SIM: 2400     # OTA: 2**13
APPLY_CFO_CORR = 1
APPLY_SFO_CORR = 1
APPLY_PHASE_CORR = 1


#########################################
#                Registers              #
#########################################
# Reset
RF_RST_REG = 48

# Correlator thresholding
CORR_THRESHOLD = 0x4
CORR_RST = 0x0
CORR_SCNT = 0x8
CORR_CONF = 60
TX_GAIN_CTRL = 88


#########################################
#             Create Plots              #
#########################################
matplotlib.rcParams.update({'font.size': 10})
# fig = plt.figure(figsize=(20, 8), dpi=120)
fig = plt.figure(figsize=(10, 20), dpi=120)
fig.subplots_adjust(hspace=.4, top=.97, bottom=.03)
gs = gridspec.GridSpec(ncols=4, nrows=5)

ax1 = fig.add_subplot(gs[0, :])
ax1.grid(True)
ax1.set_title('TX Signal')
title = ax1.text(0.5, 1, '|', ha="center")
ax1.set_ylabel('Magnitude')
ax1.set_xlabel('Sample index')
line1, = ax1.plot([], [], label='RFA', animated=True)
ax1.set_ylim(0, 1)
ax1.set_xlim(0, FIG_LEN)
ax1.legend(fontsize=10)

ax2 = fig.add_subplot(gs[1, :])
ax2.grid(True)
ax2.set_title('RX Signal')
ax2.set_xlabel('Sample index')
ax2.set_ylabel('I/Q')
line2, = ax2.plot([], [], label='I - RFA', animated=True)
line3, = ax2.plot([], [], label='Q - RFA', animated=True)
ax2.set_ylim(-1, 1)
ax2.set_xlim(0, FIG_LEN)
ax2.legend(fontsize=10)

ax3 = fig.add_subplot(gs[2, :])
ax3.grid(True)
ax3.set_title('RX Signal')
ax3.set_xlabel('Sample index')
ax3.set_ylabel('Magnitude')
line4, = ax3.plot([], [], label='RFA', animated=True)
line8, = ax3.plot([], [], '--k', label='Payload Start', animated=True)  # markers
line9, = ax3.plot([], [], '--r', label='Payload End', animated=True)  # markers
line10, = ax3.plot([], [], '--g', label='LTS Start', animated=True)  # markers
ax3.set_ylim(0, 1)
ax3.set_xlim(0, FIG_LEN)
ax3.legend(fontsize=10)

ax4 = fig.add_subplot(gs[3, :])
ax4.grid(True)
ax4.set_title('Correlation Peaks')
ax4.set_xlabel('Sample index')
ax4.set_ylabel('')
line5, = ax4.plot([], [], label='RFA', animated=True)
line11, = ax4.plot([], [], '--r', label='Thresh', animated=True)  # markers
ax4.set_ylim(0, 5)
ax4.set_xlim(0, FIG_LEN)
ax4.legend(fontsize=10)

ax5 = fig.add_subplot(gs[4, 0:2])
ax5.grid(True)
ax5.set_title('TX/RX Constellation')
ax5.set_xlabel('')
ax5.set_ylabel('')
line6, = ax5.plot([], [], 'ro', label='TXSym', animated=True)
line7, = ax5.plot([], [], 'bx', label='RXSym', animated=True)
ax5.set_ylim(-1.5, 1.5)
ax5.set_xlim(-2.8, 2.8)
ax5.legend(fontsize=10)

ax6 = fig.add_subplot(gs[4, 2:4])
ax6.grid(True)
ax6.set_title('Magnitude Channel Estimates')
ax6.set_xlabel('Baseband Freq.')
ax6.set_ylabel('')
line12, = ax6.step([], [])
ax6.set_ylim(-0.1, 5)
ax6.set_xlim(-10, 10)
ax6.legend(fontsize=10)


#########################################
#              Functions                #
#########################################
def init():
    """ Initialize plotting objects """
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    line4.set_data([], [])
    line5.set_data([], [])
    line6.set_data([], [])
    line7.set_data([], [])
    line8.set_data([], [])
    line9.set_data([], [])
    line10.set_data([], [])
    line11.set_data([], [])
    line12.set_data([], [])
    return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12


def animate(i, num_samps_rd, rxStream, sdr, ofdm_params, tx_struct, ota):

    global FIG_LEN, pkt_count, ax6

    pkt_count = pkt_count + 1

    # Init
    n_ofdm_syms = ofdm_params[0]
    cp_len = ofdm_params[1]
    data_cp_len = ofdm_params[2]
    num_sc = ofdm_params[3]
    mod_order = ofdm_params[4]
    fft_offset = ofdm_params[5]

    tx_payload = tx_struct[0]
    payload_len = len(tx_struct[0])
    data_const = tx_struct[1]
    sc_idx_all = tx_struct[2]
    tx_data = tx_struct[3]
    txSignal = tx_struct[4]
    lts_syms_len = len(tx_struct[5])
    lts_freq = tx_struct[6]
    pilots_matrix = tx_struct[8]

    data_sc = sc_idx_all[0]
    pilot_sc = sc_idx_all[1]
    n_data_syms = n_ofdm_syms * len(data_sc)

    # Read samples into this buffer
    sampsRx = [np.zeros(num_samps_rd, np.complex64), np.zeros(num_samps_rd, np.complex64)]
    buff0 = sampsRx[0]  # RF Chain 1
    buff1 = sampsRx[1]  # RF Chain 2

    if ota:
        # Over-the-air Mode
        sdr.activateStream(rxStream,
            SOAPY_SDR_END_BURST,    # flags
            0,                      # timeNs (dont care unless using SOAPY_SDR_HAS_TIME)
            buff0.size)             # numElems - this is the burst size
        sr = sdr.readStream(rxStream, [buff0, buff1], buff0.size)
        if sr.ret != buff0.size:
            print("Read RX burst of %d, requested %d" % (sr.ret, buff0.size))

    else:
        # Simulation Mode
        sampsRx[0] = txSignal + 0.01 * (np.random.randn(len(txSignal)) + np.random.randn(len(txSignal)) * 1j)
        sampsRx[1] = txSignal + 0.01 * (np.random.randn(len(txSignal)) + np.random.randn(len(txSignal)) * 1j)

    # DEBUGGING
    '''
    # Load binary file
    sampsRx[0] = uint32tocfloat(read_from_file(name='./data_out/rxsamps_SISO_OFDM', leng=num_samps_rd, offset=0))
    sampsRx[1] = uint32tocfloat(read_from_file(name='./data_out/rxsamps_SISO_OFDM', leng=num_samps_rd, offset=0))
    # Load MAT file
    # mat = spio.loadmat('matlabTx.mat', squeeze_me=True)
    # sampsRx[0] = mat['tx_vec_air']
    # sampsRx[1] = mat['tx_vec_air']

    # Store received samples in binary file (second method of storage)
    # Binary file
    write_to_file('./data_out/rxsamps_SISO_OFDM', cfloat2uint32(sampsRx[0]))
    # TXT or MAT file
    # scipy.io.savemat('test', mdict={'arr': sampsRx[0]})
    # np.savetxt('test.txt', sampsRx[0], delimiter=',')
    '''

    # DC removal
    for i in [0, 1]:
        sampsRx[i] -= np.mean(sampsRx[i])

    # Find LTS peaks (in case LTSs were sent)
    lts_thresh = 0.8
    a, b, peaks0 = find_lts(sampsRx[0], thresh=lts_thresh)
    a, b, peaks1 = find_lts(sampsRx[1], thresh=lts_thresh)

    # Check if LTS found
    if not a:
        print("SISO_OFDM: No LTS Found!")
        return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12

    # If beginning of frame was not captured in current buffer
    if (a - lts_syms_len) < 0:
        print("TOO EARLY... CONTINUE! ")
        return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12

    # Decode signal (focus on RF chain A only for now)
    rxSignal = sampsRx[0]
    payload_start = a + 1
    payload_end = payload_start + payload_len - 1  # Payload_len == (n_ofdm_syms * (num_sc + data_cp_len))
    lts_start = a - lts_syms_len + 1  # where LTS-CP start

    # Apply CFO Correction
    if APPLY_CFO_CORR:
        # Get LTS
        lts = rxSignal[lts_start: lts_start + lts_syms_len]
        lts_1 = lts[-64 + -fft_offset + np.array(range(97, 161))]
        lts_2 = lts[-fft_offset + np.array(range(97, 161))]
        # Compute CFO
        tmp = np.unwrap(np.angle(lts_2 * np.conjugate(lts_1)))
        coarse_cfo_est = np.mean(tmp)
        coarse_cfo_est = coarse_cfo_est / (2 * np.pi * 64)
    else:
        coarse_cfo_est = 0

    correction_vec = np.exp(-1j * 2 * np.pi * coarse_cfo_est * np.array(range(0, len(rxSignal))))
    rxSignal_cfo = rxSignal * correction_vec

    # Channel estimation
    # Get LTS again (after CFO correction)
    lts = rxSignal_cfo[lts_start: lts_start + lts_syms_len]
    lts_1 = lts[-64 + -fft_offset + np.array(range(97, 161))]
    lts_2 = lts[-fft_offset + np.array(range(97, 161))]

    # Average 2 LTS symbols to compute channel estimate
    tmp = np.fft.ifftshift(lts_freq)
    chan_est = np.fft.ifftshift(lts_freq) * (np.fft.fft(lts_1) + np.fft.fft(lts_2))/2

    # Assert sample position
    # NOTE: If packet found is not fully captured in current buffer read, ignore it and continue...
    if len(rxSignal_cfo) >= payload_end:
        # Retrieve payload symbols
        payload_samples = rxSignal_cfo[payload_start: payload_end + 1]
    else:
        print("TOO LATE... CONTINUE! ")
        return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12

    payload_samples_mat_cp = np.reshape(payload_samples, ((num_sc + data_cp_len), n_ofdm_syms), order="F")

    # Remove cyclic prefix
    payload_samples_mat = payload_samples_mat_cp[data_cp_len - fft_offset + 1 + np.array(range(0, num_sc)), :]

    # FFT
    rxSig_freq = np.fft.fft(payload_samples_mat, n=num_sc, axis=0)

    # Equalizer
    chan_est_tmp = chan_est.reshape(len(chan_est), 1, order="F")
    rxSig_freq_eq = rxSig_freq / np.matlib.repmat(chan_est_tmp, 1, n_ofdm_syms)

    # Apply SFO Correction
    if APPLY_SFO_CORR:
        # Extract pilots and equalize them by their nominal Tx values
        pilot_freq = rxSig_freq_eq[pilot_sc, :]
        pilot_freq_corr = pilot_freq * pilots_matrix
        # Compute phase of every RX pilot
        pilot_phase = np.angle(np.fft.fftshift(pilot_freq_corr))
        pilot_phase_uw = np.unwrap(pilot_phase)
        # Slope of pilot phase vs frequency of OFDM symbol
        pilot_shift = np.fft.fftshift(pilot_sc)
        pilot_shift_diff = np.diff(pilot_shift)
        pilot_shift_diff_mod = np.remainder(pilot_shift_diff, 64).reshape(len(pilot_shift_diff), 1)
        pilot_delta = np.matlib.repmat(pilot_shift_diff_mod, 1, n_ofdm_syms)
        pilot_slope = np.mean(np.diff(pilot_phase_uw, axis=0) / pilot_delta, axis=0)
        # Compute SFO correction phases
        tmp = np.array(range(-32, 32)).reshape(len(range(-32, 32)), 1)
        tmp2 = tmp * pilot_slope
        pilot_phase_sfo_corr = np.fft.fftshift(tmp2, 1)
        pilot_phase_corr = np.exp(-1j * pilot_phase_sfo_corr)
        # Apply correction per symbol
        rxSig_freq_eq = rxSig_freq_eq * pilot_phase_corr
    else:
        sfo_corr = np.zeros((num_sc, n_ofdm_syms))

    # Apply phase correction
    if APPLY_PHASE_CORR:
        # Extract pilots and equalize them by their nominal Tx values
        pilot_freq = rxSig_freq_eq[pilot_sc, :]
        pilot_freq_corr = pilot_freq * pilots_matrix
        # Calculate phase error for each symbol
        phase_error = np.angle(np.mean(pilot_freq_corr, axis=0))
    else:
        phase_error = np.zeros((1, n_ofdm_syms))

    phase_corr_tmp = np.matlib.repmat(phase_error, num_sc, 1)
    phase_corr = np.exp(-1j * phase_corr_tmp)
    rxSig_freq_eq_phase = rxSig_freq_eq * phase_corr
    rxSymbols_mat = rxSig_freq_eq_phase[data_sc, :]

    # Demodulation
    rxSymbols_vec = np.reshape(rxSymbols_mat, n_data_syms, order="F")       # Reshape into vector
    rx_data = demodulation(rxSymbols_vec, mod_order)

    print(" === STATS ===")
    symbol_err = np.sum(tx_data != rx_data)
    print("Frame#: {} --- Symbol Errors: {} out of {} total symbols".format(pkt_count, symbol_err, n_data_syms))

    # PLOTTING SECTION
    rx_H_est_plot = np.squeeze(np.matlib.repmat(complex('nan'), 1, len(chan_est)))
    rx_H_est_plot[data_sc] = np.squeeze(chan_est[data_sc])
    rx_H_est_plot[pilot_sc] = np.squeeze(chan_est[pilot_sc])
    x_ax = (20 / num_sc) * np.array(range(-(num_sc // 2), (num_sc // 2)))  # add 5 on each side for visualization

    # Fill out data structures with measured data
    line1.set_data(range(len(txSignal)), np.abs(txSignal))
    line2.set_data(range(len(rxSignal)), np.real(rxSignal))
    line3.set_data(range(len(rxSignal)), np.imag(rxSignal))
    line4.set_data(range(len(rxSignal)), np.abs(rxSignal))
    line5.set_data(range(len(peaks0)), np.abs(peaks0))
    line6.set_data(np.real(data_const), np.imag(data_const))
    line7.set_data(np.real(rxSymbols_mat), np.imag(rxSymbols_mat))

    line8.set_data(payload_start * np.ones(100), np.linspace(0.0, 1.0, num=100))
    line9.set_data(payload_end * np.ones(100), np.linspace(0.0, 1.0, num=100))
    line10.set_data(lts_start * np.ones(100), np.linspace(0.0, 1.0, num=100))
    line11.set_data(np.linspace(0.0, FIG_LEN, num=1000), (lts_thresh * np.max(peaks0)) * np.ones(1000))
    line12.set_data(x_ax, np.fft.fftshift(abs(rx_H_est_plot)))

    return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12


def rf_setup(rate, txgain, freq, bbfreq, serialBS, serialUE, rx_gains, UEchainedToBS):
    """
    Setup TX and RX chains on both Base Station (BS) and User Equipment (UE) boards
    """

    # Over-the-air Mode
    # Transmit from both antennas in TX boards
    txChannel = [0, 1]

    sdrBS = []
    sdrUE = []
    infoBS = []
    infoUE = []

    # Instantiate devices and get device information
    for i in range(len(serialBS)):
        sdrBS.append(SoapySDR.Device(dict(driver='iris', serial=serialBS[i])))
        infoBS.append(sdrBS[i].getHardwareInfo())

        # RF Parameters
        for c in txChannel:
            print("Writing settings for BS board {} on channel {}".format(serialBS[i], c))
            # === Setup for Base Station boards (TX chain) ===
            sdrBS[i].setFrequency(SOAPY_SDR_TX, c, 'RF', freq-.75*rate)
            sdrBS[i].setFrequency(SOAPY_SDR_TX, c, 'BB', .75*rate)
            sdrBS[i].setSampleRate(SOAPY_SDR_TX, c, rate)
            if "CBRS" in infoBS[i]["frontend"]:
                # Set CBRS front-end gains
                sdrBS[i].setGain(SOAPY_SDR_TX, c, 'ATTN', 0)  # [-18,0] by 3
                sdrBS[i].setGain(SOAPY_SDR_TX, c, 'PA2', 0)   # [0|17]
            sdrBS[i].setGain(SOAPY_SDR_TX, c, 'IAMP', 0)      # [0,12]
            sdrBS[i].setGain(SOAPY_SDR_TX, c, "PAD", txgain)

            # === Setup for Base Station boards (RX chain) ===
            sdrBS[i].setFrequency(SOAPY_SDR_RX, c, 'RF', freq-.75*rate)
            sdrBS[i].setFrequency(SOAPY_SDR_RX, c, 'BB', .75*rate)
            sdrBS[i].setSampleRate(SOAPY_SDR_RX, c, rate)
            if "CBRS" in infoBS[i]["frontend"]:
                # Set CBRS front-end gains
                sdrBS[i].setGain(SOAPY_SDR_RX, c, 'LNA2', rx_gains[5])  # [0,17]
                # sdrRx.setGain(SOAPY_SDR_RX, c, 'LNA1', 30)  # [0,33]
                sdrBS[i].setGain(SOAPY_SDR_RX, c, 'ATTN', rx_gains[3])  # [-18,0]
            sdrBS[i].setGain(SOAPY_SDR_RX, c, 'LNA', rx_gains[2])       # [0,30]
            sdrBS[i].setGain(SOAPY_SDR_RX, c, 'TIA', rx_gains[1])       # [0,12]
            sdrBS[i].setGain(SOAPY_SDR_RX, c, 'PGA', rx_gains[0])       # [-12,19]
            sdrBS[i].setAntenna(SOAPY_SDR_RX, c, "TRX")
            sdrBS[i].setDCOffsetMode(SOAPY_SDR_RX, c, True)

        # Reset
        sdrBS[i].writeRegister("IRIS30", RF_RST_REG, (1 << 29) | 0x1)
        sdrBS[i].writeRegister("IRIS30", RF_RST_REG, (1 << 29))
        sdrBS[i].writeRegister("IRIS30", RF_RST_REG, 0)

        # Use both channels for transmission/reception (consider two antennas at each BS board)
        sdrBS[i].writeSetting("SPI_TDD_MODE", "MIMO")
        # Gain control
        sdrBS[i].writeRegister("IRIS30", TX_GAIN_CTRL, 0)
        # Compute delays in chains
        sdrBS[i].writeSetting("SYNC_DELAYS", "")

    for i in range(len(serialUE)):
        sdrUE.append(SoapySDR.Device(dict(driver='iris', serial=serialUE[i])))
        infoUE.append(sdrUE[i].getHardwareInfo())

        # RF Parameters
        for c in txChannel:
            print("Writing settings for UE board {} on channel {}".format(serialUE[i], c))
            # === Setup for UE boards (TX chain) ===
            sdrUE[i].setFrequency(SOAPY_SDR_TX, c, 'RF', freq-.75*rate)
            sdrUE[i].setFrequency(SOAPY_SDR_TX, c, 'BB', .75*rate)
            sdrUE[i].setSampleRate(SOAPY_SDR_TX, c, rate)
            if "CBRS" in infoUE[i]["frontend"]:
                # Set CBRS front-end gains
                sdrUE[i].setGain(SOAPY_SDR_TX, c, 'ATTN', 0)  # [-18,0] by 3
                sdrUE[i].setGain(SOAPY_SDR_TX, c, 'PA2', 0)   # [0|17]
            sdrUE[i].setGain(SOAPY_SDR_TX, c, 'IAMP', 0)      # [0,12]
            sdrUE[i].setGain(SOAPY_SDR_TX, c, "PAD", txgain)

            # === Setup for UE boards (RX chain) ===
            sdrUE[i].setFrequency(SOAPY_SDR_RX, c, 'RF', freq-.75*rate)
            sdrUE[i].setFrequency(SOAPY_SDR_RX, c, 'BB', .75*rate)
            sdrUE[i].setSampleRate(SOAPY_SDR_RX, c, rate)
            if "CBRS" in infoUE[i]["frontend"]:
                # Set CBRS front-end gains
                sdrUE[i].setGain(SOAPY_SDR_RX, c, 'LNA2', rx_gains[5])  # [0,17]
                # sdrRx.setGain(SOAPY_SDR_RX, c, 'LNA1', 30)  # [0,33]
                sdrUE[i].setGain(SOAPY_SDR_RX, c, 'ATTN', rx_gains[3])  # [-18,0]
            sdrUE[i].setGain(SOAPY_SDR_RX, c, 'LNA', rx_gains[2])       # [0,30]
            sdrUE[i].setGain(SOAPY_SDR_RX, c, 'TIA', rx_gains[1])       # [0,12]
            sdrUE[i].setGain(SOAPY_SDR_RX, c, 'PGA', rx_gains[0])       # [-12,19]
            sdrUE[i].setAntenna(SOAPY_SDR_RX, c, "TRX")
            sdrUE[i].setDCOffsetMode(SOAPY_SDR_RX, c, True)

        # Reset
        sdrUE[i].writeRegister("IRIS30", RF_RST_REG, (1 << 29) | 0x1)
        sdrUE[i].writeRegister("IRIS30", RF_RST_REG, (1 << 29))
        sdrUE[i].writeRegister("IRIS30", RF_RST_REG, 0)

        # For the UE, we only care about channel A (consider a single antenna UE)
        sdrUE[i].writeSetting("SPI_TDD_MODE", "SISO")
        sdrUE[i].writeSetting(SOAPY_SDR_RX, 1, 'ENABLE_CHANNEL', 'false')
        sdrUE[i].writeSetting(SOAPY_SDR_TX, 1, 'ENABLE_CHANNEL', 'false')

        # Measure delays or reset correlator depending on whether the UEs are chained to the BS or separated
        if UEchainedToBS:
            sdrUE[i].writeSetting("SYNC_DELAYS", "")
        else:
            sdrUE[i].writeRegister("ARGCOR", CORR_RST, 0x1)  # reset corr

    return sdrBS, sdrUE


def sounding_setup(sdrBS, sdrUE, serialBS, serialUE, prefix_length, postfix_length, numSamps, threshold, rate, txgain,
                   ota, ofdm_params, tx_advance, UEchainedToBS):
    """
    Setup sounding procedure. Stream setup, create beacons, pilots, and generate TDD schedules
    """

    # ====== Packet size (to be read by UEs for beacons and pilots) ======
    symSamp = numSamps + prefix_length + postfix_length

    # ====== Padding (to compensate for delays) ======
    pad_pre = np.array([0] * prefix_length, np.complex64)      # To comprensate for front-end group delay
    pad_post = np.array([0] * postfix_length, np.complex64)    # To comprensate for rf path delay

    # ====== Create Beacons ======
    # Beacons to be sent from BS and correlated against in UE (for RX trigger)
    upsample = 1
    ampl = 1
    # The base station may upsample, but the mobiles won't
    preambles_bs = generate_training_seq(preamble_type='gold_ifft', seq_length=128, cp=0, upsample=upsample)
    # The correlators can run at lower rates, so we only need the downsampled signal.
    preambles = preambles_bs[:, ::upsample]
    beacon = preambles[0, :] * ampl
    bcnz = np.array([0] * (symSamp - prefix_length - len(beacon)), np.complex64)
    beaconA = np.concatenate([pad_pre, beacon, bcnz])
    beaconB = np.array([0] * symSamp, np.complex64)  # no need to send beacon from channel B as well

    # ====== Create Pilots for Channel Estimation ======
    # Sounding sequence to be sent from BS boards to UEs
    cp_len = ofdm_params[1]
    ltsSym, lts_f = generate_training_seq(preamble_type='lts', cp=cp_len, upsample=1)
    pilot = np.tile(ltsSym, numSamps//len(ltsSym)).astype(np.complex64) * ampl
    # Same pilot for both channels A and B
    pilotA = np.concatenate([pad_pre, pilot, pad_post])
    pilotB = np.concatenate([pad_pre, pilot, pad_post])

    # ====== TDD Configuration ======
    # Create schedules and write configuration
    schedBS = []
    schedUE = []

    if ota:
        # ====== Create streams ======
        # We will TX-stream from BS boards, and RX-stream at UE boards
        rxStreamUE = []
        txStreamBS = []
        rxStreamBS = []  # dummy to be used later to retrieve time
        for i in range(len(serialBS)):
            txStreamBS.append(sdrBS[i].setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, [0, 1]))
            rxStreamBS.append(sdrBS[i].setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1]))
        for i in range(len(serialUE)):
            rxStreamUE.append(sdrUE[i].setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1]))

        # Number of tx symbols (pilots in this case) in one frame - depends on the number of BS boards transmitting
        numSymsTx = len(serialBS)
        # Number of total symbols in one frame
        bs_node0_init = "PRGG"                          # Pilot + dummy "R" to enable readStream for timing
        numSyms = 2 * numSymsTx + len(bs_node0_init)  # 2 channels -> twice as many symbols
        # Location index within frame
        txSymStart = len(bs_node0_init)               # symbol starts right after init symbols (index from zero)

        max_frame = 1
        for i in range(len(serialBS)):
            # Only send beacon from one antenna in BS
            if i == 0:
                sym_init = bs_node0_init
            else:
                sym_init = join("G" * len(bs_node0_init))

            # Two TX channels (A & B), so double each symbol
            tmp_sched = sym_init + ''.join("GG" * i) + ''.join("TT") + ''.join("GG" * (numSymsTx - 1 - i))
            schedBS.append(tmp_sched)

            # Write the same for all the nodes in BS
            confBS = {"tdd_enabled": True,
                      "trigger_out": False,
                      "wait_trigger": False,                # don't care since trigger_out is false
                      "dual_pilot": False,
                      "symbol_size": symSamp,
                      "frames": [tmp_sched],
                      "max_frame": max_frame}
            sdrBS[i].writeSetting("TDD_CONFIG", json.dumps(confBS))

        for i in range(len(serialUE)):
            tmp_sched = ''.join("G" * len(bs_node0_init)) + ''.join("RR" * numSymsTx)
            schedUE.append(tmp_sched)
            confUE = {"tdd_enabled": True,
                      "trigger_out": not UEchainedToBS,     # true if separate, false if chained
                      "wait_trigger": True,                 # don't care if trigger_out is false
                      "symbol_size": symSamp,
                      "frames": [tmp_sched],
                      "max_frame": max_frame}
            sdrUE[i].writeSetting("TDD_CONFIG", json.dumps(confUE))

        print("Node 1 schedule (OTA) {} ".format(schedBS))
        print("Node 2 schedule (OTA) {} ".format(schedUE))
        print("PILOT at index {} ".format(txSymStart))

        # Delay to counteract difference between data and control paths to RF frontend
        for i in range(len(serialBS)):
            sdrBS[i].writeSetting("TX_SW_DELAY", str(30))
        for i in range(len(serialUE)):
            sdrUE[i].writeSetting("TX_SW_DELAY", str(30))

        # ===== Correlator Setup (at UEs) =====
        # Use correlator as trigger if UEs not chained to BS
        if not UEchainedToBS:
            # Coefficients to be loaded onto UEs for correlation
            # FPGA correlator takes coefficients in QI order
            coefficients = cfloat2uint32(np.conj(beacon), order='QI')

            auto_tx_gain = 0
            for iUE in range(len(serialUE)):
                # Enable correlator, w/zeros as inputs
                sdrUE[iUE].writeRegister("IRIS30", CORR_CONF, int("00004001", 16))
                for i in range(128):
                    sdrUE[iUE].writeRegister("ARGCOE", i*4, 0)
                time.sleep(0.1)
                sdrUE[iUE].writeRegister("ARGCOR", CORR_THRESHOLD, int(threshold))
                sdrUE[iUE].writeRegister("ARGCOR", CORR_RST, 0x1)  # reset corr
                sdrUE[iUE].writeRegister("ARGCOR", CORR_RST, 0x0)  # unrst corr
                for i in range(128):
                    sdrUE[iUE].writeRegister("ARGCOE", i*4, int(coefficients[i]))
                if auto_tx_gain:
                    max_gain = int(txgain)
                    min_gain = max(0, max_gain-15)
                    gain_reg = 0xF000 | (max_gain & 0x3F) << 6 | (min_gain & 0x3F)
                    print("gain reg 0x%X" % gain_reg)
                    # [15] en, [14] mode, [13:12] step, [11:6] stop, [5:0] start
                    sdrUE[iUE].writeRegister("IRIS30", TX_GAIN_CTRL, gain_reg)

                # DEV: ueTrigTime = 153 (prefix_length=0), CBRS: ueTrigTime = 235 (prefix_length=82),
                # tx_advance=prefix_length,
                # corr delay is 17 cycles
                ueTrigTime = prefix_length + len(beacon) + postfix_length + 17 + tx_advance
                sf_start = ueTrigTime // symSamp
                sp_start = ueTrigTime % symSamp
                print("UE starting symbol and sample count (%d, %d)" % (sf_start, sp_start))
                # make sure to set this after TDD mode is enabled "writeSetting("TDD_CONFIG", ..."
                sdrUE[iUE].setHardwareTime(SoapySDR.ticksToTimeNs((sf_start << 16) | sp_start, rate), "TRIGGER")

        # ===== Enable TDD (This will likely be removed in a future release) =====
        for i in range(len(serialBS)):
            sdrBS[i].writeSetting("TDD_MODE", "true")
        for i in range(len(serialUE)):
            sdrUE[i].writeSetting("TDD_MODE", "true")

        # ===== Write Beacons to RAM (only first BS board) =====
        # Over-the-air Mode
        replay_addr = 0
        sdrBS[0].writeRegisters("TX_RAM_A", replay_addr, cfloat2uint32(beaconA, order='IQ').tolist())
        sdrBS[0].writeRegisters("TX_RAM_B", replay_addr, cfloat2uint32(beaconB, order='IQ').tolist())  # all zeros

        # Enable the correlator, with inputs from adc
        # Use correlator as trigger if UEs not chained to BS
        if not UEchainedToBS:
            for i in range(len(serialUE)):
                sdrUE[i].writeRegister("IRIS30", CORR_CONF, int("00004011", 16))

        # Generate Trigger
        # Trigger only first board, this will propagate the trigger
        sdrBS[0].writeSetting("TRIGGER_GEN", "")

    else:
        # Simulation Mode - don't care about most of these variables
        rxStreamUE = []
        txStreamBS = []
        rxStreamBS = []
        txSymStart = []
        numSymsTx = len(serialBS)

    preamble_struct = [ltsSym, lts_f, pilotA, pilotB, symSamp, txSymStart, numSymsTx]

    return preamble_struct, rxStreamUE, txStreamBS, rxStreamBS


def sounding(sdrBS, sdrUE, serialBS, serialUE, preamble_struct,  rxStreamUE, txStreamBS, rxStreamBS, fft_offset, ota,
             prefix_length, postfix_length, ofdm_params):
    """
    Sound channel and compute channel estimates
    """

    # Length of LTS symbol
    lts_syms_len = len(preamble_struct[0])

    # LTS in Freq domain
    lts_freq = preamble_struct[1]

    # Pilots to be TX
    pilotA = preamble_struct[2]
    pilotB = preamble_struct[3]

    # Number of subcarriers
    nSC = ofdm_params[3]

    # Number of samples to read
    symSampAll = (len(pilotA) + len(pilotB)) * len(serialBS)  # 2 TX chains per BS board * number of samples per each pilot

    # Only one pilot per antenna, so txSym = 1
    txSymStart = preamble_struct[5]
    numSymsTx = preamble_struct[6]

    # Arrays to store samples
    dummyA = np.array([0] * symSampAll, np.uint32)
    dummyB = np.array([0] * symSampAll, np.uint32)

    if ota:
        waveRxA = np.zeros((len(serialUE), symSampAll), dtype=np.uint32)
        # Activate TX/RX streams
        flags = 0
        for i in range(len(serialBS)):
            rt = sdrBS[i].activateStream(txStreamBS[i])
            rr = sdrBS[i].activateStream(rxStreamBS[i], flags, 0)
            if rt < 0:
                print("Problem activating TX stream for BS board {}".format(serialBS[i]))
            if rr < 0:
                print("Problem activating RX stream for BS board {}".format(serialBS[i]))
        for i in range(len(serialUE)):
            rt = sdrUE[i].activateStream(rxStreamUE[i], flags, 0)
            if rt < 0:
                print("Problem activating RX stream for UE board {}".format(serialUE[i]))

        # Write TX streams
        # Only reason for first reading the stream here is to get a timestamp for TX purposes.
        # Boards are sync'ed so we only need to read from one
        sr = sdrBS[0].readStream(rxStreamBS[0], [dummyA, dummyB], symSampAll)
        if sr.ret > 0:
            txTime = sr.timeNs & 0xFFFFFFFF00000000
            txTime += (0x000000000 + (txSymStart << 16))
        else:
            txTime = 0
            print("Something is wrong here")

        # Now write the streams
        flags = SOAPY_SDR_HAS_TIME  # | SOAPY_SDR_END_BURST
        txTimeNs = txTime  # SoapySDR.ticksToTimeNs(txTime, rate)
        for i in range(len(serialBS)):
            for j in range(numSymsTx):
                st = sdrBS[i].writeStream(txStreamBS[i], [pilotA, pilotB], len(pilotA), flags, timeNs=txTimeNs)
                txTimeNs += 0x10000
                if st.ret < 0:
                    print("WriteStream: ret=%d,flags=%d,timeNs=0x%X,txTime=0x%X" %
                          (st.ret, st.flags, st.timeNs, txTimeNs))

        # Read streams at UEs - single antenna so just read from channel A
        for i in range(len(serialUE)):
            sr = sdrUE[i].readStream(rxStreamUE[i], [waveRxA[i, :], dummyB], symSampAll)
            if sr.ret < 0 or sr.ret > symSampAll:
                print("readStream returned %d" % sr.ret)

        # Deactivate and Close Streams
        for i in range(len(serialBS)):
           sdrBS[i].deactivateStream(rxStreamBS[i])
           sdrBS[i].deactivateStream(txStreamBS[i])
           sdrBS[i].closeStream(rxStreamBS[i])
           sdrBS[i].closeStream(txStreamBS[i])
        for i in range(len(serialUE)):
           sdrUE[i].deactivateStream(rxStreamUE[i])
           sdrUE[i].closeStream(rxStreamUE[i])

    else:
        # SIMULATION
        waveRxA = np.zeros((len(serialUE), symSampAll), dtype=np.complex)
        # Create RX signal at each UE
        tmp = np.concatenate([pilotA, pilotB])
        for i in range(len(serialUE)):
            waveRxA[i, :] = np.squeeze(np.matlib.repmat(tmp, 1, len(serialBS)))

    # Process Pilots
    # DC removal
    for i in range(len(serialUE)):
        waveRxA[i, :] -= np.mean(waveRxA[i, :])

    # Channel estimation
    lts_start_debug = []

    chan_est = np.zeros((len(serialUE), len(serialBS), 2, nSC))     # 2 RF channels (A & B) per BS board
    pilot_size = len(pilotA) + len(pilotB)
    lts_thresh = 0.8
    for i in range(len(serialUE)):
        sig_full = waveRxA[i, :]

        for j in range(len(serialBS)):
            # Get segment corresponding to current BS-board pilots
            sig_tmp = sig_full[j * pilot_size: (j+1) * pilot_size]

            # Find LTS peaks
            a, b, peaks0 = find_lts(sig_tmp, thresh=lts_thresh)

            for k in range(2):
                # RF channels A and B

                # Find where LTS starts
                lts_start = a - lts_syms_len + 1            # where LTS-CP start
                lts_start = lts_start + (k * len(pilotA))   # LTS from Channel B is at a full pilot + prefix + postfix

                print("LenBuffer: {}, lenSigTmp: {}, rangeStart: {}, rangeEnd: {}, lenRange: {}, a: {}, startLTS: {}".
                      format(len(sig_full),
                             len(sig_tmp),
                             j*pilot_size,
                             (j+1)*pilot_size,
                             len(range(j * pilot_size, (j+1) * pilot_size)),
                             a,
                             lts_start))

                lts_start_debug.append(lts_start)

                # Apply CFO Correction
                if APPLY_CFO_CORR:
                    # Get LTS
                    lts = sig_tmp[lts_start: lts_start + lts_syms_len]
                    lts_1 = lts[-64 + -fft_offset + np.array(range(97, 161))]
                    lts_2 = lts[-fft_offset + np.array(range(97, 161))]
                    # Compute CFO
                    tmp = np.unwrap(np.angle(lts_2 * np.conjugate(lts_1)))
                    coarse_cfo_est = np.mean(tmp)
                    coarse_cfo_est = coarse_cfo_est / (2 * np.pi * 64)
                else:
                    coarse_cfo_est = 0

                correction_vec = np.exp(-1j * 2 * np.pi * coarse_cfo_est * np.array(range(0, len(sig_tmp))))
                rxSignal_cfo = sig_tmp * correction_vec

                # Channel estimation
                # Get LTS again (after CFO correction)
                lts = rxSignal_cfo[lts_start: lts_start + lts_syms_len]
                lts_1 = lts[-64 + -fft_offset + np.array(range(97, 161))]
                lts_2 = lts[-fft_offset + np.array(range(97, 161))]

                # Average 2 LTS symbols to compute channel estimate
                chan_est[i, j, k, :] = np.fft.ifftshift(lts_freq) * (np.fft.fft(lts_1) + np.fft.fft(lts_2))/2

            # Plotting (debug)
            fig2 = plt.figure(100+j)
            ax1 = fig2.add_subplot(2, 1, 1)
            ax1.grid(True)
            ax1.plot(np.abs(sig_tmp))
            ax1.scatter(lts_start_debug, 0.25*np.ones((1,len(lts_start_db))))
            ax2 = fig2.add_subplot(2, 1, 2)
            ax2.grid(True)
            ax2.plot(np.abs(sig_full))
            plt.show(block=False)

    return chan_est


def txrx_app(ampl, ofdm_params, num_samps_rd, ota, serialUE, sdrBS, sdrUE, chan_est):
    """
    Generate TX streams, and write data into RAM for TX
    """

    # Generate TX Signal
    # Preambles
    n_ofdm_syms = ofdm_params[0]
    cp_len = ofdm_params[1]
    data_cp_len = ofdm_params[2]
    mod_order = ofdm_params[4]

    # WiFi STS Signal
    sts_sym = generate_training_seq(preamble_type='sts', reps=10)
    # WiFi LTS Signal
    lts_sym, lts_freq = generate_training_seq(preamble_type='lts', cp=cp_len, upsample=1)
    # Full preamble
    preamble = np.concatenate((sts_sym, lts_sym))
    # Init signal
    prefix = np.zeros((30,)).astype(complex)
    posfix = np.zeros((30,)).astype(complex)

    # Generate Data Signal - one data stream per RX Iris board
    tx_struct = []
    signal1_ui32 = []
    signal2_ui32 = []
    for i in range(len(serialRx)):
        sig_t, data_const, tx_data, sc_idx_all, pilots_matrix = generate_data(n_ofdm_syms, mod_order, cp_length=data_cp_len)
        sigLen = len(preamble) + len(prefix) + len(posfix) + len(sig_t)
        txSignal = np.empty(sigLen).astype(np.complex64)
        wbz = txSignal

        # Precoding HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



        txSignal = np.concatenate((prefix, preamble, sig_t, posfix))
        txSignal = ampl * txSignal / np.absolute(txSignal).max()

        tx_struct.append([sig_t, data_const, sc_idx_all, tx_data, txSignal, lts_sym, lts_freq, preamble, pilots_matrix])

        # Float to fixed point
        signal1_ui32.append(cfloat2uint32(txSignal, order='IQ'))
        signal2_ui32.append(cfloat2uint32(wbz, order='IQ'))       # Currently unused

    if ota:
        # Over-the-air Mode
        # Replay_address is the offset inside RAM where we are writing to. For basic operation with one signal being
        # transmitted from one or two antennas, make it zero
        replay_addr = 0
        if ant == 'A':
            sdrTx.writeRegisters("TX_RAM_A", replay_addr, signal1_ui32.tolist())
        elif ant == 'B':
            sdrTx.writeRegisters("TX_RAM_B", replay_addr, signal1_ui32.tolist())
        elif ant == 'AB':
            sdrTx.writeRegisters("TX_RAM_A", replay_addr, signal1_ui32.tolist())
            sdrTx.writeRegisters("TX_RAM_B", replay_addr, signal1_ui32.tolist())

        # This starts transmission
        sdrTx.writeSetting("TX_REPLAY", str(len(signal1_ui32)))

         # Setup RX stream
        rxStream = sdrRx.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0, 1])
    else:
        # Simulation Mode
        rxStream = []

    # Start animation
    anim = animation.FuncAnimation(fig, animate,
                                   init_func=init,
                                   fargs=(num_samps_rd, rxStream, sdrRx, ofdm_params, tx_struct, ota),
                                   frames=100,
                                   interval=100,
                                   blit=True)
    plt.show()


#########################################
#                 Main                  #
#########################################
def main():
    parser = OptionParser()
    parser.add_option("--mode", type="string", dest="mode", help="Simulation vs Over-the-Air (i.e., SIM/OTA)", default="SIM")
    parser.add_option("--rate", type="float", dest="rate", help="Tx and Rx sample rate", default=5e6)
    parser.add_option("--ampl", type="float", dest="ampl", help="Tx digital amplitude scale", default=1)
    parser.add_option("--txgain", type="float", dest="txgain", help="Tx gain (dB)", default=-5.0) # with CBRS -25
    parser.add_option("--LNA", type="float", dest="LNA", help="LNA gain (dB) [0:1:30]", default=15.0) # with CBRS 15
    parser.add_option("--TIA", type="float", dest="TIA", help="TIA gain (dB) [0, 3, 9, 12]", default=0.0) # with CBRS 0
    parser.add_option("--PGA", type="float", dest="PGA", help="PGA gain (dB) [-12:1:19]", default=-12.0) # with CBRS -12
    parser.add_option("--LNA1", type="float", dest="LNA1", help="BRS/CBRS Front-end LNA1 gain stage [0:33] (dB)", default=0.0)
    parser.add_option("--LNA2", type="float", dest="LNA2", help="BRS/CBRS Front-end LNA2 gain [0:17] (dB)", default=0.0)
    parser.add_option("--ATTN", type="float", dest="ATTN", help="BRS/CBRS Front-end ATTN gain stage [-18:6:0] (dB)", default=0.0)
    parser.add_option("--freq", type="float", dest="freq", help="Tx RF freq (Hz)", default=3.6e9)
    parser.add_option("--bbfreq", type="float", dest="bbfreq", help="Lime chip Baseband frequency (Hz)", default=0)
    parser.add_option("--nOFDMsym", type="int", dest="nOFDMsym", help="Number of OFDM symbols", default=20)
    parser.add_option("--ltsCpLen", type="int", dest="ltsCpLen", help="Length of Cyclic Prefix - LTS", default=32)
    parser.add_option("--corr-threshold", type="int", dest="threshold", help="Correlator Threshold Value", default=128)
    parser.add_option("--ue-tx-advance", type="int", dest="tx_advance", help="sample advance for tx vs rx (correlator)", default=68)
    parser.add_option("--dataCpLen", type="int", dest="dataCpLen", help="Length of Cyclic Prefix - Data", default=16)
    parser.add_option("--nSC", type="int", dest="nSC", help="# of subcarriers. Only supports 64 sc at the moment", default=64)
    parser.add_option("--fftOfset", type="int", dest="fftOffset", help="FFT Offset: # of CP samples for FFT", default=6)
    parser.add_option("--modOrder", type="int", dest="modOrder", help="Modulation Order 2=BPSK/4=QPSK/16=16QAM/64=64QAM", default=16)
    parser.add_option("--serialBS", type="string", dest="serialBS", help="Serial # of TX device", default=["RF3C000025", "RF3C000025"])
    parser.add_option("--serialUE", type="string", dest="serialUE", help="Serial # of RX device", default=["RF3C000042"])
    parser.add_option("--UEchainedToBS", action="store_true", dest="UEchainedToBS", help="Use chain trigger for synchronization, if UEs attached to BS", default=True)
    parser.add_option("--nSampsRead", type="int", dest="nSampsRead", help="# Samples to read", default=FIG_LEN)
    parser.add_option("--numSampsPilot", type="int", dest="numSampsPilot", help="Num samples to receive during pilot TX", default=160)  # 2.5 LTS sequences
    parser.add_option("--prefix-length", type="int", dest="prefix_length", help="prefix padding length for beacon and pilot", default=82)     # to comprensate for front-end group delay
    parser.add_option("--postfix-length", type="int", dest="postfix_length", help="postfix padding length for beacon and pilot", default=68)  # to comprensate for rf path delay

    (options, args) = parser.parse_args()

    ofdm_params = [options.nOFDMsym, options.ltsCpLen, options.dataCpLen, options.nSC, options.modOrder, options.fftOffset]
    rx_gains = [options.PGA, options.TIA, options.LNA, options.ATTN, options.LNA1, options.LNA2]

    # Verify transmission mode
    if options.mode == "OTA":
        ota = 1
    elif options.mode == "SIM":
        ota = 0
    else:
        raise Exception("Transmission Mode Not Supported. Options: OTA/SIM")

    # Verify number of subcarriers
    if not (options.nSC == 64):
        raise AssertionError("Only 64 subcarriers are currently supported")

    # Display parameters
    print("\n")
    print("========== TX PARAMETERS =========")
    print("Transmitting OFDM signal from {} to {}".format(options.serialBS, options.serialUE))
    print("Sample Rate (sps): {}".format(options.rate))
    print("Both Antennas")
    print("Tx Gain (dB): {}".format(options.txgain))
    print("Rx Gains (dB) - LNA: {}, TIA: {}, PGA: {}".format(options.LNA, options.TIA, options.PGA))
    print("CBRS/BRS Rx Gains (dB) - LNA1: {}, LNA2: {}, ATTN: {}".format(options.LNA1, options.LNA2, options.ATTN))
    print("Frequency (Hz): {}".format(options.freq))
    print("Baseband Freq. (Hz): {}".format(options.bbfreq))
    print("OFDM Parameters - # OFDM syms: {}, LTS CP length: {}, Data CP length: {}, # SC: {}, Modulation Order: {}".
          format(options.nOFDMsym, options.ltsCpLen, options.dataCpLen, options.nSC, options.modOrder))
    print("==================================")
    print("\n")

    ##########################
    #   TX/RX boards setup   #
    ##########################
    if ota:
        sdrBS, sdrUE = rf_setup(
            rate=options.rate,
            txgain=options.txgain,
            freq=options.freq,
            bbfreq=options.bbfreq,
            serialBS=options.serialBS,
            serialUE=options.serialUE,
            rx_gains=rx_gains,
            UEchainedToBS=options.UEchainedToBS,
        )
    else:
        sdrBS = []
        sdrUE = []

    ###############################
    #  Prepare channel sounding   #
    ###############################
    preamble_struct, rxStreamUE, txStreamBS, rxStreamBS = sounding_setup(
        sdrBS=sdrBS,
        sdrUE=sdrUE,
        serialBS=options.serialBS,
        serialUE=options.serialUE,
        prefix_length=options.prefix_length,
        postfix_length=options.postfix_length,
        numSamps=options.numSampsPilot,
        threshold=options.threshold,
        rate=options.rate,
        txgain=options.txgain,
        ota=ota,
        ofdm_params=ofdm_params,
        tx_advance=options.tx_advance,
        UEchainedToBS=options.UEchainedToBS,
    )

    ###############################
    #       Channel Sounding      #
    ###############################
    chan_est = sounding(
        sdrBS=sdrBS,
        sdrUE=sdrUE,
        serialBS=options.serialBS,
        serialUE=options.serialUE,
        preamble_struct=preamble_struct,
        rxStreamUE=rxStreamUE,
        txStreamBS=txStreamBS,
        rxStreamBS=rxStreamBS,
        fft_offset=options.fftOffset,
        ota=ota,
        prefix_length=options.prefix_length,
        postfix_length=options.postfix_length,
        ofdm_params=ofdm_params,
    )

    ##########################################################
    #  Start channel sounding and data transmission process  #
    ##########################################################
    txrx_app(
        ampl=options.ampl,
        ofdm_params=ofdm_params,
        num_samps_rd=options.nSampsRead,
        ota=ota,
        serialUE=options.serialUE,
        sdrBS=sdrBS,
        sdrUE=sdrUE,
        chan_est=chan_est,
    )

if __name__ == '__main__':
    main()
