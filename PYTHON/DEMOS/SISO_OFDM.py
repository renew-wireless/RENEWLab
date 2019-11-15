#!/usr/bin/python
"""
 SISO_OFDM.py

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
from macros import *
from SoapySDR import *              # SOAPY_SDR_ constants
from optparse import OptionParser
from matplotlib import animation
from data_recorder import *
from find_lts import *
from digital_rssi import *
from bandpower import *
from file_rdwr import *
from type_conv import *
from print_sensor import *
from ofdmtxrx import *
from init_fncs import *

plt.style.use('ggplot')


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
ax4.set_ylim(0, 8)
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


def animate(i, num_samps_rd, rxStream, sdr, ofdm_params, tx_struct, ota, ofdm_obj, agc_en):
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
        flags = SOAPY_SDR_END_BURST
        flags |= SOAPY_SDR_WAIT_TRIGGER
        sdr.activateStream(rxStream,
            flags,                  # flags
            0,                      # timeNs (dont care unless using SOAPY_SDR_HAS_TIME)
            buff0.size)             # numElems - this is the burst size

        if agc_en:
            sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_RESET_FLAG, 1)
            sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_RESET_FLAG, 0)
            sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NEW_FRAME, 1)
            sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NEW_FRAME, 0)

        sdr.writeSetting("TRIGGER_GEN", "")
        sr = sdr.readStream(rxStream, [buff0, buff1], buff0.size)
        if sr.ret != buff0.size:
            print("Read RX burst of %d, requested %d" % (sr.ret, buff0.size))

        # Retrieve RSSI computed in the FPGA
        rssi_fpga = int(sdr.readRegister("IRIS30", FPGA_IRIS030_RD_MEASURED_RSSI))
        # RX
        lna_rd = sdr.getGain(SOAPY_SDR_RX, 0, 'LNA')       # ChanA (0)
        tia_rd = sdr.getGain(SOAPY_SDR_RX, 0, 'TIA')       # ChanA (0)
        pga_rd = sdr.getGain(SOAPY_SDR_RX, 0, 'PGA')       # ChanA (0)
        lna1_rd = sdr.getGain(SOAPY_SDR_RX, 0, 'LNA1')     # ChanA (0)
        lna2_rd = sdr.getGain(SOAPY_SDR_RX, 0, 'LNA2')     # ChanA (0)
        attn_rd1 = sdr.getGain(SOAPY_SDR_RX, 0, 'ATTN')    # ChanA (0)
        print("RSSI: {} \t LNA: {} \t TIA: {} \t PGA: {} \t LNA1: {} \t LNA2: {} \t ATTN1: {}".format(rssi_fpga, lna_rd, tia_rd, pga_rd, lna1_rd, lna2_rd, attn_rd1))

    else:
        # Simulation Mode
        sampsRx[0] = txSignal + 0.01 * (np.random.randn(len(txSignal)) + np.random.randn(len(txSignal)) * 1j)
        sampsRx[1] = txSignal + 0.01 * (np.random.randn(len(txSignal)) + np.random.randn(len(txSignal)) * 1j)

    # DC removal
    for i in [0, 1]:
        sampsRx[i] -= np.mean(sampsRx[i])

    # Find LTS peaks (in case LTSs were sent)
    lts_thresh = 0.8
    a, b, peaks0 = find_lts(sampsRx[0], thresh=lts_thresh, flip=True)
    a, b, peaks1 = find_lts(sampsRx[1], thresh=lts_thresh, flip=True)

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
        coarse_cfo_est = ofdm_obj.cfo_correction(rxSignal, lts_start, lts_syms_len, fft_offset)
    else:
        coarse_cfo_est = 0

    correction_vec = np.exp(-1j * 2 * np.pi * coarse_cfo_est * np.array(range(0, len(rxSignal))))
    rxSignal_cfo = rxSignal * correction_vec

    # Channel estimation
    # Get LTS again (after CFO correction)
    lts = rxSignal_cfo[lts_start: lts_start + lts_syms_len]

    # Verify number of samples
    if len(lts) != 160:
        print("INCORRECT START OF PAYLOAD... CONTINUE!")
        return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12

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

    # Assert
    if len(payload_samples) != ((num_sc + data_cp_len) * n_ofdm_syms):
        print("INCORRECT START OF PAYLOAD... CONTINUE!")
        return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12
    else:
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
        rxSig_freq_eq = ofdm_obj.sfo_correction(rxSig_freq_eq, pilot_sc, pilots_matrix, n_ofdm_syms)
    else:
        sfo_corr = np.zeros((num_sc, n_ofdm_syms))

    # Apply phase correction
    if APPLY_PHASE_CORR:
        phase_error = ofdm_obj.phase_correction(rxSig_freq_eq, pilot_sc, pilots_matrix)
    else:
        phase_error = np.zeros((1, n_ofdm_syms))

    phase_corr_tmp = np.matlib.repmat(phase_error, num_sc, 1)
    phase_corr = np.exp(-1j * phase_corr_tmp)
    rxSig_freq_eq_phase = rxSig_freq_eq * phase_corr
    rxSymbols_mat = rxSig_freq_eq_phase[data_sc, :]

    # Demodulation
    rxSymbols_vec = np.reshape(rxSymbols_mat, n_data_syms, order="F")       # Reshape into vector
    rx_data = ofdm_obj.demodulation(rxSymbols_vec, mod_order)

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


def txrx_app(args, rate, ampl, ant, txgain, freq, bbfreq, serialTx, serialRx, ofdm_params, rx_gains,
             num_samps_rd, ota, ofdm_obj, agc_en):
    """
    Setup transmitter, generate TX signal, and write data into RAM for TX
    """

    if ota:
        # Over-the-air Mode
        # Device information
        sdrTx = SoapySDR.Device(dict(serial=serialTx))
        infoTx = sdrTx.getHardwareInfo()
        sdrRx = SoapySDR.Device(dict(serial=serialRx))
        infoRx = sdrRx.getHardwareInfo()

        # AGC - target might need to change for different frequencies (tested with 30 at 3.6GHz)
        rssi_target_idx = 30
        agc_init(sdrRx, rssi_target_idx, agc_en)

        # Reset
        sdrRx.writeSetting("RESET_DATA_LOGIC", "")

        if ant == 'A':
            txChannel = [0]
        elif ant == 'B':
            txChannel = [1]
        elif ant == 'AB':
            txChannel = [0, 1]
        else:
            txChannel = []

        # RF Parameters
        for c in [0, 1]: #txChannel:
            print("Writing settings for channel {}".format(c))
            sdrTx.setBandwidth(SOAPY_SDR_TX, c, rate)
            sdrTx.setFrequency(SOAPY_SDR_TX, c, freq + bbfreq)
            sdrTx.setSampleRate(SOAPY_SDR_TX, c, rate)
            #if bbfreq > 0:
            #    sdrTx.setFrequency(SOAPY_SDR_TX, c, "BB", bbfreq)
            if "CBRS" in infoTx["frontend"]:
                print("set CBRS front-end gains")
                sdrTx.setGain(SOAPY_SDR_TX, c, 'ATTN', -6)   # {-18,-12,-6,0}
            sdrTx.setGain(SOAPY_SDR_TX, c, "PAD", txgain)
            sdrTx.setGain(SOAPY_SDR_TX, c, "IAMP", -12)

            sdrRx.setBandwidth(SOAPY_SDR_RX, c, rate)
            sdrRx.setFrequency(SOAPY_SDR_RX, c, freq)
            sdrRx.setSampleRate(SOAPY_SDR_RX, c, rate)
            if "CBRS" in infoRx["frontend"]:
                sdrRx.setGain(SOAPY_SDR_RX, c, 'LNA2', rx_gains[5])  # LO: [0|17], HI:[0|14]
                sdrRx.setGain(SOAPY_SDR_RX, c, 'LNA1', rx_gains[4])  # [0,33]
                sdrRx.setGain(SOAPY_SDR_RX, c, 'ATTN', rx_gains[3])  # {-18,-12,-6,0}
            sdrRx.setGain(SOAPY_SDR_RX, c, 'LNA', rx_gains[2])       # [0,30]
            sdrRx.setGain(SOAPY_SDR_RX, c, 'TIA', rx_gains[1])       # [0,12]
            sdrRx.setGain(SOAPY_SDR_RX, c, 'PGA', rx_gains[0])       # [-12,19]
            sdrRx.setAntenna(SOAPY_SDR_RX, c, "TRX")
            sdrRx.setDCOffsetMode(SOAPY_SDR_RX, c, True)

            sdrTx.writeSetting(SOAPY_SDR_TX, c, "CALIBRATE", 'SKLK')
            sdrRx.writeSetting(SOAPY_SDR_RX, c, "CALIBRATE", 'SKLK')
    else:
        # Simulation Mode
        sdrRx = []

    if agc_en:
        sdrRx.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_ENABLE, 1)
        sdrRx.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_ENABLE_FLAG, 1)
        sdrRx.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NEW_FRAME, 1)
        sdrRx.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NEW_FRAME, 0)

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

    # Generate Data Signal
    sig_t, data_const, tx_data, sc_idx_all, pilots_matrix = \
        ofdm_obj.generate_data(n_ofdm_syms, mod_order, cp_length=data_cp_len)
    sigLen = len(preamble) + len(prefix) + len(posfix) + len(sig_t)
    txSignal = np.empty(sigLen).astype(np.complex64)
    wbz = txSignal
    txSignal = np.concatenate((prefix, preamble, sig_t, posfix))
    txSignal = ampl * txSignal / np.absolute(txSignal).max()

    tx_struct = [sig_t, data_const, sc_idx_all, tx_data, txSignal, lts_sym, lts_freq, preamble, pilots_matrix]

    # Float to fixed point
    signal1_ui32 = cfloat2uint32(txSignal, order='QI')
    signal2_ui32 = cfloat2uint32(wbz)       # Currently unused

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
                                   fargs=(num_samps_rd, rxStream, sdrRx, ofdm_params, tx_struct, ota, ofdm_obj, agc_en),
                                   frames=100,
                                   interval=100,
                                   blit=True)
    plt.show()


#########################################
#                 Main                  #
#########################################
def main():
    parser = OptionParser()
    parser.add_option("--args", type="string", dest="args", help="Device factor arguments", default="")
    parser.add_option("--rate", type="float", dest="rate", help="Tx and Rx sample rate", default=5e6)
    parser.add_option("--ampl", type="float", dest="ampl", help="Tx digital amplitude scale", default=1)
    parser.add_option("--ant", type="string", dest="ant", help="Optional Tx antenna", default="A")
    parser.add_option("--txgain", type="float", dest="txgain", help="Tx gain (dB)", default=40.0)
    parser.add_option("--LNA", type="float", dest="LNA", help="LNA gain (dB) [0:1:30]", default=10.0)
    parser.add_option("--TIA", type="float", dest="TIA", help="TIA gain (dB) [0, 3, 9, 12]", default=0.0)
    parser.add_option("--PGA", type="float", dest="PGA", help="PGA gain (dB) [-12:1:19]", default=0.0)
    parser.add_option("--LNA1", type="float", dest="LNA1", help="BRS/CBRS Front-end LNA1 gain stage [0:33] (dB)", default=33.0)
    parser.add_option("--LNA2", type="float", dest="LNA2", help="BRS/CBRS Front-end LNA2 gain [0:17] (dB)", default=14.0)
    parser.add_option("--ATTN", type="float", dest="ATTN", help="BRS/CBRS Front-end ATTN gain stage [-18:6:0] (dB)", default=-18.0)
    parser.add_option("--freq", type="float", dest="freq", help="Tx RF freq (Hz)", default=3.597e9)
    parser.add_option("--bbfreq", type="float", dest="bbfreq", help="Lime chip Baseband frequency (Hz)", default=0)
    parser.add_option("--nOFDMsym", type="int", dest="nOFDMsym", help="Number of OFDM symbols", default=20)
    parser.add_option("--ltsCpLen", type="int", dest="ltsCpLen", help="Length of Cyclic Prefix - LTS", default=32)
    parser.add_option("--dataCpLen", type="int", dest="dataCpLen", help="Length of Cyclic Prefix - Data", default=16)
    parser.add_option("--nSC", type="int", dest="nSC", help="# of subcarriers. Only supports 64 sc at the moment", default=64)
    parser.add_option("--fftOfset", type="int", dest="fftOffset", help="FFT Offset: # of CP samples for FFT", default=6)
    parser.add_option("--modOrder", type="int", dest="modOrder", help="Modulation Order 2=BPSK/4=QPSK/16=16QAM/64=64QAM", default=16)
    parser.add_option("--serialTx", type="string", dest="serialTx", help="Serial # of TX device", default="RF3C000064")
    parser.add_option("--serialRx", type="string", dest="serialRx", help="Serial # of RX device", default="RF3C000029")
    parser.add_option("--nSampsRead", type="int", dest="nSampsRead", help="# Samples to read", default=FIG_LEN)
    parser.add_option("--mode", type="string", dest="mode", help="Simulation vs Over-the-Air (i.e., SIM/OTA)", default="OTA")
    parser.add_option("--agc_en", action="store_true", dest="agc_en", help="Flag to enable AGC", default=False)
    (options, args) = parser.parse_args()

    ofdm_params = [options.nOFDMsym, options.ltsCpLen, options.dataCpLen, options.nSC, options.modOrder, options.fftOffset]
    rx_gains = [options.PGA, options.TIA, options.LNA, options.ATTN, options.LNA1, options.LNA2]

    # OFDM object
    ofdm_obj = ofdmTxRx()

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
    print("Transmitting OFDM signal from {} to {}".format(options.serialTx, options.serialRx))
    print("Sample Rate (sps): {}".format(options.rate))
    print("Antenna: {}".format(options.ant))
    print("Tx Gain (dB): {}".format(options.txgain))
    print("Rx Gains (dB) - LNA: {}, TIA: {}, PGA: {}".format(options.LNA, options.TIA, options.PGA))
    print("CBRS/BRS Rx Gains (dB) - LNA1: {}, LNA2: {}, ATTN: {}".format(options.LNA1, options.LNA2, options.ATTN))
    print("Frequency (Hz): {}".format(options.freq))
    print("Baseband Freq. (Hz): {}".format(options.bbfreq))
    print("OFDM Parameters - # OFDM syms: {}, LTS CP length: {}, Data CP length: {}, # SC: {}, Modulation Order: {}".format(options.nOFDMsym, options.ltsCpLen, options.dataCpLen, options.nSC, options.modOrder))
    print("==================================")
    print("\n")

    txrx_app(
        args=options.args,
        rate=options.rate,
        ampl=options.ampl,
        ant=options.ant,
        txgain=options.txgain,
        freq=options.freq,
        bbfreq=options.bbfreq,
        serialTx=options.serialTx,
        serialRx=options.serialRx,
        ofdm_params=ofdm_params,
        rx_gains=rx_gains,
        num_samps_rd=options.nSampsRead,
        ota=ota,
        ofdm_obj=ofdm_obj,
        agc_en=options.agc_en,
    )


if __name__ == '__main__':
    main()
