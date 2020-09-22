#!/usr/bin/python3
"""

    MMIMO_DOWNLINK.py

    This script demonstrates a (non real-time) downlink beamforming operation.
    Beamforming is peformed during 2 consecutive frame bursts! In the first frame
    a reciprocity measurement among BS antennas takes place and then uplink pilots
    are sent from clients. If successful, downlink beamforming vectors are computed
    and downlink precoding and IFFT is performed. In the second frame, the precoded
    downlink data is transmitted and received by the client(s). If successful, 
    equalization is done for each received client stream.

    NOTE ON GAINS:
    Gain settings will vary depending on RF frontend board being used
    If using CBRS:
    rxgain: at 2.5GHz [3:1:105], at 3.6GHz [3:1:102]
    txgain: at 2.5GHz [16:1:93], at 3.6GHz [15:1:102]

    If using only Dev Board:
    rxgain: at both frequency bands [0:1:30]
    txgain: at both frequency bands [0:1:42]


    Example Usage:
    python3 MMIMO_DOWNLINK.py --bnodes="../IrisUtils/data_in/bs_serials.txt"
                              --cnodes="../IrisUtils/data_in/cl_serials.txt"

    We have added a couple of files listing serial numbers of
    base station nodes, as well as client nodes. These need to
    be modified according to the nodes being used for each experiment

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
sys.path.append('../IrisUtils/')
sys.path.append('../IrisUtils/data_in/')
import numpy as np
from optparse import OptionParser
import SoapySDR
from SoapySDR import * #SOAPY_SDR_ constants
import time
import math
import datetime
import json
import signal
from scipy.linalg import hadamard
import scipy.io as sio 
from array import array
import matplotlib
matplotlib.rcParams.update({'font.size': 10})
import matplotlib.pyplot as plt
from matplotlib import animation
from find_lts import *
from type_conv import cfloat2uint32
from ofdmtxrx import *

plt.style.use('ggplot')  # customize your plots style

TDD_CONF_REG = 120
SCH_ADDR_REG = 136
SCH_MODE_REG = 140

RUNNING = True
TOT_FRAMES = 1000 
LTS_THRESH = 0.8


def signal_handler(signum, frame):
    global RUNNING
    RUNNING = False


def init(hub, bnodes, cnodes, ref_ant, ampl, rate, freq, txgain, rxgain, cp, plotter, numSamps, prefix_length, postfix_length, tx_advance, mod_order, threshold, use_trig):
    if hub != "": hub_dev = SoapySDR.Device(dict(driver="remote", serial = hub)) # device that triggers bnodes and ref_node
    bsdrs = [SoapySDR.Device(dict(driver="iris", serial = serial)) for serial in bnodes] # base station sdrs
    csdrs = [SoapySDR.Device(dict(driver="iris", serial = serial)) for serial in cnodes] # client sdrs
    # assume trig_sdr is part of the master nodes
    trig_dev = None
    if hub != "":
        trig_dev = hub_dev
    else:
        trig_dev = bsdrs[0]

    #set params on both channels
    for sdr in bsdrs+csdrs:
        info = sdr.getHardwareInfo()
        print("%s settings on device" % (info["frontend"]))
        for ch in [0, 1]:
            sdr.setBandwidth(SOAPY_SDR_TX, ch, 2.5*rate)
            sdr.setBandwidth(SOAPY_SDR_RX, ch, 2.5*rate)
            sdr.setSampleRate(SOAPY_SDR_TX, ch, rate)
            sdr.setSampleRate(SOAPY_SDR_RX, ch, rate)
            # sdr.setFrequency(SOAPY_SDR_TX, ch, freq)
            # sdr.setFrequency(SOAPY_SDR_RX, ch, freq)
            sdr.setFrequency(SOAPY_SDR_TX, ch, 'RF', freq-.75*rate)
            sdr.setFrequency(SOAPY_SDR_RX, ch, 'RF', freq-.75*rate)
            sdr.setFrequency(SOAPY_SDR_TX, ch, 'BB', .75*rate)
            sdr.setFrequency(SOAPY_SDR_RX, ch, 'BB', .75*rate)
            sdr.setAntenna(SOAPY_SDR_RX, ch, "TRX")
            sdr.setDCOffsetMode(SOAPY_SDR_RX, ch, True)

            sdr.setGain(SOAPY_SDR_TX, ch, 'PAD', txgain)
            sdr.setGain(SOAPY_SDR_RX, ch, 'LNA', rxgain)

            if "CBRS" in info["frontend"]:
                sdr.setGain(SOAPY_SDR_TX, ch, 'ATTN', -6)
                sdr.setGain(SOAPY_SDR_RX, ch, 'LNA2', 14)
                if freq < 3e9:
                    sdr.setGain(SOAPY_SDR_RX, ch, 'ATTN', -12)
                else:
                    sdr.setGain(SOAPY_SDR_RX, ch, 'ATTN', 0)

            # Read initial gain settings
            readLNA = sdr.getGain(SOAPY_SDR_RX, 0, 'LNA')
            readTIA = sdr.getGain(SOAPY_SDR_RX, 0, 'TIA')
            readPGA = sdr.getGain(SOAPY_SDR_RX, 0, 'PGA')
            print("INITIAL GAIN - LNA: {}, \t TIA:{}, \t PGA:{}".format(readLNA, readTIA, readPGA))

        sdr.writeSetting("RESET_DATA_LOGIC", "")
        sdr.writeSetting(SOAPY_SDR_RX, 1, 'ENABLE_CHANNEL', 'false')
        sdr.writeSetting(SOAPY_SDR_TX, 1, 'ENABLE_CHANNEL', 'false')

    trig_dev.writeSetting("SYNC_DELAYS", "")

    sym_samps = numSamps + prefix_length + postfix_length
    print("numSamps = %d"%sym_samps)
    M = len(bsdrs)
    K = len(csdrs)
    N = 64
    D = 1 # number of downlink symbols

    pad1 = np.array([0]*(prefix_length), np.complex64) # to comprensate for front-end group delay
    pad2 = np.array([0]*(postfix_length), np.complex64) # to comprensate for rf path delay
    wbz = np.array([0]*(sym_samps), np.complex64)
    # OFDM object
    ofdm_obj = ofdmTxRx()


    #### Generate Pilot
    cp_len = 32 if cp else 0
    ofdm_len = 2*N + cp_len
    lts_rep = numSamps//(ofdm_len)
    zeros = np.array([0]*(numSamps-ofdm_len))
    lts_sym, lts_f = generate_training_seq(preamble_type='lts', cp=cp_len, upsample=1)
    pilot = np.concatenate((lts_sym, zeros))
    wb_pilot = ampl * pilot
    wb_pilot1 = np.concatenate([pad1, wb_pilot, pad2])
    lts_t = lts_sym[-64:]
    lts_t_cp = np.concatenate((lts_t[len(lts_t) - 16:], lts_t))

    #### Generate Beacon and hadamard weights
    upsample = 1
    preambles_bs = generate_training_seq(preamble_type='gold_ifft', seq_length=128, cp=0, upsample=upsample)
    preambles = preambles_bs[:,::upsample]
    beacon = preambles[0,:]
    coe = cfloat2uint32(np.conj(beacon), order='QI')
    bcnz = np.array([0]*(sym_samps-prefix_length-len(beacon)), np.complex64)  
    beacon1 = np.concatenate([pad1,beacon*.5,bcnz])
    beacon2 = wbz  

    possible_dim = []
    possible_dim.append(2**(np.ceil(np.log2(M))))
    h_dim = min(possible_dim)
    hadamard_matrix = hadamard(h_dim)
    beacon_weights = hadamard_matrix[0:M, 0:M]
    beacon_weights = beacon_weights.astype(np.uint32)

    #### Generate Data
    data_cp_len = 16 if cp else 0
    data_ofdm_len = N + data_cp_len
    n_ofdm_syms = (numSamps // data_ofdm_len)
    sig_t, data_const, tx_data, sc_idx_all, pilots_matrix = \
        ofdm_obj.generate_data(n_ofdm_syms - 2, mod_order, cp_length=data_cp_len)
    data_sc = sc_idx_all[0]
    pilot_sc = sc_idx_all[1]
    tx_dl_data = np.zeros((N, n_ofdm_syms, K)).astype(complex)
    for k in range(K):
        tx_dl_data[data_sc, 2:, k] = data_const
        tx_dl_data[pilot_sc, 2:, k] = pilots_matrix
        tx_dl_data[:, 0, k] = lts_f
        tx_dl_data[:, 1, k] = lts_f
    tx_dl_ifft = np.zeros((M, n_ofdm_syms, N)).astype(complex)
    print("n_ofdm_syms %d, data_ofdm_len %d"%(n_ofdm_syms, data_ofdm_len))

    # received data params 
    lts_thresh = 0.8
    n_data_ofdm_syms = n_ofdm_syms - 2
    payload_len = n_data_ofdm_syms * data_ofdm_len
    lts_len = 2 * data_ofdm_len
    fft_offset = 0

    #### Configure tdd mode
    guardSize = (len(csdrs)) % 2 + 1
    frameLen = len(csdrs) + len(bsdrs)*2 + 4 + guardSize

    # BS frame config
    for i,sdr in enumerate(bsdrs):
        beacon_sch = "BG"
        if i == ref_ant:
            ref_ul_pilot_sch = "PG"
            ref_dl_pilot_sch = ''.join("RG" * (M - 1))
            ul_pilot_sch = ''.join("R" * K)
        else:
            ref_ul_pilot_sch = "RG"
            new_i = i - (i > ref_ant)
            ref_dl_pilot_sch = ''.join("GG" * new_i) + "PG" + ''.join("GG" * (M-(new_i+2)))
            ul_pilot_sch = ''.join("R" * K)

        frame_sch1 = beacon_sch + ref_ul_pilot_sch + ref_dl_pilot_sch + ul_pilot_sch + 'G' 

        dl_data_sch = "PG" + ''.join("G" * (2 * M + K - (2 * D)))
        frame_sch2 = beacon_sch + dl_data_sch + 'G'

        print("BS node %d frame schedule (%s, %s)" % (i, frame_sch1, frame_sch2))
        bconf = {"tdd_enabled": True, 
                "frame_mode": "triggered", 
                "symbol_size" : sym_samps, 
                "frames": [frame_sch1, frame_sch2],
                "beacon_start" : prefix_length,
                "beacon_stop" : prefix_length+len(beacon),
                "max_frame" : 2}
        sdr.writeSetting("TDD_CONFIG", json.dumps(bconf))
        sdr.writeSetting("TDD_MODE", "true")

    # Client frame config
    for i, sdr in enumerate(csdrs):
        det_sch = "GG"
        ref_pilot_sch = ''.join("GG" * M)
        ul_pilot_sch = ''.join("G" * i) + "P" + ''.join("G" * (K-(i+1)))
        frame_sch1 = det_sch + ref_pilot_sch + ul_pilot_sch + 'G'

        dl_data_sch = "RG" * D + ''.join("G" * (2 * M + K - (2 * D)))
        frame_sch2 = det_sch + dl_data_sch + 'G'

        print("Client %d frame schedule  (%s, %s)"%(i, frame_sch1, frame_sch2))
        cconf = {"tdd_enabled": True,
                 "frame_mode": "triggered",
                 "symbol_size" : sym_samps,
                 "frames": [frame_sch1, frame_sch2],
                 "max_frame" : 0}
        sdr.writeSetting("TDD_CONFIG", json.dumps(cconf))
        sdr.writeSetting("TDD_MODE", "true")

    for sdr in bsdrs+csdrs:
        sdr.writeSetting("TX_SW_DELAY", str(30))

    if not use_trig:
        for sdr in csdrs:
            # enable the correlator, with zeros as inputs
            corr_conf = {"corr_enabled" : True,
                        "corr_threshold" : 1}
            sdr.writeSetting("CORR_CONFIG", json.dumps(corr_conf))
            sdr.writeRegisters("CORR_COE", 0, coe.tolist())

            # DEV: ueTrigTime = 153 (prefix_length=0),
            # CBRS: ueTrigTime = 235 (prefix_length=82), tx_advance=prefix_length,
            # corr delay is 17 cycles
            #cl_trig_time = prefix_length + len(beacon) + postfix_length + 17 + postfix_length
            cl_trig_time = 256 + 250
            sf_start = cl_trig_time // sym_samps
            sp_start = cl_trig_time % sym_samps
            print("UE starting symbol and sample count (%d, %d)" % (sf_start, sp_start))
            # make sure to set this after TDD mode is enabled "writeSetting("TDD_CONFIG", ..."
            sdr.setHardwareTime(SoapySDR.ticksToTimeNs((sf_start << 16) | sp_start, rate), "TRIGGER")
    else:
        for sdr in csdrs:
            sdr.setHardwareTime(0, "TRIGGER")

    for i,sdr in enumerate(bsdrs):
        sdr.setHardwareTime(0, "TRIGGER")

    replay_addr = 0
    pilot_uint = cfloat2uint32(wb_pilot1, order='QI').tolist()
    beacon_uint = cfloat2uint32(beacon1, order='QI').tolist()
    zero_uint = cfloat2uint32(wbz, order='QI').tolist() 
    for i, sdr in enumerate(bsdrs):
        sdr.writeRegisters("BEACON_RAM", 0, beacon_uint)
        sdr.writeRegisters("BEACON_RAM_WGT_A", 0, beacon_weights[i].tolist())
        sdr.writeSetting("BEACON_START", str(M))

    for sdr in csdrs:
        sdr.writeRegisters("TX_RAM_A", replay_addr, pilot_uint)
        sdr.writeRegisters("TX_RAM_B", replay_addr, zero_uint)

    # Create and activate streams
    rx_stream_ul = [sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0, 1]) for sdr in bsdrs]
    rx_stream_dl = [sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0, 1]) for sdr in csdrs]
    flags = 0
    for i, sdr in enumerate(bsdrs):
        sdr.activateStream(rx_stream_ul[i], flags, 0)
    for i, sdr in enumerate(csdrs):
        sdr.activateStream(rx_stream_dl[i], flags, 0)

    #############
    #initialize array and matrixes to hold rx and processed data
    #############
    calib_rx_dl = [np.array([1]*sym_samps).astype(np.complex64) for m in range(M)]
    calib_rx_ul = [np.array([1]*sym_samps).astype(np.complex64) for m in range(M)]
    data_rx_dl = [[np.array([0]*sym_samps).astype(np.complex64) for k in range(K)] for m in range(D)]
    pilot_rx_ul = [[np.array([0]*sym_samps).astype(np.complex64) for m in range(M)] for k in range(K)]
    dummy_rx = np.array([0]*sym_samps).astype(np.complex64)

    ul_cal_offset = np.array([0]*M, np.int32)
    dl_cal_offset = np.array([0]*M, np.int32)

    ul_offset = [np.array([0]*K, np.int32) for m in range(M)] 
    dl_offset = [np.array([0]*K, np.int32) for m in range(M)] 

    ul_csi_mat = np.empty([K, M, N], dtype=np.complex64)
    rx_f_cal_dl = np.empty([M, N], dtype=np.complex64)
    rx_f_cal_ul = np.empty([M, N], dtype=np.complex64)

    w_zf_dl = np.empty([M, K, N], dtype=np.complex64)
    w_conj_dl = np.empty([M, K, N], dtype=np.complex64)

    calib = np.empty((M, N)).astype(np.complex64)

    rxSymbols_mat = np.empty([len(data_sc), D * n_data_ofdm_syms, K], dtype=np.complex64)

    cont_plotter = plotter

    if cont_plotter:
        fig1, axes1 = plt.subplots(nrows=M, ncols=2, figsize=(9, 12))
        axes1[0, 0].set_title('Pilot Uplink (Re)')
        axes1[0, 1].set_title('Pilot Uplink (Im)')
        for m in range(M):
            axes1[m, 0].set_xlim(0, sym_samps)
            axes1[m, 0].set_ylim(-1, 1)
            if m == ref_ant:
                axes1[m, 0].set_ylabel('Ant %d (ref)'%m)
            else:
                axes1[m, 0].set_ylabel('Ant %d'%m)
            axes1[m, 0].legend(fontsize=10)
            axes1[m, 1].set_xlim(0, sym_samps)
            axes1[m, 1].set_ylim(-1, 1)
            axes1[m, 1].legend(fontsize=10)
        lines11 = [[axes1[m, 0].plot(range(sym_samps), np.real(pilot_rx_ul[k][m]), label="User %d (real)"%k)[0] for k in range(K)] for m in range(M)]
        lines12 = [[axes1[m, 1].plot(range(sym_samps), np.imag(pilot_rx_ul[k][m]), label="User %d (imag)"%k)[0] for k in range(K)] for m in range(M)]
        fig1.show()

        fig2, axes2 = plt.subplots(nrows=M, ncols=2, figsize=(9, 12))
        axes2[0, 0].set_title('Calibration Downlink')
        axes2[0, 1].set_title('Calibration Uplink')
        for m in range(M):
            axes2[m, 0].set_xlim(0, sym_samps)
            axes2[m, 0].set_ylim(-1, 1)
            if m == ref_ant:
                axes2[m, 0].set_ylabel('Ant %d (ref)'%m)
            else:
                axes2[m, 0].set_ylabel('Ant %d'%m)
            axes2[m, 0].legend(fontsize=10)
            axes2[m, 1].set_xlim(0, sym_samps)
            axes2[m, 1].set_ylim(-1, 1)
            axes2[m, 1].legend(fontsize=10)
        lines20 = [axes2[m, 0].plot(range(sym_samps), calib_rx_dl[m][:sym_samps])[0] for m in range(M)]
        lines21 = [axes2[m, 1].plot(range(sym_samps), calib_rx_ul[m][:sym_samps])[0] for m in range(M)]
        lines24 = [axes2[m, 0].plot(range(sym_samps), calib_rx_dl[m][:sym_samps])[0] for m in range(M)]
        lines25 = [axes2[m, 1].plot(range(sym_samps), calib_rx_ul[m][:sym_samps])[0] for m in range(M)]
        fig2.show()

        fig3, axes3 = plt.subplots(nrows=K, ncols=1, figsize=(6, 6))
        for k in range(K):
            if K == 1:
                ax3 = axes3
            else:
                ax3 = axes3[k]
            ax3.grid(True)
            ax3.set_title('TX/RX Constellation')
            ax3.set_xlabel('')
            ax3.set_ylabel('')
            ax3.set_ylim(-5.5, 5.5)
            ax3.set_xlim(-5.8, 5.8)
            ax3.legend(fontsize=10)

        if K == 1:
            line31, = axes3.plot([], [], 'ro', label='TXSym')
            line32, = axes3.plot([], [], 'bx', label='RXSym')
        else:
            line31 = [axes3[k].plot([], [], 'ro', label='TXSym')[0] for k in range(K)]
            line32 = [axes3[k].plot([], [], 'bx', label='RXSym')[0] for k in range(K)]

        fig3.show()

        fig4, axes4 = plt.subplots(nrows=K, ncols=1, figsize=(6, 6))
        for k in range(K):
            if K == 1:
                ax4 = axes4
            else:
                ax4 = axes4[k]
            ax4.grid(True)
            ax4.set_title('Received Downlink')
            ax4.set_xlabel('')
            ax4.set_ylabel('')
            ax4.set_ylim(-1, 1)
            ax4.set_xlim(0, sym_samps)
            ax4.legend(fontsize=10)

        if K == 1:
            line41, = axes4.plot(range(sym_samps), data_rx_dl[0][0], label='RX Downlink')
            line42, = axes4.plot(range(sym_samps), data_rx_dl[0][0])
        else:
            line41 = [axes4[k].plot(range(sym_samps), data_rx_dl[0][k], label='RX Downlink')[0] for k in range(K)]
            line42 = [axes4[k].plot(range(sym_samps), data_rx_dl[0][k])[0] for k in range(K)]

        fig4.show()


    cur_frame = 0
    signal.signal(signal.SIGINT, signal_handler)
    tstart = datetime.datetime.now()

    while(RUNNING):

        ## disarm correlator in the clients
        if not use_trig:    
            for i, sdr in enumerate(csdrs):
                sdr.writeSetting("CORR_CONFIG", json.dumps({"corr_enabled": False}))

        bad_recip_read = False
        bad_frame = False
        ## arm correlator in the clients, inputs from adc
        if not use_trig:    
            for i, sdr in enumerate(csdrs):
                sdr.writeSetting("CORR_START", "A")

        for i, sdr in enumerate(bsdrs):
            sdr.writeRegisters("TX_RAM_A", 0, pilot_uint)
            sdr.writeRegisters("TX_RAM_B", 0, zero_uint)

        trig_dev.writeSetting("TRIGGER_GEN", "")

        ## collect reciprocity pilots from antenna m
        for m in range(M):
            if bad_recip_read: break
            if m != ref_ant:
                sr = bsdrs[m].readStream(rx_stream_ul[m], [calib_rx_ul[m], dummy_rx], sym_samps)
                if sr.ret < sym_samps:
                    print("Calib: m %d ret %d"%(m,sr.ret))
                    bad_recip_read = True

        for m in range(M):
            if bad_recip_read: break
            if m != ref_ant:
                sr = bsdrs[ref_ant].readStream(rx_stream_ul[ref_ant], [calib_rx_dl[m], dummy_rx], sym_samps)
                if sr.ret < sym_samps:
                    print("Calib: m %d ret %d"%(m,sr.ret))
                    bad_recip_read = True

        if bad_recip_read: 
            print("BAD RECIPROCAL PILOT READ... CONTINUE! ")
            continue

        ## collect uplink pilots
        bad_pilot_read = False
        for k in range(K):
            if bad_pilot_read: break
            for m in range(M):
                sr = bsdrs[m].readStream(rx_stream_ul[m], [pilot_rx_ul[k][m], dummy_rx], sym_samps)
                if sr.ret < sym_samps:
                    print("PilotUP: k: %d, m %d ret %d"%(k,m,sr.ret))
                    bad_pilot_read = True

        if bad_pilot_read: 
            print("BAD PILOT READ... CONTINUE! ")
            continue


        ## process downlink signal 
        # processing the received calibration samples
        print("frame %d"%(cur_frame))
        for m in range(M):
            if ref_ant == m: 
                calib[m,:] = np.array([1]*N, np.complex64) #continue
                rx_f_cal_dl[m,:] = np.array([0]*N, np.complex64) 
                rx_f_cal_ul[m,:] = np.array([0]*N, np.complex64) 
                continue 
            calib_rx_dl[m] -= np.mean(calib_rx_dl[m])
            calib_rx_ul[m] -= np.mean(calib_rx_ul[m])
             
            best_peak_dl, _, _ = find_lts(calib_rx_dl[m], thresh=LTS_THRESH)
            best_peak_ul, _, _ = find_lts(calib_rx_ul[m], thresh=LTS_THRESH)
            dl_cal_offset[m] = 0 if not best_peak_dl else best_peak_dl - len(lts_sym) + cp_len 
            ul_cal_offset[m] = 0 if not best_peak_ul else best_peak_ul - len(lts_sym) + cp_len 
            if (dl_cal_offset[m] < 150 or ul_cal_offset[m] < 150):
                bad_frame = True
            lts_dn_1 = calib_rx_dl[m][dl_cal_offset[m]:dl_cal_offset[m]+N]
            lts_dn_2 = calib_rx_dl[m][dl_cal_offset[m]+N:dl_cal_offset[m]+2*N]
            lts_up_1 = calib_rx_ul[m][ul_cal_offset[m]:ul_cal_offset[m]+N]
            lts_up_2 = calib_rx_ul[m][ul_cal_offset[m]+N:ul_cal_offset[m]+2*N]

            rx_f_cal_dl1 = np.fft.fftshift(np.fft.fft(lts_dn_1, N, 0), 0)
            rx_f_cal_dl2 = np.fft.fftshift(np.fft.fft(lts_dn_2, N, 0), 0)
            rx_f_cal_ul1 = np.fft.fftshift(np.fft.fft(lts_up_1, N, 0), 0)
            rx_f_cal_ul2 = np.fft.fftshift(np.fft.fft(lts_up_2, N, 0), 0)
            rx_f_cal_dl[m, :] = (rx_f_cal_dl1 + rx_f_cal_dl2)/2
            rx_f_cal_ul[m, :] = (rx_f_cal_ul1 + rx_f_cal_ul2)/2
            calib[m,:] = np.divide(rx_f_cal_dl[m,:], rx_f_cal_ul[m,:])
 
        # processing uplink pilot received samples
        for k in range(K):
            for m in range(M):
                pilot_rx_ul[k][m] -= np.mean(pilot_rx_ul[k][m]) 
                best_peak, _, _ = find_lts(pilot_rx_ul[k][m], thresh=LTS_THRESH)
                ul_offset[m][k] = 0 if not best_peak else (best_peak - len(lts_sym) + cp_len) 
                if ul_offset[m][k] < 150:
                    bad_frame = True 
                lts_ul_1 = pilot_rx_ul[k][m][ul_offset[m][k]:ul_offset[m][k]+N]
                lts_ul_2 = pilot_rx_ul[k][m][ul_offset[m][k]+N:ul_offset[m][k]+2*N]
                rx_f_ul1 = np.fft.fftshift(np.fft.fft(lts_ul_1, N, 0), 0)
                rx_f_ul2 = np.fft.fftshift(np.fft.fft(lts_ul_2, N, 0), 0)
                ul_csi_mat[k,m,:] = (rx_f_cal_ul1 + rx_f_cal_ul2) * lts_f / 2 

        # processing beamforming vectors and downlink transmit data
        for l in range(n_ofdm_syms):
            for n in range(N):
                dl_csi = np.matmul(ul_csi_mat[:, :, n], np.diag(calib[:, n]))
                w_zf_dl[:, :, n] = np.linalg.pinv(dl_csi)
                tx_dl_ifft[:, l, n] = np.matmul(w_zf_dl[:, :, n], tx_dl_data[n, l, :])

        tx_sym = np.fft.ifft(tx_dl_ifft, axis=2)
        tx_sym = np.concatenate((tx_sym[:,:,-data_cp_len:], tx_sym), axis=2)

        # send downlink signal
        for i, sdr in enumerate(bsdrs):
            tx_sig = np.reshape(tx_sym[i, :, :], (1, n_ofdm_syms * data_ofdm_len)) 
            tx_sig = np.concatenate([pad1, tx_sig[0, :], pad2])
            sdr.writeRegisters("TX_RAM_A", 0, cfloat2uint32(tx_sig, order='QI').tolist())
            sdr.writeRegisters("TX_RAM_B", 0, zero_uint)

        trig_dev.writeSetting("TRIGGER_GEN", "")

        # collect downlink data from antenna k
        bad_dl_read = False
        for d in range(D):
            if bad_dl_read: break
            for k in range(K):
                sr = csdrs[k].readStream(rx_stream_dl[k], [data_rx_dl[d][k], dummy_rx], sym_samps)
                if sr.ret < sym_samps:
                    print("DL DATA: symbol %d, k %d, ret %d"%(d, k, sr.ret))
                    bad_dl_read = True

        if bad_dl_read: 
            print("BAD DL READ... CONTINUE! ")
            continue

        # DC removal
        # Find LTS peaks (in case LTSs were sent)
        bad_dl_data = False
        for k in range(K):
            if bad_dl_data: break
            for d in range(D):
                data_rx_dl[d][k] -= np.mean(data_rx_dl[d][k])
                best_peak_dl, b, peaks0 = find_lts(data_rx_dl[d][k], thresh=lts_thresh, flip=True, lts_seq=lts_t_cp)
                if use_trig:
                    dl_offset[k] = 163 + 160 #0 if not best_peak_dl else best_peak_dl
                else:
                    dl_offset[k] = 0 if not best_peak_dl else best_peak_dl
                if dl_offset[k] < lts_len:
                    bad_dl_data = True
                    print("NO VALID DOWNLINK... CONTINUE! ")
                    break
                payload_start = dl_offset[k]
                payload_end = payload_start + payload_len  # Payload_len == (n_ofdm_syms * (num_sc + data_cp_len))
                lts_start = payload_start - lts_len  # where LTS-CP start
                lts = data_rx_dl[d][k][lts_start: payload_start]
                if len(lts) < lts_len:
                    print("BAD DOWNLINK PILOT... CONTINUE!  ")
                    bad_dl_data = True
                    break
                lts_1 = lts[16 + -fft_offset + np.array(range(0, 64))]
                lts_2 = lts[96 + -fft_offset + np.array(range(0, 64))]

                # Average 2 LTS symbols to compute channel estimate
                tmp = np.fft.ifftshift(lts_f)
                chan_est = np.fft.ifftshift(lts_f) * (np.fft.fft(lts_1) + np.fft.fft(lts_2))/2
                if len(data_rx_dl[d][k]) >= payload_end:
                    # Retrieve payload symbols
                    payload_samples = data_rx_dl[d][k][payload_start: payload_end]
                else:
                    bad_dl_data = True
                    print("TOO LATE (payload_end %d)... CONTINUE! "%payload_end)
                    break
                payload_samples_mat_cp = np.reshape(payload_samples, (data_ofdm_len, n_data_ofdm_syms), order="F")

                # Remove cyclic prefix
                payload_samples_mat = payload_samples_mat_cp[data_cp_len - fft_offset + np.array(range(0, N)), :]

                # FFT
                rxSig_freq = np.fft.fft(payload_samples_mat, n=N, axis=0)

                # Equalizer
                chan_est_tmp = chan_est.reshape(len(chan_est), 1, order="F")
                rxSig_freq_eq = rxSig_freq / np.matlib.repmat(chan_est_tmp, 1, n_data_ofdm_syms)
                phase_error = ofdm_obj.phase_correction(rxSig_freq_eq, pilot_sc, pilots_matrix)

                phase_corr_tmp = np.matlib.repmat(phase_error, N, 1)
                phase_corr = np.exp(-1j * phase_corr_tmp)
                rxSig_freq_eq_phase = rxSig_freq_eq * phase_corr
                rxSymbols_mat[:, d * n_data_ofdm_syms : (d + 1) * n_data_ofdm_syms, k] = rxSig_freq_eq_phase[data_sc, :]

        if bad_dl_data:
            continue

        evm_mat = np.power(np.abs(rxSymbols_mat - np.tile(tx_dl_data[data_sc, 2:, :], (1, D, 1))), 2)
        evm_per_user = np.mean(np.reshape(evm_mat, (len(data_sc) * n_data_ofdm_syms * D, K)), axis=0)
        evm_per_user_db = 10 * np.log10(evm_per_user)
        print('EVM (dB) per user')
        print([ "{:2.2f}".format(x) for x in evm_per_user_db ])
        print('')

        cur_frame += 1
        if cur_frame >= TOT_FRAMES: break
        if cont_plotter:

            for m in range(M):
                if m == ref_ant:
                    lines20[m].set_ydata(np.real(wb_pilot1))
                    lines21[m].set_ydata(np.real(wb_pilot1))
                    continue
                lines20[m].set_ydata(np.real(calib_rx_dl[m]))
                lines21[m].set_ydata(np.real(calib_rx_ul[m]))
                lines24[m].set_data(dl_cal_offset[m], np.linspace(-1.0, 1.0, num=100))
                lines25[m].set_data(ul_cal_offset[m], np.linspace(-1.0, 1.0, num=100))

            for m in range(M):
                for k in range(K):
                    lines11[m][k].set_ydata(np.real(pilot_rx_ul[k][m]))
                    lines12[m][k].set_ydata(np.imag(pilot_rx_ul[k][m]))

            if K == 1:
                txSyms_all = tx_dl_data[data_sc, 2:, 0].flatten()
                rxSyms_all = rxSymbols_mat[:, :, 0].flatten()
                line31.set_data(np.real(txSyms_all), np.imag(txSyms_all))
                line32.set_data(np.real(rxSyms_all), np.imag(rxSyms_all))
                line41.set_data(range(sym_samps), np.real(data_rx_dl[0][0]))
                line42.set_data(dl_offset[0], np.linspace(-1.0, 1.0, num=100))
            else:
                for k in range(K):
                    txSyms_all = tx_dl_data[data_sc, 2:, k].flatten()
                    rxSyms_all = rxSymbols_mat[:, :, k].flatten()
                    line31[k].set_data(np.real(txSyms_all), np.imag(txSyms_all))
                    line32[k].set_data(np.real(rxSyms_all), np.imag(rxSyms_all))
                    line41[k].set_data(range(sym_samps), np.real(data_rx_dl[0][k]))
                    line42[k].set_data(dl_offset[k], np.linspace(-1.0, 1.0, num=100))

            fig1.canvas.draw()
            fig1.show()

            fig2.canvas.draw()
            fig2.show()

            fig3.canvas.draw()
            fig3.show()

            fig4.canvas.draw()
            fig4.show()

    tend = datetime.datetime.now()
    c = tend - tstart
    print("Elapsed %d (secs)"%c.total_seconds())

    # clear up fpga states 
    tdd_conf = {"tdd_enabled" : False}
    corr_conf = {"corr_enabled" : False}
    for sdr in csdrs:
        if not use_trig:
            sdr.writeSetting("CORR_CONFIG", json.dumps(corr_conf))
    for sdr in bsdrs+csdrs:
        sdr.writeSetting("TDD_CONFIG", json.dumps(tdd_conf))
        sdr.writeSetting("TDD_MODE", "false")
        sdr.writeSetting("RESET_DATA_LOGIC", "")

    # close streams and exit
    for i, sdr in enumerate(bsdrs):
        sdr.closeStream(rx_stream_ul[i])
    for i, sdr in enumerate(csdrs):
        sdr.closeStream(rx_stream_dl[i])

def main():
    parser = OptionParser()
    parser.add_option("--args", type="string", dest="args", help="arguments", default="")
    parser.add_option("--bnodes", type="string", dest="bnodes", help="file name containing serials on the base station", default="../IrisUtils/data_in/bs_serials.txt")
    parser.add_option("--cnodes", type="string", dest="cnodes", help="file name containing serials to be used as clients", default="../IrisUtils/data_in/cl_serials.txt")
    parser.add_option("--hub", type="string", dest="hub", help="Hub node", default="")
    parser.add_option("--ref-ant", type="int", dest="ref_ant", help="Calibration reference antenna", default=0)
    parser.add_option("--ampl", type="float", dest="ampl", help="Amplitude coefficient for downCal/upCal", default=0.5)
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--freq", type="float", dest="freq", help="Optional Tx freq (Hz)", default=2.5e9)
    parser.add_option("--txgain", type="float", dest="txgain", help="Optional Tx gain (dB)", default=40.0)
    parser.add_option("--rxgain", type="float", dest="rxgain", help="Optional Rx gain (dB)", default=20.0)
    parser.add_option("--bw", type="float", dest="bw", help="Optional Tx filter bw (Hz)", default=10e6)
    parser.add_option("--cp", action="store_true", dest="cp", help="adds cyclic prefix to tx symbols", default=True)
    parser.add_option("--plotter", action="store_true", dest="plotter", help="continuously plots all signals and stats",default=False)
    parser.add_option("--numSamps", type="int", dest="numSamps", help="Number of samples in Symbol", default=400)
    parser.add_option("--prefix-length", type="int", dest="prefix_length", help="prefix padding length for beacon and pilot", default=82)
    parser.add_option("--postfix-length", type="int", dest="postfix_length", help="postfix padding length for beacon and pilot", default=68)
    parser.add_option("--tx-advance", type="int", dest="tx_advance", help="symbol advance for tx", default=2)
    parser.add_option("--corr-threshold", type="int", dest="threshold", help="Correlator Threshold Value", default=1)
    parser.add_option("--use-trig", action="store_true", dest="use_trig", help="uses chain triggers for synchronization",default=False)
    parser.add_option("--modOrder", type="int", dest="modOrder", help="Modulation Order 2=BPSK/4=QPSK/16=16QAM/64=64QAM", default=2)
    (options, args) = parser.parse_args()

    bserials = []
    with open(options.bnodes, "r") as f:
        for line in f.read().split():
            if line[0] != '#':
                bserials.append(line)
            else:
                continue      

    cserials = []
    with open(options.cnodes, "r") as f:
        for line in f.read().split():
            if line[0] != '#':
                cserials.append(line)
            else:
                continue

    init(hub=options.hub,
         bnodes=bserials,
         cnodes=cserials,
         ref_ant=options.ref_ant,
         ampl=options.ampl,
         rate=options.rate,
         freq=options.freq,
         txgain=options.txgain,
         rxgain=options.rxgain,
         cp=options.cp,
         plotter=options.plotter,
         numSamps=options.numSamps,
         prefix_length=options.prefix_length,
         postfix_length=options.postfix_length,
         tx_advance=options.tx_advance,
         mod_order=options.modOrder,
         threshold=options.threshold,
         use_trig=options.use_trig)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
