#!/usr/bin/python
"""
    sample_offset_cal_prototype.py

    ** NOTE **
    This script is used to test sample offset and calibration. Not a Demo script
    so don't expect it to work as is

    One-time calibration procedure to align samples from all radios in the base station

    Example:
        python3 sample_offset_cal_prototype.py --filename="./data_in/calibrate_bs_radios.txt"

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
sys.path.append('../IrisUtils/')

import SoapySDR
from SoapySDR import *  # SOAPY_SDR_ constants
from optparse import OptionParser
from collections import Counter
import numpy as np
import matplotlib.pyplot as plt
import json
import time
import copy
from type_conv import *
from generate_sequence import *
from find_lts import *

plt.style.use('ggplot')  # customize your plots style

#########################################
#                Registers              #
#########################################
# TDD Register Set
RF_RST_REG = 48
TDD_CONF_REG = 120
SCH_ADDR_REG = 136
SCH_MODE_REG = 140
TX_GAIN_CTRL = 88
FPGA_IRIS30_TRIGGERS = 44
FPGA_IRIS30_INCR_TIME = (1 << 2)
FPGA_IRIS30_DECR_TIME = (1 << 3)


#########################################
#              Functions                #
#########################################
def txrx_app(serials, ref_node_idx, hub_serial, rate, freq, txgain, rxgain, numSamps, prefix_pad, postfix_pad):
    """
    Function to configure Iris boards, generate pilot to be transmitted,
    write pilots to RAM, set up schedule (frame), transmit, and receive.
    """
    serials_all = serials.copy()
    ref_serial = serials[ref_node_idx]
    serials.remove(ref_serial)

    # Get SDR objects (msdr: master SDR, ssdr: slave SDRs)
    ssdr = [SoapySDR.Device(dict(driver="iris", serial=serial)) for serial in serials]
    msdr = SoapySDR.Device(dict(driver="iris", serial=ref_serial))

    if hub_serial != "":
        hub = SoapySDR.Device(dict(driver='faros', serial=hub_serial))

    # eror flag
    flag = 0

    # Some default sample rates
    for i, sdr in enumerate(ssdr + [msdr]):
        info = sdr.getHardwareInfo()
        print("%s settings on device %d" % (info["frontend"], i))
        for ch in [0, 1]:
            sdr.setSampleRate(SOAPY_SDR_TX, ch, rate)
            sdr.setSampleRate(SOAPY_SDR_RX, ch, rate)
            sdr.setFrequency(SOAPY_SDR_TX, ch, 'RF', freq-.75*rate)
            sdr.setFrequency(SOAPY_SDR_RX, ch, 'RF', freq-.75*rate)
            sdr.setFrequency(SOAPY_SDR_TX, ch, 'BB', .75*rate)
            sdr.setFrequency(SOAPY_SDR_RX, ch, 'BB', .75*rate)
            if "CBRS" in info["frontend"]:
                sdr.setGain(SOAPY_SDR_TX, ch, 'ATTN', 0)  # [-18,0] by 3
                sdr.setGain(SOAPY_SDR_TX, ch, 'PA1', 15)  # [0|15]
                sdr.setGain(SOAPY_SDR_TX, ch, 'PA2', 0)   # [0|15]
                sdr.setGain(SOAPY_SDR_TX, ch, 'PA3', 30)  # [0|30]
            sdr.setGain(SOAPY_SDR_TX, ch, 'IAMP', 12)     # [0,12]
            sdr.setGain(SOAPY_SDR_TX, ch, 'PAD', txgain)  # [-52,0]

            if "CBRS" in info["frontend"]:
                sdr.setGain(SOAPY_SDR_RX, ch, 'ATTN', 0)   # [-18,0]
                sdr.setGain(SOAPY_SDR_RX, ch, 'LNA1', 30)  # [0,33]
                sdr.setGain(SOAPY_SDR_RX, ch, 'LNA2', 17)  # [0,17]
            sdr.setGain(SOAPY_SDR_RX, ch, 'LNA', rxgain)   # [0,30]
            sdr.setGain(SOAPY_SDR_RX, ch, 'TIA', 0)        # [0,12]
            sdr.setGain(SOAPY_SDR_RX, ch, 'PGA', 0)        # [-12,19]
            sdr.setAntenna(SOAPY_SDR_RX, ch, "TRX")
            sdr.setDCOffsetMode(SOAPY_SDR_RX, ch, True)
        # ADC_rst, stops the tdd time counters, makes sure next time runs in a clean slate
        sdr.writeRegister("IRIS30", RF_RST_REG, (1 << 29) | 0x1)
        sdr.writeRegister("IRIS30", RF_RST_REG, (1 << 29))
        sdr.writeRegister("IRIS30", RF_RST_REG, 0)

    # Generate pilot to be TX
    # WiFi LTS Signal - Cyclic prefix of 32
    lts_sym, lts_freq = generate_training_seq(preamble_type='lts', cp=32, upsample=1)
    pilot = np.tile(lts_sym, numSamps // len(lts_sym)).astype(complex)
    pilot = pilot / max(pilot)  # Normalize amplitude
    pad1 = np.array([0] * prefix_pad, np.complex64)
    pad2 = np.array([0] * postfix_pad, np.complex64)
    pilot1 = np.concatenate([pad1, pilot, pad2])
    pilot2 = np.array([0] * len(pilot1), np.complex64)
    # Packet size
    symSamp = len(pilot) + prefix_pad + postfix_pad
    print("num pilot samps = %d" % len(pilot))
    print("num total samps = %d" % symSamp)

    # Synchronization delays
    if hub_serial != "":
        hub.writeSetting("SYNC_DELAYS", "")
    else:
        msdr.writeSetting("SYNC_DELAYS", "")

    # SW Delays
    [sdr.writeSetting("TX_SW_DELAY", str(30)) for sdr in (ssdr + [msdr])]

    # TDD Mode
    [sdr.writeSetting("TDD_MODE", "true") for sdr in (ssdr + [msdr])]

    # Write Pilot to RAM
    replay_addr = 0
    [sdr.writeRegisters("TX_RAM_A", replay_addr, cfloat2uint32(pilot1, order='IQ').tolist()) for sdr in (ssdr + [msdr])]
    [sdr.writeRegisters("TX_RAM_B", replay_addr, cfloat2uint32(pilot2, order='IQ').tolist()) for sdr in (ssdr + [msdr])]

    # Set Schedule
    ref_sched = "PGRGRG"
    ref2_sched = "RGPGRG"
    other_sched = "RGRGPG"
    frame_len = len(ref_sched)                  # Number of subframes (e.g., 3 if PGP, 4 if PGPG)
    num_rx_pilots = other_sched.count('R')      # Two R's for "other" base station radios
    print("Ref node schedule %s " % ref_sched)
    print("Ref node2 schedule %s " % ref2_sched)
    print("Other node schedule %s " % other_sched)
    # Send one frame (set max_frame to 1)
    ref_conf = {"tdd_enabled": True, "trigger_out": False, "symbol_size": symSamp, "frames": [ref_sched],
                "max_frame": 1}
    ref2_conf = {"tdd_enabled": True, "trigger_out": False, "symbol_size": symSamp, "frames": [ref2_sched],
                "max_frame": 1}
    other_conf = {"tdd_enabled": True, "trigger_out": False, "dual_pilot": False, "symbol_size": symSamp,
                  "frames": [other_sched], "max_frame": 1}

    # Create RX streams
    # CS16 makes sure the 4-bit lsb are samples are being sent
    m_rxStream = msdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1], dict(WIRE=SOAPY_SDR_CS16))
    s_rxStream = [sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1], dict(WIRE=SOAPY_SDR_CS16)) for sdr in ssdr]

    # Write TDD settings (schedule and parameters)
    msdr.writeSetting("TDD_CONFIG", json.dumps(ref_conf))
    ssdr[0].writeSetting("TDD_CONFIG", json.dumps(ref2_conf))
    for i, sdr in enumerate(ssdr[1::]):
        sdr.writeSetting("TDD_CONFIG", json.dumps(other_conf))

    # Begin pilot+calibration process: Send multiple pilots and pick the correlation index that occurs more frequently
    num_cal_tx = 100
    num_ver_tx = 100
    pass_thresh = 0.8*num_cal_tx
    # Aggregate over iterations (for calibration and for verification)
    waveRxA_agg = np.zeros((num_cal_tx, num_rx_pilots*len(serials_all), symSamp))
    waveRxB_agg = np.zeros((num_cal_tx, num_rx_pilots*len(serials_all), symSamp))
    waveRxA_ver = np.zeros((num_ver_tx, num_rx_pilots*len(serials_all), symSamp))
    waveRxB_ver = np.zeros((num_ver_tx, num_rx_pilots*len(serials_all), symSamp))

    for idx in range(num_cal_tx+num_ver_tx):
        # Initialize RX arrays
        # Reference + other nodes. Assume num_rx_pilots entries for each node, regardless of whether we use them or not
        waveRxA = [np.array([0] * symSamp, np.uint32) for i in range(num_rx_pilots) for j in range(len(ssdr) + 1)]
        waveRxB = [np.array([0] * symSamp, np.uint32) for i in range(num_rx_pilots) for j in range(len(ssdr) + 1)]

        # Activate streams
        flags = 0
        r1 = msdr.activateStream(m_rxStream, flags, 0)
        if r1 < 0:
            print("Problem activating stream #1")
        for i, sdr in enumerate(ssdr):
            r2 = sdr.activateStream(s_rxStream[i], flags, 0)
            if r2 < 0:
                print("Problem activating stream #2")

        # Generate trigger
        if hub_serial != "":
            hub.writeSetting("TRIGGER_GEN", "")
        else:
            msdr.writeSetting("TRIGGER_GEN", "")

        # Read Streams
        # B0 <-- B1
        curr_rx_idx = 0
        r1 = msdr.readStream(m_rxStream, [waveRxA[curr_rx_idx], waveRxB[curr_rx_idx]], symSamp)
        print("reading stream ref antenna: ({})".format(r1))
        # B0 <-- B2
        curr_rx_idx = 1
        r1 = msdr.readStream(m_rxStream, [waveRxA[curr_rx_idx], waveRxB[curr_rx_idx]], symSamp)
        print("reading stream ref antenna: ({})".format(r1))
        # B1 <-- B0
        curr_rx_idx = 2
        r2 = ssdr[0].readStream(s_rxStream[0], [waveRxA[curr_rx_idx], waveRxB[curr_rx_idx]], symSamp)
        print("reading stream from second ref antenna: ({})".format(r2))
        # B1 <-- B2
        curr_rx_idx = 3
        r2 = ssdr[0].readStream(s_rxStream[0], [waveRxA[curr_rx_idx], waveRxB[curr_rx_idx]], symSamp)
        print("reading stream from second ref antenna: ({})".format(r2))
        # B2 <-- B0
        # B2 <-- B1
        # ...
        for i, sdr in enumerate(ssdr[1::]):
            for j in range(num_rx_pilots):
                curr_rx_idx = curr_rx_idx + 1
                r3 = sdr.readStream(s_rxStream[i+1], [waveRxA[curr_rx_idx], waveRxB[curr_rx_idx]], symSamp)
                print("reading stream other nodes: ({})".format(r3))

        # Timestamps
        print("Timestamps:")
        for sdr in ([msdr] + ssdr):
            print(hex(SoapySDR.timeNsToTicks(sdr.getHardwareTime(""), rate)))

        waveRxA = uint32tocfloat(waveRxA)
        waveRxB = uint32tocfloat(waveRxB)

        if idx < num_cal_tx:
            waveRxA_agg[idx, :, :] = waveRxA
            waveRxB_agg[idx, :, :] = waveRxB

            if idx == num_cal_tx-1:
                # Find correlation indexes
                idx_mat_tmp = find_corr_idx(waveRxA_agg, waveRxB_agg)
                idx_mat_cal = idx_mat_tmp
                # indexes = [0, 2] + list(range(4, idx_mat_tmp.shape[1]))     # FIXME - back in
                # idx_mat = idx_mat_tmp[:, indexes]                           # FIXME - back in
                idx_mat = idx_mat_tmp                                         # FIXME - out

                # Find most frequent occurrence (most common correlation index) for two RX boards
                #                  [     0       1       2       3       4           5           6           7      ...]
                # idx_mat columns: [B0_rxPilot, [], B1_rxPilot, [], B2_rxPilot, B2_rxPilot, B3_rxPilot, B3_rxPilot, ...]
                corr_idx_mat_re = np.zeros((num_rx_pilots+1, len(serials_all)))  # FIXME - remove +1
                freq_mat_re = np.zeros((num_rx_pilots+1, len(serials_all)))        # FIXME - remove +1
                most_freq = np.zeros(idx_mat.shape[1])
                num_occurr = np.zeros(idx_mat.shape[1])
                corr_idx_mat_re[:] = np.nan
                freq_mat_re[:] = np.nan
                el0 = list(range(2*(len(serials_all)-1+1)))   # FIXME - remove +1 # list column elements in one-dimensional structure
                el1 = list(range(len(serials_all)))         # list column elements in two-dimensional structure
                for colIdx in range(idx_mat.shape[1]):
                    occurr_cnt = Counter(idx_mat[:, colIdx])
                    most_freq[colIdx] = (occurr_cnt.most_common(1))[0][0]
                    num_occurr[colIdx] = (occurr_cnt.most_common(1))[0][1]

                corr_idx_mat_re[0, [1] + el1[2::]] = most_freq[[2, 4]]  # most_freq[[1] + el0[2::2]]  # FIXME swap back
                corr_idx_mat_re[1, [0] + el1[2::]] = most_freq[[0, 5]]  # most_freq[[0] + el0[3::2]]  # FIXME swap back
                corr_idx_mat_re[2, [0] + [1]] = most_freq[[1, 3]]  # FIXME- remove line
                freq_mat_re[0, [1] + el1[2::]] = num_occurr[[2, 4]]  # num_occurr[[1] + el0[2::2]]# FIXME swap back
                freq_mat_re[1, [0] + el1[2::]] = num_occurr[[0, 5]]  # num_occurr[[0] + el0[3::2]]# FIXME swap back
                freq_mat_re[2, [0] + [1]] = num_occurr[[1, 3]]  # FIXME- remove line

                # Check if we didn't meet "PASSING" threshold (i.e., confidence on rx pilots)
                if any(num_occurr < pass_thresh):
                    cleanup([msdr] + ssdr, frame_len, m_rxStream, s_rxStream)
                    flag = -1
                    return flag, corr_idx_mat_re

                # Perform calibration
                cal_coeff = calibrate(corr_idx_mat_re, ref_node_idx, ssdr)

        elif idx >= num_cal_tx:
            waveRxA_ver[idx-num_cal_tx, :, :] = waveRxA
            waveRxB_ver[idx-num_cal_tx, :, :] = waveRxB

            if idx == num_cal_tx + num_ver_tx - 1:
                # Find correlation indexes
                idx_mat_tmp = find_corr_idx(waveRxA_ver, waveRxB_ver)
                # indexes = [0, 2] + list(range(4, idx_mat_tmp.shape[1]))
                # idx_mat = idx_mat_tmp[:, indexes]
                idx_mat_ver = idx_mat_tmp

    # Show results
    if 1:
        print("Index Matrix: \n {}".format(corr_idx_mat_re))
        print("Cal Coefficient: \n {}".format(cal_coeff))

        print("MEGD VER: \n {}".format(idx_mat_ver))

        fig = plt.figure(figsize=(20, 8), dpi=120)
        fig.subplots_adjust(hspace=.5, top=.85)
        num_rows = 2
        num_cols = 3

        ax11 = fig.add_subplot(num_rows, num_cols, 1)
        ax11.grid(True)
        ax11.set_title("Calibrate Ref TX")
        ax11.plot(idx_mat_cal[:, 2], '-.sb', label='Node2', alpha=0.7)
        ax11.plot(idx_mat_cal[:, 4], '-+r', label='Node3', alpha=0.7)
        ax11.set_ylim(324, 334)
        ax11.legend(fontsize=10)

        ax12 = fig.add_subplot(num_rows, num_cols, 2)
        ax12.grid(True)
        ax12.set_title("Calibrate Node2 TX")
        ax12.plot(idx_mat_cal[:, 0], '--*g', label='Node1', alpha=1)
        ax12.plot(idx_mat_cal[:, 5], '-+r', label='Node3', alpha=0.7)
        ax12.set_ylim(324, 334)
        ax12.legend(fontsize=10)

        ax13 = fig.add_subplot(num_rows, num_cols, 3)
        ax13.grid(True)
        ax13.set_title("Calibrate Node3 TX")
        ax13.plot(idx_mat_cal[:, 1], '--*g', label='Node1', alpha=1)
        ax13.plot(idx_mat_cal[:, 3], '-.sb', label='Node2', alpha=0.5)
        ax13.set_ylim(324, 334)
        ax13.legend(fontsize=10)

        ax21 = fig.add_subplot(num_rows, num_cols, 4)
        ax21.grid(True)
        ax21.set_title("Verify Ref TX")
        ax21.plot(idx_mat_ver[:, 2], '-sb', label='Node2', alpha=0.7)
        ax21.plot(idx_mat_ver[:, 4], '-+r', label='Node3', alpha=0.7)
        ax21.set_ylim(324, 334)
        ax21.legend(fontsize=10)

        ax22 = fig.add_subplot(num_rows, num_cols, 5)
        ax22.grid(True)
        ax22.set_title("Verify Node2 TX")
        ax22.plot(idx_mat_ver[:, 0], '--*g', label='Node1', alpha=1)
        ax22.plot(idx_mat_ver[:, 5], '-+r', label='Node3', alpha=0.7)
        ax22.set_ylim(324, 334)
        ax22.legend(fontsize=10)

        ax23 = fig.add_subplot(num_rows, num_cols, 6)
        ax23.grid(True)
        ax23.set_title("Verify Node3 TX")
        ax23.plot(idx_mat_ver[:, 1], '--*g', label='Node1', alpha=1)
        ax23.plot(idx_mat_ver[:, 3], '-.sb', label='Node2', alpha=0.5)
        ax23.set_ylim(324, 334)
        ax23.legend(fontsize=10)

        plt.show()

    cleanup([msdr] + ssdr, frame_len, m_rxStream, s_rxStream)
    return flag, corr_idx_mat_re


def find_corr_idx(waveRxA, waveRxB):
    """
    Find correlation indexes
    Input:
        serials - serial number of all base station boards (including reference)
        waveRxA -
        waveRxB -

    Output:

    """
    idx_mat = np.empty((waveRxA.shape[0], waveRxA.shape[1]))   # Matrix: 2x#radios. two pilots
    idx_mat[:] = np.nan
    lts_thresh = 0.8

    # Iterate over the multiple entries
    for i in range(waveRxA.shape[0]):
        for j in range(waveRxA.shape[1]):
            # Both channels
            best_peakA, all_peaksA, corrA = find_lts(waveRxA[i, j], thresh=lts_thresh)
            best_peakB, all_peaksB, corrB = find_lts(waveRxB[i, j], thresh=lts_thresh)

            #print("Best peak BOARD: {},  A: {}    and B: {}".format(j, best_peakA, best_peakB))
            #plt.figure(count)
            #plt.plot(corrA)
            #plt.plot(corrB)
            #plt.show()

            # Check if LTS found
            if not best_peakA and not best_peakB:
                print("No LTS Found! Board")
                continue
            if not (best_peakA == best_peakB):
                print("Same board, different indexes. Wut??")

            idx_mat[i, j] = best_peakA if best_peakA else best_peakB
    return idx_mat


def calibrate(corr_idx_mat, ref_node, ssdr):
    """
    Calibrate

    Find correction factor between reference board and the rest:
        1) Find difference between second board and another board
        2) Find difference between reference board and the rest
        3) Use combined info to find difference between reference board and second board

    Input:


    Output:

    """
    diff_board2 = corr_idx_mat[0, ref_node+1] - corr_idx_mat[0, 2]
    diff_all = corr_idx_mat[1, ref_node] - corr_idx_mat[1, :]

    diff_all[1] = diff_all[2] - diff_board2
    cal_coeff = diff_all

    # BY INCREASING ONCE, THE SAMPLE JUMPS (INCREASES) BY ONE IN ALL CASES. E.G., FROM 328 TO 329
    # SIMILARLY BY DECREASING ONCE, THE SAMPLE JUMPS (DECREASES) BY ONE IN ALL CASES. E.G., FROM 326 TO 325
    # ANOTHER OBSERVATION: FIRST ITERATION IS ALWAYS DIFFERENT TO THE REST. THE REST ARE THE SAME THOUGH
    if 0:
        for i, sdr in enumerate(ssdr):
            ioffset = int(cal_coeff[i+1])       # ignore reference node
            num_iter = abs(ioffset)
            for j in range(num_iter):
                if ioffset > 0:
                    sdr.writeRegister("IRIS30", FPGA_IRIS30_TRIGGERS, FPGA_IRIS30_INCR_TIME)
                elif ioffset < 0:
                    sdr.writeRegister("IRIS30", FPGA_IRIS30_TRIGGERS, FPGA_IRIS30_DECR_TIME)

        time.sleep(5)

    # FIXME - test
    if 1:
        target = np.round(np.mean(corr_idx_mat[0, 1::]))
        print("****** TARGET: {}  *******".format(target))
        offset = target - corr_idx_mat[0, :]
        for i, sdr in enumerate(ssdr):
            ioffset = int(offset[i+1])       # ignore reference node
            num_iter = abs(ioffset)
            for j in range(num_iter):
                if ioffset > 0:
                    sdr.writeRegister("IRIS30", FPGA_IRIS30_TRIGGERS, FPGA_IRIS30_INCR_TIME)
                elif ioffset < 0:
                    sdr.writeRegister("IRIS30", FPGA_IRIS30_TRIGGERS, FPGA_IRIS30_DECR_TIME)
    return cal_coeff


def cleanup(sdrs, frame_len, m_rxStream, s_rxStream):
    for sdr in sdrs:
        sdr.writeRegister("IRIS30", RF_RST_REG, (1 << 29) | 0x1)
        sdr.writeRegister("IRIS30", RF_RST_REG, (1 << 29))
        sdr.writeRegister("IRIS30", RF_RST_REG, 0)
        for i in range(frame_len):
            sdr.writeRegister("RFCORE", SCH_ADDR_REG, i)  # subframe 0
            sdr.writeRegister("RFCORE", SCH_MODE_REG, 0)  # 01 replay
        sdr.writeRegister("RFCORE", TDD_CONF_REG, 0)

    # Deactivate and close streams
    sdrs[0].deactivateStream(m_rxStream)
    sdrs[0].closeStream(m_rxStream)
    for i, sdr in enumerate(sdrs[1::]):
        sdr.deactivateStream(s_rxStream[i])
        sdr.closeStream(s_rxStream[i])


def plotter(symSamp, waveRxA, waveRxB, serials):
    """
    Output: None
    """
    # Plotter
    fig = plt.figure(figsize=(20, 8), dpi=120)
    fig.subplots_adjust(hspace=.5, top=.85)

    # Plot results from a max of 5 boards
    nplot = min(5, len(serials))

    ax = []
    for idx, serial in enumerate(serials):
        ax.append(fig.add_subplot(nplot, 1, idx+1))
        ax[idx].grid(True)
        ax[idx].set_title('Board Serial: %s' % serials[idx])
        ax[idx].set_ylabel('Signal (units)')
        ax[idx].set_xlabel('Sample index')
        ax[idx].plot(range(len(waveRxA[idx])), np.real(waveRxA[idx]), label='ChA I Node 1')
        ax[idx].plot(range(len(waveRxB[idx])), np.real(waveRxB[idx]), label='ChB I Node 1')
        ax[idx].set_ylim(-1, 1)
        ax[idx].set_xlim(0, symSamp)
        ax[idx].legend(fontsize=10)
    plt.show()


#########################################
#                  Main                 #
#########################################
def main():
    parser = OptionParser()
    parser.add_option("--filename", type="string", dest="filename", help="file containing serial number of all radios in base station", default="./data_in/bs_serial_num.txt")
    parser.add_option("--refNode", type="int", dest="ref_node", help="Index of reference node", default=0)
    parser.add_option("--hubSerial", type="string", dest="hub_serial", help="Serial of Faros HUB", default="")
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--txgain", type="float", dest="txgain", help="Optional Tx gain (dB)", default=25.0)
    parser.add_option("--rxgain", type="float", dest="rxgain", help="Optional Rx gain (dB)", default=25.0)
    parser.add_option("--freq", type="float", dest="freq", help="Optional Tx freq (Hz)", default=3.5e9)
    parser.add_option("--numPilotSamps", type="int", dest="nPilotSamps", help="Num actual pilot samples to tx (len of LTS is 160 samps)", default=160)
    parser.add_option("--prefix-pad", type="int", dest="prefix_length", help="prefix padding length for beacon and pilot", default=100)
    parser.add_option("--postfix-pad", type="int", dest="postfix_length", help="postfix padding length for beacon and pilot", default=100)
    (options, args) = parser.parse_args()

    # Get all serial numbers for Base Station boards
    serials = []
    with open(options.filename, "r") as f:
        for line in f.read().split():
            serials.append(line)

    # Transmit/Receive pilots
    corr_idx_mat_all = []
    for idxTx in range(1):
        flag = -1
        while flag == -1:
            flag, corr_idx_mat = txrx_app(
                serials=serials.copy(),
                ref_node_idx=options.ref_node,
                hub_serial=options.hub_serial,
                rate=options.rate,
                freq=options.freq,
                txgain=options.txgain,
                rxgain=options.rxgain,
                numSamps=options.nPilotSamps,
                prefix_pad=options.prefix_length,
                postfix_pad=options.postfix_length,
            )
            if flag == -1:
                print("Calibration failed. Re-run!")
                # sys.exit(0)

        corr_idx_mat_all.append(corr_idx_mat)
    print("Final Corr Mat: \n {}".format(corr_idx_mat_all))

    # Plot results
    #plotter(symSamp, waveRxA, waveRxB, serials)


if __name__ == '__main__':
    main()

