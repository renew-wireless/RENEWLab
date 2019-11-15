#!/usr/bin/python
"""
    sample_offset_cal_prototype.py

    ** NOTE **
    This script is used to test sample offset and calibration.

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
def txrx_app(serials, ref_node_idx, hub_serial, rate, freq, txgain, rxgain, numSamps, prefix_pad, postfix_pad, debug):
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
            sdr.setGain(SOAPY_SDR_TX, ch, txgain)
            sdr.setGain(SOAPY_SDR_RX, ch, rxgain)
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
    ref_sched = "PG"
    other_sched = "RG"
    frame_len = len(ref_sched)                  # Number of subframes (e.g., 3 if PGP, 4 if PGPG)
    num_rx_pilots = other_sched.count('R')      # Two R's for "other" base station radios
    print("Ref node schedule %s " % ref_sched)
    print("Other nodes schedule %s " % other_sched)
    # Send one frame (set max_frame to 1)
    ref_conf = {"tdd_enabled": True, "trigger_out": False, "dual_pilot": False, "symbol_size": symSamp,
                "frames": [ref_sched], "max_frame": 1}
    other_conf = {"tdd_enabled": True, "trigger_out": False, "dual_pilot": False, "symbol_size": symSamp,
                  "frames": [other_sched], "max_frame": 1}

    # Create RX streams
    # CS16 makes sure the 4-bit lsb are samples are being sent
    m_rxStream = msdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1], dict(WIRE=SOAPY_SDR_CS16))
    s_rxStream = [sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1], dict(WIRE=SOAPY_SDR_CS16)) for sdr in ssdr]

    # Write TDD settings (schedule and parameters)
    msdr.writeSetting("TDD_CONFIG", json.dumps(ref_conf))
    for i, sdr in enumerate(ssdr):
        sdr.writeSetting("TDD_CONFIG", json.dumps(other_conf))

    # Begin pilot+calibration process: Send multiple pilots and pick the correlation index that occurs more frequently
    num_cal_tx = 100
    num_ver_tx = 100
    pass_thresh = 0.8*num_cal_tx
    # Aggregate over iterations (for calibration and for verification)
    waveRxA_agg = np.zeros((num_cal_tx, num_rx_pilots*(len(serials_all)-1), symSamp))
    waveRxB_agg = np.zeros((num_cal_tx, num_rx_pilots*(len(serials_all)-1), symSamp))
    waveRxA_ver = np.zeros((num_ver_tx, num_rx_pilots*(len(serials_all)-1), symSamp))
    waveRxB_ver = np.zeros((num_ver_tx, num_rx_pilots*(len(serials_all)-1), symSamp))

    for idx in range(num_cal_tx+num_ver_tx):
        # Initialize RX arrays
        # All nodes except ref. Assume num_rx_pilots entries for each node, regardless of whether we use them or not
        waveRxA = [np.array([0] * symSamp, np.uint32) for i in range(num_rx_pilots) for j in range(len(ssdr))]
        waveRxB = [np.array([0] * symSamp, np.uint32) for i in range(num_rx_pilots) for j in range(len(ssdr))]

        dummyA = np.array([0] * symSamp, np.uint32)
        dummyB = np.array([0] * symSamp, np.uint32)

        # Activate streams
        flags = 0
        r1 = msdr.activateStream(m_rxStream, flags, 0)
        if r1 < 0:
            print("Problem activating RefNode stream (Node0)")
        for i, sdr in enumerate(ssdr):
            r2 = sdr.activateStream(s_rxStream[i], flags, 0)
            if r2 < 0:
                print("Problem activating stream at RxNode #{}".format(i+1))

        # Drain buffers
        for i, sdr in enumerate([msdr] + ssdr):
            valid = 0
            while valid is not -1:
                if i == 0:
                    r0 = sdr.readStream(m_rxStream, [dummyA, dummyB], symSamp)
                else:
                    r0 = sdr.readStream(s_rxStream[i-1], [dummyA, dummyB], symSamp)
                valid = r0.ret
                if debug:
                    print("draining buffers: ({}). Board: {}".format(r0, i))

        # Generate trigger
        if hub_serial != "":
            hub.writeSetting("TRIGGER_GEN", "")
        else:
            msdr.writeSetting("TRIGGER_GEN", "")

        # Read Streams
        # B0 <-- B1 (NOT REALLY NEEDED WITH THE CURRENT SCHEDULE)
        # curr_rx_idx = 0
        # r1 = msdr.readStream(m_rxStream, [waveRxA[curr_rx_idx], waveRxB[curr_rx_idx]], symSamp)
        # print("reading stream ref antenna: ({})".format(r1))

        curr_rx_idx = 0
        for i, sdr in enumerate(ssdr):
            for j in range(num_rx_pilots):
                r1 = sdr.readStream(s_rxStream[i], [waveRxA[curr_rx_idx], waveRxB[curr_rx_idx]], symSamp)
                curr_rx_idx = curr_rx_idx + 1
                if debug:
                    print("reading stream non-reference nodes: ({})".format(r1))

        # Timestamps
        if debug:
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
                idx_mat_cal = find_corr_idx(waveRxA_agg, waveRxB_agg)

                # Find most common value at each board
                most_freq = np.zeros(idx_mat_cal.shape[1])
                num_occurr = np.zeros(idx_mat_cal.shape[1])
                for colIdx in range(idx_mat_cal.shape[1]):
                    occurr_cnt = Counter(idx_mat_cal[:, colIdx])
                    most_freq[colIdx] = (occurr_cnt.most_common(1))[0][0]
                    num_occurr[colIdx] = (occurr_cnt.most_common(1))[0][1]

                # Re-assign
                corr_idx_vec_cal = most_freq

                # Check if we didn't meet "PASSING" threshold (i.e., confidence on rx pilots)
                if any(num_occurr < pass_thresh):
                    cleanup([msdr] + ssdr, frame_len, m_rxStream, s_rxStream)
                    flag = -1
                    return flag, corr_idx_vec_cal, idx_mat_cal, num_occurr

                # Perform calibration
                cal_coeff = calibrate(most_freq, ssdr)

        elif idx >= num_cal_tx:
            waveRxA_ver[idx-num_cal_tx, :, :] = waveRxA
            waveRxB_ver[idx-num_cal_tx, :, :] = waveRxB

            if idx == num_cal_tx + num_ver_tx - 1:
                # Find correlation indexes
                idx_mat = find_corr_idx(waveRxA_ver, waveRxB_ver)
                idx_mat_ver = idx_mat

                # Find most common value at each board
                most_freq = np.zeros(idx_mat_ver.shape[1])
                num_occurr = np.zeros(idx_mat_ver.shape[1])
                for colIdx in range(idx_mat_ver.shape[1]):
                    occurr_cnt = Counter(idx_mat_ver[:, colIdx])
                    most_freq[colIdx] = (occurr_cnt.most_common(1))[0][0]
                    num_occurr[colIdx] = (occurr_cnt.most_common(1))[0][1]

                # Re-assign
                corr_idx_vec_ver = most_freq

    cleanup([msdr] + ssdr, frame_len, m_rxStream, s_rxStream)
    return flag, corr_idx_vec_cal, corr_idx_vec_ver, cal_coeff


def find_corr_idx(waveRxA, waveRxB):
    """
    Find correlation indexes
    Input:
        waveRxA - vector of IQ samples from RF chain A
        waveRxB - vector of IQ samples from RF chain B

    Output:
        idx_mat - matrix with correlation indexes at all boards

    """
    idx_mat = np.empty((waveRxA.shape[0], waveRxA.shape[1]))   # Matrix: 2x#radios. two pilots
    idx_mat[:] = np.nan
    lts_thresh = 0.8

    # Iterate over the multiple entries
    for i in range(waveRxA.shape[0]):
        for j in range(waveRxA.shape[1]):
            # Both channels
            best_peakA, all_peaksA, corrA = find_lts(waveRxA[i, j], thresh=lts_thresh, flip=False)
            best_peakB, all_peaksB, corrB = find_lts(waveRxB[i, j], thresh=lts_thresh, flip=False)

            # print("Best peak BOARD: {},  A: {}    and B: {}".format(j, best_peakA, best_peakB))
            # plt.figure(100)
            # plt.plot(corrA)
            # plt.plot(corrB)
            # plt.show()

            # Check if LTS found
            if not best_peakA: # and not best_peakB:
                print("No LTS Found!")
                continue
            if not (best_peakA == best_peakB):
                print("Same board, different indexes. Wut??")

            idx_mat[i, j] = best_peakA # best_peakA if best_peakA else best_peakB
    return idx_mat


def calibrate(corr_idx_vec, ssdr):
    """
    Calibrate

    Find correction factor between reference board and the rest:

    Input:
        corr_idx_vec - vector of absolute correlation indexes for all boards
        ssdr         - sdr objects

    Output:
        cal_coeff    - vector of sample index offsets from reference board

    """
    cal_coeff = corr_idx_vec[0] - corr_idx_vec

    # BY INCREASING ONCE, THE SAMPLE JUMPS (INCREASES) BY ONE IN ALL CASES. E.G., FROM 328 TO 329
    # SIMILARLY BY DECREASING ONCE, THE SAMPLE JUMPS (DECREASES) BY ONE IN ALL CASES. E.G., FROM 326 TO 325
    # ANOTHER OBSERVATION: FIRST ITERATION IS ALWAYS DIFFERENT TO THE REST. THE REST ARE THE SAME THOUGH
    # First board transmitted pilots, second board becomes reference for computing offset, adjustment
    # starts at ssdr[1]
    for i, sdr in enumerate(ssdr[1::]):
        ioffset = int(cal_coeff[i+1])       # ignore reference node
        num_iter = abs(ioffset)
        for j in range(num_iter):
            if ioffset > 0:
                sdr.writeRegister("IRIS30", FPGA_IRIS30_TRIGGERS, FPGA_IRIS30_INCR_TIME)
            elif ioffset < 0:
                sdr.writeRegister("IRIS30", FPGA_IRIS30_TRIGGERS, FPGA_IRIS30_DECR_TIME)

    time.sleep(1)

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


#########################################
#                  Main                 #
#########################################
def main():
    parser = OptionParser()
    parser.add_option("--filename", type="string", dest="filename", help="file containing serial number of all radios in base station", default="./data_in/bs_serial_num.txt")
    parser.add_option("--refNode", type="int", dest="ref_node", help="Index of reference node", default=0)
    parser.add_option("--hubSerial", type="string", dest="hub_serial", help="Serial of Faros HUB", default="")
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--txgain", type="float", dest="txgain", help="Optional Tx gain (dB) w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]", default=30.0)
    parser.add_option("--rxgain", type="float", dest="rxgain", help="Optional Rx gain (dB) w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]", default=30.0)
    parser.add_option("--freq", type="float", dest="freq", help="Optional Tx freq (Hz)", default=3.5e9)
    parser.add_option("--numPilotSamps", type="int", dest="nPilotSamps", help="Num actual pilot samples to tx (len of LTS is 160 samps)", default=160)
    parser.add_option("--prefix-pad", type="int", dest="prefix_length", help="prefix padding length for beacon and pilot", default=100)
    parser.add_option("--postfix-pad", type="int", dest="postfix_length", help="postfix padding length for beacon and pilot", default=100)
    parser.add_option("--debug", action="store_true", dest="debug", help="Debug flag", default=False)
    (options, args) = parser.parse_args()

    # Get all serial numbers for Base Station boards
    serials = []
    with open(options.filename, "r") as f:
        for line in f.read().split():
            serials.append(line)

    # Plotter
    fig = plt.figure(figsize=(20, 8), dpi=120)
    fig.subplots_adjust(hspace=.5, top=.85)
    num_rows = 2
    num_cols = 1

    # Transmit/Receive pilots
    for idxTx in range(1):
        flag = -1
        while flag == -1:
            flag, corr_cal, corr_ver, cal_coeff = txrx_app(
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
                debug=options.debug,
            )
            if flag == -1:
                bad_mat = corr_ver     # returns different vals if fails
                bad_occur = cal_coeff  # returns different vals if fails
                print("==== Calibration failed. Automatic Re-run! ====")
                print("{}".format(bad_mat))
                print("{}".format(bad_occur))
                time.sleep(5)

        cal_offset = corr_cal[0] - corr_cal
        ver_offset = corr_ver[0] - corr_ver

        if any(ver_offset != 0):
            print("**** CALIBRATION FAILED !!! ****")

        # Show results
        if 1:
            print("Cal Matrix: \n {}".format(corr_cal))
            print("Cal Coefficient: \n {}".format(cal_coeff))
            print("Ver Matrix: \n {}".format(corr_ver))

            ax1 = fig.add_subplot(num_rows, num_cols, 1)
            ax1.grid(True)
            ax1.set_title("Offset Before Calibration (indexAll[0] - indexAll)")
            ax1.plot(cal_offset, '--o', alpha=0.7)
            ax1.set_ylim(-5, 5)
            ax1.legend(fontsize=10)

            ax2 = fig.add_subplot(num_rows, num_cols, 2)
            ax2.grid(True)
            ax2.set_title("Offset After Calibration (indexAll[0] - indexAll)")
            ax2.plot(ver_offset, '--o', alpha=0.7)
            ax2.set_ylim(-5, 5)
            ax2.legend(fontsize=10)

            plt.show()


if __name__ == '__main__':
    main()
