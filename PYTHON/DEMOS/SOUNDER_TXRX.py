#!/usr/bin/python
"""
 SOUNDER_TXRX.py

  Basic channel sounding test:
  It program two Irises in TDD mode (one as a base station node,
  and the other as a UE/client node) to transmit and receive according
  to the following schedule/frame:

    Node 1 (BS) schedule PGRGGGGGGGGGGGGGGGRG
    Node 2 (UE) schedule GGPGGGGGGGGGGGGGGGTG

  where the BS is required to first send a beacon signal (initial "P").
  This beacon consists of a signal that has been pre-loaded to the
  BS FPGA buffer. Similarly, the conjugate of the beacon signal has
  been pre-loaded to the UE's buffer in order to enable any correlation
  operation on the UE side.

  The BS node is configured for non-recurring triggers,
  i.e., a trigger is used only for starting frame counting.
  On the other hand, the UE will rely on an FPGA-based correlator
  to trigger itself and count frames. Given the delay between base station
  and UE (group delay at front-end, and RF path delay) there is a
  mechanism for adjusting the frame time using setHardwareTime
  which sets the start symbol and start count within the
  symbol at the time of a correlator trigger. This frame-time also
  accounts for the time advance between the UE and base station
  node.

  The pilot signal (currently a programmable repition of a WiFi LTS)
  transmitted from the UE to the BS is pre-loaded into the FPGA buffer
  and is transmitted in symbol 3 of each frame ("P" in third slot).

  The symbol "T" shown in the second-to-last slot corresponds to
  a sinusoid signal `streamed` from the host to the UE Iris using a
  separate thread. The UE thus transmits this symbol (configured using
  the txSymNum option) right after sending a pilot.

  The "use_trig" flag, allows for two chained Irises to do the same
  operation described above, except that correlator is not
  needed and both nodes are configured for automatic frame counting
  using a non-recurring trigger.

  TDD operation starts with a trigger and runs perpetually
  until it stopped by the user (Ctrl-c). Received pilots and data are
  stored in binary files and they can be inspected using plt_simp.py
  in the IrisUtils folder


  Example:
    python3 SOUNDER_TXRX.py --serial1="RF3C000042" --serial2="RF3C000025"


  Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
             Oscar Bejarano: obejarano@rice.edu

---------------------------------------------------------------------
 Copyright (c) 2018-2019, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
sys.path.append('../IrisUtils/')
sys.path.append('../IrisUtils/data_in/')

import random as rd
import threading
import SoapySDR
from SoapySDR import *  # SOAPY_SDR_ constants
from optparse import OptionParser
import numpy as np
import time
import os
import signal
import math
import pdb
import json
import matplotlib.pyplot as plt
import pickle
import scipy.io as sio 
from functools import partial
from type_conv import *
from file_rdwr import *
from generate_sequence import *


#########################################
#                Registers              #
#########################################
# CORR THRESHOLDING REGS
CORR_THRESHOLD = 92
CORR_RST = 64
#CORR_SCNT = 0x8
CORR_CONF = 60

""" Registers """
# TDD Register Set
RF_RST_REG = 48
TDD_CONF_REG = 120
SCH_ADDR_REG = 136
SCH_MODE_REG = 140
TX_GAIN_CTRL = 88

# AGC registers Set
FPGA_IRIS030_WR_AGC_ENABLE_FLAG = 232
FPGA_IRIS030_WR_AGC_RESET_FLAG = 236
FPGA_IRIS030_WR_IQ_THRESH = 240
FPGA_IRIS030_WR_NUM_SAMPS_SAT = 244
FPGA_IRIS030_WR_MAX_NUM_SAMPS_AGC = 248
FPGA_IRIS030_WR_RSSI_TARGET = 252
FPGA_IRIS030_WR_WAIT_COUNT_THRESH = 256
FPGA_IRIS030_WR_AGC_SMALL_JUMP = 260
FPGA_IRIS030_WR_AGC_BIG_JUMP = 264
FPGA_IRIS030_WR_AGC_TEST_GAIN_SETTINGS = 268
FPGA_IRIS030_WR_AGC_LNA_IN = 272
FPGA_IRIS030_WR_AGC_TIA_IN = 276
FPGA_IRIS030_WR_AGC_PGA_IN = 280

# RSSI register Set
FPGA_IRIS030_RD_MEASURED_RSSI = 284

# Packet Detect Register Set
FPGA_IRIS030_WR_PKT_DET_THRESH = 288
FPGA_IRIS030_WR_PKT_DET_NUM_SAMPS = 292
FPGA_IRIS030_WR_PKT_DET_ENABLE = 296
FPGA_IRIS030_WR_PKT_DET_NEW_FRAME = 300


#########################################
#            Global Parameters          #
#########################################
running = True
record = True
exit_plot = False
bsdr = None
msdr = None
txStreamM = None
rxStreamB = None
rxStreamM = None


#########################################
#              Functions                #
#########################################
def tx_thread(sdr, rate, txStream, rxStream, waveTx, numSamps, numSyms, txSymNum, startSymbol):
    global running
    firstTime = True
    waveRxA = np.array([0]*numSamps, np.uint32)
    waveRxB = np.array([0]*numSamps, np.uint32)
    flags = 0
    sdr.activateStream(txStream)
    sdr.activateStream(rxStream, flags, 0)
    while(running):
        if txSymNum == 0:
            continue
        sr = sdr.readStream(rxStream, [waveRxA, waveRxB], numSamps)
        print("CL: readStream returned %d" % sr.ret)
        if sr.ret > 0:
            txTime = sr.timeNs & 0xFFFFFFFF00000000
            txTime += (0x000000000 + (startSymbol << 16))
            if firstTime:
                print("first receive time 0x%X" % sr.timeNs)
                print("first transmit time 0x%X" % txTime)
                firstTime = False
        else:
            continue

        flags = SOAPY_SDR_HAS_TIME # | SOAPY_SDR_END_BURST
        for j in range(txSymNum):
            txTimeNs = txTime      # SoapySDR.ticksToTimeNs(txTime, rate)
            st = sdr.writeStream(txStream, [waveTx, waveTx], numSamps, flags, timeNs=txTimeNs)
            # sts = sdr.readStreamStatus(txStream)
            txTime += 0x10000
            if st.ret < 0:
                print("ret=%d,flags=%d,timeNs=0x%X,txTime=0x%X" % (st.ret, st.flags, st.timeNs, txTime))
    print("Exiting TX Thread")


def rx_thread(sdr, rxStream, numSamps, txSymNum, both_channels):
    global running
    cwd = os.getcwd()
    fip = open(cwd + '/data_out/rxpilot_sounder.bin', 'wb')
    fid = open(cwd + '/data_out/rxdata_sounder.bin', 'wb')
    if both_channels:
        fip2 = open(cwd + '/data_out/rxpilotB_sounder.bin', 'wb')
        fid2 = open(cwd + '/data_out/rxdataB_sounder.bin', 'wb')
    rxFrNum = 0
    pilotSymNum = 2 if both_channels else 1
    waveRxA = np.array([0]*numSamps, np.uint32)
    waveRxB = np.array([0]*numSamps, np.uint32)
    flags = 0
    r1 = sdr.activateStream(rxStream, flags, 0)
    if r1<0:
        print("Problem activating stream #1")
    while (running):
        readLNA = sdr.getGain(SOAPY_SDR_RX, 0, 'LNA')
        readTIA = sdr.getGain(SOAPY_SDR_RX, 0, 'TIA')
        readPGA = sdr.getGain(SOAPY_SDR_RX, 0, 'PGA')
        # print("LNA: {}, \t TIA:{}, \t PGA:{}".format(readLNA, readTIA, readPGA))
        for j in range(pilotSymNum):
            sr = sdr.readStream(rxStream, [waveRxA, waveRxB], numSamps)
            print("BS: readStream returned %d" % sr.ret)
            if sr.ret < 0 or sr.ret > numSamps:
                print("BS - BAD: readStream returned %d"%sr.ret)
            for i, a in enumerate(waveRxA):
                pickle.dump(a, fip)
                if both_channels: pickle.dump(waveRxB[i], fip2)
        for j in range(txSymNum):
            sr = sdr.readStream(rxStream, [waveRxA, waveRxB], numSamps)
            if sr.ret < 0 or sr.ret > numSamps:
                print("BS: readStream returned %d"%sr.ret)
            for i, a in enumerate(waveRxA):
                pickle.dump(a, fid)
                if both_channels: pickle.dump(waveRxB[i], fid2)
        rxFrNum += 1
    fip.close()
    fid.close()
    if both_channels:
        fip2.close()
        fid2.close()
    sdr.deactivateStream(rxStream)
    sdr.closeStream(rxStream)
    print("Exiting RX Thread, Read %d Frames" % rxFrNum)


def siso_sounder(serial1, serial2, rate, freq, txgain, rxgain, numSamps, numSyms, txSymNum, threshold, tx_advance,
                 prefix_length, postfix_length, both_channels, wait_trigger, calibrate, record, use_trig, auto_tx_gain, agc_en):
    global bsdr, msdr, txStreamM, rxStreamB

    print("setting %s as eNB and %s as UE" % (serial1, serial2))
    bsdr = SoapySDR.Device(dict(serial=serial1))
    msdr = SoapySDR.Device(dict(serial=serial2))

    for i, sdr in enumerate([bsdr, msdr]):
        # AGC SETUP (Init)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_ENABLE_FLAG, 0)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_RESET_FLAG, 1)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_RESET_FLAG, 0)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_IQ_THRESH, 10300)        # 10300 about -6dBm
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_NUM_SAMPS_SAT, 3)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_MAX_NUM_SAMPS_AGC, 20)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_WAIT_COUNT_THRESH, 160)  # gain settle takes about 20 samps (val=20)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_RSSI_TARGET, 14)         # ideally around 14
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_SMALL_JUMP, 5)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_BIG_JUMP, 15)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_TEST_GAIN_SETTINGS, 0)

        # PACKET DETECT SETUP
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_THRESH, 0)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NUM_SAMPS, 5)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_ENABLE, 0)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NEW_FRAME, 0)

    # Some default sample rates
    for i, sdr in enumerate([bsdr, msdr]):
        info = sdr.getHardwareInfo()
        print("%s settings on device %d" % (info["frontend"], i))
        for ch in [0, 1]:
            sdr.setSampleRate(SOAPY_SDR_TX, ch, rate)
            sdr.setSampleRate(SOAPY_SDR_RX, ch, rate)
            # sdr.setFrequency(SOAPY_SDR_TX, ch, freq)
            # sdr.setFrequency(SOAPY_SDR_RX, ch, freq)
            sdr.setFrequency(SOAPY_SDR_TX, ch, 'RF', freq-.75*rate)
            sdr.setFrequency(SOAPY_SDR_RX, ch, 'RF', freq-.75*rate)
            sdr.setFrequency(SOAPY_SDR_TX, ch, 'BB', .75*rate)
            sdr.setFrequency(SOAPY_SDR_RX, ch, 'BB', .75*rate)
            if "CBRS" in info["frontend"]:
                sdr.setGain(SOAPY_SDR_TX, ch, 'ATTN', -6)  # {-18,-12,-6,0}
            sdr.setGain(SOAPY_SDR_TX, ch, 'PAD', txgain)   # [0,52]

            if "CBRS" in info["frontend"]:
                sdr.setGain(SOAPY_SDR_RX, ch, 'ATTN', -6)   # {-18,-12,-6,0}
                sdr.setGain(SOAPY_SDR_RX, ch, 'LNA2', 14)  # LO: [0|17], HI:[0|14]

            # LMS gains
            if agc_en:
                # Set gains to max (initially)
                sdr.setGain(SOAPY_SDR_RX, ch, 'LNA', 30)       # [0,30]
                sdr.setGain(SOAPY_SDR_RX, ch, 'TIA', 12)       # [0,12]
                sdr.setGain(SOAPY_SDR_RX, ch, 'PGA', 19)       # [-12,19]
            else:
                sdr.setGain(SOAPY_SDR_RX, ch, 'LNA', rxgain)   # [0,30]
                sdr.setGain(SOAPY_SDR_RX, ch, 'TIA', 0)       # [0,12]
                sdr.setGain(SOAPY_SDR_RX, ch, 'PGA', 0)       # [-12,19]

            sdr.setAntenna(SOAPY_SDR_RX, ch, "TRX")
            sdr.setDCOffsetMode(SOAPY_SDR_RX, ch, True)

            # Read initial gain settings
            readLNA = sdr.getGain(SOAPY_SDR_RX, 0, 'LNA')
            readTIA = sdr.getGain(SOAPY_SDR_RX, 0, 'TIA')
            readPGA = sdr.getGain(SOAPY_SDR_RX, 0, 'PGA')
            print("INITIAL GAIN - LNA: {}, \t TIA:{}, \t PGA:{}".format(readLNA, readTIA, readPGA))

        for ch in [0, 1]:
            if calibrate:
                sdr.writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", 'SKLK')
                sdr.writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", '')

        sdr.writeRegister("IRIS30", RF_RST_REG, (1 << 29) | 0x1)
        sdr.writeRegister("IRIS30", RF_RST_REG, (1 << 29))
        sdr.writeRegister("IRIS30", RF_RST_REG, 0)

    # pdb.set_trace()
    msdr.writeRegister("IRIS30", TX_GAIN_CTRL, 0)  
    if use_trig:
        bsdr.writeSetting("SYNC_DELAYS", "")
        # bsdr.writeSetting("FPGA_DIQ_MODE", "PATTERN")

    # Packet size
    symSamp = numSamps + prefix_length + postfix_length
    print("numSamps = %d" % symSamp)
    print("txSymNum = %d" % txSymNum)
    upsample = 1
    Ts = 1/rate
    s_freq = 1e6
    s_time_vals = np.array(np.arange(0, numSamps)).transpose()*Ts
    nb_data = np.exp(s_time_vals*1j*2*np.pi*s_freq).astype(np.complex64)*.25

    # Create streams
    rxStreamM = msdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1])  
    txStreamM = msdr.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, [0, 1])  
    if record:
        rxStreamB = bsdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1])

    for i, sdr in enumerate([bsdr, msdr]):
        # ENABLE PKT DETECT AND AGC
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_ENABLE, agc_en)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_ENABLE_FLAG, agc_en)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NEW_FRAME, 1)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NEW_FRAME, 0)

    # preambles to be sent from BS and correlated against in UE
    # the base station may upsample, but the mobiles won't
    preambles_bs = generate_training_seq(preamble_type='gold_ifft', seq_length=128, cp=0, upsample=1)
    # the correlators can run at lower rates, so we only need the downsampled signal.
    preambles = preambles_bs[:, ::upsample]

    ampl = 0.5
    beacon = preambles[0, :]
    coe = cfloat2uint32(np.conj(beacon), order='QI')     # FPGA correlator takes coefficients in QI order
    ltsSym, lts_f = generate_training_seq(preamble_type='lts', cp=32, upsample=1)
    # ltsSym = lts.genLTS(upsample=1, cp=0)
    pad1 = np.array([0]*(prefix_length), np.complex64)   # to comprensate for front-end group delay
    pad2 = np.array([0]*(postfix_length), np.complex64)  # to comprensate for rf path delay
    wb_pilot = np.tile(ltsSym, numSamps//len(ltsSym)).astype(np.complex64)*ampl
    wbz = np.array([0]*(symSamp), np.complex64)
    wb_pilot1 = np.concatenate([pad1, wb_pilot, pad2])
    wb_pilot2 = wbz  # wb_pilot1 if both_channels else wbz
    bcnz = np.array([0]*(symSamp-prefix_length-len(beacon)), np.complex64)  
    beacon1 = np.concatenate([pad1, beacon*ampl, bcnz])
    beacon2 = wbz  # beacon1 if both_channels else wbz

    bsched = "PGR"+''.join("G"*(numSyms-txSymNum-4))+''.join("R"*txSymNum)+"G" 
    msched = "GRP"+''.join("G"*(numSyms-txSymNum-4))+''.join("T"*txSymNum)+"G"
    if both_channels:
        bsched = "PGRR"+''.join("G"*(numSyms-txSymNum-5))+''.join("R"*txSymNum)+"G" 
        msched = "GGPP"+''.join("G"*(numSyms-txSymNum-5))+''.join("T"*txSymNum)+"G"
    print("Node 1 schedule %s " % bsched) 
    print("Node 2 schedule %s " % msched)
    bconf = {"tdd_enabled": True, "frame_mode": "free_running", "symbol_size": symSamp, "frames": [bsched]}
    mconf = {"tdd_enabled": True, "frame_mode": "free_running" if use_trig else "triggered" if wait_trigger else "continuous_resync", "dual_pilot": both_channels, "symbol_size" : symSamp, "frames": [msched]}
    bsdr.writeSetting("TDD_CONFIG", json.dumps(bconf))
    msdr.writeSetting("TDD_CONFIG", json.dumps(mconf))

    for sdr in [bsdr, msdr]:
        sdr.writeSetting("TX_SW_DELAY", str(30))

    if not use_trig:
        msdr.writeRegister("IRIS30", CORR_CONF, int("00004001", 16))  # enable the correlator, with zeros as inputs
        for i in range(128):
            msdr.writeRegister("ARGCOE", i*4, 0)
        time.sleep(0.1)
        #msdr.writeRegister("ARGCOR", CORR_THRESHOLD, int(threshold))
        #msdr.writeRegister("ARGCOR", CORR_RST, 0x1)  # reset corr
        #msdr.writeRegister("ARGCOR", CORR_RST, 0x0)  # unrst corr
        msdr.writeRegister("IRIS30", CORR_RST, 0x1)  # reset corr
        msdr.writeRegister("IRIS30", CORR_RST, 0x0)  # unrst corr
        msdr.writeRegister("IRIS30", CORR_THRESHOLD, int(np.log2(threshold)))
        for i in range(128):
            msdr.writeRegister("ARGCOE", i*4, int(coe[i]))
        if auto_tx_gain:
            max_gain = int(txgain)
            min_gain = max(0, max_gain-15)
            gain_reg = 0xF000 | (max_gain & 0x3F) << 6 | (min_gain & 0x3F)
            print("gain reg 0x%X" % gain_reg)
            # [15] en, [14] mode, [13:12] step, [11:6] stop, [5:0] start
            msdr.writeRegister("IRIS30", TX_GAIN_CTRL, gain_reg)

        # DEV: ueTrigTime = 153 (prefix_length=0), CBRS: ueTrigTime = 235 (prefix_length=82), tx_advance=prefix_length,
        # corr delay is 17 cycles
        ueTrigTime = prefix_length + len(beacon) + postfix_length + 17 + tx_advance 
        sf_start = ueTrigTime // symSamp
        sp_start = ueTrigTime % symSamp
        print("UE starting symbol and sample count (%d, %d)" % (sf_start, sp_start))
        # make sure to set this after TDD mode is enabled "writeSetting("TDD_CONFIG", ..."
        msdr.setHardwareTime(SoapySDR.ticksToTimeNs((sf_start << 16) | sp_start, rate), "TRIGGER")

    msdr.writeSetting("TDD_MODE", "true")
    bsdr.writeSetting("TDD_MODE", "true")

    replay_addr = 0
    bsdr.writeRegisters("TX_RAM_A", replay_addr, cfloat2uint32(beacon1, order='QI').tolist())
    bsdr.writeRegisters("TX_RAM_B", replay_addr, cfloat2uint32(beacon2, order='QI').tolist())

    msdr.writeRegisters("TX_RAM_A", replay_addr, cfloat2uint32(wb_pilot1, order='QI').tolist())
    msdr.writeRegisters("TX_RAM_B", replay_addr, cfloat2uint32(wbz, order='QI').tolist())
    if both_channels:
        msdr.writeRegisters("TX_RAM_A", replay_addr+2048, cfloat2uint32(wbz, order='QI').tolist())
        msdr.writeRegisters("TX_RAM_B", replay_addr+2048, cfloat2uint32(wb_pilot2, order='QI').tolist())

    if not use_trig:    
        msdr.writeRegister("IRIS30", CORR_CONF, int("00004011", 16))  # enable the correlator, with inputs from adc

    signal.signal(signal.SIGINT, partial(signal_handler, rate, numSyms, txSymNum))
    bsdr.writeSetting("TRIGGER_GEN", "")
    txth = threading.Thread(target=tx_thread, args=(msdr, rate, txStreamM, rxStreamM, nb_data, symSamp, numSyms, txSymNum, numSyms-txSymNum-1))
    txth.start()
    if record:
        rxth = threading.Thread(target=rx_thread, args=(bsdr, rxStreamB, symSamp, txSymNum, both_channels))
        rxth.start()
    #signal.pause()
    num_trig = 0
    while True:
        time.sleep(1)
        t = SoapySDR.timeNsToTicks(msdr.getHardwareTime(""),rate) >> 32 #trigger count is top 32 bits.
        print("%d new triggers, %d total" % (t - num_trig, t))
        num_trig = t


def signal_handler(rate, numSyms, txSymNum, signal, frame):
    global bsdr, msdr, running, txStreamM, rxStreamB, exit_plot
    msdr.writeRegister("IRIS30", CORR_CONF, 0)  # stop mobile correlator first, to prevent from the tdd manager going
    msdr.writeRegister("IRIS30", TX_GAIN_CTRL, 0)  
    # stop tx/rx threads
    running = False

    print("printing number of frames")
    print("NB 0x%X" % SoapySDR.timeNsToTicks(bsdr.getHardwareTime(""), rate))
    print("UE 0x%X" % SoapySDR.timeNsToTicks(msdr.getHardwareTime(""), rate))
    # print("UE SCNT: 0x%X" % msdr.readRegister("ARGCOR", CORR_SCNT))
    # ADC_rst, stops the tdd time counters
    for sdr in [bsdr, msdr]:
        sdr.writeRegister("IRIS30", RF_RST_REG, (1<<29)| 0x1)
        sdr.writeRegister("IRIS30", RF_RST_REG, (1<<29))
        sdr.writeRegister("IRIS30", RF_RST_REG, 0)
    for i in range(numSyms):
        msdr.writeRegister("RFCORE", SCH_ADDR_REG, i)  # subframe 0
        msdr.writeRegister("RFCORE", SCH_MODE_REG, 0)  # 01 replay
        bsdr.writeRegister("RFCORE", SCH_ADDR_REG, i)  # subframe 0
        bsdr.writeRegister("RFCORE", SCH_MODE_REG, 0)  # 01 replay
    bsdr.writeRegister("RFCORE", TDD_CONF_REG, 0) 
    msdr.writeRegister("RFCORE", TDD_CONF_REG, 0) 
    msdr.writeSetting("TDD_MODE", "false")
    bsdr.writeSetting("TDD_MODE", "false")

    for i, sdr in enumerate([bsdr, msdr]):
        # Reset
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_ENABLE_FLAG, 0)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_RESET_FLAG, 0)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_RESET_FLAG, 0)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_IQ_THRESH, 0)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_NUM_SAMPS_SAT, 0)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_MAX_NUM_SAMPS_AGC, 0)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_WAIT_COUNT_THRESH, 0)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_RSSI_TARGET, 0)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_SMALL_JUMP, 0)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_BIG_JUMP, 0)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_TEST_GAIN_SETTINGS, 0)

        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_THRESH, 0)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NUM_SAMPS, 5)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_ENABLE, 0)
        sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NEW_FRAME, 0)

    bsdr = None
    msdr = None

    if exit_plot:
        fig_len = 256
        pilot = np.zeros(fig_len)
        rxdata = np.zeros(fig_len)
        fig = plt.figure(figsize=(20, 8), dpi=100)
        ax1 = fig.add_subplot(2, 1, 1)
        ax2 = fig.add_subplot(2, 1, 2)
        pilot = uint32tocfloat(read_from_file("data_out/rxpilot_sounder", leng=fig_len, offset=0))
        ax1.plot(np.real(pilot), label='pilot i')
        ax1.plot(np.imag(pilot), label='pilot q')

        if txSymNum > 0:
            rxdata = uint32tocfloat(read_from_file("data_out/rxdata_sounder", leng=fig_len, offset=0))
        ax2.plot(np.real(rxdata), label='rx data i')
        ax2.plot(np.imag(rxdata), label='rx data q')

        plt.show()
    sys.exit(0)


#########################################
#                  Main                 #
#########################################
def main():
    parser = OptionParser()
    parser.add_option("--serial1", type="string", dest="serial1", help="serial number of the master device", default="RF3C000042")
    parser.add_option("--serial2", type="string", dest="serial2", help="serial number of the slave device", default="RF3C000025")
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--txgain", type="float", dest="txgain", help="Optional Tx gain (dB)", default=30.0)
    parser.add_option("--rxgain", type="float", dest="rxgain", help="Optional Rx gain (dB) - only used if agc disabled", default=20.0)
    parser.add_option("--freq", type="float", dest="freq", help="Optional Tx freq (Hz)", default=3.6e9)
    parser.add_option("--numSamps", type="int", dest="numSamps", help="Num samples to receive", default=512)
    parser.add_option("--prefix-length", type="int", dest="prefix_length", help="prefix padding length for beacon and pilot", default=82)     # to compensate for front-end group delay
    parser.add_option("--postfix-length", type="int", dest="postfix_length", help="postfix padding length for beacon and pilot", default=68)  # to compensate for rf path delay
    parser.add_option("--numSyms", type="int", dest="numSyms", help="Number of symbols in one sub-frame", default=20)
    parser.add_option("--txSymNum", type="int", dest="txSymNum", help="Number of tx sub-frames in one frame", default=0)
    parser.add_option("--corr-threshold", type="int", dest="threshold", help="Correlator Threshold Value", default=2)
    parser.add_option("--ue-tx-advance", type="int", dest="tx_advance", help="sample advance for tx vs rx", default=68)
    parser.add_option("--both-channels", action="store_true", dest="both_channels", help="transmit from both channels", default=False)
    parser.add_option("--calibrate", action="store_true", dest="calibrate", help="transmit from both channels", default=False)
    parser.add_option("--use-trig", action="store_true", dest="use_trig", help="uses chain triggers for synchronization", default=True)
    parser.add_option("--wait-trigger", action="store_true", dest="wait_trigger", help="wait for a trigger to start a frame", default=False)
    parser.add_option("--auto-tx-gain", action="store_true", dest="auto_tx_gain", help="automatically go over tx gains", default=False)
    parser.add_option("--record", action="store_true", dest="record", help="record received pilots and data", default=True)
    parser.add_option("--agc-enable", action="store_true", dest="agc_en", help="Enable AGC flag", default=False)
    (options, args) = parser.parse_args()

    siso_sounder(
        serial1=options.serial1,
        serial2=options.serial2,
        rate=options.rate,
        freq=options.freq,
        txgain=options.txgain,
        rxgain=options.rxgain,
        numSamps=options.numSamps,
        numSyms=options.numSyms,
        txSymNum=options.txSymNum,
        threshold=options.threshold,
        tx_advance=options.tx_advance,
        prefix_length=options.prefix_length,
        postfix_length=options.postfix_length,
        both_channels=options.both_channels,
        wait_trigger=options.wait_trigger,
        calibrate=options.calibrate,
        record=options.record,
        auto_tx_gain=options.auto_tx_gain,
        use_trig=options.use_trig,
        agc_en=options.agc_en,
    )


if __name__ == '__main__':
    main()
 
