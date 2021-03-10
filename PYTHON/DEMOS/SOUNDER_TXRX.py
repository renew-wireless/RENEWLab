#!/usr/bin/python3
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
    python3 SOUNDER_TXRX.py --bsnode="RF3C000042" --clnode="RF3C000025"

  where bsnode corresponds to a base station node and clnode corresponds
  to a client node.

  OUTPUT:
  The script will generate binary files that can be analyzed using the
  plt_simp.py script in folder PYTHON/IrisUtils/

  IMPORTANT NOTE:
  The client firmware has different features than the base station node
  firmware. Therefore, ONLY bsnode can transmit a beacon whereas ONLY
  cnode can correlate against such beacon. This means it is critical to
  set bsnode to the serial number of a base station node and clnode to the 
  serial number of a client node!

  NOTE ON GAINS:
  Gain settings will vary depending on RF frontend board being used
  If using CBRS:
  rxgain: at 2.5GHz [3:1:105], at 3.6GHz [3:1:102]
  txgain: at 2.5GHz [16:1:90], at 3.6GHz [15:1:95]

  If using only Dev Board:
  rxgain: at both frequency bands [0:1:30]
  txgain: at both frequency bands [0:1:42]

  The code assumes both TX and RX have the same type of RF frontend board.
  Also, currently AGC only supports the CBRS RF frontend. It cannot be used
  with the Iris Dev Board or UHF board

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
import pickle
import scipy.io as sio 
from functools import partial
from type_conv import *
from file_rdwr import *
from generate_sequence import *


#########################################
#            Global Parameters          #
#########################################
running = True
record = True
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
        #print("CL: readStream returned %d" % sr.ret)
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
        for j in range(pilotSymNum):
            sr = sdr.readStream(rxStream, [waveRxA, waveRxB], numSamps)
            #print("BS: readStream returned %d" % sr.ret)
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


def siso_sounder(hub, serial1, serial2, rate, freq, txgain, rxgain, numSamps, numSyms, txSymNum, threshold, tx_advance,
                 prefix_length, postfix_length, both_channels, wait_trigger, calibrate, record, use_trig, tx_power_loop, agc_en):
    global bsdr, msdr, txStreamM, rxStreamB

    print("setting %s as eNB and %s as UE" % (serial1, serial2))
    bsdr = SoapySDR.Device(dict(serial=serial1))
    msdr = SoapySDR.Device(dict(serial=serial2))
    if hub != "":
        trig_dev = SoapySDR.Device(dict(serial=hub, driver="remote"))
    else:
        trig_dev = bsdr

    # Some default sample rates
    for i, sdr in enumerate([bsdr, msdr]):
        info = sdr.getHardwareInfo()
        print("%s settings on device %d" % (info["frontend"], i))
        for ch in [0, 1]:
            sdr.setBandwidth(SOAPY_SDR_TX, ch, 2.5*rate)
            sdr.setBandwidth(SOAPY_SDR_RX, ch, 2.5*rate)
            sdr.setSampleRate(SOAPY_SDR_TX, ch, rate)
            sdr.setSampleRate(SOAPY_SDR_RX, ch, rate)
            #sdr.setFrequency(SOAPY_SDR_TX, ch, freq)
            #sdr.setFrequency(SOAPY_SDR_RX, ch, freq)
            sdr.setFrequency(SOAPY_SDR_TX, ch, 'RF', freq-.75*rate)
            sdr.setFrequency(SOAPY_SDR_RX, ch, 'RF', freq-.75*rate)
            sdr.setFrequency(SOAPY_SDR_TX, ch, 'BB', .75*rate)
            sdr.setFrequency(SOAPY_SDR_RX, ch, 'BB', .75*rate)
            sdr.setAntenna(SOAPY_SDR_RX, ch, "TRX")
            sdr.setDCOffsetMode(SOAPY_SDR_RX, ch, True)

            if "CBRS" in info["frontend"]:
                # Set gains to high val (initially)
                if agc_en: rxgain = 100
                sdr.setGain(SOAPY_SDR_TX, ch, txgain)
                sdr.setGain(SOAPY_SDR_RX, ch, rxgain)
            else:
                # No CBRS board gains, only changing LMS7 gains
                # AGC only supported for CBRS boards
                agc_en = False
                sdr.setGain(SOAPY_SDR_TX, ch, "PAD", txgain)    # [0:1:42]
                sdr.setGain(SOAPY_SDR_TX, ch, "IAMP", 0)        # [-12:1:3]
                sdr.setGain(SOAPY_SDR_RX, ch, "LNA", rxgain)    # [0:1:30]
                sdr.setGain(SOAPY_SDR_RX, ch, "TIA", 0)         # [0, 3, 9, 12]
                sdr.setGain(SOAPY_SDR_RX, ch, "PGA", -10)       # [-12:1:19]

            # Read initial gain settings
            readLNA = sdr.getGain(SOAPY_SDR_RX, 0, 'LNA')
            readTIA = sdr.getGain(SOAPY_SDR_RX, 0, 'TIA')
            readPGA = sdr.getGain(SOAPY_SDR_RX, 0, 'PGA')
            print("INITIAL GAIN - LNA: {}, \t TIA:{}, \t PGA:{}".format(readLNA, readTIA, readPGA))

        for ch in [0, 1]:
            if calibrate:
                sdr.writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", 'SKLK')

        sdr.writeSetting("RESET_DATA_LOGIC", "")

    # pdb.set_trace()
    if use_trig:
        trig_dev.writeSetting("SYNC_DELAYS", "")

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

    tpc_conf = {"tpc_enabled" : False}
    msdr.writeSetting("TPC_CONFIG", json.dumps(tpc_conf))
    agc_conf = {"agc_enabled" : agc_en}
    msdr.writeSetting("AGC_CONFIG", json.dumps(agc_conf))

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

    if both_channels:
        beacon_weights = hadamard(2)
    else:
        beacon_weights = np.eye(2, dtype=np.uint32)
    beacon_weights = beacon_weights.astype(np.uint32)

    bsched = "BGR"+''.join("G"*(numSyms-txSymNum-4))+''.join("R"*txSymNum)+"G"
    msched = "GGP"+''.join("G"*(numSyms-txSymNum-4))+''.join("T"*txSymNum)+"G"
    if both_channels:
        bsched = "BGRR"+''.join("G"*(numSyms-txSymNum-5))+''.join("R"*txSymNum)+"G"
        msched = "GGPP"+''.join("G"*(numSyms-txSymNum-5))+''.join("T"*txSymNum)+"G"

    print("Iris 1 schedule %s " % bsched) 
    print("Iris 2 schedule %s " % msched)

    bconf = {"tdd_enabled": True,
             "frame_mode": "free_running",
             "symbol_size": symSamp,
             "frames": [bsched],
             "beacon_start" : prefix_length,
             "beacon_stop" : prefix_length+len(beacon),
             "max_frame" : 0}
    mconf = {"tdd_enabled": True,
             "frame_mode": "free_running" if use_trig else "triggered" if wait_trigger else "continuous_resync",
             "dual_pilot": both_channels,
             "symbol_size" : symSamp,
             "frames": [msched],
             "max_frame" : 0}

    bsdr.writeSetting("TDD_CONFIG", json.dumps(bconf))
    msdr.writeSetting("TDD_CONFIG", json.dumps(mconf))

    bsdr.writeRegisters("BEACON_RAM", 0, cfloat2uint32(beacon, order='QI').tolist())
    bsdr.writeRegisters("BEACON_RAM_WGT_A", 0, beacon_weights[0].tolist())
    bsdr.writeRegisters("BEACON_RAM_WGT_B", 0, beacon_weights[1].tolist())
    numAnt = 2 if both_channels else 1
    bsdr.writeSetting("BEACON_START", str(numAnt))

    for sdr in [bsdr, msdr]:
        sdr.writeSetting("TX_SW_DELAY", str(30))
        sdr.writeSetting("TDD_MODE", "true")

    if not use_trig:
        corr_conf = {"corr_enabled" : True, "corr_threshold" : threshold}
        msdr.writeSetting("CORR_CONFIG", json.dumps(corr_conf))
        msdr.writeRegisters("CORR_COE", 0, coe.tolist())

        # DEV: ueTrigTime = 153 (prefix_length=0), CBRS: ueTrigTime = 235 (prefix_length=82), tx_advance=prefix_length,
        # corr delay is 17 cycles
        ueTrigTime = len(beacon) + 200 # prefix_length + len(beacon) + postfix_length + 17 + tx_advance + 150
        sf_start = ueTrigTime // symSamp
        sp_start = ueTrigTime % symSamp
        print("UE starting symbol and sample count (%d, %d)" % (sf_start, sp_start))
        # make sure to set this after TDD mode is enabled "writeSetting("TDD_CONFIG", ..."
        msdr.setHardwareTime(SoapySDR.ticksToTimeNs((sf_start << 16) | sp_start, rate), "TRIGGER")

    if tx_power_loop:
        tcp_conf = {"tpc_enabled" : True,
                    "max_gain" : int(tx_gain),
                    "min_gain" : max(0, max_gain-12)}
        msdr.writeSetting("TPC_CONFIG", json.dumps(tpc_conf))

    msdr.writeRegisters("TX_RAM_A", 0, cfloat2uint32(wb_pilot1, order='QI').tolist())
    if both_channels:
        msdr.writeRegisters("TX_RAM_B", 0, cfloat2uint32(wb_pilot2, order='QI').tolist())

    if not use_trig:
        msdr.writeSetting("CORR_START", "A")

    signal.signal(signal.SIGINT, partial(signal_handler, rate, numSyms, use_trig))
    trig_dev.writeSetting("TRIGGER_GEN", "")
    txth = threading.Thread(target=tx_thread, args=(msdr, rate, txStreamM, rxStreamM, nb_data, symSamp, numSyms, txSymNum, numSyms-txSymNum-1))
    txth.start()
    if record:
        rxth = threading.Thread(target=rx_thread, args=(bsdr, rxStreamB, symSamp, txSymNum, both_channels))
        rxth.start()
    num_trig = 0
    while True:
        time.sleep(1)
        t = SoapySDR.timeNsToTicks(msdr.getHardwareTime(""),rate) >> 32 #trigger count is top 32 bits.
        print("%d new triggers, %d total" % (t - num_trig, t))
        num_trig = t


def signal_handler(rate, numSyms, use_trig, signal, frame):
    global bsdr, msdr, running, txStreamM, rxStreamB
    corr_conf = {"corr_enabled" : False}
    if not use_trig:
        msdr.writeSetting("CORR_CONFIG", json.dumps(corr_conf))
    tpc_conf = {"tpc_enabled" : False}
    msdr.writeSetting("TPC_CONFIG", json.dumps(tpc_conf))
    # stop tx/rx threads
    running = False

    print("printing number of frames")
    print("NB 0x%X" % SoapySDR.timeNsToTicks(bsdr.getHardwareTime(""), rate))
    print("UE 0x%X" % SoapySDR.timeNsToTicks(msdr.getHardwareTime(""), rate))
    # ADC_rst, stops the tdd time counters
    tdd_conf = {"tdd_enabled" : False}
    for sdr in [bsdr, msdr]:
        sdr.writeSetting("RESET_DATA_LOGIC", "")
        sdr.writeSetting("TDD_CONFIG", json.dumps(tdd_conf))
        sdr.writeSetting("TDD_MODE", "false")

    agc_conf = {"agc_enabled" : False}
    msdr.writeSetting("AGC_CONFIG", json.dumps(agc_conf))

    bsdr = None
    msdr = None

    sys.exit(0)


#########################################
#                  Main                 #
#########################################
def main():
    parser = OptionParser()
    parser.add_option("--bsnode", type="string", dest="bsnode", help="serial number of the master (base station node) device", default="RF3E000103")
    parser.add_option("--clnode", type="string", dest="clnode", help="serial number of the slave (client node) device", default="RF3E000157")
    parser.add_option("--hub", type="string", dest="hub", help="Hub node", default="")
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--txgain", type="float", dest="txgain", help="Tx gain (dB)", default=80.0)  # Check top of file for info on gain range
    parser.add_option("--rxgain", type="float", dest="rxgain", help="Rx gain (dB)", default=70.0)  # Check top of file for info on gain range
    parser.add_option("--freq", type="float", dest="freq", help="Optional Tx freq (Hz)", default=0)
    parser.add_option("--numSamps", type="int", dest="numSamps", help="Num samples to receive", default=512)
    parser.add_option("--prefix-length", type="int", dest="prefix_length", help="prefix padding length for beacon and pilot", default=100)     # to compensate for front-end group delay
    parser.add_option("--postfix-length", type="int", dest="postfix_length", help="postfix padding length for beacon and pilot", default=100)  # to compensate for rf path delay
    parser.add_option("--numSyms", type="int", dest="numSyms", help="Number of symbols in one sub-frame", default=20)
    parser.add_option("--txSymNum", type="int", dest="txSymNum", help="Number of tx sub-frames in one frame", default=0)
    parser.add_option("--corr-threshold", type="int", dest="threshold", help="Correlator Threshold Value", default=1)
    parser.add_option("--ue-tx-advance", type="int", dest="tx_advance", help="sample advance for tx vs rx", default=68)
    parser.add_option("--both-channels", action="store_true", dest="both_channels", help="transmit from both channels", default=False)
    parser.add_option("--calibrate", action="store_true", dest="calibrate", help="transmit from both channels", default=False)
    parser.add_option("--use-trig", action="store_true", dest="use_trig", help="uses chain triggers for synchronization", default=False)
    parser.add_option("--wait-trigger", action="store_true", dest="wait_trigger", help="wait for a trigger to start a frame", default=False)
    parser.add_option("--tx-power-loop", action="store_true", dest="tx_power_loop", help="loop over a set of tx gains in consecutive frames", default=False)
    parser.add_option("--record", action="store_true", dest="record", help="record received pilots and data", default=True)
    parser.add_option("--agc-enable", action="store_true", dest="agc_en", help="Enable AGC flag", default=False)
    (options, args) = parser.parse_args()

    if options.freq == 0:
        print("[ERROR] Please provide RF Freq (Hz). POWDER users must set to 2.5e9")
        exit(0)

    siso_sounder(
        hub=options.hub,
        serial1=options.bsnode,
        serial2=options.clnode,
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
        tx_power_loop=options.tx_power_loop,
        use_trig=options.use_trig,
        agc_en=options.agc_en,
    )


if __name__ == '__main__':
    main()
 
