#!/usr/bin/python
"""
    SISO_TXRX_TDD.py

    NOTE: IRIS BOARDS MUST BE CHAINED FOR THIS SCRIPT TO WORK.
    ORDER MATTERS; FIRST BOARD (SERIAL1) IS THE ONE SENDING THE TRIGGER.
    TESTED WITH BOTH BOARDS USING BASE STATION ROOTFS IMAGE (NOT UE) AND
    ONLY FIRST BOARD CONNECTED TO HOST VIA ETHERNET

    This script is useful for testing the TDD operation.
    It programs two Irises in TDD mode with the following framing
    schedule:
        Iris 1: PGRG
        Iris 2: RGPG

    where P means a pilot or a pre-loaded signal, G means Guard
    band (no Tx or Rx action), R means Rx, and T means Tx,
    though not used in this script.

    The above determines the operation for each frame and each
    letter determines one symbol. Although every 16 consecutive
    frames can be scheduled separately.
    The pilot signal in this case is a sinusoid which is written
    into FPGA buffer (TX_RAM_A & TX_RAM_B for channels A & B)
    before the start trigger.

    The script programs the Irises in a one-shot mode, i.e.
    they run for just one frame. This means that each frame starts
    with a separate trigger. After the end of the frame,
    the script plots the two Rx symbols which are supposedly
    what each of the Iris boards received from each other (as shown
    in the schedule above).

    NOTE ON GAINS:
    Gain settings will vary depending on RF frontend board being used
    If using CBRS:
    rxgain: at 2.5GHz [3:1:105], at 3.6GHz [3:1:102]
    txgain: at 2.5GHz [16:1:90], at 3.6GHz [15:1:95]

    If using only Dev Board:
    rxgain: at both frequency bands [0:1:30]
    txgain: at both frequency bands [0:1:42]

    The code assumes both TX and RX have the same type of RF frontend board.

    Example:
        python3 SISO_TXRX_TDD.py --serial1="RF3C000042" --serial2="RF3C000025"

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
import numpy as np
import time
import os
import math
import json
import matplotlib.pyplot as plt
from type_conv import *
from macros import *


#########################################
#              Functions                #
#########################################
def siso_tdd_burst(serial1, serial2, rate, freq, txgain, rxgain, numSamps, prefix_pad, postfix_pad):
    bsdr = SoapySDR.Device(dict(driver='iris', serial=serial1))
    msdr = SoapySDR.Device(dict(driver='iris', serial=serial2))

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
                sdr.setGain(SOAPY_SDR_TX, ch, txgain)  # txgain: at 2.5GHz [16:1:93], at 3.6GHz [15:1:102]
                sdr.setGain(SOAPY_SDR_RX, ch, rxgain)  # rxgain: at 2.5GHz [3:1:105], at 3.6GHz [3:1:102]
            else:
                # No CBRS board gains, only changing LMS7 gains
                sdr.setGain(SOAPY_SDR_TX, ch, "PAD", txgain) # [0:1:42] txgain
                sdr.setGain(SOAPY_SDR_TX, ch, "ATTN", -6)
                sdr.setGain(SOAPY_SDR_RX, ch, "LNA", rxgain) # [0:1:30] rxgain
                sdr.setGain(SOAPY_SDR_RX, ch, "LNA2", 14)
                sdr.setGain(SOAPY_SDR_RX, ch, "ATTN", 0 if freq > 3e9 else -18)

    # SYNC_DELAYS
    bsdr.writeSetting("SYNC_DELAYS", "")

    # Packet size
    symSamp = numSamps + prefix_pad + postfix_pad
    print("numSamps = %d" % numSamps)
    print("symSamps = %d" % symSamp)

    # Generate sinusoid to be TX
    Ts = 1 / rate
    s_freq = 1e5
    s_time_vals = np.array(np.arange(0, numSamps)).transpose()*Ts
    pilot = np.exp(s_time_vals*1j*2*np.pi*s_freq).astype(np.complex64)*1
    pad1 = np.array([0]*prefix_pad, np.complex64)
    pad2 = np.array([0]*postfix_pad, np.complex64)
    wbz = np.array([0]*symSamp, np.complex64)
    pilot1 = np.concatenate([pad1, pilot, pad2])
    pilot2 = wbz

    # Initialize RX arrays
    waveRxA1 = np.array([0]*symSamp, np.uint32)
    waveRxB1 = np.array([0]*symSamp, np.uint32)
    waveRxA2 = np.array([0]*symSamp, np.uint32)
    waveRxB2 = np.array([0]*symSamp, np.uint32)

    # Create RX streams
    # CS16 makes sure the 4-bit lsb are samples are being sent
    rxStreamB = bsdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1])
    rxStreamM = msdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1])

    # Set Schedule
    bsched = "PGRG"
    msched = "RGPG"
    print("Node 1 schedule %s " % bsched)
    print("Node 2 schedule %s " % msched)
    # Send one frame (set mamx_frame to 1)
    bconf = {"tdd_enabled": True,
             "frame_mode": "free_running",
             "symbol_size": symSamp,
             "frames": [bsched],
             "max_frame": 1}
    mconf = {"tdd_enabled": True,
             "frame_mode": "free_running",
             "dual_pilot": False,
             "symbol_size": symSamp,
             "frames": [msched],
             "max_frame": 1}
    bsdr.writeSetting("TDD_CONFIG", json.dumps(bconf))
    msdr.writeSetting("TDD_CONFIG", json.dumps(mconf))

    # SW Delays
    for sdr in [bsdr, msdr]:
        sdr.writeSetting("TX_SW_DELAY", str(30))

    msdr.writeSetting("TDD_MODE", "true")
    bsdr.writeSetting("TDD_MODE", "true")

    for sdr in [bsdr, msdr]:
        sdr.writeRegisters("TX_RAM_A", 0, cfloat2uint32(pilot1, order='QI').tolist())
        sdr.writeRegisters("TX_RAM_B", 0, cfloat2uint32(pilot2, order='QI').tolist())

    flags = 0
    r1 = bsdr.activateStream(rxStreamB, flags, 0)
    r2 = msdr.activateStream(rxStreamM, flags, 0)
    if r1 < 0:
        print("Problem activating stream #1")
    if r2 < 0:
        print("Problem activating stream #2")

    bsdr.writeSetting("TRIGGER_GEN", "")

    r1 = msdr.readStream(rxStreamM, [waveRxA1, waveRxB1], symSamp)
    print("reading stream #1 ({})".format(r1))
    r2 = bsdr.readStream(rxStreamB, [waveRxA2, waveRxB2], symSamp)
    print("reading stream #2 ({})".format(r2))
 
    # ADC_rst, stops the tdd time counters, makes sure next time runs in a clean slate
    tdd_conf = {"tdd_enabled" : False}
    for sdr in [bsdr, msdr]:
        sdr.writeSetting("RESET_DATA_LOGIC", "")
        sdr.writeSetting("TDD_CONFIG", json.dumps(tdd_conf))
        sdr.writeSetting("TDD_MODE", "false")

    msdr.deactivateStream(rxStreamM)
    bsdr.deactivateStream(rxStreamB)
    msdr.closeStream(rxStreamM)
    bsdr.closeStream(rxStreamB)
    msdr = None
    bsdr = None

    fig = plt.figure(figsize=(20, 8), dpi=120)
    fig.subplots_adjust(hspace=.5, top=.85)
    ax1 = fig.add_subplot(2, 1, 1)
    ax1.grid(True)
    ax1.set_title('Serials: (%s, %s)' % (serial1, serial2))
    ax1.set_ylabel('Signal (units)')
    ax1.set_xlabel('Sample index')
    ax1.plot(range(len(waveRxA1)), np.real(uint32tocfloat(waveRxA1)), label='ChA I Node 1')
    ax1.plot(range(len(waveRxB1)), np.real(uint32tocfloat(waveRxB1)), label='ChB I Node 1')
    ax1.set_ylim(-1, 1)
    ax1.set_xlim(0, symSamp)
    ax1.legend(fontsize=10)
    ax2 = fig.add_subplot(2, 1, 2)
    ax2.grid(True)
    ax2.set_ylabel('Signal (units)')
    ax2.set_xlabel('Sample index')
    ax2.plot(range(len(waveRxA2)), np.real(uint32tocfloat(waveRxA2)), label='ChA I Node 2')
    ax2.plot(range(len(waveRxB2)), np.real(uint32tocfloat(waveRxB2)), label='ChB I Node 2')
    ax2.set_ylim(-1, 1)
    ax2.set_xlim(0, symSamp)
    ax2.legend(fontsize=10)
    plt.show()


#########################################
#                  Main                 #
#########################################
def main():
    parser = OptionParser()
    parser.add_option("--serial1", type="string", dest="serial1", help="serial number of the device 1", default="RF3E000143")
    parser.add_option("--serial2", type="string", dest="serial2", help="serial number of the device 2", default="RF3E000160")
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--txgain", type="float", dest="txgain", help="Tx gain (dB)", default=50.0)  # See documentation at top of file for info on gain range
    parser.add_option("--rxgain", type="float", dest="rxgain", help="Rx gain (dB)", default=60.0)  # See documentation at top of file for info on gain range
    parser.add_option("--freq", type="float", dest="freq", help="Optional Tx freq (Hz)", default=0)
    parser.add_option("--numSamps", type="int", dest="numSamps", help="Num samples to receive", default=512)
    parser.add_option("--prefix-pad", type="int", dest="prefix_length", help="prefix padding length for beacon and pilot", default=82)
    parser.add_option("--postfix-pad", type="int", dest="postfix_length", help="postfix padding length for beacon and pilot", default=68)

    if options.freq == 0:
        print("[ERROR] Please provide RF Freq (Hz). POWDER users must set to 2.5e9")
        exit(0)

    (options, args) = parser.parse_args()
    siso_tdd_burst(
        serial1=options.serial1,
        serial2=options.serial2,
        rate=options.rate,
        freq=options.freq,
        txgain=options.txgain,
        rxgain=options.rxgain,
        numSamps=options.numSamps,
        prefix_pad=options.prefix_length,
        postfix_pad=options.postfix_length,
    )


if __name__ == '__main__':
    main()
 
