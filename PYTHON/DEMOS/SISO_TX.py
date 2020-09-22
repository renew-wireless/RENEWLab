#!/usr/bin/python3
"""
 SISO_TX.py

 Simple transmitter. Generate signal and transmit it from one of the
 antennas in the Iris Board.
 Supported signal types:
   LTE, WiFi LTS, WiFi STS, Sine

 Usage example: python3 SISO_TX.py --serial="RF3C000047"

  NOTE ON GAINS:
  Gain settings will vary depending on RF frontend board being used
  If using CBRS:
  gain: at 2.5GHz [16:1:93], at 3.6GHz [15:1:102]

  If using only Dev Board:
  gain: at both frequency bands [0:1:42]

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
import time
import datetime
import os
import math
import signal
import threading
import matplotlib.pyplot as plt
import pickle
import json
import pdb
import LTE5_re
import LTE5_im
from SoapySDR import *              # SOAPY_SDR_ constants
from optparse import OptionParser
from functools import partial
from type_conv import *
from print_sensor import *
from generate_sequence import *


#########################################
#            Global Parameters          #
#########################################
sdr = None
running = True
txStream = None

#########################################
#              Functions                #
#########################################
def print_thread(sdr, info):
    """
    Continuously print sensor information
    """
    global running
    while running and "CBRS" in info["frontend"]:
        print('-'*80)
        print_sensor([sdr], 'LMS7_TEMP')
        print_sensor([sdr], 'ZYNQ_TEMP')
        time.sleep(2)


def siggen_app(args, rate, ampl, ant, gain, freq, bbfreq, waveFreq, numSamps, serial, sigType, lo_tone):
    """
    Generate signal and write stream to RAM for TX
    """
    global sdr

    # Device information
    sdr = SoapySDR.Device(dict(serial=serial))
    info = sdr.getHardwareInfo()
    amplFixed = int(ampl*(1 << 13))
    if ant == 'A':
        txChannel = [0]
    elif ant == 'B':
        txChannel = [1]
    elif ant == 'AB':
        txChannel = [0, 1]
    else:
        txChannel = []

    # Settings
    for c in txChannel:
        print("Writing settings for channel {}".format(c))
        sdr.setBandwidth(SOAPY_SDR_TX, c, 2.5*rate)
        sdr.setSampleRate(SOAPY_SDR_TX, c, rate)
        sdr.setFrequency(SOAPY_SDR_TX, c, "RF", freq-.75*rate)
        sdr.setFrequency(SOAPY_SDR_TX, c, "BB", .75*rate)
        #sdr.setFrequency(SOAPY_SDR_TX, c, "RF", freq+bbfreq)
        #sdr.setFrequency(SOAPY_SDR_TX, c, "BB", bbfreq)
        sdr.setAntenna(SOAPY_SDR_TX, c, "TRX")

        if lo_tone:
            sdr.writeSetting(SOAPY_SDR_TX, c, 'TSP_TSG_CONST', str(amplFixed))
            sdr.writeSetting(SOAPY_SDR_TX, c, 'TX_ENB_OVERRIDE', 'true')

        if "CBRS" in info["frontend"]:
            sdr.setGain(SOAPY_SDR_TX, c, gain)
        else:
            # No CBRS board gains, only changing LMS7 gains
            sdr.setGain(SOAPY_SDR_TX, c, "PAD", gain)  # [0:1:42]
            sdr.setGain(SOAPY_SDR_TX, c, "IAMP", 0)    # [-12:1:3]

    # Generate TX signal
    txSignal = np.empty(numSamps).astype(np.complex64)
    wbz = txSignal
    if sigType == "LTE":
        # LTE signal
        for i in range(numSamps):
            txSignal[i] = np.complex(LTE5_re.lte5i[i]/32768.0, LTE5_im.lte5q[i]/32768.0)
    elif sigType == "LTS":
        # WiFi LTS Signal
        ltsSym, lts_f = generate_training_seq(preamble_type='lts', cp=32, upsample=1)
        txSignal = np.tile(ltsSym, numSamps//len(ltsSym)).astype(np.complex64) * ampl
    elif sigType == "STS":
        # WiFi STS Signal
        stsSym = generate_training_seq(preamble_type='sts', reps=10)
        txSignal = np.tile(stsSym, numSamps//len(stsSym)).astype(np.complex64) * 5
    elif sigType == "SINE":
        # Sine Waveform
        Ts = 1 / rate
        if waveFreq is None:
            waveFreq = rate / 20
        x = 20
        numSamps = int(x * rate / waveFreq)  # x period worth of samples
        s_freq = waveFreq
        s_time_vals = np.array(np.arange(0, numSamps)).transpose() * Ts
        txSignal = np.exp(s_time_vals * 1j * 2 * np.pi * s_freq).astype(np.complex64) * ampl
        if bbfreq > 0:
            txSignal = np.array([0]*numSamps, np.complex64)   # use with cordic
            txSignal += .1
    else:
        raise Exception("Signal type not supported. Valid entries: LTE/LTS/STS/SINE")

    # Float to fixed point
    pilot1_ui32 = cfloat2uint32(txSignal, order='QI')
    pilot2_ui32 = cfloat2uint32(wbz)

    if not lo_tone:
        replay_addr = 0
        if ant == 'A':
            sdr.writeRegisters("TX_RAM_A", replay_addr, pilot1_ui32.tolist())
        elif ant == 'B':
            sdr.writeRegisters("TX_RAM_B", replay_addr, pilot1_ui32.tolist())
        elif ant == 'AB':
            sdr.writeRegisters("TX_RAM_A", replay_addr, pilot1_ui32.tolist())
            sdr.writeRegisters("TX_RAM_B", replay_addr, pilot1_ui32.tolist())
        sdr.writeSetting("TX_REPLAY", str(numSamps)) # this starts transmission

    # Show current gains
    if "CBRS" in info["frontend"]:
        IAMP = sdr.getGain(SOAPY_SDR_TX, 0, "IAMP")
        PAD = sdr.getGain(SOAPY_SDR_TX, 0, "PAD")
        PA1 = sdr.getGain(SOAPY_SDR_TX, 0, "PA1")
        PA2 = sdr.getGain(SOAPY_SDR_TX, 0, "PA2")
        PA3 = sdr.getGain(SOAPY_SDR_TX, 0, "PA3")
        ATTN = sdr.getGain(SOAPY_SDR_TX, 0, "ATTN")
        print("GAINS CHAN0: PA1 {}, PA3 {}, PA2 {}, ATTN {}, PAD {}, IAMP {}".format(PA1, PA3, PA2, ATTN, PAD, IAMP))
    else:
        IAMP = sdr.getGain(SOAPY_SDR_TX, 0, "IAMP")
        PAD = sdr.getGain(SOAPY_SDR_TX, 0, "PAD")
        print("GAINS CHAN0: PAD {}, IAMP {}".format(PAD, IAMP))

    # Plot signal
    debug = 0
    if debug:
        fig = plt.figure(figsize=(20, 8), dpi=100)
        ax1 = fig.add_subplot(2, 1, 1)
        ax1.plot(np.real(txSignal), label='pilot i')
        ax1.plot(np.imag(txSignal), label='pilot q')
        ax2 = fig.add_subplot(2, 1, 2)
        ax2.plot(np.abs(txSignal), label='abs(signal)')
        plt.show(block=False)

    # Stop/Close/Cleanup
    signal.signal(signal.SIGINT, signal_handler)
    pth = threading.Thread(target=print_thread, args=(sdr, info))
    pth.start()
    print("ctrl-c to stop ...")
    signal.pause()


def signal_handler(signal, frame):
    global sdr, txStream, running
    running = False
    #cleanup txStream
    print("Cleanup txStreams")
    if txStream is not None:
        sdr.deactivateStream(txStream)
        sdr.closeStream(txStream)


#########################################
#                 Main                  #
#########################################
def main():
    parser = OptionParser()
    parser.add_option("--args", type="string", dest="args", help="device factor arguments", default="")
    parser.add_option("--rate", type="float", dest="rate", help="Tx and Rx sample rate", default=5e6)
    parser.add_option("--ampl", type="float", dest="ampl", help="Tx digital amplitude scale", default=1)
    parser.add_option("--ant", type="string", dest="ant", help="Optional Tx antenna", default="A")
    parser.add_option("--gain", type="float", dest="gain", help="Tx gain [0:105] (dB)", default=42.0)
    parser.add_option("--freq", type="float", dest="freq", help="Tx RF freq (Hz)", default=2.5e9)
    parser.add_option("--bbfreq", type="float", dest="bbfreq", help="Lime chip Baseband frequency (Hz)", default=0)
    parser.add_option("--waveFreq", type="float", dest="waveFreq", help="Baseband waveform freq (Hz)", default=None)
    parser.add_option("--numSamps", type="int", dest="numSamps", help="Num samples to receive", default=1024)
    parser.add_option("--serial", type="string", dest="serial", help="serial number of the device", default="RF3E000157")
    parser.add_option("--sigType", type="string", dest="sigType", help="Signal Type: LTE/LTS/STS/SINE", default="SINE")
    parser.add_option("--lo-tone", action="store_true", dest="lo_tone", help="generate tone using the LO ", default=False)
    (options, args) = parser.parse_args()

    # Display parameters
    print("\n")
    print("========== TX PARAMETERS =========")
    print("Transmitting {} signal from board {}".format(options.sigType, options.serial))
    print("Sample Rate (sps): {}".format(options.rate))
    print("Antenna: {}".format(options.ant))
    print("Tx Gain (dB): {}".format(options.gain))
    print("Frequency (Hz): {}".format(options.freq))
    print("Baseband Freq. (Hz): {}".format(options.bbfreq))
    print("Number of Samples: {}".format(options.numSamps))
    print("==================================")
    print("\n")

    siggen_app(
        args=options.args,
        rate=options.rate,
        ampl=options.ampl,
        ant=options.ant,
        gain=options.gain,
        freq=options.freq,
        bbfreq=options.bbfreq,
        waveFreq=options.waveFreq,
        numSamps=options.numSamps,
        serial=options.serial,
        sigType=options.sigType,
        lo_tone=options.lo_tone,
    )


if __name__ == '__main__':
    main()
