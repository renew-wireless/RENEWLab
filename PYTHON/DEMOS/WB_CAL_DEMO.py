#!/usr/bin/python3
import sys
sys.path.append('../IrisUtils/')

import numpy as np
from optparse import OptionParser
import SoapySDR
from SoapySDR import * #SOAPY_SDR_ constants
import time
import os
import math
import datetime
import json
import signal
import pdb
import matplotlib
from scipy.linalg import hadamard
import scipy.io as sio 
from array import array
matplotlib.rcParams.update({'font.size': 10})
import matplotlib.pyplot as plt
from matplotlib import animation
import collections
from csi_lib import *
from find_lts import *
from digital_rssi import *
from bandpower import *
from file_rdwr import *
from ofdmtxrx import *
from type_conv import *
from print_sensor import *

plt.style.use('ggplot')  # customize your plots style

RF_RST_REG = 48
CORR_CONF = 60
CORR_RST = 64
CORR_THRESHOLD = 92
TDD_CONF_REG = 120
SCH_ADDR_REG = 136
SCH_MODE_REG = 140

NumBufferSamps = 3000
lts_thresh = 0.8
running = True

def signal_handler(signum, frame):
    global running
    running = False

def init(hub, bnodes, ref_ant, ampl, rate, freq, txgain, rxgain, cp, numSamps, prefix_length, postfix_length, both_channels, plotter):
    R = len(bnodes)
    ant = 2 if both_channels else 1
    M = R * ant

    if hub != "": hub_dev = SoapySDR.Device(dict(driver="remote", serial = hub)) # device that triggers bnodes and ref_node
    bsdrs = [SoapySDR.Device(dict(driver="iris", serial = serial)) for serial in bnodes] # base station sdrs

    # assume trig_sdr is part of the master nodes
    trig_dev = None
    if hub != "":
        trig_dev = hub_dev
    else:
        trig_dev = bsdrs[0]

    #set params on both channels
    for sdr in bsdrs:
        info = sdr.getHardwareInfo()
        print("%s settings on device" % (info["frontend"]))
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
                if freq < 3e9: sdr.setGain(SOAPY_SDR_RX, ch, 'ATTN', -18)   # {-18,-12,-6,0}
                else: sdr.setGain(SOAPY_SDR_RX, ch, 'ATTN', 0)   # {-18,-12,-6,0}
                sdr.setGain(SOAPY_SDR_RX, ch, 'LNA2', 17)  # LO: [0|17], HI:[0|14]

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

        #for ch in [0, 1]:
        #    if calibrate:
        #        sdr.writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", 'SKLK')
        #        sdr.writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", '')

        sdr.writeRegister("IRIS30", RF_RST_REG, (1 << 29) | 0x1)
        sdr.writeRegister("IRIS30", RF_RST_REG, (1 << 29))
        sdr.writeRegister("IRIS30", RF_RST_REG, 0)
        if not both_channels:
            if info["serial"].find("RF3E") < 0:
                print("SPI TDD MODE")
                #sdr.writeSetting("SPI_TDD_MODE", "SISO")
            sdr.writeSetting(SOAPY_SDR_RX, 1, 'ENABLE_CHANNEL', 'false')
            sdr.writeSetting(SOAPY_SDR_TX, 1, 'ENABLE_CHANNEL', 'false')

    trig_dev.writeSetting("SYNC_DELAYS", "")

    symSamp = numSamps + prefix_length + postfix_length
    print("numSamps = %d" % symSamp)

    fft_size = 64
    cp_len = 32 if cp else 0
    ofdm_len = 2*fft_size + cp_len
    zeros = np.array([0]*(numSamps-ofdm_len))
    ltsSym, lts_f = generate_training_seq(preamble_type='lts', cp=cp_len, upsample=1)
    pilot = np.concatenate((ltsSym, zeros)).astype(np.complex64)
    wb_pilot = 0.5 * pilot
    wbz = np.array([0]*(symSamp), np.complex64)
    pad1 = np.array([0]*(prefix_length), np.complex64) # to comprensate for front-end group delay
    pad2 = np.array([0]*(postfix_length), np.complex64) # to comprensate for rf path delay
    zeroPlot = [[np.array([0]*symSamp).astype(np.complex64) for r in range(M)] for s in range(M)]
    wb_pilot_pad = np.concatenate([pad1, wb_pilot, pad2]).astype(np.complex64)
    pilot_subcarriers = [7, 21, 43, 57]
    pilot_sc_num = len(pilot_subcarriers)

    # Create streams
    txStreams = [sdr.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, [0, 1]) for sdr in bsdrs]
    rxStreams = [sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0, 1]) for sdr in bsdrs]
    for r,sdr in enumerate(bsdrs):
        sdr.activateStream(txStreams[r])

    num_sdrs = len(bsdrs)
    num_ants = num_sdrs * ant
    sampsRx = [[np.empty(symSamp).astype(np.complex64) for r in range(num_ants)] for t in range(num_ants)]
    csi = [[np.empty(fft_size).astype(np.complex64) for r in range(num_ants)] for t in range(num_ants)]
    calibMag = [np.empty(fft_size).astype(np.complex64) for r in range(num_ants)]
    calibAng = [np.empty(fft_size).astype(np.complex64) for r in range(num_ants)]
    magBuffer = [[collections.deque(maxlen=NumBufferSamps) for i in range(pilot_sc_num)] for j in range(num_ants)]
    angBuffer = [[collections.deque(maxlen=NumBufferSamps) for i in range(len(pilot_subcarriers))] for j in range(num_ants)]
    dummy = np.empty(symSamp).astype(np.complex64)

    if plotter:
        fig1, axes1 = plt.subplots(nrows=M, ncols=2, figsize=(12,8))
        axes1[0,0].set_title('Reciprocity Calibration Magnitude')
        axes1[0,1].set_title('Reciprocity Calibration Phase')
        for m in range(M):
            axes1[m,0].set_xlim(0, NumBufferSamps)
            axes1[m,1].set_xlim(0, NumBufferSamps)
            axes1[m,0].set_ylim(0,5)
            axes1[m,1].set_ylim(-np.pi,np.pi)
            if m == ref_ant:
                axes1[m,0].set_ylabel('Ant %d (ref)'%(m))
            else:
                axes1[m,0].set_ylabel('Ant %d'%(m))

        lines10 = [[axes1[m,0].plot(range(NumBufferSamps), np.zeros(NumBufferSamps), label='SC %d'%(p))[0] for p in range(pilot_sc_num)] for m in range(M)]
        lines11 = [[axes1[m,1].plot(range(NumBufferSamps), np.zeros(NumBufferSamps), label='SC %d'%(p))[0] for p in range(pilot_sc_num)] for m in range(M)]
        for m in range(M):
            for l in range(2):
                axes1[m,l].legend(fontsize=10)

        #fig2, axes2 = plt.subplots(nrows=M, ncols=M, figsize=(12,12))
        #for m in range(M):
        #    for l in range(M):
        #        axes2[m,l].set_xlim(0, symSamp)
        #        axes2[m,l].set_ylim(-.1,.1)
        #        axes2[m,l].set_ylabel('Tx Ant %d, Rx Ant %d'%(m,l))
        #        axes2[m,l].legend(fontsize=10)

        #lines20 = [[axes2[m,l].plot(range(symSamp), np.real(zeroPlot[m][l]), label='Pilot TxAnt %d RxAnt %d (real)'%(m,l))[0] for l in range(M)] for m in range(M)]
        #lines21 = [[axes2[m,l].plot(range(symSamp), np.imag(zeroPlot[m][l]), label='Pilot TxAnt %d RxAnt %d (imag)'%(m,l))[0] for l in range(M)] for m in range(M)]
        ##lines22 = [[axes2[m,l].plot(range(symSamp), symSamp*[lts_thresh])[0] for m in range(M)] for l in range(M)] 
        #fig2.show()

    cround = 1
    signal.signal(signal.SIGINT, signal_handler)
    begin = time.time()
    prev_time = begin
    while(running):
        for m in range(num_sdrs):
            ref_sdr = bsdrs[m]
            flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST

            # transmit pilot from node m
            sr = ref_sdr.writeStream(txStreams[m], [wb_pilot_pad, zeroPlot[0][0]], symSamp, flags)
            if sr.ret == -1:
                print("bad write")
            
            flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST
            for r,sdr in enumerate(bsdrs):
                if r != m: 
                    sdr.activateStream(rxStreams[r], flags, 0, symSamp)

            trig_dev.writeSetting("TRIGGER_GEN", "")
            
            for r,sdr in enumerate(bsdrs):
                if r != m:
                    sr = sdr.readStream(rxStreams[r], [sampsRx[m][r], dummy], symSamp, timeoutUs=int(1e6))
                    if sr.ret != symSamp:
                        print("bad read %d"%sr.ret)

        bad_round = False
        for m in range(M):
            if bad_round: break
            for p in range(M):
                if m != p:
                    sampsRx[m][p] -= np.mean(sampsRx[m][p])
                    a0, _, _ = find_lts(sampsRx[m][p], thresh=lts_thresh, flip=True)
                    offset = 0 if not a0 else a0 - len(ltsSym) + cp_len
                    if offset < 150:
                        print("bad round, skip")
                        bad_round = True
                        break
                    H1 = np.fft.fft(sampsRx[m][p][offset:offset+fft_size], fft_size, 0) 
                    H2 = np.fft.fft(sampsRx[m][p][offset+fft_size:offset+2*fft_size], fft_size, 0)
                    csi[m][p] = (H1+H2)/2

        if bad_round: continue

        for m in range(M):
            if m == ref_ant:
                calibMag[m] = np.ones(fft_size)
                calibAng[m] = np.zeros(fft_size)
            else: 
                calibMag[m] = np.divide(np.abs(csi[m][ref_ant]), np.abs(csi[ref_ant][m]))
                calibAng[m] = np.angle(csi[m][ref_ant]*np.conj(csi[ref_ant][m]))
            for c in range(pilot_sc_num):
                s = pilot_subcarriers[c]
                magBuffer[m][c].append(calibMag[m][s])
                angBuffer[m][c].append(calibAng[m][s])

        if plotter:
            for m in range(num_ants):
                for p in range(pilot_sc_num):
                    lines10[m][p].set_data(range(len(magBuffer[m][p])), magBuffer[m][p])
                    lines11[m][p].set_data(range(len(magBuffer[m][p])), angBuffer[m][p])
            #for m in range(num_ants):
            #    for p in range(num_ants):
            #        lines20[m][p].set_ydata(np.real(sampsRx[m][p]))
            #        lines21[m][p].set_ydata(np.imag(sampsRx[m][p]))
            #        #lines22[m][p].set_data(offset[m,p], np.linspace(-1.0, 1.0, num=100))
            fig1.canvas.draw()
            fig1.show()
            #fig2.canvas.draw()
            #fig2.show()

        cround += 1
        cur_time = time.time()
        if (cur_time - prev_time > 3):
            print("%d rounds, %f secs elapsed"%(cround, cur_time-begin))
            prev_time = cur_time

    [bsdrs[r].closeStream(txStreams[r]) for r in range(len(bsdrs))]
    [bsdrs[r].closeStream(rxStreams[r]) for r in range(len(bsdrs))]

def main():
    parser = OptionParser()
    parser.add_option("--bnodes", type="string", dest="bnodes", help="file name containing serials on the base station", default="bs_serials.txt")
    parser.add_option("--hub", type="string", dest="hub", help="Hub node", default="")
    parser.add_option("--ref-ant", type="int", dest="ref_ant", help="Calibration reference antenna", default=0)
    parser.add_option("--ampl", type="float", dest="ampl", help="Amplitude coefficient for downCal/upCal", default=5.0)
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--freq", type="float", dest="freq", help="Optional Tx freq (Hz)", default=3.6e9)
    parser.add_option("--txgain", type="float", dest="txgain", help="Optional Tx gain (dB)", default=30.0)
    parser.add_option("--rxgain", type="float", dest="rxgain", help="Optional Rx gain (dB)", default=20.0)
    parser.add_option("--bw", type="float", dest="bw", help="Optional Tx filter bw (Hz)", default=10e6)
    parser.add_option("--cp", action="store_true", dest="cp", help="adds cyclic prefix to tx symbols", default=True)
    parser.add_option("--numSamps", type="int", dest="numSamps", help="Number of samples in Symbol", default=400)
    parser.add_option("--prefix-length", type="int", dest="prefix_length", help="prefix padding length for beacon and pilot", default=82)
    parser.add_option("--postfix-length", type="int", dest="postfix_length", help="postfix padding length for beacon and pilot", default=68)
    parser.add_option("--both-channels", action="store_true", dest="both_channels", help="transmit from both channels",default=False)
    parser.add_option("--plotter", action="store_true", dest="plotter", help="continuously plots all signals and stats",default=False)
    (options, args) = parser.parse_args()

    bserials = []
    with open(options.bnodes, "r") as f:
        for line in f.read().split():
            if line[0] != '#':
                bserials.append(line)
            else:
                continue      


    init(
	hub=options.hub,
	bnodes=bserials,
	ref_ant=options.ref_ant,
	ampl=options.ampl,
	rate=options.rate,
	freq=options.freq,
	txgain=options.txgain,
	rxgain=options.rxgain,
        cp=options.cp,
	numSamps=options.numSamps,
        prefix_length=options.prefix_length,
        postfix_length=options.postfix_length,
        both_channels=options.both_channels,
	plotter=options.plotter
    )

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()

