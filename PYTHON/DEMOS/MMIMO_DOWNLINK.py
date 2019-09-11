
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

running = True

def signal_handler(signum, frame):
    global running
    running = False

def init(hub, bnodes, cnodes, ref_ant, ampl, rate, freq, txgain, rxgain, cp, wait_trigger, numSamps, prefix_length, postfix_length, tx_advance, both_channels, threshold, use_trig, samp_cal, recip_cal, plotter):
    R = len(bnodes)
    ant = 2 if both_channels else 1
    M = R * ant
    K = len(cnodes)
    if not recip_cal and K == 0:
        print("Either specify a client node or enable --recip-cal")
    else:
        print("(R,M,K) = (%d,%d,%d)"%(R,M,K))

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
                sdr.writeSetting("SPI_TDD_MODE", "SISO")
            sdr.writeSetting(SOAPY_SDR_RX, 1, 'ENABLE_CHANNEL', 'false')
            sdr.writeSetting(SOAPY_SDR_TX, 1, 'ENABLE_CHANNEL', 'false')

    trig_dev.writeSetting("SYNC_DELAYS", "")

    symSamp = numSamps + prefix_length + postfix_length
    print("numSamps = %d"%symSamp)

    fft_size = 64
    cp_len = 32 if cp else 0
    ofdm_len = 2*fft_size + cp_len
    zeros = np.array([0]*(numSamps-ofdm_len))
    ltsSym, lts_f = generate_training_seq(preamble_type='lts', cp=cp_len, upsample=1)
    pilot = np.concatenate((ltsSym, zeros)).astype(np.complex64)
    #a0, a1, corr = find_lts(ltsSym, flip=True)
    #print(len(corr))
    #plt.plot(corr)
    #plt.show()

    upsample = 1
    preambles_bs = generate_training_seq(preamble_type='gold_ifft', seq_length=128, cp=0, upsample=upsample)
    preambles = preambles_bs[:,::upsample] #the correlators can run at lower rates, so we only need the downsampled signal.
    beacon = preambles[0,:]
    pad1 = np.array([0]*(prefix_length), np.complex64) # to comprensate for front-end group delay
    pad2 = np.array([0]*(postfix_length), np.complex64) # to comprensate for rf path delay
    wbz = np.array([0]*(symSamp), np.complex64)
    bcnz = np.array([0]*(symSamp-prefix_length-len(beacon)), np.complex64)  
    beacon1 = np.concatenate([pad1,beacon*.5,bcnz]).astype(np.complex64)
    beacon2 = wbz #beacon1 if both_channels else wbz   

    wb_pilot = 0.25 * pilot
    wbz = np.array([0]*(symSamp), np.complex64)
    wb_pilot_pad = np.concatenate([pad1, wb_pilot, pad2]).astype(np.complex64)

    L = fft_size - 12
    lts_thresh = 0.8

    possible_dim = []
    nRadios = 2*len(bsdrs) if both_channels else len(bsdrs)
    possible_dim.append(2**(np.ceil(np.log2(nRadios))))
    h_dim = min(possible_dim)
    hadamard_matrix = hadamard(h_dim)       #hadamard matrix : http://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.linalg.hadamard.html
    beacon_weights = hadamard_matrix[0:nRadios, 0:nRadios]
    beacon_weights = beacon_weights.astype(np.uint32)

    # DEV: ueTrigTime = 153 (prefix_length=0), CBRS: ueTrigTime = 235 (prefix_length=82), tx_advance=prefix_length,
    # corr delay is 17 cycles
    rf_roundtrip = prefix_length + len(beacon) + postfix_length + 17 + postfix_length
 
    # Create streams
    txBsStreams = [sdr.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, [0, 1]) for sdr in bsdrs]
    rxBsStreams = [sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0, 1]) for sdr in bsdrs]
    rxClStreams = [sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0, 1]) for sdr in csdrs]

    zeroPlot = [[np.array([0]*symSamp).astype(np.complex64) for r in range(M)] for s in range(M)]
    peaks = [[np.array([0]*613, np.int32) for i in range(M)] for j in range(M)] 
    peaks1 = [[np.array([0]*613, np.int32) for i in range(2*K)] for j in range(M)] 

    if plotter and K > 0:
        fig1, axes1 = plt.subplots(nrows=M, ncols=2*K, figsize=(12,12))
        axes1[0,0].set_title('Downlink Pilot')
        axes1[0,K].set_title('Uplink Pilot')
        for m in range(M):
            for l in range(K):
                axes1[m,l].set_xlim(0, symSamp)
                axes1[m,l+K].set_xlim(0, symSamp)
                axes1[m,l].set_ylim(-1,1)
                axes1[m,l+K].set_ylim(-1,1)
                axes1[m,l].set_ylabel('Downlink BS Ant %d, Cl Ant %d'%(m,l))
                axes1[m,l+K].set_ylabel('Uplink BS Ant %d, Cl Ant %d'%(m,l))

        lines10 = [[axes1[m,l].plot(range(symSamp), np.real(zeroPlot[m][l]), label='Pilot TxAnt %d RxAnt %d (real)'%(m,l))[0] for l in range(2*K)] for m in range(M)]
        lines11 = [[axes1[m,l].plot(range(symSamp), np.imag(zeroPlot[m][l]), label='Pilot TxAnt %d RxAnt %d (imag)'%(m,l))[0] for l in range(2*K)] for m in range(M)]
        lines12 = [[axes1[m,l].plot(range(symSamp), symSamp*[lts_thresh])[0] for l in range(2*K)] for m in range(M)] 
        for m in range(M):
            for l in range(2*K):
                axes1[m,l].legend(fontsize=10)
        fig1.show()

    if plotter and recip_cal:
        fig2, axes2 = plt.subplots(nrows=M, ncols=M, figsize=(12,12))
        for m in range(M):
            for l in range(M):
                axes2[m,l].set_xlim(0, symSamp)
                axes2[m,l].set_ylim(-1,1)
                axes2[m,l].set_ylabel('Tx Ant %d, Rx Ant %d'%(m,l))
                axes2[m,l].legend(fontsize=10)

        lines20 = [[axes2[m,l].plot(range(symSamp), np.real(zeroPlot[m][l]), label='Pilot TxAnt %d RxAnt %d (real)'%(m,l))[0] for l in range(M)] for m in range(M)]
        lines21 = [[axes2[m,l].plot(range(symSamp), np.imag(zeroPlot[m][l]), label='Pilot TxAnt %d RxAnt %d (imag)'%(m,l))[0] for l in range(M)] for m in range(M)]
        lines22 = [[axes2[m,l].plot(range(symSamp), symSamp*[lts_thresh])[0] for m in range(M)] for l in range(M)] 
        #lines23 = [[axes2[m,l].plot(range(symSamp), peaks[m][l][:symSamp], label='Offset %d'%m)[0] for m in range(M)] for l in range(M)]  
        fig2.show()

    if recip_cal: calibObj = CalibCSI(bsdrs, trig_dev, txBsStreams, rxBsStreams, ant, symSamp, wb_pilot_pad)
    if K > 0: csiObj = CSI(bsdrs, csdrs, trig_dev, txBsStreams, rxBsStreams, rxClStreams, not use_trig, ant, rate, symSamp, wb_pilot_pad, beacon1, beacon, None if use_trig else beacon_weights, rf_roundtrip)

    frame = 1000 
    f = 0
    forward_sync = False
    reverse_sync = False
    offset = np.empty((M,M), np.int32)
    offset1 = np.empty((M,2*K), np.int32)
    rssi = np.empty((M,M), np.float32)
    signal.signal(signal.SIGINT, signal_handler)
    while(running):
        if recip_cal:
            calibObj.setup()
            _, _, sampsRx = calibObj.collect_calib_pilots()
            for m in range(M):
                for p in range(M):
                    if m != p:
                        sampsRx[m][p] -= np.mean(sampsRx[m][p])
                        a0, a1, peaks[m][p] = find_lts(sampsRx[m][p], thresh=lts_thresh, flip=True)
                        offset[m,p] = 0 if not a0 else a0 - len(ltsSym) + cp_len
                        H1 = np.fft.fft(sampsRx[m][p][offset[m,p]:offset[m,p]+fft_size], fft_size, 0) 
                        H2 = np.fft.fft(sampsRx[m][p][offset[m,p]+fft_size:offset[m,p]+2*fft_size], fft_size, 0)
                        rssi[m,p] = np.mean((np.abs(H1)+np.abs(H2))/2)
                        #print("[%d,%d]"%(m,p))
                        #print(sampsRx[m][p])
                    else:
                        sampsRx[m][p] = wb_pilot_pad
                        a0, a1, peaks[m][p] = find_lts(sampsRx[m][p], thresh=lts_thresh, flip=True)
                        offset[m,p] = 0 if not a0 else a0 - len(ltsSym) + cp_len
                        rssi[m,p] = 0 
                    #print("(%d, %d): %d"%(m,p,offset[m][p]))
                    #print(peaks[m][p])

            if samp_cal: 
                print("rx offsets at ref_ant %d"%ref_ant)
                print(offset[:,ref_ant])
                print("tx offsets at ref_ant %d"%ref_ant)
                print(offset[ref_ant,:])
                ref_offset = 1 if ref_ant == 0 else 0
                if not forward_sync: forward_sync = calibObj.sample_cal(offset[ref_ant,:], ref_ant)
                if forward_sync and not reverse_sync: reverse_sync = calibObj.sample_cal(offset[:,ref_ant], ref_ant, offset[ref_ant, ref_offset], False) 
            calibObj.close()

            if plotter:
                for m in range(M):
                    for p in range(M):
                        lines20[m][p].set_ydata(np.real(sampsRx[m][p]))
                        lines21[m][p].set_ydata(np.imag(sampsRx[m][p]))
                        lines22[m][p].set_data(offset[m,p], np.linspace(-1.0, 1.0, num=100))
                        #lines23[m][p].set_ydata(np.real(peaks[m][p][:symSamp])/np.max(peaks[m][p][:symSamp]))

            #print("offset matrix")
            #print(offset)
            #print("rssi matrix")
            #print(rssi)
            #print("")
            #print("")

        if K > 0:
            csiObj.setup()
            bsRxSamps, clRxSamps = csiObj.collectCSI()
            csiObj.close()
            for m in range(M):
                for k in range(K):
                    bsRxSamps[m][k] -= np.mean(bsRxSamps[m][k])
                    a0, a1, peaks1[m][k+K] = find_lts(bsRxSamps[m][k], thresh=lts_thresh, flip=False)
                    offset1[m,k+K] = 0 if not a0 else a0 - len(ltsSym) + cp_len
                    if plotter: lines10[m][k+K].set_ydata(np.real(bsRxSamps[m][k]))
                    if plotter: lines11[m][k+K].set_ydata(np.imag(bsRxSamps[m][k]))
                    if plotter: lines12[m][k+K].set_data(offset1[m,k+K], np.linspace(-1.0, 1.0, num=100))
                for k in range(K):
                    clRxSamps[m][k] -= np.mean(clRxSamps[m][k])
                    a0, a1, peaks1[m][k] = find_lts(clRxSamps[m][k], thresh=lts_thresh, flip=False)
                    offset1[m,k] = 0 if not a0 else a0 - len(ltsSym) + cp_len
                    if plotter: lines10[m][k].set_ydata(np.real(clRxSamps[m][k]))
                    if plotter: lines11[m][k].set_ydata(np.imag(clRxSamps[m][k]))
                    if plotter: lines12[m][k].set_data(offset1[m,k], np.linspace(-1.0, 1.0, num=100))

            if plotter:
                fig1.canvas.draw()
                fig1.show()

        if recip_cal:
            if plotter:
                fig2.canvas.draw()
                fig2.show()

        print("frame %d"%f)
        print("")
        f += 1
 
    if recip_cal:
        calibObj.close()

    if K > 0: 
        csiObj.close()
        [csdrs[r].closeStream(rxClStreams[r]) for r in range(len(csdrs))]

    [bsdrs[r].closeStream(txBsStreams[r]) for r in range(len(bsdrs))]
    [bsdrs[r].closeStream(rxBsStreams[r]) for r in range(len(bsdrs))]


def main():
    parser = OptionParser()
    parser.add_option("--bnodes", type="string", dest="bnodes", help="file name containing serials on the base station", default="bs_serials.txt")
    parser.add_option("--cnodes", type="string", dest="cnodes", help="file name containing serials to be used as clients", default="client_serials.txt")
    parser.add_option("--hub", type="string", dest="hub", help="Hub node", default="")
    parser.add_option("--ref-ant", type="int", dest="ref_ant", help="Calibration reference antenna", default=0)
    parser.add_option("--ampl", type="float", dest="ampl", help="Amplitude coefficient for downCal/upCal", default=5.0)
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--freq", type="float", dest="freq", help="Optional Tx freq (Hz)", default=2.5e9)
    parser.add_option("--txgain", type="float", dest="txgain", help="Optional Tx gain (dB)", default=40.0)
    parser.add_option("--rxgain", type="float", dest="rxgain", help="Optional Rx gain (dB)", default=20.0)
    parser.add_option("--bw", type="float", dest="bw", help="Optional Tx filter bw (Hz)", default=10e6)
    parser.add_option("--cp", action="store_true", dest="cp", help="adds cyclic prefix to tx symbols", default=True)
    parser.add_option("--wait-trigger", action="store_true", dest="wait_trigger", help="wait for a trigger to start a frame",default=False)
    parser.add_option("--numSamps", type="int", dest="numSamps", help="Number of samples in Symbol", default=400)
    parser.add_option("--prefix-length", type="int", dest="prefix_length", help="prefix padding length for beacon and pilot", default=82)
    parser.add_option("--postfix-length", type="int", dest="postfix_length", help="postfix padding length for beacon and pilot", default=68)
    parser.add_option("--tx-advance", type="int", dest="tx_advance", help="symbol advance for tx", default=2)
    parser.add_option("--both-channels", action="store_true", dest="both_channels", help="transmit from both channels",default=False)
    parser.add_option("--corr-threshold", type="int", dest="threshold", help="Correlator Threshold Value", default=1)
    parser.add_option("--use-trig", action="store_true", dest="use_trig", help="uses chain triggers for synchronization",default=False)
    parser.add_option("--recip-cal", action="store_true", dest="recip_cal", help="perform reciprocity calibration procedure",default=False)
    parser.add_option("--sample-cal", action="store_true", dest="samp_cal", help="perform sample offset calibration",default=False)
    parser.add_option("--plotter", action="store_true", dest="plotter", help="continuously plots all signals and stats",default=False)
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

    init(
	hub=options.hub,
	bnodes=bserials,
	cnodes=cserials,
	ref_ant=options.ref_ant,
	ampl=options.ampl,
	rate=options.rate,
	freq=options.freq,
	txgain=options.txgain,
	rxgain=options.rxgain,
        cp=options.cp,
	wait_trigger=options.wait_trigger,
	numSamps=options.numSamps,
        prefix_length=options.prefix_length,
        postfix_length=options.postfix_length,
        tx_advance=options.tx_advance,
        both_channels=options.both_channels,
        threshold=options.threshold,
        use_trig=options.use_trig,
        recip_cal=options.recip_cal,
        samp_cal=options.samp_cal,
	plotter=options.plotter
    )

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
#    except Exception as e:
#	print e
#	exit()

