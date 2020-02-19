#!/usr/bin/python3
"""
    NB_CAL_DEMO.py

    NOTE: IRIS BOARDS MUST BE CHAINED FOR THIS SCRIPT TO WORK.
    ORDER MATTERS; FIRST NON-REF BOARD (SERIALS) IS THE ONE SENDING THE TRIGGER.

    This script is used for watching the recprocity amplitude and phases of
    narrow-band (sine) signals between all boards and a reference board.
    The reference board will be the first serial that appears in the bs_serials.txt
    file.
    It programs two Irises in TDD mode with the a framing schedule.

    Example:
        python3 NB_CAL_DEMO.py --serials=../IrisUtils/data_in/bs_serials.txt

    where "xxxx" is the serial number of an Iris node in the base station.

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
sys.path.append('../IrisUtils/')
sys.path.append('../IrisUtils/data_in/')

import SoapySDR
from SoapySDR import * #SOAPY_SDR_ constants
from optparse import OptionParser
import numpy as np
import time
import os
import math
import json
import signal
import matplotlib.pyplot as plt
from matplotlib import animation
from type_conv import *

running = True


def signal_handler(signum, frame):
    global running
    running = False


def calibrate_array(hub_serial, serials, ref_serial, rate, freq, txgain, rxgain, numSamps, prefix_pad, postfix_pad, second_channel):
    global running
    bsdr = [SoapySDR.Device(dict(driver='iris',serial=serial)) for serial in serials]
    ref_sdr = SoapySDR.Device(dict(driver='iris',serial=ref_serial))
    if hub_serial != "": 
        hub = SoapySDR.Device(dict(driver='remote',serial=hub_serial))
    sdrs = [ref_sdr] + bsdr
    #some default sample rates
    for i, sdr in enumerate(sdrs):
        info = sdr.getHardwareInfo()
        print("%s settings on device %d" % (info["frontend"], i))
        channel = [1] if second_channel else [0]
        for ch in channel:
            sdr.setFrequency(SOAPY_SDR_TX, ch, freq)
            sdr.setFrequency(SOAPY_SDR_RX, ch, freq)
            sdr.setSampleRate(SOAPY_SDR_TX, ch, rate)
            sdr.setSampleRate(SOAPY_SDR_RX, ch, rate)

            sdr.setGain(SOAPY_SDR_TX, ch, txgain)
            sdr.setGain(SOAPY_SDR_RX, ch, rxgain)

            sdr.setAntenna(SOAPY_SDR_RX, ch, "TRX")
            sdr.setDCOffsetMode(SOAPY_SDR_RX, ch, True)

        sdr.writeSetting("RESET_DATA_LOGIC", "")

    if hub_serial != "":
        hub.writeSetting("SYNC_DELAYS", "")
    else:
        bsdr[0].writeSetting("SYNC_DELAYS", "")
    num_bs_ant = len(bsdr)
    symSamp = numSamps + prefix_pad + postfix_pad
    print("numSamps = %d"%numSamps)
    print("symSamps = %d"%symSamp)

    DnA = [np.array([0]*symSamp, np.uint32) for i in range(num_bs_ant)]
    DnB = [np.array([0]*symSamp, np.uint32) for i in range(num_bs_ant)]
    UpA = [np.array([0]*symSamp, np.uint32) for i in range(num_bs_ant)]
    UpB = [np.array([0]*symSamp, np.uint32) for i in range(num_bs_ant)]  


    #phaseDeltaDn = np.empty(num_bs_ant).astype(np.float)
    #phaseDeltaUp = np.empty(num_bs_ant).astype(np.float)
    #calibPhase = np.empty(num_bs_ant).astype(np.float)  
    #ampDn = np.empty(num_bs_ant).astype(np.float)
    #ampUp = np.empty(num_bs_ant).astype(np.float)
    #calibAmp = np.empty(num_bs_ant).astype(np.float)  

    # CS16 makes sure the 4-bit lsb are samples are being sent
    bs_rx_stream = [sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1],  dict(WIRE=SOAPY_SDR_CS16)) for sdr in bsdr]
    ref_rx_stream = ref_sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1],  dict(WIRE=SOAPY_SDR_CS16))  

    Ts = 1/rate
    s_freq = 1e5
    s_time_vals = np.array(np.arange(0,numSamps)).transpose()*Ts
    pilot = np.exp(s_time_vals*1j*2*np.pi*s_freq).astype(np.complex64)*.2
    pad1 = np.array([0]*prefix_pad, np.complex64)
    pad2 = np.array([0]*postfix_pad, np.complex64)
    wbz = np.array([0]*(symSamp), np.complex64)
    pilot1 = np.concatenate([pad1,pilot,pad2]) if not second_channel else wbz  
    pilot2 = np.concatenate([pad1,pilot,pad2]) if second_channel else wbz
  
    # configure tdd mode 
    frameLen = len(bsdr) + 1 + 1 + 1;
    for i,sdr in enumerate(bsdr):
        bsched = "RG"+''.join("G"*i)+"P"+''.join("G"*(frameLen-i-3))
        print("node %d schedule: %s" % (i,bsched))
        bconf = {"tdd_enabled": True, 
                 "frame_mode": "triggered", 
                 "symbol_size" : symSamp, 
                 "frames": [bsched], 
                 "max_frame": 0}
        sdr.writeSetting("TDD_CONFIG", json.dumps(bconf))

    msched = "PG"+''.join("R"*len(bsdr))+"G"
    print("ref node schedule: %s" % (msched))
    mconf = {"tdd_enabled": True, 
             "frame_mode": "triggered", 
             "symbol_size" : symSamp, 
             "frames": [msched], 
             "max_frame": 0}
    ref_sdr.writeSetting("TDD_CONFIG", json.dumps(mconf))

    for sdr in sdrs:
        sdr.writeSetting("TDD_MODE", "true")

    for sdr in sdrs:
        sdr.writeSetting("TX_SW_DELAY", str(30))

    replay_addr = 0
    for sdr in sdrs:
        sdr.writeRegisters("TX_RAM_A", replay_addr, cfloat2uint32(pilot1, order='QI').tolist())
        sdr.writeRegisters("TX_RAM_B", replay_addr, cfloat2uint32(pilot2, order='QI').tolist())

    flags = 0
    [sdr.activateStream(bs_rx_stream[i], flags, 0) for i,sdr in enumerate(bsdr)]
    ref_sdr.activateStream(ref_rx_stream, flags, 0)

    fig, axes= plt.subplots(nrows=len(serials), ncols=2)# figsize=(16,16))
    axes[0,0].set_title('Downlink')
    axes[0,1].set_title('Uplink')
    for i in range(len(serials)):
        #axes[i, 0].set_ylabel('Amplitude')
        axes[i, 0].set_ylim(-.1, .1)
        axes[i, 0].set_xlim(0, symSamp)
        axes[i, 0].legend(fontsize=10)
        #axes[i, 1].set_ylabel('Amplitude')
        axes[i, 1].set_ylim(-.1, .1)
        axes[i, 1].set_xlim(0, symSamp)
        axes[i, 1].legend(fontsize=10)


    interval = range(100,symSamp)
    lineI0 = [axes[i, 0].plot(np.real(uint32tocfloat(UpA[i][interval])), label='ChA(I) Ref->Node %d'%i)[0] for i in range(num_bs_ant)]
    lineQ0 = [axes[i, 0].plot(np.imag(uint32tocfloat(UpA[i][interval])), label='ChA(Q) Ref->Node %d'%i)[0] for i in range(num_bs_ant)]  
    lineI1 = [axes[i, 1].plot(np.real(uint32tocfloat(DnA[i][interval])), label='ChA(I) Node %d->Ref'%i)[0] for i in range(num_bs_ant)]  
    lineQ1 = [axes[i, 1].plot(np.imag(uint32tocfloat(DnA[i][interval])), label='ChA(Q) Node %d->Ref'%i)[0] for i in range(num_bs_ant)]  
                                                                                                                  
    fig.show()
    #print("Timestamps:")
    #for sdr in sdrs:
    #    print(hex(SoapySDR.timeNsToTicks(sdr.getHardwareTime(""), rate)))
    plot_size = len(interval)
    H1 = [np.empty(plot_size).astype(np.complex64) for r in range(num_bs_ant)]
    H2 = [np.empty(plot_size).astype(np.complex64) for r in range(num_bs_ant)]
    calib = [np.empty(plot_size).astype(np.complex64) for r in range(num_bs_ant)]

    fg,ax = plt.subplots(nrows=2,ncols=1) #, figsize=(16,8))
    ax[0].set_ylabel('calibration amplitude')
    ax[0].set_xlim(interval[0],interval[-1])
    ax[0].set_ylim(-.1,.1)
    ax[1].set_ylabel('calibration phase')
    ax[1].set_xlim(interval[0],interval[-1])
    ax[1].set_ylim(-np.pi,np.pi)
    line0 = [ax[0].plot([0]*plot_size, label='Amplitude %d'%i)[0] for i in range(num_bs_ant)]
    line1 = [ax[1].plot([0]*plot_size, label='Phase %d'%i)[0] for i in range(num_bs_ant)]  
    fg.show() 

    signal.signal(signal.SIGINT, signal_handler)
    #def animate(i):
    while(running):
        if hub_serial != "":
            hub.writeSetting("TRIGGER_GEN", "")
        else:
            bsdr[0].writeSetting("TRIGGER_GEN", "")

        for i,sdr in enumerate(bsdr):
            sdr.readStream(bs_rx_stream[i], [UpA[i], UpB[i]], symSamp)
        for i in range(len(bsdr)):
            ref_sdr.readStream(ref_rx_stream, [DnA[i], DnB[i]], symSamp)
        for i in range(num_bs_ant):
            H1[i] = uint32tocfloat(DnA[i][interval] if not second_channel else DnB[i][interval])
            H2[i] = uint32tocfloat(UpA[i][interval] if not second_channel else UpB[i][interval])
            calib[i] = H1[i]*np.conj(H2[i])
            #phaseDeltaDn[i] = np.mean(np.angle(DnA[i][interval] * np.conj(pilot[interval])))
            #phaseDeltaUp[i] = np.mean(np.angle(UpA[i][interval] * np.conj(pilot[interval])))
            #calibPhase[i]   = np.mean(np.angle(DnA[i][interval] * np.conj(UpA[i][interval])))
            #ampDn[i]        = np.sum(np.abs(DnA[i][interval]))/np.sum(np.abs(pilot[interval]))
            #ampUp[i]        = np.sum(np.abs(UpA[i][interval]))/np.sum(np.abs(pilot[interval]))
            #calibAmp[i]     = np.sum(np.abs(UpA[i][interval]))/np.sum(np.abs(DnA[interval]))
            #np.set_printoptions(precision=2)
            lineI0[i].set_ydata(np.real(H1[i]))
            lineQ0[i].set_ydata(np.imag(H1[i]))
            lineI1[i].set_ydata(np.real(H2[i]))
            lineQ1[i].set_ydata(np.imag(H2[i]))
            line0[i].set_ydata(np.abs(calib[i]))
            line1[i].set_ydata((np.angle(calib[i])))
        fig.canvas.draw()
        fg.canvas.draw()
        fg.show()
        fig.show()
        #return line0, line1,
    #anim = animation.FuncAnimation(fig, animate, init_func=None, 
    #                           frames=100, interval=100, blit=True)

    #plt.show()

    tdd_conf = {"tdd_enabled" : False}
    for sdr in sdrs:
        sdr.writeSetting("RESET_DATA_LOGIC", "")
        sdr.writeSetting("TDD_CONFIG", json.dumps(tdd_conf))
        sdr.writeSetting("TDD_MODE", "false")

    ref_sdr.closeStream(ref_rx_stream)
    for i,sdr in enumerate(bsdr):
        sdr.closeStream(bs_rx_stream[i])
    ref_sdr = None
    bsdr = None


def main():
    parser = OptionParser()
    parser.add_option("--hub-serial", type="string", dest="hub_serial", help="hub serial number of the base station", default="")
    parser.add_option("--serials", type="string", dest="serials", help="serial number of all devices, reference node will be first serial", default="../IrisUtils/data_in/bs_serials.txt")
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=1e6)
    parser.add_option("--txgain", type="float", dest="txgain", help="Optional Tx gain (dB) w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]", default=30.0)
    parser.add_option("--rxgain", type="float", dest="rxgain", help="Optional Rx gain (dB) w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]", default=30.0)
    parser.add_option("--freq", type="float", dest="freq", help="Optional Tx freq (Hz)", default=2.5e9)
    parser.add_option("--numSamps", type="int", dest="numSamps", help="Num samples to receive", default=512)
    parser.add_option("--prefix-pad", type="int", dest="prefix_length", help="prefix padding length for beacon and pilot", default=82)
    parser.add_option("--postfix-pad", type="int", dest="postfix_length", help="postfix padding length for beacon and pilot", default=68)
    parser.add_option("--second-channel", action="store_true", dest="second_channel", help="transmit/receive from 2nd channel",default=False)
    (options, args) = parser.parse_args()

    bserials = []
    with open(options.serials, "r") as f:
        for line in f.read().split():
            if line[0] != '#':
                bserials.append(line)
            else:
                continue      

    ref_serial = bserials[0]
    if len(bserials) > 1:
        bserials = bserials[1::]
    else:
        print("Need more than one board (in bs_serials.txt) to run calibration")
        sys.exit(0)

    print("SERIALS (EXCEPT REFERENCE NODE): {}".format(bserials))
    print("REFERENCE NODE SERIAL: {}".format(ref_serial))

    calibrate_array(
        hub_serial=options.hub_serial,
        serials=bserials,
        ref_serial=ref_serial,
        rate=options.rate,
        freq=options.freq,
        txgain=options.txgain,
        rxgain=options.rxgain,
        numSamps=options.numSamps,
        prefix_pad=options.prefix_length,
        postfix_pad=options.postfix_length,
        second_channel=options.second_channel
    )


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
    except Exception as e:
        print(e)
        exit()
