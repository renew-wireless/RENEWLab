#!/usr/bin/python3
"""
 plt_simp.py

  Plot output from SOUNDER_TXRX.py demo script
  The script will read the following files:

  "../DEMOS/data_out/rxpilot_sounder"
  "../DEMOS/data_out/rxdata_sounder"

  for both RF chains (if both chains are being used in SOUNDER_TXRX.py)
  and plot the waveforms


  Example:
    python3 plt_simp.py

---------------------------------------------------------------------
 Copyright (c) 2018-2019, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import pickle
import sys

sys.path.append('../DEMOS/data_out/')

import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
from generate_sequence import *
from type_conv import *
from file_rdwr import *
from find_lts import *


def plot_data(pilot, rxdata, channelnum, framelen=512, framenum=100, pilotsymnum=1, uplinksymnum=0):
    # dc removal
    sampsRx = pilot[0].flatten()
    # find LTS assumes signal was generated according to this:
    # generate_training_seq(preamble_type='lts', cp=32, upsample=1)
    # a, b, peaks = lts.findLTS(sampsRx)
    a, b, peaks = find_lts(sampsRx, thresh=0.8, us=1, cp=32)

    fig, axes= plt.subplots(nrows=pilotsymnum+uplinksymnum+2+1, ncols=1)
    sig_power = np.empty([pilotsymnum,framenum], dtype=float)
    sig_phase = np.empty([pilotsymnum,framenum], dtype=float)
    offset = 82+128
    sc = 2 # subcarrier
    for i, ax in enumerate(axes):
        if i < pilotsymnum:
            label = "pilot %d " % i
            p = pilot[i].flatten()
            ax.plot(np.real(p), label=label+"I")
            ax.plot(np.imag(p), label=label+"Q")
            ax.legend()
            for k in range(framenum):
                signal = pilot[i,k,offset:offset+64]
                sig_power[i, k] = np.mean(signal * np.conj(signal))
                sig_phase[i, k] = np.angle(np.fft.fftshift(np.fft.fft(signal)))[sc]
        elif uplinksymnum > 0 and i < pilotsymnum + uplinksymnum:
            j = i - pilotsymnum
            d = rxdata[j].flatten()
            label = "rx data %d " % j
            ax.plot(np.real(d), label=label+"I")
            ax.plot(np.imag(d), label=label+"Q")
            ax.legend()
        elif i == pilotsymnum+uplinksymnum:
            for m in range(pilotsymnum):
                ax.plot(sig_power[m], label='pilot %d power' % m )
            ax.legend()
        elif i == pilotsymnum+uplinksymnum+1:
            for m in range(pilotsymnum):
                ax.plot(sig_phase[m], label='pilot %d phase sc %d' % (m, sc) )
            ax.legend()
        else:
            ax.plot(np.real(peaks), label='Corr Peaks')
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        if len(sys.argv) != 6:
            print("format: ./plt_simp.py framelen framenum channelnum pilotsymnum uplinksymnum")
            sys.exit(0)
        framelen=int(sys.argv[1])
        framenum=int(sys.argv[2])
        channelnum = int(sys.argv[3])
        pilotsymnum = int(sys.argv[4])
        uplinksymnum = int(sys.argv[5])
    else:
        framelen = 662
        framenum = 100
        channelnum = 1
        pilotsymnum = 1
        uplinksymnum = 0
    if pilotsymnum > 0:
        pilot = uint32tocfloat(read_from_file("../DEMOS/data_out/rxpilot_sounder", framelen*framenum*pilotsymnum, 0))
        pilot = np.reshape(pilot, (framenum, pilotsymnum, framelen))
        pilot = np.transpose(pilot, (1,0,2)) 
        if channelnum == 2:
            pilotB = uint32tocfloat(read_from_file("../DEMOS/data_out/rxpilotB_sounder", framelen*framenum*pilotsymnum, 0))
            pilotB = np.reshape(pilotB, (framenum, pilotsymnum, framelen))
            pilotB = np.transpose(pilotB, (1,0,2))
            pilot = np.concatenate((pilot,pilotB), axis=0)
    else:
        pilot=None
    if uplinksymnum > 0:
        rxdata = uint32tocfloat(read_from_file("../DEMOS/data_out/rxdata_sounder", framelen*framenum*uplinksymnum, 0))
        rxdata = np.reshape(rxdata, (framenum, uplinksymnum, framelen))
        rxdata = np.transpose(rxdata, (1,0,2)) 
        if channelnum == 2:
            rxdataB = uint32tocfloat(read_from_file("../DEMOS/data_out/rxdataB_sounder", framelen*framenum*uplinksymnum, 0))
            rxdataB = np.reshape(rxdataB, (framnum, uplinksymnum, framelen))
            rxdataB = np.transpose(rxdataB, (1,0,2)) 
            rxdata = np.concatenate((rxdata, rxdataB), axis=0)
    else:
        rxdata=None
    plot_data(pilot, rxdata, channelnum, framelen, framenum, pilotsymnum, uplinksymnum)
    # sio.savemat('rxTest.mat', {'pilot':pilot, 'rxdata':rxdata})

 
