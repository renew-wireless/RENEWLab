#!/usr/bin/python3
"""
 mMIMO_receiver.py

 Simple massive MIMO receiver.
 
 -Two modes: simulation and real-time
     a) Sim:       Read HDF5 file and run mMIMO receiver on the
                   collected data
     b) Real-Time: Continuously read RX buffer as UEs are transmitting

 - Procedure:
     a) Read IQ
     b) Find Pilots
     c) Channel Estimator (Get CSI)
     d) ZF/MMSE Weight Computation
     e) Separate Streams (Demultiplexing)
     f) Demodulate data if available
     g) Plotter (Optional: Animated)
         i)   Rx Signal Full (IQ)
         ii)  Pilots
         iii) Constellation



     TODO:
         2) Create Plotter Class ??

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
import numpy as np
import h5py
import matplotlib.pyplot as plt
from hdf_dump import *
from radio_lib import *


#########################################
#             Create Plots              #
#########################################
matplotlib.rcParams.update({'font.size': 10})
# fig = plt.figure(figsize=(20, 8), dpi=120)
fig = plt.figure(figsize=(10, 20), dpi=120)
fig.subplots_adjust(hspace=.4, top=.97, bottom=.03)
gs = gridspec.GridSpec(ncols=4, nrows=5)

ax1 = fig.add_subplot(gs[0, :])
ax1.grid(True)
ax1.set_title('TX Signal')
title = ax1.text(0.5, 1, '|', ha="center")
ax1.set_ylabel('Magnitude')
ax1.set_xlabel('Sample index')
line1, = ax1.plot([], [], label='RFA', animated=True)
ax1.set_ylim(0, 1)
ax1.set_xlim(0, FIG_LEN)
ax1.legend(fontsize=10)

ax2 = fig.add_subplot(gs[1, :])
ax2.grid(True)
ax2.set_title('RX Signal')
ax2.set_xlabel('Sample index')
ax2.set_ylabel('I/Q')
line2, = ax2.plot([], [], label='I - RFA', animated=True)
line3, = ax2.plot([], [], label='Q - RFA', animated=True)
ax2.set_ylim(-1, 1)
ax2.set_xlim(0, FIG_LEN)
ax2.legend(fontsize=10)

ax3 = fig.add_subplot(gs[2, :])
ax3.grid(True)
ax3.set_title('RX Signal')
ax3.set_xlabel('Sample index')
ax3.set_ylabel('Magnitude')
line4, = ax3.plot([], [], label='RFA', animated=True)
line8, = ax3.plot([], [], '--k', label='Payload Start', animated=True)  # markers
line9, = ax3.plot([], [], '--r', label='Payload End', animated=True)  # markers
line10, = ax3.plot([], [], '--g', label='LTS Start', animated=True)  # markers
ax3.set_ylim(0, 1)
ax3.set_xlim(0, FIG_LEN)
ax3.legend(fontsize=10)

ax4 = fig.add_subplot(gs[3, :])
ax4.grid(True)
ax4.set_title('Correlation Peaks')
ax4.set_xlabel('Sample index')
ax4.set_ylabel('')
line5, = ax4.plot([], [], label='RFA', animated=True)
line11, = ax4.plot([], [], '--r', label='Thresh', animated=True)  # markers
ax4.set_ylim(0, 5)
ax4.set_xlim(0, FIG_LEN)
ax4.legend(fontsize=10)

ax5 = fig.add_subplot(gs[4, 0:2])
ax5.grid(True)
ax5.set_title('TX/RX Constellation')
ax5.set_xlabel('')
ax5.set_ylabel('')
line6, = ax5.plot([], [], 'ro', label='TXSym', animated=True)
line7, = ax5.plot([], [], 'bx', label='RXSym', animated=True)
ax5.set_ylim(-1.5, 1.5)
ax5.set_xlim(-2.8, 2.8)
ax5.legend(fontsize=10)

ax6 = fig.add_subplot(gs[4, 2:4])
ax6.grid(True)
ax6.set_title('Magnitude Channel Estimates')
ax6.set_xlabel('Baseband Freq.')
ax6.set_ylabel('')
line12, = ax6.step([], [])
ax6.set_ylim(-0.1, 5)
ax6.set_xlim(-10, 10)
ax6.legend(fontsize=10)


#########################################
#              Functions                #
#########################################
def init():
    """ Initialize plotting objects """
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    line4.set_data([], [])
    line5.set_data([], [])
    line6.set_data([], [])
    line7.set_data([], [])
    line8.set_data([], [])
    line9.set_data([], [])
    line10.set_data([], [])
    line11.set_data([], [])
    line12.set_data([], [])
    return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12
 
    
def read_rx_samples(rx_mode, filename):
    """
    Read IQ samples received at Base Station.
    Two modes: simulation and real-time
     a) Sim:             Read previously collected HDF5 file and run mMIMO
                         receiver on the collected data
     b) Real-Time (OTA): Call radio library to retrieve samples as they
                         are received
    """
    if rx_mode == "SIM":
        hdf5 = hdfDump(filename)
        hdf5.get_hdf5()
        hdf5.parse_hdf5()
        raw_data = hdf5.data

        # Check which data we have available
        data_types_avail = []
        pilots_avail = bool(raw_data['Pilot_Samples'])
        ul_data_avail = bool(raw_data['UplinkData'])

        if pilots_avail:
            data_types_avail.append("PILOTS")
            print("PILOT Data Available")
        if ul_data_avail:
            data_types_avail.append("UL_DATA")
            print("Uplink Data Available")

        # Empty structure
        if not data_types_avail:
            raise Exception(' **** No pilots or uplink data found **** ')

        # Retrieve attributes
        hdf5.get_attributes()
        metadata = hdf5.metadata

        # Get raw samples
        hdf5.get_samples(data_types_avail)
        samples = hdf5.samples
        
    else:
        # FIXME - retrieve config data (metadata) and actual samples
        radLib = radioLib()
        samples = radLib.data
        
    return metadata, samples


def pilot_finder(meta_pilots, samps_pilots):
    """

    return:
    """
    pilot_type = meta_pilots['PILOT_TYPE']
    pilot_cp = meta_pilots['PILOT_CP']
    pilot_tx = generate_training_seq(preamble_type=pilot_type, seq_length=[], cp=32, upsample=1, reps=[])
    pilot_rx = samps_pilots
    
    return x


def estimate_channel():
    """

    return:
    """
    x=1
    return x


def bf_weights_calc():
    """

    return:
    """
    x=1
    return x


def demultiplexing():
    """

    return:
    """
    x=1
    return x


def demodulate_data():
    """

    return:
    """
    x=1
    return x


def plotter():
    """

    return:
    """
    # Compute CSI from IQ samples
    # csi, samps = samps2csi(samples, num_ue_tmp, symbol_length, symbol_length, offset=0, bound=0)

    # Verify default_frame does not exceed max number of collected frames
    frame_to_plot = min(default_frame, samps.shape[0])

    # Plotter
    # Samps Dimensions: (Frame, Cell, User, Pilot Rep, Antenna, Sample)
    fig, axes = plt.subplots(nrows=5, ncols=len(data_types_avail))
    axes[0, idx].set_title('PILOTS - Cell 0')
    axes[0, idx].set_ylabel('Frame %d ant 0 (Re)' % frame_to_plot)
    axes[0, idx].plot(np.real(samps[frame_to_plot, 0, 0, 0, 1, :]))

    axes[1, idx].set_ylabel('Frame %d ant 1 (Re)' % frame_to_plot)
    axes[1, idx].plot(np.real(samps[frame_to_plot, 0, 0, 0, 1, :]))

    axes[2, idx].set_ylabel('All Frames ant 0 (Re)')
    axes[2, idx].plot(np.real(samps[:, 0, 0, 0, 0, :]).flatten())

    axes[3, idx].set_ylabel('All Frames ant 1 (Re)')
    axes[3, idx].plot(np.real(samps[:, 0, 0, 0, 1, :]).flatten())

    axes[4, idx].set_ylabel('Amplitude')
    for i in range(samps.shape[4]):
        axes[4, idx].plot(np.mean(np.abs(samps[:, 0, 0, 0, i, :]), axis=1).flatten())
    axes[4, idx].set_xlabel('Sample')
    plt.show()

def rx_app(rx_mode, filename):
    """
    Main function

    Input:
        a) rx_mode:  SIM or OTA
        b) filename: HDF5 file to read from 
    """

    # Read Received Samples
    meta_pilots, samps_pilots, meta_ulData, samps_ulData = read_rx_samples(rx_mode, filename)

    # Find potential pilots
    pilot_finder(meta_pilots, samps_pilots)

    
#########################################
#                 Main                  #
#########################################
if __name__ == '__main__':
    if len(sys.argv) > 1:
        if sys.argv[1] == "-h":
            print('format: ./mMIMO_receiver.py <mode (SIM/OTA)> <filename (if SIM, empty otherwise)>')
            print('SIM: Simulation mode. Need to specify HDF5 file to read from')
            print('OTA: Real-Time mode. Clients must be active. Test configuration specified in ???????????')  # FIXME!!
            sys.exit(0)
            
        rx_mode = sys.argv[1]
        if rx_mode == "SIM":
            if len(sys.argv) < 2:
                raise Exception("SIM mode requires HDF5-filename argument: ./mMIMO_receiver.py SIM <filename>")
            filename = sys.argv[2]
        elif rx_mode == "OTA":
            filename = []
        else:
            raise Exception("Mode not recognized. Options: SIM/OTA")    

        # Rx Application
        rx_app(rx_mode, filename)

    else:
        raise Exception('Please specify mode. For more details: ./mMIMO_receiver.py -h')

