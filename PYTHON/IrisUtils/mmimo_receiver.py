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


    Currently only supports one cell system

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
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import gridspec
from hdf_dump import *
# from radio_lib import *
from find_lts import *
from generate_sequence import *
from ofdmtxrx import *


#########################################
#           Global Parameters           #
#########################################
FFT_OFFSET = 4
APPLY_CFO_CORR = 1
APPLY_SFO_CORR = 1
APPLY_PHASE_CORR = 1


#########################################
#             Create Plots              #
#########################################



#########################################
#              Functions                #
#########################################
def read_rx_samples(rx_mode, filename):
    """
    Read IQ samples received at Base Station.

    Input:
        rx_mode - Two modes: simulation and real-time
                    a) Sim:             Read previously collected HDF5 file
                                        and run mMIMO receiver on the collected
                                        data
                    b) Real-Time (OTA): Call radio library to retrieve samples
                                        as they are received

        filename - Name of file to process

    Output:
        metadata - Attributes from hdf5 file
        samples  - Raw samples from pilots and data
    """

    if rx_mode == "SIM":
        hdf5 = hdfDump(filename)
        hdf5.get_hdf5()
        hdf5.parse_hdf5()

        # Check which data we have available
        data_types_avail = []
        pilots_avail = bool(hdf5.data['Pilot_Samples'])
        ul_data_avail = bool(hdf5.data['UplinkData'])

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


def pilot_finder(samples, pilot_type):
    """
    Find pilots from clients to each of the base station antennas

    Input:
        metadata - Attributes from hdf5 file
        samples  - Raw samples from pilots and data.
                   Dimensions: (frames, numCells, numClients, numAntennasAtBS, numSamplesPerSymbol*2)
    """

    if pilot_type == 'lts':
        lts_thresh = 0.8
        best_pk, lts_pks, lts_corr = find_lts(samples, thresh=lts_thresh)
        lts, lts_f = generate_training_seq(preamble_type='lts', cp=32, upsample=1)
        lts_syms_len = len(lts)

        # We'll need the transmitted version of the pilot (for channel estimation, for example)
        tx_pilot = [lts, lts_f]

        # Check if LTS found
        if not best_pk:
            print("SISO_OFDM: No LTS Found! Continue...")
            pilot = np.array([])
            return pilot, tx_pilot
        # If beginning of frame was not captured in current buffer
        if (best_pk - lts_syms_len) < 0:
            print("TOO EARLY. Continue... ")
            pilot = np.array([])
            return pilot, tx_pilot

        # Get pilot
        lts_start = best_pk - lts_syms_len + 1  # where LTS-CP start
        pilot = samples[lts_start:best_pk+1]

    else:
        raise Exception("Only LTS Pilots supported at the moment")

    return pilot, tx_pilot


def estimate_channel(this_pilot, tx_pilot, ofdm_obj):
    """
    Estimate channel from received pilots

    Input:
        this_pilot - received pilot (vector)
        tx_pilot   - time (tx_pilot[0]) and frequency (tx_pilot[1]) domain transmitted pilot sequences

    Output:
        chan_est - Vector containing channel estimates computed from this particular RX pilot (dim: fft_size x 1)
    """
    global FFT_OFFSET, APPLY_CFO_CORR

    # Retrieve sent pilot (freq domain)
    pilot_freq = tx_pilot[1]

    # Apply coarse CFO Correction
    lts_start = 0
    lts_syms_len = len(this_pilot)

    if APPLY_CFO_CORR:
        coarse_cfo_est = ofdm_obj.cfo_correction(this_pilot, lts_start, lts_syms_len, FFT_OFFSET)
    else:
        coarse_cfo_est = 0

    correction_vec = np.exp(-1j * 2 * np.pi * coarse_cfo_est * np.array(range(0, len(this_pilot))))
    pilot_cfo = this_pilot * correction_vec

    # Channel estimation
    # Get LTS again (after CFO correction)
    lts = pilot_cfo[lts_start: lts_start + lts_syms_len]
    lts_1 = lts[-64 + -FFT_OFFSET + np.array(range(97, 161))]
    lts_2 = lts[-FFT_OFFSET + np.array(range(97, 161))]

    # Average 2 LTS symbols to compute channel estimate
    chan_est = np.fft.ifftshift(pilot_freq) * (np.fft.fft(lts_1) + np.fft.fft(lts_2)) / 2

    return chan_est


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
    metadata, samples = read_rx_samples(rx_mode, filename)

    # OFDM object
    ofdm_obj = ofdmTxRx()

    # Get attributes we care about
    pilot_type = metadata['PILOT_SEQ']
    pilot_samples = samples['PILOT_SAMPS']
    data_samples = samples['UL_SAMPS']
    num_cells = metadata['BS_NUM_CELLS']
    num_cl = metadata['NUM_CLIENTS']
    num_bs_ant = metadata['BS_NUM_ANT']
    sym_len = metadata['SYM_LEN']
    fft_size = metadata['FFT_SIZE']
    cl_frame_sched = metadata['CL_FRAME_SCHED']
    pilot_dim = pilot_samples.shape
    num_frames = pilot_dim[0]

    # Verify dimensions
    assert pilot_dim[1] == num_cells
    assert pilot_dim[2] == num_cl
    assert pilot_dim[3] == num_bs_ant
    assert pilot_dim[4] == 2 * sym_len  # No complex values in HDF5, x2 to account for IQ

    # Prepare samples to iterate over all received frames
    chan_est = np.empty([num_cells, num_bs_ant, num_cl, num_frames, fft_size], dtype=complex)
    if rx_mode == "SIM":
        for frameIdx in range(num_frames):
            for clIdx in range(num_cl):
                for antIdx in range(num_bs_ant):
                    # Put I/Q together
                    # Dims pilots: (frames, numCells, numClients, numAntennasAtBS, numSamplesPerSymbol*2)
                    I = pilot_samples[frameIdx, num_cells-1, clIdx, antIdx, 0:sym_len*2-1:2]/2**16
                    Q = pilot_samples[frameIdx, num_cells-1, clIdx, antIdx, 1:sym_len*2:2]/2**16
                    IQ = I + (Q * 1j)

                    # Find potential pilots
                    this_pilot, tx_pilot = pilot_finder(IQ, pilot_type)
                    if this_pilot.size == 0:
                        continue

                    # Channel estimation from pilots
                    chan_est[num_cells-1, antIdx, clIdx, frameIdx, :] = estimate_channel(this_pilot, tx_pilot, ofdm_obj)

                    # Get data samples
                    # How many data symbols transmitted by each client?
                    this_cl_sched = cl_frame_sched[clIdx]
                    sym_found_idx = this_cl_sched.find('U')
                    sym_found_idx2 = this_cl_sched.find('G')
                    num_ul_syms = len(sym_found_idx)

                    # Dims data: (frames, numCells, ulSymsPerFrame, numAntennasAtBS, numSamplesPerSymbol*2)
                    for ulSymIdx in range(num_ul_syms):
                        I = data_samples[frameIdx, num_cells-1, clIdx, antIdx, 0:sym_len*2-1:2]/2**16
                        Q = data_samples[frameIdx, num_cells-1, clIdx, antIdx, 1:sym_len*2:2]/2**16


        stop = 1


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

