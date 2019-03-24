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
from optparse import OptionParser
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
        samples    - Raw samples from pilots and data.
                     Dimensions: (frames, numCells, numClients, numAntennasAtBS, numSamplesPerSymbol*2)
        pilot_type - Type of TX pilot (e.g., 802.11 LTS)

    Output:
        pilot     - Received pilot (from multiple clients)
        tx_pilot  - Transmitted pilot (same pilot sent by all clients)
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


def estimate_channel(this_pilot, tx_pilot, ofdm_obj, user_params):
    """
    Estimate channel from received pilots

    Input:
        this_pilot  - received pilot (vector)
        tx_pilot    - time (tx_pilot[0]) and frequency (tx_pilot[1]) domain transmitted pilot sequences (vectors)
        ofdm_obj    - OFDM object
        user_params - set of parameters defined by user. See main function

    Output:
        chan_est - Vector containing channel estimates computed from this particular RX pilot (dim: fft_size x 1)
    """
    fft_offset = user_params[5]
    apply_cfo_corr = user_params[2]

    # Retrieve sent pilot (freq domain)
    pilot_freq = tx_pilot[1]

    # Apply coarse CFO Correction
    lts_start = 0
    lts_syms_len = len(this_pilot)

    if apply_cfo_corr:
        coarse_cfo_est = ofdm_obj.cfo_correction(this_pilot, lts_start, lts_syms_len, fft_offset)
    else:
        coarse_cfo_est = 0

    correction_vec = np.exp(-1j * 2 * np.pi * coarse_cfo_est * np.array(range(0, len(this_pilot))))
    pilot_cfo = this_pilot * correction_vec

    # Channel estimation
    # Get LTS again (after CFO correction)
    lts = pilot_cfo[lts_start: lts_start + lts_syms_len]
    lts_1 = lts[-64 + -fft_offset + np.array(range(97, 161))]
    lts_2 = lts[-fft_offset + np.array(range(97, 161))]

    # Average 2 LTS symbols to compute channel estimate
    chan_est = np.fft.ifftshift(pilot_freq) * (np.fft.fft(lts_1) + np.fft.fft(lts_2)) / 2

    return chan_est


def beamforming_weights(chan_est, user_params):
    """
    Compute beam steering weights

    Input:
        chan_est    - Channel estimate vector.
                      Dimensions: chan_est[current cell, num clients, num antennas, current frame, fft_size]
        user_params - Set of parameters defined by user. See main function

    Output
        WW - BF weights matrix. Dimensions: WW[num BS antennas, num clients, num subcarriers]
    """
    # Collapse into dimensions [numCl, numBSant, fft_size]
    H_tmp = np.squeeze(chan_est)
    H_tmp_shape = H_tmp.shape
    num_sc = H_tmp_shape[2]
    num_ant = H_tmp_shape[1]
    num_cl = H_tmp_shape[0]

    bf_scheme = user_params[1]
    power_norm = 0

    if bf_scheme == "ZF":
        # Zero Forcing
        WW = np.empty((num_ant, num_cl, num_sc), dtype=complex)
        for scIdx in range(num_sc):
            H = H_tmp[:, :, scIdx]
            HH = np.matrix.getH(H)
            W = HH.dot(np.linalg.pinv(H.dot(HH)))

            if power_norm:
                # Normalize (equal power allocation across users)
                P = 1
                for k in range(num_cl):
                    W[:, k] = np.sqrt(P / num_cl) * (W[:, k] / np.linalg.norm(V[:, k]))

            WW[:, :, scIdx] = W
    else:
        raise Exception("Only Zero Forcing is currently implemented")

    return WW


def demultiplex(samples, bf_weights, user_params, metadata):
    """
    Separate data streams by applying beamforming weights previously computed.
    Requires us to perform FFT prior to applying weights

    Input:
        samples     - IQ data. Dimensions: samples[num BS antennas, num samps including padding]
        bf_weights  - Beamforming weights. Dimensions: bf_weights[num antennas, num clients, num subcarriers]
        user_params - set of parameters defined by user. See main function
        metadata    - Attributes from hdf5 file

    Output
        streams     - Per client data streams. Dimensions: streams[num clients, num subcarriers, num ofdm symbols]
    """

    w_dim = bf_weights.shape
    num_sc = w_dim[2]
    num_cl = int(metadata['NUM_CLIENTS'])
    num_ant = int(metadata['BS_NUM_ANT'])
    data_cp_len = int(metadata['CP_LEN'])
    fft_size = int(metadata['FFT_SIZE'])
    num_samps = int(metadata['SYM_LEN_NO_PAD'])
    prefix = int(metadata['PREFIX_LEN'])
    ofdm_size = fft_size + data_cp_len
    n_ofdm_syms = num_samps//ofdm_size
    fft_offset = user_params[5]

    # Remove padding
    payload = samples[:, prefix+1:prefix+1+num_samps]

    # Reshape into matrix. Dim: [num clients, num_sc+cp, num_ofdm]
    payload_samples_mat_cp = np.reshape(payload, (num_ant, (num_sc + data_cp_len), n_ofdm_syms), order="F")

    # Remove cyclic prefix
    payload_samples_mat = payload_samples_mat_cp[:, data_cp_len - fft_offset + 1 + np.array(range(0, num_sc)), :]

    # FFT
    rxSig_freq = np.empty((num_ant, payload_samples_mat.shape[1], payload_samples_mat.shape[2]), dtype=complex)
    for antIdx in range(num_ant):
        rxSig_freq[antIdx, :, :] = np.fft.fft(payload_samples_mat[antIdx, :, :], n=num_sc, axis=0)

    # Demultiplexing (iterate over clients and ofdm symbols)
    x = np.empty((num_cl, rxSig_freq.shape[1], n_ofdm_syms), dtype=complex)
    for symIdx in range(n_ofdm_syms):
        for scIdx in range(rxSig_freq.shape[1]):
            this_w = np.squeeze(bf_weights[:, :, scIdx])
            y = np.squeeze(rxSig_freq[:, scIdx, symIdx])
            x[:, scIdx, symIdx] = np.transpose(np.dot(np.transpose(this_w), y))

    streams = x
    return streams


def demodulate_data(streams, ofdm_obj, user_params, metadata):
    """
    Given complex data streams for all users, demodulate signals

    Input:
        streams     - Per client data streams. Dimensions: streams[num clients, num subcarriers, num ofdm symbols]
        ofdm_obj    - OFDM object
        user_params - Set of parameters defined by user. See main function
        metadata    - Attributes from hdf5 file

    Output
        demod_syms  - Demodulated data symbols
        symbol_err  - Data symbol error (needs TX data to determine this)
    """

    fft_size = int(metadata['FFT_SIZE'])
    data_cp_len = int(metadata['CP_LEN'])
    num_samps = int(metadata['SYM_LEN_NO_PAD'])
    num_sc = int(metadata['FFT_SIZE'])
    mod_order_str = metadata['CL_MODULATION']
    ofdm_size = fft_size + data_cp_len
    n_ofdm_syms = num_samps//ofdm_size
    n_data_syms = n_ofdm_syms * len(data_sc)

    if mod_order_str == "BPSK":
        mod_order = 2
    elif mod_order_str == "QPSK":
        mod_order = 4
    elif mod_order_str == "16QAM":
        mod_order = 16
    elif mod_order_str == "64QAM":
        mod_order = 64

    # REMOVE ME !!!!! WE NEED TO OBTAIN THIS FROM HDF5 FILE INSTEAD
    # pilot_sc = metadata['OFDM_PILOT_SC']
    # pilots_matrix = metadata['OFDM_PILOT_MATRIX']
    # tx_data = metadata['OFDM_TX_DATA']
    # = metadata['OFDM_DATA_SC']

    data_subcarriers = list(range(1, 7)) + list(range(8, 21)) + list(range(22, 27)) + \
                       list(range(38, 43)) + list(range(44, 57)) + list(range(58, 64))
    pilot_sc = [7, 21, 43, 57]
    pilots = np.array([1, 1, -1, 1]).reshape(4, 1, order="F")
    pilots_matrix = np.matlib.repmat(pilots, 1, n_ofdm_syms)
    # REMOVE ME END !!!!! WE NEED TO OBTAIN THIS FROM HDF5 FILE INSTEAD


    # Correction Flags
    apply_sfo_corr = user_params[3]
    apply_phase_corr = user_params[4]

    for clIdx in range(streams.shape[0]):

        rxSig_freq_eq = streams[clIdx, :, :]
        # Apply SFO Correction
        if apply_sfo_corr:
            rxSig_freq_eq = ofdm_obj.sfo_correction(rxSig_freq_eq, pilot_sc, pilots_matrix, n_ofdm_syms)
        else:
            sfo_corr = np.zeros((num_sc, n_ofdm_syms))

        # Apply phase correction
        if apply_phase_corr:
            phase_error = ofdm_obj.phase_correction(rxSig_freq_eq, pilot_sc, pilots_matrix)
        else:
            phase_error = np.zeros((1, n_ofdm_syms))

        phase_corr_tmp = np.matlib.repmat(phase_error, num_sc, 1)
        phase_corr = np.exp(-1j * phase_corr_tmp)
        rxSig_freq_eq_phase = rxSig_freq_eq * phase_corr
        rxSymbols_mat = rxSig_freq_eq_phase[data_sc, :]

        # Demodulation
        rxSymbols_vec = np.reshape(rxSymbols_mat, n_data_syms, order="F")       # Reshape into vector
        rx_data = ofdm_obj.demodulation(rxSymbols_vec, mod_order)

        # print(" === STATS ===")
        symbol_err = np.sum(tx_data != rx_data)
        # print("Frame#: {} --- Symbol Errors: {} out of {} total symbols".format(pkt_count, symbol_err, n_data_syms))


    return demod_syms, symbol_err


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


def rx_app(filename, user_params):
    """
    Main function

    Input:
        a) filename    - HDF5 file to read from
        b) user_params - set of parameters defined by user. See main function

    Output:
        None
    """
    rx_mode = user_params[0]

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
    chan_est = np.empty([num_cells, num_cl, num_bs_ant, num_frames, fft_size], dtype=complex)
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
                    chan_est[num_cells-1, clIdx, antIdx, frameIdx, :] = estimate_channel(this_pilot, tx_pilot, ofdm_obj, user_params)

            # PER FRAME
            # Steering weight computation after collecting pilots at all antennas from all clients
            bf_weights = beamforming_weights(chan_est[num_cells-1, :, :, frameIdx, :], user_params)

            # Get data samples
            # How many data symbols transmitted by each client?
            this_cl_sched = cl_frame_sched[clIdx]
            num_ul_syms = this_cl_sched.count('U')

            # Dims data: (frames, numCells, ulSymsPerFrame, numAntennasAtBS, numSamplesPerSymbol*2)
            for ulSymIdx in range(num_ul_syms):
                I = data_samples[frameIdx, num_cells-1, ulSymIdx, :, 0:sym_len*2-1:2]/2**16
                Q = data_samples[frameIdx, num_cells-1, ulSymIdx, :, 1:sym_len*2:2]/2**16
                IQ = I + (Q * 1j)

                # Demultiplexing - Separate streams
                streams = demultiplex(IQ, bf_weights, user_params, metadata)
                rx_data = demodulate_data(streams, ofdm_obj, user_params, metadata)

                plotter(IQ, chan_est, bf_weights, streams, rx_data, user_params)
        stop = 1


#########################################
#                 Main                  #
#########################################
if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--mode",             type="string",          dest="mode",        default="SIM", help="Options: SIM/OTA [default: %default]")
    parser.add_option("--file",             type="string",          dest="file",        default="./data_in/Argos-2019-2-20-14-55-22_1x8x2.hdf5", help="HDF5 filename to be read in SIM mode [default: %default]")
    parser.add_option("--bfScheme",         type="string",          dest="bf_scheme",   default="ZF", help="Beamforming Scheme. Options: ZF (for now) [default: %default]")
    parser.add_option("--cfoCorrection",    action="store_true",    dest="cfo_corr",    default=True, help="Apply CFO correction [default: %default]")
    parser.add_option("--sfoCorrection",    action="store_true",    dest="sfo_corr",    default=False, help="Apply SFO correction [default: %default]")
    parser.add_option("--phaseCorrection",  action="store_true",    dest="phase_corr",  default=False, help="Apply phase correction [default: %default]")
    parser.add_option("--fftOfset",         type="int",             dest="fft_offset",  default=4,    help="FFT Offset:# CP samples for FFT [default: %default]")
    (options, args) = parser.parse_args()

    user_params = [options.mode,
                   options.bf_scheme,
                   options.cfo_corr,
                   options.sfo_corr,
                   options.phase_corr,
                   options.fft_offset
                   ]

    filename = options.file

    # Rx Application
    rx_app(filename, user_params)


