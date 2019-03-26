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
import threading
import signal
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
from plotter import *


#########################################
#              Global Vars              #
#########################################
running = True


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

        pilot_thresh = lts_thresh * np.max(lts_corr)

        # We'll need the transmitted version of the pilot (for channel estimation, for example)
        tx_pilot = [lts, lts_f]

        # Check if LTS found
        if not best_pk:
            print("SISO_OFDM: No LTS Found! Continue...")
            pilot = np.array([])
            return pilot, tx_pilot, lts_corr, pilot_thresh
        # If beginning of frame was not captured in current buffer
        if (best_pk - lts_syms_len) < 0:
            print("TOO EARLY. Continue... ")
            pilot = np.array([])
            return pilot, tx_pilot, lts_corr, pilot_thresh

        # Get pilot
        lts_start = best_pk - lts_syms_len + 1  # where LTS-CP start
        pilot = samples[lts_start:best_pk+1]

    else:
        raise Exception("Only LTS Pilots supported at the moment")

    return pilot, tx_pilot, lts_corr, pilot_thresh


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
        rx_data_all   - TX Data. Dims: rx_data_all[num clients, num data syms]
        rxSymbols_all - Demodulated data symbols. Dims: rx_data_all[num clients, num data syms]
        symbol_err    - Data symbol error. Nneeds TX data to determine this. Dims:symbol_err[num clients, num data syms]
    """

    fft_size = int(metadata['FFT_SIZE'])
    data_cp_len = int(metadata['CP_LEN'])
    num_samps = int(metadata['SYM_LEN_NO_PAD'])
    num_sc = int(metadata['FFT_SIZE'])
    mod_order_str = metadata['CL_MODULATION']
    ofdm_size = fft_size + data_cp_len
    n_ofdm_syms = num_samps//ofdm_size

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
    data_sc = list(range(1, 7)) + list(range(8, 21)) + list(range(22, 27)) + \
                       list(range(38, 43)) + list(range(44, 57)) + list(range(58, 64))
    pilot_sc = [7, 21, 43, 57]
    pilots = np.array([1, 1, -1, 1]).reshape(4, 1, order="F")
    pilots_matrix = np.matlib.repmat(pilots, 1, n_ofdm_syms)
    # REMOVE ME END !!!!! WE NEED TO OBTAIN THIS FROM HDF5 FILE INSTEAD

    n_data_syms = n_ofdm_syms * len(data_sc)

    # Correction Flags
    apply_sfo_corr = user_params[3]
    apply_phase_corr = user_params[4]

    rx_data_all = np.empty((streams.shape[0], n_data_syms), dtype=int)
    rxSymbols_all = np.empty((streams.shape[0], n_data_syms), dtype=complex)
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

        rxSymbols_all[clIdx, :] = rxSymbols_vec
        rx_data_all[clIdx, :] = rx_data
        # print(" === STATS ===")
        symbol_err = []  # np.sum(tx_data != rx_data)
        # print("Frame#: {} --- Symbol Errors: {} out of {} total symbols".format(pkt_count, symbol_err, n_data_syms))

    return rx_data_all, rxSymbols_all, symbol_err, rxSymbols_mat, pilot_sc, data_sc


def compute_correlation(chan_est):
    """
    Debug plot that is useful for checking sync.

    Input:
        chan_est - Channel estimates. Dims: chan_est[num_cells, num clients, num BS ant, num frames, num subcarriers]

    Output:
        corr_total - Correlation. Dims: [num frames, num clients]
    """
    """Input samps dims: Frame, Cell, Antenna, User, Sample"""
    """Returns iq with Frame, Cell, User, Pilot Rep, Antenna, Sample"""
    """Returns csi with Frame, Cell, User, Pilot Rep, Antenna, Subcarrier"""

    this_cell = 0
    ref_frame = 0
    chan_est_ref = np.squeeze(chan_est[this_cell, :, :, ref_frame, :])
    corr_vec = np.transpose(np.conj(chan_est_ref), (1, 0, 2))   # Convert to [#bs ant, #clients, #subcarriers]

    userCSI = np.transpose(np.squeeze(chan_est[this_cell, :, :, :, :]), (2, 0, 1, 3))  # from [#clients, #ant, #frames, #sc]
    sig_intf = np.empty((userCSI.shape[0], userCSI.shape[1], userCSI.shape[1], userCSI.shape[3]), dtype='float32')
    for sc in range(userCSI.shape[3]):
        sig_intf[:, :, :, sc] = np.abs(np.dot(userCSI[:, :, :, sc], corr_vec[:, :, sc])) / np.dot(
            np.abs(userCSI[:, :, :, sc]), np.abs(corr_vec[:, :, sc]))

    # gets correlation of subcarriers for each user across bs antennas
    sig_sc = np.diagonal(sig_intf, axis1=1, axis2=2)
    sig_sc = np.swapaxes(sig_sc, 1, 2)
    corr_total = np.mean(sig_sc, axis=2)  # averaging corr across users
    return corr_total


def rx_app(filename, user_params, this_plotter):
    """
    Main function

    Input:
        filename    - HDF5 file to read from
        user_params - set of parameters defined by user. See main function
        plot_vec    - vector of flags to determine what will be plotted

    Output:
        None
    """
    global running
    rx_mode = user_params[0]

    # Read Received Samples
    metadata, samples = read_rx_samples(rx_mode, filename)

    # OFDM object
    ofdm_obj = ofdmTxRx()

    # Get attributes we care about
    prefix_len = metadata['PREFIX_LEN']
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

    while running:
        # Prepare samples to iterate over all received frames
        chan_est = np.empty([num_cells, num_cl, num_bs_ant, num_frames, fft_size], dtype=complex)
        lts_corr = np.empty([num_cl, num_bs_ant, sym_len+fft_size-1])
        IQ_pilots = np.empty([num_cells, num_cl, num_bs_ant, sym_len], dtype=complex)
        pilot_thresh = np.empty([num_cl, num_bs_ant])
        corr_total = np.empty([num_frames, num_cl])
        if rx_mode == "SIM":
            for frameIdx in range(num_frames):
                for clIdx in range(num_cl):
                    for antIdx in range(num_bs_ant):
                        # Put I/Q together
                        # Dims pilots: (frames, numCells, numClients, numAntennasAtBS, numSamplesPerSymbol*2)
                        I = pilot_samples[frameIdx, num_cells-1, clIdx, antIdx, 0:sym_len*2-1:2]/2**16
                        Q = pilot_samples[frameIdx, num_cells-1, clIdx, antIdx, 1:sym_len*2:2]/2**16
                        IQ = I + (Q * 1j)
                        IQ_pilots[num_cells-1, clIdx, antIdx, :] = IQ   # For 'plotter' use

                        # Find potential pilots. tx_pilot is a "struct" with dims:  [lts_time seq, lts_freq seq]
                        this_pilot, tx_pilot, lts_corr_tmp, pilot_thresh[clIdx, antIdx] = pilot_finder(IQ, pilot_type)
                        lts_corr[clIdx, antIdx, :] = lts_corr_tmp
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
                    rx_data, rxSymbols, symbol_err, rxSymbols_mat, pilot_sc, data_sc = demodulate_data(streams, ofdm_obj, user_params, metadata)

                    # Plotter
                    # Correlation across frames
                    sc_of_interest = np.sort(pilot_sc + data_sc)
                    corr_total[frameIdx, :] = compute_correlation(chan_est[:, :, :, sc_of_interest])

                    cl_plot = 0
                    ant_plot = 0
                    cell_plot = 0
                    # Grab RX frame at one antenna. Need to put together pilots from all users and data IQ
                    rx_data = []
                    for clIdx in range(num_cl):
                        rx_data.extend(IQ_pilots[cell_plot, clIdx, ant_plot, :])
                    rx_data.extend(IQ[ant_plot, :])

                    this_plotter.set_data(np.squeeze(tx_pilot[0]),                               # tx domain LTS
                                          np.squeeze(rx_data),                                   # [numBsAnt, symLen]
                                          chan_est[num_cells-1, cl_plot, ant_plot, frameIdx, :], # [numCells, numCl, numBsAnt, numFrame, numSC]
                                          lts_corr[cl_plot, ant_plot, :],                        # [numCl, numBsAnt, sym_len+fft_size-1]
                                          pilot_thresh[cl_plot, ant_plot],                       # [numCl, numBsAnt]
                                          #bf_weights[ant_plot, cl_plot, :],                     # [numBsAnt, numCl, numSC]
                                          #rx_data[cl_plot, :],                                  # [numCl, numDataSyms (numOfdmSyms*numDataSC)]
                                          rxSymbols_mat,
                                          np.squeeze(corr_total[:, cl_plot, :]),                 # [numDataPilotSC, numCl, numFrames]
                                          #rxSymbols[cl_plot, :],                                # [numCl, numDataSyms (numOfdmSyms*numDataSC)]
                                          #symbol_err,                                           # scalar
                                          user_params,
                                          metadata, pilot_sc, data_sc)                            # add pilot sc and data sc to metadata
    print("Exiting RX Thread")


def signal_handler(sig, frame):
    """
    SIGINT signal handler

    Input:
        None

    Output:
        None
    """
    print("SIG HANDLER!!!")
    global running
    print('Caught signal %d' % sig)
    # stop tx/rx threads
    running = False
    signal.pause()

#########################################
#                 Main                  #
#########################################
if __name__ == '__main__':

    # Start main program
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    print('To terminate, press Ctrl+C')

    parser = OptionParser()
    # Params
    parser.add_option("--file",       type="string",       dest="file",       default="./data_in/Argos-2019-2-20-14-55-22_1x8x2.hdf5", help="HDF5 filename to be read in SIM mode [default: %default]")
    parser.add_option("--mode",       type="string",       dest="mode",       default="SIM", help="Options: SIM/OTA [default: %default]")
    parser.add_option("--bfScheme",   type="string",       dest="bf_scheme",  default="ZF",  help="Beamforming Scheme. Options: ZF (for now) [default: %default]")
    parser.add_option("--cfoCorr",    action="store_true", dest="cfo_corr",   default=True,  help="Apply CFO correction [default: %default]")
    parser.add_option("--sfoCorr",    action="store_true", dest="sfo_corr",   default=True,  help="Apply SFO correction [default: %default]")
    parser.add_option("--phaseCorr",  action="store_true", dest="phase_corr", default=True,  help="Apply phase correction [default: %default]")
    parser.add_option("--fftOfset",   type="int",          dest="fft_offset", default=4,     help="FFT Offset:# CP samples for FFT [default: %default]")
    # Plotter
    parser.add_option("--pConst",     action="store_true", dest="plot_const", default=True,  help="Plot data symbol constellation [default: %default]")
    parser.add_option("--pTxIQ",      action="store_true", dest="plot_tx_iq", default=True,  help="Plot TX IQ samples [default: %default]")
    parser.add_option("--pRxIQ",      action="store_true", dest="plot_rx_iq", default=True,  help="Plot RX IQ samples [default: %default]")
    parser.add_option("--pCorr",      action="store_true", dest="plot_corr",  default=True,  help="Plot Sample Correlation [default: %default]")
    parser.add_option("--pCSI",       action="store_true", dest="plot_csi",   default=True,  help="Plot Channel Estimates (IQ and Magnitude) [default: %default]")
    parser.add_option("--pPhase",     action="store_true", dest="plot_phase", default=True,  help="Plot Phase Error Estimates [default: %default]")
    parser.add_option("--pPilotCorr", action="store_true", dest="plot_pilot", default=True,  help="Plot Pilot Correlation and Threshold  [default: %default]")
    (options, args) = parser.parse_args()

    # Params
    user_params = [options.mode,
                   options.bf_scheme,
                   options.cfo_corr,
                   options.sfo_corr,
                   options.phase_corr,
                   options.fft_offset
                   ]
    # Plotter
    plot_vec = []
    if options.plot_const:
        plot_vec.append("CONSTELLATION")
    if options.plot_tx_iq:
        plot_vec.append("TX_IQ")
    if options.plot_rx_iq:
        plot_vec.append("RX_IQ")
    if options.plot_corr:
        plot_vec.append("CORRELATION")
    if options.plot_csi:
        plot_vec.append("CSI")
    if options.plot_phase:
        plot_vec.append("PHASE")
    if options.plot_pilot:
        plot_vec.append("PILOT_CORR")

    # File
    filename = options.file

    # Rx Application. Matplotlib GUI needs to run on main thread.
    num_cl_plot = 2     # number of clients to plot
    this_plotter = Plotter(plot_vec, num_cl_plot)

    # RX app thread
    rxth = threading.Thread(target=rx_app, args=(filename, user_params, this_plotter))
    rxth.start()

    # Start animation
    this_plotter.animate()


