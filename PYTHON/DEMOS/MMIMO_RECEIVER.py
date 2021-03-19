#!/usr/bin/python3
"""
 MMIMO_RECEIVER.py

 Simple massive MIMO receiver. Tested only for two clients but code can be expanded to more clients.
 
 -Two modes: simulation and real-time
     a) Sim (AWGN):      Simulation mode/Debug mode. Take time domain TX samples from specified
                         HDF5 file and pass them through an AWGN channel. Tested for up to 2 clients and a
                         variable number of BS antennas.
     a) Replay (REPLAY): Read HDF5 file and run mMIMO receiver on the
                         collected data

 - Procedure:
     a) Read IQ
     b) Find Pilots
     c) Channel Estimator (Get CSI)
     d) ZF Weight Computation
     e) Separate Streams (Demultiplexing)
     f) Demodulate data
     g) Plotter

    Currently only supports one-cell system (one Base Station).
    *** NOTE ***
    We recommend to use this script with datasets generated from a joint BS/Client config file. This allows the sounder
    to record the original data transmitted by the client and its parameters. Otherwise, certain functionalities like
    EVM, and even the AWGN mode won't work.

    Usage example:
    1) Run sounder script ("./CC/Sounder/sounder ./CC/Sounder/files/<config>.json")
    This will run both the Base station and clients from same machine and will generate a log
    file inside "/CC/Sounder/logs".
    Use file as input to this script: "python3 MMIMO_RECEIVER.py --file <path/filename>"

    2) Run sounder script in two terminals, one for base station config only, and one for client config only. The base
    station will record a log file under "/CC/Sounder/logs".
    Use file as input to this script: "python3 MMIMO_RECEIVER.py --file <path/filename>"

    To terminate - CLOSE FIGURE
---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

#########################################
#                Include                #
#########################################
import sys
sys.path.append('../IrisUtils/')
sys.path.append('../IrisUtils/data_in/')

import numpy as np
import threading
import signal
from optparse import OptionParser
import matplotlib.pyplot as plt
from hdf5_lib import *
# from radio_lib import *
from find_lts import *
from generate_sequence import *
from ofdmtxrx import *
from ofdm_plotter import *
import matplotlib.cm as cm


#########################################
#              Global Vars              #
#########################################
global running
running = True


#########################################
#              Functions                #
#########################################
def read_rx_samples(rx_mode, filename):
    """
    Read IQ samples received at Base Station.

    Input:
        rx_mode - Three modes:
                    a) Sim (AWGN):      Read previously collected HDF5 file
                                        and run mMIMO receiver on the collected
                                        data
                    b) Replay (REPLAY): Read HDF5 file and run mMIMO receiver on the
                                        collected data
                    b) Real-Time (OTA): Call radio library to retrieve samples
                                        as they are received

        filename - Name of file to process

    Output:
        metadata - Attributes from hdf5 file
        samples  - Raw samples from pilots and data
    """

    if rx_mode == "AWGN" or rx_mode == "REPLAY":
        hdf5 = hdf5_lib(filename)
        hdf5.get_data()

        # Check which data we have available
        data_types_avail = []
        pilots_avail = bool(hdf5.data['Pilot_Samples'])
        if len(hdf5.data.keys()) > 1:
            if bool(hdf5.data['UplinkData']):
                ul_data_avail = bool(hdf5.data['UplinkData'])
        else:
            ul_data_avail = False

        samples = dict()
        if pilots_avail:
            data_types_avail.append("PILOTS")
            samples.update({"PILOT_SAMPS": hdf5.pilot_samples})
            print("PILOT Data Available")
        if ul_data_avail:
            data_types_avail.append("UL_DATA")
            samples.update({"UL_DATA": hdf5.uplink_samples})
            print("Uplink Data Available")
        else:
            print("No data found! exit now...")
            sys.exit(-1)

        # Empty structure
        if not data_types_avail:
            raise Exception(' **** No pilots or uplink data found **** ')

        # Retrieve attributes
        metadata = hdf5.metadata

    else:
        # If OTA
        # TODO - retrieve config data (metadata) and actual samples
        raise Exception("Realtime (OTA) not yet supported")
        radLib = radioLib()
        samples = radLib.data
        
    return metadata, samples


def pilot_finder(samples, pilot_type, flip=False, pilot_seq=[]):
    """
    Find pilots from clients to each of the base station antennas

    Input:
        samples    - Raw samples from pilots and data.
                     Dimensions: vector [1 x num samples]
        pilot_type - Type of TX pilot (e.g., 802.11 LTS)
        flip       - Needed for finding LTS function

    Output:
        pilot     - Received pilot (from multiple clients)
        tx_pilot  - Transmitted pilot (same pilot sent by all clients)
    """
    if pilot_type.find('lts') != -1:
        # LTS-based pilot
        lts_thresh = 0.8
        best_pk, lts_pks, lts_corr = find_lts(samples, thresh=lts_thresh, flip=flip, lts_seq=pilot_seq)

        # full lts contains 2.5 64-sample-LTS sequences, we need only one symbol
        lts, lts_f = generate_training_seq(preamble_type='lts', cp=32, upsample=1)

        if not (pilot_seq.size == 0):
            # pilot provided, overwrite the one returned above
            lts = pilot_seq

        lts_syms_len = len(lts)
        pilot_thresh = lts_thresh * np.max(lts_corr)
        # We'll need the transmitted version of the pilot (for channel estimation, for example)
        tx_pilot = [lts, lts_f]
        lts_start = 0

        # Check if LTS found
        if not best_pk:
            print("SISO_OFDM: No LTS Found! Continue...")
            pilot = np.array([])
            return pilot, tx_pilot, lts_corr, pilot_thresh, best_pk, lts_start
        # If beginning of frame was not captured in current buffer
        if (best_pk - (2 * lts_syms_len)) < 0:
            print("TOO EARLY. Continue... ")
            pilot = np.array([])
            return pilot, tx_pilot, lts_corr, pilot_thresh, best_pk, lts_start
        if (best_pk - lts_syms_len) > len(samples):
            print("TOO LATE. Continue... ")
            pilot = np.array([])
            return pilot, tx_pilot, lts_corr, pilot_thresh, best_pk, lts_start

        # Get pilot
        cp = 16
        # go back from second peak to start of sequence where pilot begins
        lts_start = best_pk - (2 * (lts_syms_len + cp)) + 1
        pilot = samples[lts_start: lts_start + 2 * (lts_syms_len + cp)]

    else:
        raise Exception("Only LTS Pilots supported at the moment")

    return pilot, tx_pilot, lts_corr, pilot_thresh, best_pk, lts_start


def estimate_channel(this_pilot, tx_pilot, ofdm_obj, user_params, metadata):
    """
    Estimate channel from received pilots

    Input:
        this_pilot  - received pilot (vector)
        tx_pilot    - time (tx_pilot[0]) and frequency (tx_pilot[1]) domain transmitted pilot sequences (vectors)
        ofdm_obj    - OFDM object
        user_params - set of parameters defined by user. See main function


    Output:
        chan_est - Vector containing channel estimates computed from this particular RX pilot (dim: fft_size x 1)
        cfo_est  - Coarse CFO estimate
        lts_evm  - Estimate of Error Vector Magnitude over LTS
    """

    fft_offset = user_params[5]
    apply_cfo_corr = user_params[2]

    # Retrieve sent pilot (freq domain)
    pilot_freq = tx_pilot[1]

    cp_len = int(metadata['CP_LEN'])
    pilot_f_len = len(pilot_freq)

    # Apply coarse CFO Correction
    lts_start = 0
    lts_syms_len = len(this_pilot)

    if apply_cfo_corr:
        try:
            coarse_cfo_est = ofdm_obj.cfo_correction(this_pilot, lts_start, lts_syms_len, fft_offset)
        except:
            chan_est = np.zeros(len(pilot_freq))
            cfo_est = 0
            lts_evm = 0
            return chan_est, cfo_est, lts_evm
    else:
        coarse_cfo_est = 0

    correction_vec = np.exp(-1j * 2 * np.pi * coarse_cfo_est * np.array(range(0, len(this_pilot))))
    pilot_cfo = this_pilot * correction_vec
    cfo_est = coarse_cfo_est

    # Channel estimation
    # Get LTS again (after CFO correction)
    if lts_syms_len == pilot_f_len:  # + cp_len:
        # 80-sample long
        # Half sequence (80-sample long LTS)
        lts = pilot_cfo[lts_start: lts_start + lts_syms_len]
        lts_1 = lts[-pilot_f_len + -fft_offset + np.array(range(lts_syms_len, (2*lts_syms_len)))]
        # lts_1 = lts[-64 + -fft_offset + np.array(range(80, 160-16))]

        # Average 2 LTS symbols to compute channel estimate
        chan_est = np.fft.ifftshift(pilot_freq) * np.fft.fft(lts_1)

        # Compute an estimate of EVM based on TX/RX LTS samples
        lts1_f = np.fft.fft(lts_1)
        lts_tx = np.fft.ifftshift(pilot_freq)
        evm_tmp1 = abs(lts1_f - lts_tx) ** 2
        lts_evm = np.mean(evm_tmp1)

    elif lts_syms_len == 160:
        lts = pilot_cfo[lts_start: lts_start + lts_syms_len]
        lts_1 = lts[-(64+cp_len) + -fft_offset + np.array(range(97, 161))]
        lts_2 = lts[-fft_offset + np.array(range(97, 161))]

        # Average 2 LTS symbols to compute channel estimate
        # chan_est = np.fft.ifftshift(pilot_freq) * np.fft.fft(lts_1)
        chan_est = np.fft.ifftshift(pilot_freq) * (np.fft.fft(lts_1) + np.fft.fft(lts_2)) / 2

        # Compute an estimate of EVM based on TX/RX LTS samples
        lts1_f = np.fft.fft(lts_1)
        lts_tx = np.fft.ifftshift(pilot_freq)
        evm_tmp1 = abs(lts1_f - lts_tx) ** 2
        lts_evm = np.mean(evm_tmp1)

    else:
        print("Invalid pilot sequence length")
        chan_est = np.zeros(len(pilot_freq))
        cfo_est = 0
        lts_evm = 0

    return chan_est, cfo_est, lts_evm


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
    # Collapse into dimensions [numBSant, numCl, fft_size]
    H_tmp = np.transpose(chan_est, (1, 0, 2))
    H_tmp_shape = H_tmp.shape
    num_sc = H_tmp_shape[2]
    num_ant = H_tmp_shape[0]
    num_cl = H_tmp_shape[1]

    bf_scheme = user_params[1]
    power_norm = 0

    WW = np.zeros((num_cl, num_ant, num_sc), dtype=complex)
    if bf_scheme == "ZF":
        # Zero Forcing
        # inv(H^H*H)*H^H*y
        for scIdx in range(num_sc):
            H = H_tmp[:, :, scIdx]
            HH = np.matrix.getH(H)
            W = np.matmul(HH, np.linalg.pinv(np.matmul(H, HH)))
            # W = (np.linalg.pinv(HH.dot(H))).dot(HH)

            if power_norm:
                # Normalize (equal power allocation across users)
                P = 1 * np.ones(num_cl)
                for k in range(num_cl):
                    W[:, k] = np.sqrt(P[k] / num_cl) * (W[:, k] / np.linalg.norm(W[:, k]))

            WW[:, :, scIdx] = W
            A = H.dot(W)

    elif bf_scheme == "MMSE":
        # MMSE
        # inv(H^H*H + sigma^2*I)*H^H*y
        for scIdx in range(num_sc):
            H = H_tmp[:, :, scIdx]
            HH = np.matrix.getH(H)
            sigma2I = 1.0 * np.eye(num_ant, dtype=complex)  # TODO: measure noise
            W = np.matmul(HH, np.linalg.pinv(np.matmul(H, HH) + sigma2I))

            if power_norm:
                # Normalize
                P = [0.3, 0.2]
                for k in range(num_cl):
                    W[:, k] = np.sqrt(P[k] / num_cl) * (W[:, k] / np.linalg.norm(W[:, k]))

            WW[:, :, scIdx] = W
            A = H.dot(W)

    else:
        raise Exception("Only Zero Forcing and MMSE currently implemented")

    return WW


def demultiplex(samples, bf_weights, user_params, metadata, chan_est, lts_start):
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
    num_cl = int(metadata['CL_NUM'])
    num_ant = metadata['BS_ANT_NUM_PER_CELL'].astype(int)[0]
    data_cp_len = int(metadata['CP_LEN'])
    fft_size = int(metadata['FFT_SIZE'])
    num_samps = int(metadata['SYMBOL_LEN_NO_PAD'])
    prefix_len = int(metadata['PREFIX_LEN'])
    ofdm_size = fft_size + data_cp_len
    n_ofdm_syms = num_samps//ofdm_size
    fft_offset = user_params[5]
    rx_mode = user_params[0]

    if rx_mode == "AWGN":
        samp_offset = np.ones((num_cl, num_ant)).astype(int) * prefix_len
    else:
        # Sample offset for data should be the same as for the pilots
        samp_offset = lts_start.astype(int)

    # Reshape into matrix. Dim: [num clients, num_sc+cp, num_ofdm]
    payload = samples
    payload_samples_mat_cp = np.zeros((num_ant, num_sc + data_cp_len, n_ofdm_syms)).astype(complex)

    for antIdx in range(num_ant):
        # Vector -> Matrix
        if ((n_ofdm_syms * (num_sc + data_cp_len)) + samp_offset[0, antIdx]) > len(payload[antIdx, :]):
            # Invalid offset, just use a dummy value
            this_offset = 60
        else:
            this_offset = samp_offset[0, antIdx]

        tmp_range = range(this_offset, n_ofdm_syms * (num_sc + data_cp_len) + this_offset)

        debug = 0
        if debug:
            print("RANGE: {} TO {}. Len(tmp_range): {}".format(tmp_range[0], tmp_range[-1], len(tmp_range)))
            plt.figure(100+antIdx)
            plt.plot(np.abs(payload[antIdx, :]))
            plt.show()

        payload_samples_mat_cp[antIdx, :, :] = np.reshape(payload[antIdx, tmp_range],
                                                          (num_sc + data_cp_len, n_ofdm_syms),
                                                           order="F")

    # Remove cyclic prefix
    payload_samples_mat = payload_samples_mat_cp[:, data_cp_len - fft_offset + 1 + np.array(range(0, num_sc)), :]       # TODO: That +1 seems to be a golden value, used to be +0... ??

    # FFT
    rxSig_freq = np.zeros((payload_samples_mat.shape[0], payload_samples_mat.shape[1], payload_samples_mat.shape[2]),
                          dtype=complex)

    for antIdx in range(num_ant):
        for symIdx in range(n_ofdm_syms):
            tmp = np.squeeze(payload_samples_mat[antIdx, :, symIdx])
            rxSig_freq[antIdx, :, symIdx] = np.fft.fft(tmp)

    # Demultiplexing (iterate over clients and ofdm symbols)
    x = np.zeros((num_cl, num_sc, n_ofdm_syms), dtype=complex)
    for symIdx in range(n_ofdm_syms):
        for scIdx in range(num_sc):
            this_w = np.squeeze(bf_weights[:, :, scIdx])
            y = np.squeeze(rxSig_freq[:, scIdx, symIdx])
            x[:, scIdx, symIdx] = np.dot(this_w, y)  # np.transpose(np.matmul(this_w, y))

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
    num_samps = int(metadata['SYMBOL_LEN_NO_PAD'])
    num_sc = int(metadata['FFT_SIZE'])
    mod_order_str = metadata['CL_MODULATION'].astype(str)
    data_sc = metadata['OFDM_DATA_SC']
    pilot_sc = metadata['OFDM_PILOT_SC']
    pilot_sc_vec = metadata['OFDM_PILOT_SC_VALS'].reshape(4, 1, order="F")
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
    else:
        sys.exit("Invalid Modulation")

    pilots_matrix = np.matlib.repmat(pilot_sc_vec, 1, n_ofdm_syms)
    n_data_syms = n_ofdm_syms * len(data_sc)

    # Correction Flags
    apply_sfo_corr = user_params[3]
    apply_phase_corr = user_params[4]

    rx_data_all = np.zeros((streams.shape[0], n_data_syms), dtype=int)
    rxSymbols_all = np.zeros((streams.shape[0], n_data_syms), dtype=complex)
    phase_error_all = np.zeros((streams.shape[0], n_ofdm_syms), dtype=float)
    rxSymbols_mat_allCl = []
    rxSymbols_mat_allCl_noCP = []
    for clIdx in range(streams.shape[0]):

        rxSig_freq_eq = streams[clIdx, :, :]
        # Apply SFO Correction
        if apply_sfo_corr:
            rxSig_freq_eq = ofdm_obj.sfo_correction(rxSig_freq_eq, pilot_sc, pilots_matrix, n_ofdm_syms)
        else:
            sfo_corr = np.ones((num_sc, n_ofdm_syms)) * -999

        # Apply phase correction
        if apply_phase_corr:
            phase_error = ofdm_obj.phase_correction(rxSig_freq_eq, pilot_sc, pilots_matrix)
        else:
            phase_error = np.zeros((1, n_ofdm_syms))

        phase_corr_tmp = np.matlib.repmat(phase_error, num_sc, 1)
        phase_corr = np.exp(-1j * phase_corr_tmp)
        rxSig_freq_eq_phase = rxSig_freq_eq * phase_corr
        rxSymbols_mat = rxSig_freq_eq_phase[data_sc, :]
        rxSymbols_mat_allCl.append(rxSymbols_mat)
        rxSymbols_mat_allCl_noCP.append(rxSig_freq_eq_phase)

        # Demodulation
        rxSymbols_vec = np.reshape(rxSymbols_mat, n_data_syms, order="F")       # Reshape into vector
        rx_data = ofdm_obj.demodulation(rxSymbols_vec, mod_order)

        rxSymbols_all[clIdx, :] = rxSymbols_vec
        rx_data_all[clIdx, :] = rx_data
        phase_error_all[clIdx, :] = phase_error

    return rx_data_all, rxSymbols_all, rxSymbols_mat_allCl, pilot_sc, data_sc, phase_error_all, rxSymbols_mat_allCl_noCP


def compute_correlation(chan_est, frameIdx, ref_frame):
    """
    Debug plot that is useful for checking sync.

    Input:
        chan_est - Channel estimates. Dims: chan_est[num_cells, num clients, num BS ant, num frames, num subcarriers]
        frameIdx - Index of frame being currently processed

    Output:
        corr_total - Correlation. Dims: [num frames, num clients]
    """
    """Input samps dims: Frame, Cell, Antenna, User, Sample"""
    """Returns iq with Frame, Cell, User, Pilot Rep, Antenna, Sample"""
    """Returns csi with Frame, Cell, User, Pilot Rep, Antenna, Subcarrier"""

    this_cell = 0
    chan_est_ref = chan_est[this_cell, :, :, ref_frame, :]  # [#clients, #BS ant, #frames, #subcarriers]
    corr_vec = np.transpose(np.conj(chan_est_ref), (1, 0, 2))           # Convert to [#bs ant, #clients, #subcarriers]

    userCSI = chan_est[this_cell, :, :, :, :]                           # [#clients, #BS ant, #frames, #subcarriers]
    userCSI = np.transpose(userCSI, (2, 0, 1, 3))                       # to [#frames, #clients, #ant, #sc]
    userCSI = userCSI[frameIdx, :, :, :]                                # [#clients, #ant, #sc]

    sig_intf = np.empty((userCSI.shape[0], userCSI.shape[0], userCSI.shape[2]), dtype='float32')
    for sc in range(userCSI.shape[2]):
        num = np.abs(np.dot(userCSI[:, :, sc], corr_vec[:, :, sc]))
        den = np.dot(np.abs(userCSI[:, :, sc]), np.abs(corr_vec[:, :, sc]))
        sig_intf[:, :, sc] = num / den

    # gets correlation of subcarriers for each user across bs antennas
    sig_sc = np.diagonal(sig_intf, axis1=0, axis2=1)
    sig_sc = np.swapaxes(sig_sc, 0, 1)
    corr_total = np.mean(sig_sc, axis=1)  # averaging corr across users?/subcarriers?
    return corr_total


def rx_stats(tx_syms, rx_data, cfo_est, lts_evm, metadata, n_ofdm_syms, ofdm_obj, phase_error):
    """
        Print stats
    """

    # Symbol error
    ofdm_data_sc = metadata['OFDM_DATA_SC']
    num_cl = int(metadata['CL_NUM'])
    num_sc = int(metadata['FFT_SIZE'])
    mod_order_str = metadata['CL_MODULATION']
    rate = int(metadata['RATE'])
    num_bs_ant = metadata['BS_ANT_NUM_PER_CELL'].astype(int)[0]

    if mod_order_str == "BPSK":
        mod_order = 2
    elif mod_order_str == "QPSK":
        mod_order = 4
    elif mod_order_str == "16QAM":
        mod_order = 16
    elif mod_order_str == "64QAM":
        mod_order = 64

    # Get tx data symbols and reshape
    tx_syms_data = np.zeros((num_cl, len(ofdm_data_sc) * n_ofdm_syms)).astype(complex)
    sym_err_rate = np.zeros(num_cl)
    cfo = np.zeros((num_cl, num_bs_ant))
    for idxCl in range(num_cl):
        tmp = np.reshape(tx_syms[idxCl], (num_sc, n_ofdm_syms), order="F")
        tx_syms_data[idxCl, :] = np.reshape(tmp[ofdm_data_sc, :], (len(ofdm_data_sc) * n_ofdm_syms), order="F")
        tx_data = ofdm_obj.demodulation(tx_syms_data[idxCl, :], mod_order)
        sym_error = (tx_data != rx_data[idxCl]).astype(int)
        sym_err_rate[idxCl] = 100 * sum(sym_error)/len(sym_error)
        cfo[idxCl, :] = cfo_est[idxCl, :] * np.squeeze(rate)

    print("======= STATS ========")
    print("Error Rate: {}".format(sym_err_rate))


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
    pilot_scaling = user_params[7]

    ###########################
    #  Read Received Samples  #
    ###########################
    metadata, samples = read_rx_samples(rx_mode, filename)

    ###########################
    #        OFDM object      #
    ###########################
    ofdm_obj = ofdmTxRx()

    ###########################
    #        Attributes       #
    ###########################
    if "CL_SDR_ID" in metadata.keys():
        cl_present = True
    else:
        cl_present = False

    prefix_len = int(metadata['PREFIX_LEN'])
    postfix_len = int(metadata['POSTFIX_LEN'])
    pilot_type = metadata['PILOT_SEQ_TYPE'].astype(str)[0]
    num_bs_ant = metadata['BS_ANT_NUM_PER_CELL'].astype(int)[0]
    pilot_samples = samples['PILOT_SAMPS']
    data_samples = samples['UL_DATA']
    num_cells = int(metadata['BS_NUM_CELLS'])
    num_cl = int(metadata['CL_NUM'])
    sym_len = int(metadata['SYMBOL_LEN'])
    sym_len_no_pad = int(metadata['SYMBOL_LEN_NO_PAD'])
    fft_size = int(metadata['FFT_SIZE'])
    cp_len = int(metadata['CP_LEN'])
    ofdm_data_sc = metadata['OFDM_DATA_SC']
    ofdm_pilot_sc = metadata['OFDM_PILOT_SC']
    ofdm_pilot = np.array(metadata['OFDM_PILOT'])
    ofdm_pilot_cp = np.concatenate((ofdm_pilot[-16:], ofdm_pilot))

    if not cl_present:
        cl_frame_sched = metadata['BS_FRAME_SCHED']
        # print('ERROR: Script needs client metadata. Sounder must be run in joint mode (BS and client together)')
        print('WARNING: Client(s) metadata is not available. Demodulation will not be available.')
        # sys.exit()
    else:
        ofdm_data = []
        ofdm_data_time = []
        cl_frame_sched = metadata['CL_FRAME_SCHED']

        for idx in range(num_cl):
            # Freq domain TX data (Does not contain cyclic prefix or prefix/postfix)
            ofdm_data.append(metadata['OFDM_DATA_CL' + str(idx)][idx])
            ofdm_data_time.append(metadata['OFDM_DATA_TIME_CL' + str(idx)][idx])

    pilot_dim = pilot_samples.shape
    num_frames = pilot_dim[0]

    # Frames to analyze
    tmp = user_params[6]
    if tmp.find(':') >= 0:
        frame0 = int(tmp.split(':')[0])
        frameEnd = tmp.split(':')[1]
        if frameEnd.find('end') >= 0 or int(frameEnd) > num_frames:
            iframes = range(frame0, num_frames)
        else:
            iframes = range(frame0, int(frameEnd))
    else:
        frame0 = 0
        iframes = range(int(tmp), int(tmp)+1)
        print("Using frame zero as reference!")

    # Verify dimensions
    assert pilot_dim[1] == num_cells
    assert pilot_dim[2] == num_cl
    assert pilot_dim[3] == num_bs_ant
    assert pilot_dim[4] == 2 * sym_len  # No complex values in HDF5, x2 to account for IQ

    ###########################
    #     Build TX signals    #
    ###########################
    # Process TX freq domain samples (from HDF5). These are the samples generated for transmission and stored in file,
    # not what has been received
    num_samps_freq_dom = (fft_size+cp_len)*(sym_len_no_pad//(fft_size+cp_len))
    n_ofdm_syms = num_samps_freq_dom//(fft_size+cp_len)

    # Pilots
    rep = sym_len_no_pad//len(ofdm_pilot_cp)
    frac = sym_len_no_pad % len(ofdm_pilot_cp)

    full_pilot = np.concatenate((np.zeros(prefix_len),
                                 np.squeeze(np.matlib.repmat(ofdm_pilot_cp, 1, rep)),
                                 ofdm_pilot_cp[0:frac],
                                 np.zeros(postfix_len)))

    # Note:
    # One pilot per client + overlapping data including prefix etc (for showing purposes)
    tx_sig = np.zeros((num_cl, (num_cl*sym_len + len(ofdm_data_time[0]))), dtype=complex)

    if cl_present:
        ofdm_data_mat = []
        for clIdx in range(num_cl):
            data_freq = ofdm_data[clIdx]
            ofdm_data_mat.append(np.reshape(np.squeeze(data_freq), (fft_size, n_ofdm_syms), order='F'))
            # ofdm_data_mat_time = np.fft.ifft(ofdm_data_mat[clIdx], axis=0)
            # ofdm_data_vec_time = np.reshape(ofdm_data_mat_time,
            # (1, ofdm_data_mat_time.shape[0]*ofdm_data_mat_time.shape[1]), order='F')
            tx_sig[clIdx, (clIdx*len(full_pilot)):(clIdx+1)*len(full_pilot)] = full_pilot * pilot_scaling
            tx_sig[clIdx, num_cl*len(full_pilot)::] = ofdm_data_time[clIdx] / np.max(ofdm_data_time[clIdx])
            # np.concatenate((np.zeros(prefix_len), np.squeeze(ofdm_data_vec_time)))

    # Remove pilots
    ofdm_tx_syms = np.empty((num_cl, len(ofdm_data_sc)*n_ofdm_syms)).astype(complex)
    if cl_present:
        for clIdx in range(num_cl):
            tmp = np.reshape(ofdm_data[clIdx], (fft_size, n_ofdm_syms), order='F')
            tmp = tmp[ofdm_data_sc, :]
            ofdm_tx_syms[clIdx, :] = np.reshape(tmp, (1, len(ofdm_data_sc)*n_ofdm_syms), order='F')

    # Number of uplink data symbols. Assume all clients are transmitting the same number of data symbols
    if type(cl_frame_sched) == list or type(cl_frame_sched) == numpy.ndarray:
        this_cl_sched = str(cl_frame_sched[0])  # Client index 0
    else:
        this_cl_sched = str(cl_frame_sched)
    num_ul_syms = this_cl_sched.count('U')


    ###########################
    #       Update Plots      #
    ###########################
    fig_len = 3 * (prefix_len + sym_len_no_pad + postfix_len)
    pilot_len = sym_len_no_pad
    this_plotter.update_tx_signal_fig(fig_len)
    this_plotter.update_rx_signal_fig(fig_len)
    this_plotter.update_corr_peaks(pilot_len)
    this_plotter.update_frame_corr(fig_len)
    this_plotter.update_phaser_err(n_ofdm_syms)

    ###########################
    #    Process RX Signals   #
    ###########################
    # Running flag. For demo purposes
    while running:
        # Prepare samples to iterate over all received frames
        chan_est = np.zeros([num_cells, num_cl, num_bs_ant, num_frames, fft_size], dtype=complex)
        cfo_est = np.zeros([num_cells, num_cl, num_bs_ant, num_frames])
        lts_evm = np.zeros([num_cells, num_cl, num_bs_ant, num_frames])
        lts_corr = np.zeros([num_cl, num_bs_ant, sym_len+fft_size-1])
        peak_index = np.zeros([num_cl, num_bs_ant, num_frames])
        IQ_pilots = np.zeros([num_cells, num_cl, num_bs_ant, sym_len], dtype=complex)
        pilot_thresh = np.zeros([num_cl, num_bs_ant])
        lts_start = np.zeros([num_cl, num_bs_ant])
        corr_total = np.zeros([num_frames, num_cl])
        corr_total[:] = np.nan
        lts_start_all = np.zeros([num_cells, num_cl, num_bs_ant, num_frames])

        if rx_mode == "AWGN":
            if not cl_present:
                print("AWGN not supported when client and base station are run separately (No client data available). "
                      "Please run on REPLAY mode.")
                sys.exit(-1)
            for frameIdx in range(num_frames):
                # PER FRAME
                # Code for debugging. Supports up to 2 clients and 8 BS ant. Uses TX symbols from HDF5 and passes them
                # through and AWGN channel
                chan_est_dbg = np.zeros([num_cl, num_bs_ant, fft_size], dtype=complex)
                # (1) Put pilot and data together
                tx_data_sim = np.zeros((num_bs_ant, num_cl*len(full_pilot)+len(ofdm_data_time[0]))).astype(complex)

                if num_cl == 1:
                    tx_data_sim_cl1 = np.concatenate([full_pilot, np.squeeze(ofdm_data_time[0]/np.max(ofdm_data_time[0]))])
                    tx_data_sim_cl2 = 0 * tx_data_sim_cl1
                elif num_cl == 2:
                    # tx_data_sim_cl1 = np.concatenate([full_pilot, np.zeros(len(full_pilot)), np.squeeze(ofdm_data_time[0])])
                    # tx_data_sim_cl2 = np.concatenate([np.zeros(len(full_pilot)), full_pilot, np.squeeze(ofdm_data_time[1])])
                    tx_data_sim_cl1 = tx_sig[0, :]/np.max(tx_sig[0, :])
                    tx_data_sim_cl2 = tx_sig[1, :]/np.max(tx_sig[1, :])

                # Merge signals (adding data of both)
                mult1 = 0.5
                mult2 = 0.8
                for andIdx in range(num_bs_ant):
                    # Alternate magnitude from one antenna to the next
                    tx_data_sim[andIdx, :] = (mult1 if andIdx % 2 == 0 else mult2) * tx_data_sim_cl1 + \
                                             (mult2 if andIdx % 2 == 0 else mult1) * tx_data_sim_cl2

                # (2) Pass it through AWGN Channel (each client and BS antenna path independently)
                num_samps_full_frame = len(tx_data_sim[0, :])
                ofdm_rx_syms_awgn = np.zeros((num_bs_ant, num_samps_full_frame)).astype(complex)
                for antIdx in range(num_bs_ant):
                    noise = 0.015 * (np.random.randn(num_samps_full_frame) + np.random.randn(num_samps_full_frame) * 1j)
                    ofdm_rx_syms_awgn[antIdx, :] = tx_data_sim[antIdx, :] + noise
                    # Remove DC
                    ofdm_rx_syms_awgn[antIdx, :] -= np.mean(ofdm_rx_syms_awgn[antIdx, :])

                    # (3) Find pilot
                    for clIdx in range(num_cl):
                        this_pilot = ofdm_rx_syms_awgn[antIdx, clIdx*len(full_pilot):(clIdx+1)*len(full_pilot)]
                        # Flip needed for AWGN data (due to the way we are writing the HDF5 files)
                        this_pilot, tx_pilot, lts_corr_tmp, pilot_thresh[clIdx, antIdx], best_pk, lts_start[clIdx, antIdx] = pilot_finder(this_pilot, pilot_type, flip=True, pilot_seq=ofdm_pilot)
                        lts_corr[clIdx, antIdx, :] = lts_corr_tmp
                        if this_pilot.size == 0:
                            continue

                        # (4) Channel estimation
                        chan_est_dbg[clIdx, antIdx, :], cfo_est_tmp, lts_evm_tmp = estimate_channel(this_pilot, tx_pilot, ofdm_obj, user_params, metadata)
                        chan_est[num_cells-1, clIdx, antIdx, frameIdx, :] = chan_est_dbg[clIdx, antIdx, :]
                        cfo_est[num_cells - 1, clIdx, antIdx, frameIdx] = cfo_est_tmp
                        lts_evm[num_cells - 1, clIdx, antIdx, frameIdx] = lts_evm_tmp

                # (5) Beamsteering weights
                bf_weights = beamforming_weights(chan_est[num_cells - 1, :, :, frameIdx, :], user_params)

                # (6) Re-assign
                rx_data = ofdm_rx_syms_awgn[:, num_cl*len(full_pilot)::]  # [pilot_cl1, pilot_cl2, data_combined]
                full_rx_frame = ofdm_rx_syms_awgn

                # (7) Demultiplex streams
                streams = demultiplex(rx_data, bf_weights, user_params, metadata, chan_est[num_cells - 1, :, :, frameIdx, :], lts_start)

                # (8) Demodulate streams
                rx_data_val, rxSymbols, rxSyms_mat, pilot_sc, data_sc, phase_error, rxSymsWithCP = demodulate_data(streams, ofdm_obj, user_params, metadata)

                # (9) Plotter
                rxSyms_vec = np.zeros((num_cl, len(data_sc) * n_ofdm_syms)).astype(complex)
                for idxCl in range(num_cl):
                    rxSyms_vec[idxCl, :] = np.reshape(rxSyms_mat[idxCl], (len(data_sc) * n_ofdm_syms), order="F")

                ant_plot = 0
                # Correlation across frames.
                sc_of_interest = np.sort(np.ndarray.tolist(pilot_sc) + np.ndarray.tolist(data_sc))
                H = chan_est_dbg[:, :, sc_of_interest]
                Htmp = chan_est[:, :, :, :, sc_of_interest]
                # corr_total: one column per client
                corr_total[frameIdx, :] = compute_correlation(Htmp, frameIdx, frame0)
                # Manipulation of channel estimates
                chan_est_vec = []
                rx_H_est_plot = []
                rx_H_est_plot_tmp = []
                evm_mat = []
                avg_evm = []
                snr_from_evm = []
                for clIdx in range(num_cl):
                    evm_mat.append(abs(rxSymsWithCP[clIdx] - ofdm_data_mat[clIdx]) ** 2)
                    avg_evm.append(np.mean(evm_mat[clIdx]))
                    snr_from_evm.append(10 * np.log10(1 / avg_evm[clIdx]))
                    # Dim: chan_est_dbg[numCl, numBsAnt, numSC]
                    chan_est_vec.append(chan_est_dbg[clIdx, ant_plot, :])
                    rx_H_est_plot.append(np.squeeze(np.matlib.repmat(complex('nan'), 1, len(chan_est_vec[clIdx]))))
                    rx_H_est_plot[clIdx][data_sc] = np.squeeze(chan_est_vec[clIdx][data_sc])
                    rx_H_est_plot[clIdx][pilot_sc] = np.squeeze(chan_est_vec[clIdx][pilot_sc])
                    rx_H_est_plot_tmp.append(rx_H_est_plot[clIdx])
                    rx_H_est_plot[clIdx] = np.fft.fftshift(abs(rx_H_est_plot[clIdx]))
                # Re-assign
                rx_data = full_rx_frame[ant_plot, :]

                # Calculate the Demmel condition number.
                if num_cl > 1:
                    demmelNumber = np.empty((1, chan_est.shape[4]), dtype='float32')
                    chan_est_tmp = np.squeeze(chan_est[num_cells - 1, :, :, frameIdx, :])
                    for sc in range(chan_est_tmp.shape[2]):
                        # covariance matrix
                        cov = np.matmul(chan_est_tmp[:, :, sc],
                                        np.transpose(np.squeeze(chan_est_tmp[:, :, sc]), [1, 0]).conj())
                        eigenvalues = np.abs(np.linalg.eigvals(cov))
                        demmelNumber[0, sc] = np.sum(eigenvalues) / np.min(eigenvalues)
                    tmp = demmelNumber[0, list(ofdm_pilot_sc) + list(ofdm_data_sc)]
                    demmelNumber_ave = np.average(tmp)
                else:
                    demmelNumber_ave = -999

                    # Update plotter data
                this_plotter.set_data(frameIdx,
                                      num_cl,  # of clients
                                      tx_sig,  # tx[num clients][num samples]
                                      rx_data,  # [numBsAnt, symLen]
                                      chan_est_vec,  # [numCl][fft size]
                                      rx_H_est_plot,  # rx_H_est_plot[numCl][fft_size]
                                      lts_corr[:, ant_plot, :],  # [numCl, numBsAnt, sym_len+fft_size-1]
                                      pilot_thresh[:, ant_plot],  # [numCl, numBsAnt]
                                      rxSyms_vec,  # [numCl, num data sc * num ofdm sym]
                                      corr_total,  # [num frames, numCl]
                                      ofdm_tx_syms,  # tx symbols [numClients, data length]
                                      user_params,
                                      metadata,
                                      phase_error,
                                      avg_evm,
                                      snr_from_evm,
                                      demmelNumber_ave)

        elif rx_mode == "REPLAY":
            for frameIdx in iframes:
                print("FRAME: {}".format(frameIdx))
                if not running:
                    sys.exit(0)
                for clIdx in range(num_cl):
                    for antIdx in range(num_bs_ant):
                        # print("FRAME: {} \t Client: {} \t Antenna: {}".format(frameIdx, clIdx, antIdx))
                        # Put I/Q together
                        # Dims pilots: (frames, numCells, numClients, numAntennasAtBS, numSamplesPerSymbol*2)
                        I = pilot_samples[frameIdx, num_cells - 1, clIdx, antIdx, 0:sym_len * 2:2] / 2 ** 15
                        Q = pilot_samples[frameIdx, num_cells - 1, clIdx, antIdx, 1:sym_len * 2:2] / 2 ** 15
                        IQ = I + (Q * 1j)

                        IQ_pilots[num_cells - 1, clIdx, antIdx, :] = IQ * pilot_scaling   # For 'plotter' use

                        # Find potential pilots. tx_pilot is a "struct" with dims:  [lts_time seq, lts_freq seq]
                        # No need to flip for OTA captures
                        this_pilot, tx_pilot, lts_corr_tmp, pilot_thresh[clIdx, antIdx], best_pk, lts_start[clIdx, antIdx] = pilot_finder(IQ, pilot_type, flip=True, pilot_seq=ofdm_pilot)
                        if this_pilot.size == 0:
                            continue

                        lts_corr[clIdx, antIdx, :] = lts_corr_tmp
                        peak_index[clIdx, antIdx, frameIdx] = best_pk

                        # Channel estimation from pilots
                        chan_est_tmp, cfo_est_tmp, lts_evm_tmp = estimate_channel(this_pilot, tx_pilot, ofdm_obj, user_params, metadata)
                        chan_est[num_cells - 1, clIdx, antIdx, frameIdx, :] = chan_est_tmp
                        cfo_est[num_cells - 1, clIdx, antIdx, frameIdx] = cfo_est_tmp
                        lts_evm[num_cells - 1, clIdx, antIdx, frameIdx] = lts_evm_tmp
                        lts_start_all[num_cells - 1, clIdx, antIdx, frameIdx] = lts_start[clIdx, antIdx]

                        # Measure noise at each BS antenna - for MMSE
                        # TODO

                # PER FRAME
                # Steering weight computation after collecting pilots at all antennas from all clients
                bf_weights = beamforming_weights(chan_est[num_cells-1, :, :, frameIdx, :], user_params)

                # Get data samples
                # Dims data: (frames, numCells, ulSymsPerFrame, numAntennasAtBS, numSamplesPerSymbol*2)
                for ulSymIdx in range(num_ul_syms):
                    Q = data_samples[frameIdx, num_cells-1, ulSymIdx, :, 0:sym_len*2:2] / 2 ** 15   # 32768
                    I = data_samples[frameIdx, num_cells-1, ulSymIdx, :, 1:sym_len*2:2] / 2 ** 15   # 32768
                    IQ = Q + (I * 1j)   # QI, not IQ

                    # Demultiplexing - Separate streams
                    this_chan_est = chan_est[num_cells - 1, :, :, frameIdx, :]

                    streams = demultiplex(IQ, bf_weights, user_params, metadata, this_chan_est, lts_start)

                    rx_data_val, rxSymbols, rxSyms_mat, pilot_sc, data_sc, phase_error, rxSymsWithCP = demodulate_data(streams, ofdm_obj, user_params, metadata)
                    rxSyms_vec = np.reshape(rxSyms_mat, (num_cl, len(data_sc) * n_ofdm_syms), order="F")

                    # Plotter
                    ant_plot = 0
                    cell_plot = 0
                    # Correlation across frames.
                    sc_of_interest = np.sort(np.ndarray.tolist(pilot_sc) + np.ndarray.tolist(data_sc))
                    H = chan_est[:, :, :, :, sc_of_interest]
                    # corr_total: one column per client
                    corr_total[frameIdx, :] = compute_correlation(H, frameIdx, frame0)

                    # Manipulation of channel estimates
                    chan_est_vec = []
                    rx_H_est_plot = []
                    rx_H_est_plot_tmp = []
                    rx_data = []
                    evm_mat = []
                    avg_evm = []
                    snr_from_evm = []
                    for clIdx in range(num_cl):
                        if cl_present:
                            evm_mat.append(abs(rxSymsWithCP[clIdx] - ofdm_data_mat[clIdx]) ** 2)
                            avg_evm.append(np.mean(evm_mat[clIdx]))
                            snr_from_evm.append(10 * np.log10(1 / avg_evm[clIdx]))
                        else:
                            avg_evm.append(-999)
                            snr_from_evm.append(-999)

                        # Dim: chan_est[numCells, numCl, numBsAnt, numFrame, numSC]
                        chan_est_vec.append(chan_est[num_cells - 1, clIdx, ant_plot, frameIdx, :])
                        rx_H_est_plot.append(np.squeeze(np.matlib.repmat(complex('nan'), 1, len(chan_est_vec[clIdx]))))
                        rx_H_est_plot[clIdx][data_sc] = np.squeeze(chan_est_vec[clIdx][data_sc])
                        rx_H_est_plot[clIdx][pilot_sc] = np.squeeze(chan_est_vec[clIdx][pilot_sc])
                        rx_H_est_plot_tmp.append(rx_H_est_plot[clIdx])
                        rx_H_est_plot[clIdx] = np.fft.fftshift(abs(rx_H_est_plot[clIdx]))

                        # Grab RX frame at one antenna. Need to put together pilots from all users and data IQ
                        rx_data.extend(IQ_pilots[cell_plot, clIdx, ant_plot, :])
                    rx_data.extend(IQ[ant_plot, :])

                    # Calculate the Demmel condition number.
                    if num_cl > 1:
                        demmelNumber = np.empty((1, chan_est.shape[4]), dtype='float32')
                        chan_est_tmp = np.squeeze(chan_est[num_cells - 1, :, :, frameIdx, :])
                        for sc in range(chan_est_tmp.shape[2]):
                            # covariance matrix
                            cov = np.matmul(chan_est_tmp[:, :, sc], np.transpose(np.squeeze(chan_est_tmp[:, :, sc]), [1, 0]).conj())
                            eigenvalues = np.abs(np.linalg.eigvals(cov))
                            demmelNumber[0, sc] = np.sum(eigenvalues) / np.min(eigenvalues)
                        tmp = demmelNumber[0, list(ofdm_pilot_sc) + list(ofdm_data_sc)]
                        demmelNumber_ave = np.average(tmp)
                    else:
                        demmelNumber_ave = -999

                    # TODO: Calculate Statistics
                    lts_evm_tmp = lts_evm[num_cells - 1, :, :, frameIdx]
                    # rx_stats(rxSymbols, ofdm_data, rx_data_val, lts_evm_tmp, metadata, n_ofdm_syms, ofdm_obj)

                    # Update plotter data
                    this_plotter.set_data(frameIdx,
                                          num_cl,  # of clients
                                          tx_sig,                                # tx[num clients][num samples]
                                          rx_data,                               # [numBsAnt, symLen]
                                          chan_est_vec,                          # [numCl][fft size]
                                          rx_H_est_plot,                         # rx_H_est_plot[numCl][fft_size]
                                          lts_corr[:, ant_plot, :],              # [numCl, numBsAnt, sym_len+fft_size-1]
                                          pilot_thresh[:, ant_plot],             # [numCl, numBsAnt]
                                          rxSyms_vec,                            # [numCl, num data sc * num ofdm sym]
                                          corr_total,                            # [num frames, numCl]
                                          ofdm_tx_syms,                          # tx symbols [numClients, data length]
                                          user_params,
                                          metadata,
                                          phase_error,
                                          avg_evm,
                                          snr_from_evm,
                                          demmelNumber_ave)

            # Analyze pilot offsets
            # lts_start_all[0, clIdx, antIdx, frameIdx]
            debug = 0
            if debug:
                y0 = [lts_start_all[0, 0, i, iframes] for i in range(1)]
                colors = cm.rainbow(np.linspace(0, 1, num_bs_ant))
                plt.figure(5000)
                for y, c in zip(y0, colors):
                    plt.plot(iframes, y, 'o', color=c)
                plt.show(block=False)

        else:
            # else if real-time (OTA)
            raise Exception("Realtime (OTA) not yet supported")

    print("Exiting RX Thread")


def signal_handler(sig):
    """
    SIGINT signal handler
    """
    print("SIG HANDLER!")
    global running
    print('Caught signal %d' % sig)
    # stop tx/rx threads
    running = False
    sys.exit(0)


#########################################
#                 Main                  #
#########################################
if __name__ == '__main__':

    # Start main program
    print('CLOSE FIGURE TO TERMINATE')

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    parser = OptionParser()
    # Params
    parser.add_option("--file",       type="string",       dest="file",       default="../IrisUtils/data_in/<filename>.hdf5", help="HDF5 filename to be read in AWGN or REPLAY mode [default: %default]")
    parser.add_option("--mode",       type="string",       dest="mode",       default="AWGN", help="Options: REPLAY/AWGN [default: %default]")
    parser.add_option("--bfScheme",   type="string",       dest="bf_scheme",  default="ZF",  help="Beamforming Scheme. Options: ZF (for now) [default: %default]")
    parser.add_option("--cfoCorr",    action="store_true", dest="cfo_corr",   default=False,  help="Apply CFO correction (not recommended) [default: %default]")
    parser.add_option("--sfoCorr",    action="store_true", dest="sfo_corr",   default=False,  help="Apply SFO correction [default: %default]")
    parser.add_option("--phaseCorr",  action="store_true", dest="phase_corr", default=True,  help="Apply phase correction [default: %default]")
    parser.add_option("--fftOfset",   type="int",          dest="fft_offset", default=1,     help="FFT Offset:# CP samples for FFT [default: %default]")
    parser.add_option("--numClPlot",  type="int",          dest="num_cl_plot",default=2,     help="Number of clients to plot. Max of 2. [default: %default]")
    parser.add_option("--pilotScale", type="int",          dest="pilot_scale",default=0.7,   help="Scaling factor for pilots. [default: %default]")
    parser.add_option("--frame",      type="string",       dest="frame",      default="100:end",  help="Range of frames to analyze (as string). Examples: --frame='5' to analyze a single frame, --frame='5:200' for range from frame 5 to 200, --frame='5:end' all frames starting at frame 5 [default: %default]")
    (options, args) = parser.parse_args()

    # Params
    user_params = [options.mode,
                   options.bf_scheme,
                   options.cfo_corr,
                   options.sfo_corr,
                   options.phase_corr,
                   options.fft_offset,
                   options.frame,
                   options.pilot_scale
                   ]

    # File
    filename = options.file
    threads_arr = []

    # Rx Application. Matplotlib GUI needs to run on main thread.
    num_cl_plot = options.num_cl_plot     # number of clients to plot
    this_plotter = OFDMplotter(num_cl_plot)

    # Enable if you need to plot anything else other than the main output figure
    debug = 0
    if debug:
        rx_app(filename, user_params, this_plotter)
    else:
        # RX app thread
        rxth = threading.Thread(target=rx_app, args=(filename, user_params, this_plotter))
        threads_arr.append(rxth)
        rxth.daemon = True
        rxth.start()

    # Start animation
    ret = this_plotter.animate()
    sys.exit(0)

