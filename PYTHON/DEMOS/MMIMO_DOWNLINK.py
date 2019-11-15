
import sys
sys.path.append('../IrisUtils/')

import numpy as np
from optparse import OptionParser
import SoapySDR
from SoapySDR import * #SOAPY_SDR_ constants
import time
import math
import datetime
import json
import signal
from scipy.linalg import hadamard
import scipy.io as sio 
from array import array
import matplotlib
matplotlib.rcParams.update({'font.size': 10})
import matplotlib.pyplot as plt
from matplotlib import animation
from find_lts import *
from type_conv import cfloat2uint32

plt.style.use('ggplot')  # customize your plots style

TDD_CONF_REG = 120
SCH_ADDR_REG = 136
SCH_MODE_REG = 140

RUNNING = True
TOT_FRAMES = 1000 
LTS_THRESH = 0.8

def signal_handler(signum, frame):
    global RUNNING
    RUNNING = False

def init(hub, bnodes, cnodes, ref_ant, ampl, rate, freq, txgain, rxgain, cp, plotter, numSamps, prefix_length, postfix_length, tx_advance, threshold, use_trig):
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
            sdr.setBandwidth(SOAPY_SDR_TX, ch, 2.5*rate)
            sdr.setBandwidth(SOAPY_SDR_RX, ch, 2.5*rate)
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
                sdr.setGain(SOAPY_SDR_TX, ch, 'PA2', 0)    # LO: [0|17], HI:[0|14]
            sdr.setGain(SOAPY_SDR_TX, ch, 'IAMP', 12)       # [-12,12]
            sdr.setGain(SOAPY_SDR_TX, ch, 'PAD', txgain)   # [0,52]

            if "CBRS" in info["frontend"]:
                sdr.setGain(SOAPY_SDR_RX, ch, 'ATTN', 0)   # {-18,-12,-6,0}
                sdr.setGain(SOAPY_SDR_RX, ch, 'LNA1', 30)  # [0,33]
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

        sdr.writeSetting("RESET_DATA_LOGIC", "")
        if info["serial"].find("RF3E") < 0:
            print("SPI TDD MODE")
            #sdr.writeSetting("SPI_TDD_MODE", "SISO")
        sdr.writeSetting(SOAPY_SDR_RX, 1, 'ENABLE_CHANNEL', 'false')
        sdr.writeSetting(SOAPY_SDR_TX, 1, 'ENABLE_CHANNEL', 'false')

    trig_dev.writeSetting("SYNC_DELAYS", "")

    sym_samps = numSamps + prefix_length + postfix_length
    print("numSamps = %d"%sym_samps)

    fft_size = 64
    cp_len = 32 if cp else 0
    ofdm_len = 2*fft_size + cp_len
    lts_rep = numSamps//(ofdm_len)
    zeros = np.array([0]*(numSamps-ofdm_len))
    lts_sym, lts_f = generate_training_seq(preamble_type='lts', cp=cp_len, upsample=1)
    pilot = np.concatenate((lts_sym, zeros))

    upsample = 1
    preambles_bs = generate_training_seq(preamble_type='gold_ifft', seq_length=128, cp=0, upsample=upsample)
    preambles = preambles_bs[:,::upsample] #the correlators can run at lower rates, so we only need the downsampled signal.
    beacon = preambles[0,:]
    coe = cfloat2uint32(np.conj(beacon), order='QI') # FPGA correlator takes coefficients in QI order
    pad1 = np.array([0]*(prefix_length), np.complex64) # to comprensate for front-end group delay
    pad2 = np.array([0]*(postfix_length), np.complex64) # to comprensate for rf path delay
    wbz = np.array([0]*(sym_samps), np.complex64)
    bcnz = np.array([0]*(sym_samps-prefix_length-len(beacon)), np.complex64)  
    beacon1 = np.concatenate([pad1,beacon*.5,bcnz])
    beacon2 = wbz  

    wb_pilot = ampl * pilot
    wbz = np.array([0]*(sym_samps), np.complex64)
    wb_pilot1 = np.concatenate([pad1, wb_pilot, pad2])

    L = fft_size
    M = len(bsdrs)
    K = len(csdrs)

    # configure tdd mode
    guardSize = (len(csdrs)) % 2 + 1
    frameLen = len(csdrs) + len(bsdrs)*2 + 4 + guardSize

    # BS frame config
    ref_sdr = bsdrs.pop(ref_ant)
    for i,sdr in enumerate(bsdrs):
        beacon_sch = "BG"
        ul_pilot_sch = ''.join("R"*(len(csdrs)))
        ref_pilot_sch = "GR"+''.join("G"*guardSize)
        dl_pilot_sch = ''.join("GG"*i)+"PG"+''.join("GG"*(len(bsdrs)-(i+1))) 
        frame_sch = beacon_sch + ul_pilot_sch + ref_pilot_sch + dl_pilot_sch
        print("BS node %d frame schedule  %s"%(i, frame_sch))
        bconf = {"tdd_enabled": True, 
                "frame_mode": "free_running", 
                "symbol_size" : sym_samps, 
                "frames": [frame_sch],
                "beacon_start" : prefix_length,
                "beacon_stop" : prefix_length+len(beacon),
                "max_frame" : 1}
        sdr.writeSetting("TDD_CONFIG", json.dumps(bconf))
        sdr.writeSetting("TDD_MODE", "true")

    # Client frame config
    for i, sdr in enumerate(csdrs):
        det_sch = "GG"
        ul_pilot_sch = ''.join("G"*i)+"P"+''.join("G"*(len(csdrs)-i-1))
        ref_pilot_sch = "GR"+''.join("G"*(guardSize))
        dl_pilot_sch = ''.join("RG"*len(bsdrs))
        frame_sch = det_sch + ul_pilot_sch + ref_pilot_sch + dl_pilot_sch
        print("Client %d frame schedule  %s"%(i, frame_sch))
        cconf = {"tdd_enabled": True,
                 "frame_mode": "triggered",
                 "symbol_size" : sym_samps,
                 "frames": [frame_sch],
                 "max_frame" : 0}
        sdr.writeSetting("TDD_CONFIG", json.dumps(cconf))
        sdr.writeSetting("TDD_MODE", "true")

    # REF frame config
    det_sch = "GG"
    ul_pilot_sch = ''.join("R"*(len(csdrs)))
    ref_pilot_sch = "GP"+''.join("G"*(guardSize))
    dl_pilot_sch = ''.join("RG"*len(bsdrs))
    frame_sch = det_sch + ul_pilot_sch + ref_pilot_sch + dl_pilot_sch
    print("Ref node frame schedule  %s"%frame_sch)
    rconf = {"tdd_enabled": True,
             "frame_mode": "free_running",
             "symbol_size" : sym_samps,
             "frames": [frame_sch],
             "max_frame" : 1}
    ref_sdr.writeSetting("TDD_CONFIG", json.dumps(rconf))
    ref_sdr.writeSetting("TDD_MODE", "true")

    for sdr in bsdrs+csdrs+[ref_sdr]:
        sdr.writeSetting("TX_SW_DELAY", str(30))

    if not use_trig:
        for sdr in csdrs:
            # enable the correlator, with zeros as inputs
            corr_conf = {"corr_enabled" : True,
                        "corr_threshold" : 1}
            sdr.writeSetting("CORR_CONFIG", json.dumps(corr_conf))
            sdr.writeRegisters("CORR_COE", 0, coe.tolist())

            # DEV: ueTrigTime = 153 (prefix_length=0),
            # CBRS: ueTrigTime = 235 (prefix_length=82), tx_advance=prefix_length,
            # corr delay is 17 cycles
            cl_trig_time = prefix_length + len(beacon) + postfix_length + 17 + postfix_length
            sf_start = cl_trig_time // sym_samps
            sp_start = cl_trig_time % sym_samps
            print("UE starting symbol and sample count (%d, %d)" % (sf_start, sp_start))
            # make sure to set this after TDD mode is enabled "writeSetting("TDD_CONFIG", ..."
            sdr.setHardwareTime(SoapySDR.ticksToTimeNs((sf_start << 16) | sp_start, rate), "TRIGGER")
    else:
        for sdr in csdrs:
            sdr.setHardwareTime(0, "TRIGGER")

    for i,sdr in enumerate(bsdrs):
        sdr.setHardwareTime(0, "TRIGGER")
    ref_sdr.setHardwareTime(0, "TRIGGER")

    possible_dim = []
    possible_dim.append(2**(np.ceil(np.log2(M))))
    h_dim = min(possible_dim)
    hadamard_matrix = hadamard(h_dim)
    beacon_weights = hadamard_matrix[0:M, 0:M]
    beacon_weights = beacon_weights.astype(np.uint32)

    replay_addr = 0
    pilot_uint = cfloat2uint32(wb_pilot1, order='QI').tolist()
    beacon_uint = cfloat2uint32(beacon1, order='QI').tolist()
    zero_uint = cfloat2uint32(wbz, order='QI').tolist() 
    for i, sdr in enumerate(bsdrs):
        sdr.writeRegisters("TX_RAM_A", 0, pilot_uint)
        sdr.writeRegisters("TX_RAM_B", 0, zero_uint)

        sdr.writeRegisters("BEACON_RAM", 0, beacon_uint)
        sdr.writeRegisters("BEACON_RAM_WGT_A", 0, beacon_weights[i].tolist())
        sdr.writeSetting("BEACON_START", str(M))

    for sdr in csdrs:
        sdr.writeRegisters("TX_RAM_A", replay_addr, pilot_uint)
        sdr.writeRegisters("TX_RAM_B", replay_addr, zero_uint)

    ref_sdr.writeRegisters("TX_RAM_A", replay_addr, pilot_uint)
    ref_sdr.writeRegisters("TX_RAM_B", replay_addr, zero_uint)

    # Create and activate streams
    bsdrs = bsdrs[:ref_ant]+[ref_sdr]+bsdrs[ref_ant:]
    rx_stream_ul = [sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0, 1]) for sdr in bsdrs]
    rx_stream_dl = [sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0, 1]) for sdr in csdrs]
    flags = 0
    for i, sdr in enumerate(bsdrs):
        sdr.activateStream(rx_stream_ul[i], flags, 0)
    for i, sdr in enumerate(csdrs):
        sdr.activateStream(rx_stream_dl[i], flags, 0)

    #############
    #initialize array and matrixes to hold rx and processed data
    #############
    calib_rx_dl = [np.array([0]*sym_samps).astype(np.complex64) for m in range(M)]
    calib_rx_ul = [np.array([0]*sym_samps).astype(np.complex64) for m in range(M)]
    pilot_rx_dl = [[np.array([0]*sym_samps).astype(np.complex64) for k in range(K)] for m in range(M)]
    pilot_rx_ul = [[np.array([0]*sym_samps).astype(np.complex64) for m in range(M)] for k in range(K)]
    #cl_rx_noise = [np.array([0]*sym_samps).astype(np.complex64) for k in range(K)]
    #bs_rx_noise = [np.array([0]*sym_samps).astype(np.complex64) for m in range(M)]
    dummy_rx = np.array([0]*sym_samps).astype(np.complex64)

    ul_cal_offset = np.array([0]*M, np.int32)
    dl_cal_offset = np.array([0]*M, np.int32)

    ul_offset = [np.array([0]*K, np.int32) for m in range(M)] 
    dl_offset = [np.array([0]*K, np.int32) for m in range(M)] 

    dl_csi_mat = np.empty([M, K, L], dtype=np.complex64)
    ul_csi_mat = np.empty([K, M, L], dtype=np.complex64)
    rx_f_cal_dl = np.empty([M, L], dtype=np.complex64)
    rx_f_cal_ul = np.empty([M, L], dtype=np.complex64)

    w_zf_dl = np.empty([M, K, L], dtype=np.complex64)
    w_zf_dl_cur = np.empty([M, K, L], dtype=np.complex64)
    w_zf_ul = np.empty([M, K, L], dtype=np.complex64)
    w_conj_dl = np.empty([M, K, L], dtype=np.complex64)
    w_conj_dl_cur = np.empty([M, K, L], dtype=np.complex64)
    w_conj_ul = np.empty([M, K, L], dtype=np.complex64)

    dl_csi_err = np.empty([M, K, L], dtype=np.complex64)
    dl_csi_err_abs = np.empty([M, K, L], dtype=np.float64)
    dl_csi_err_abs_mean = np.empty([K, L], dtype=np.complex64)

    dl_csi_err_cur = np.empty([M, K, L], dtype=np.complex64)
    dl_csi_err_cur_abs = np.empty([M, K, L], dtype=np.float64)
    dl_csi_err_cur_abs_mean = np.empty([K, L], dtype=np.complex64)
    #dl_csi_err_diff = np.empty([M, K, L], dtype=np.float64)
    #dl_csi_err_mean_diff = np.empty([K, L], dtype=np.complex64)

    calib = np.empty((M, L)).astype(np.complex64)
    calib_first = np.empty((M, L)).astype(np.complex64)
    calib_correction = np.empty((K, L)).astype(np.complex64)
    signal_power_cal = np.array([0]*2*M).astype(np.float64)
    noise_power_cal = np.array([0]*2*M).astype(np.float64)
    cal_snr = np.array([0]*2*M).astype(np.float64)
    cal_snr_min = np.array([0]*TOT_FRAMES).astype(np.float64)

    tx_data = np.matlib.repmat(lts_f, K, 1)
    tx_sig = np.empty([M, K, L], dtype=np.complex64)
    rx_sig = np.empty([K, K, L], dtype=np.complex64)
    rx_sig_power = np.empty([6, K, L], dtype=np.float64)
    rx_sig_intf = np.empty([6, K, L], dtype=np.float64)
    noise_power = np.empty([K, L], dtype=np.float64)
    sinr = np.zeros((6, TOT_FRAMES, K), dtype=np.float64)
    first = True
    cont_plotter = plotter

    if cont_plotter:
        fig1, axes1= plt.subplots(nrows=M+1, ncols=2, figsize=(9, 12))
        axes1[0, 0].set_title('Implicit vs Explicit Error (fresh cal)')
        axes1[0, 1].set_title('Implicit vs Explicit Error (first cal)')
        #axes1[0, 2].set_title('Difference of the two methods')
        for m in range(M+1):
            axes1[m, 0].set_xlim(0, L)
            axes1[m, 0].set_ylim(0, 2)
            if m == ref_ant:
                axes1[m, 0].set_ylabel('Ant %d (ref)'%m)
            else:
                axes1[m, 0].set_ylabel('Ant %d'%m)
            axes1[m, 0].legend(fontsize=10)
            axes1[m, 1].set_xlim(0, L)
            axes1[m, 1].set_ylim(0, 2)
            axes1[m, 1].legend(fontsize=10)
            #axes1[m, 2].set_xlim(0, L)
            #axes1[m, 2].set_ylim(0, 1)
            #axes1[m, 2].legend(fontsize=10)
        axes1[M, 0].set_ylabel('Mean')
        lines10 = [[axes1[m,0].plot(range(L), dl_csi_err_cur_abs[m,k,:])[0] for k in range(K)] for m in range(M)]
        lines11 = [[axes1[m,1].plot(range(L), dl_csi_err_abs[m,k,:])[0] for k in range(K)] for m in range(M)]
        #lines12 = [[axes1[m,2].plot(range(L), dl_csi_err_diff[m,k,:], label='Err Ant %d User %d'%(m,k))[0] for k in range(K)] for m in range(M)]
        linesMean10 = [axes1[M,0].plot(range(L), dl_csi_err_cur_abs_mean[k,:])[0] for k in range(K)]
        linesMean11 = [axes1[M,1].plot(range(L), dl_csi_err_abs_mean[k,:])[0] for k in range(K)]
        #linesMean12 = [axes1[M,2].plot(range(L), dl_csi_err_mean_diff[k,:], label='Mean Err User %d'%k)[0] for k in range(K)]
        fig1.show()
        fig2, axes2 = plt.subplots(nrows=M, ncols=2, figsize=(9, 12))
        axes2[0, 0].set_title('Calibration Downlink')
        axes2[0, 1].set_title('Calibration Uplink')
        for m in range(M):
            axes2[m, 0].set_xlim(0, sym_samps)
            axes2[m, 0].set_ylim(-1, 1)
            if m == ref_ant:
                axes2[m, 0].set_ylabel('Ant %d (ref)'%m)
            else:
                axes2[m, 0].set_ylabel('Ant %d'%m)
            axes2[m, 0].legend(fontsize=10)
            axes2[m, 1].set_xlim(0, sym_samps)
            axes2[m, 1].set_ylim(-1, 1)
            axes2[m, 1].legend(fontsize=10)
        lines20 = [axes2[m, 0].plot(range(sym_samps), calib_rx_dl[m][:sym_samps])[0] for m in range(M)]
        lines21 = [axes2[m, 1].plot(range(sym_samps), calib_rx_ul[m][:sym_samps])[0] for m in range(M)]
        lines24 = [axes2[m, 0].plot(range(sym_samps), calib_rx_dl[m][:sym_samps])[0] for m in range(M)]
        lines25 = [axes2[m, 1].plot(range(sym_samps), calib_rx_ul[m][:sym_samps])[0] for m in range(M)]
        fig2.show()

        fig3, axes3 = plt.subplots(nrows=M, ncols=2, figsize=(9, 12))
        axes3[0,0].set_title('Pilot Downlink')
        axes3[0,1].set_title('Pilot Uplink')
        for m in range(M):
            axes3[m, 0].set_xlim(0, sym_samps)
            axes3[m, 0].set_ylim(-1, 1)
            if m == ref_ant:
                axes3[m, 0].set_ylabel('Ant %d (ref)'%m)
            else:
                axes3[m, 0].set_ylabel('Ant %d'%m)
            axes3[m, 0].legend(fontsize=10)
            axes3[m, 1].set_xlim(0, sym_samps)
            axes3[m, 1].set_ylim(-1, 1)
            axes3[m, 1].legend(fontsize=10)
        lines30 = [[axes3[m, 0].plot(range(sym_samps), np.real(pilot_rx_dl[m][k]), label="User %d (real)"%k)[0] for k in range(K)] for m in range(M)]
        lines31 = [[axes3[m, 1].plot(range(sym_samps), np.real(pilot_rx_ul[k][m]), label="User %d (real)"%k)[0] for k in range(K)] for m in range(M)]
        #lines34 = [[axes3[m, 0].plot(range(sym_samps), calib_rx_dl[m][:sym_samps])[0] for k in range(K)] for m in range(M)]
        #lines35 = [[axes3[m, 1].plot(range(sym_samps), calib_rx_ul[m][:sym_samps])[0] for k in range(K)] for m in range(M)]
        fig3.show()

    cur_frame = 0
    signal.signal(signal.SIGINT, signal_handler)
    tstart = datetime.datetime.now()

    while(RUNNING):

        # disarm correlator in the clients
        if not use_trig:    
            for i, sdr in enumerate(csdrs):
                sdr.writeSetting("CORR_CONFIG", json.dumps({"corr_enabled": False}))

        bad_read = False
        bad_frame = False
        # arm correlator in the clients, inputs from adc
        if not use_trig:    
            for i, sdr in enumerate(csdrs):
                sdr.writeSetting("CORR_START", "A")

        trig_dev.writeSetting("TRIGGER_GEN", "")

        # collect uplink pilots
        for k in range(K):
            if bad_read: break
            for m in range(M):
                sr = bsdrs[m].readStream(rx_stream_ul[m], [pilot_rx_ul[k][m], dummy_rx], sym_samps)
                if sr.ret < sym_samps:
                    print("PilotUP: k: %d, m %d ret %d"%(k,m,sr.ret))
                    bad_read = True

        for m in range(M):
            if m == ref_ant: continue
            if bad_read: break
            sr = bsdrs[m].readStream(rx_stream_ul[m], [calib_rx_ul[m], dummy_rx], sym_samps)
            if sr.ret < sym_samps:
                print("PilotUP: k: %d, m %d ret %d"%(k,m,sr.ret))
                bad_read = True
        for k in range(K):
            if bad_read: break
            sr = csdrs[k].readStream(rx_stream_dl[k], [pilot_rx_dl[ref_ant][k], dummy_rx], sym_samps)
            if sr.ret < sym_samps:
                print("PilotDn: m: %d, k %d ret %d"%(ref_ant,k,sr.ret))
                bad_read = True
        #print("Done with reading uplink pilots")

        # collect downlink/reciprocity pilots from antenna m
        for m in range(M):
            if m == ref_ant: continue
            if bad_read: break
            for k in range(K):
                sr = csdrs[k].readStream(rx_stream_dl[k], [pilot_rx_dl[m][k], dummy_rx], sym_samps)
                if sr.ret < sym_samps:
                    print("PilotDn: m: %d, k %d ret %d"%(m,k,sr.ret))
                    bad_read = True
            sr = bsdrs[ref_ant].readStream(rx_stream_ul[ref_ant], [calib_rx_dl[m], dummy_rx], sym_samps)
            if sr.ret < sym_samps:
                bad_read = True

        ## collect noise 
        #for m in range(M):
        #    sr = bsdrs[m].readStream(rx_stream_ul[m], [bs_rx_noise[m], dummy_rx], sym_samps)
        #    if sr.ret < sym_samps:
        #        bad_read = True
        #for k in range(K):
        #    sr = csdrs[k].readStream(rx_stream_dl[k], [cl_rx_noise[k], dummy_rx], sym_samps)
        #    if sr.ret < sym_samps:
        #        bad_read = True
        ##print("Done with reading noise")

        if bad_read: 
            print("bad read")
            continue
        #print("Done with readings")

        # processing the received calibration samples
        print("frame %d"%(cur_frame))
        for m in range(M):
            if ref_ant == m: 
                calib[m,:] = np.array([1]*L, np.complex64) #continue
                rx_f_cal_dl[m,:] = np.array([0]*L, np.complex64) 
                rx_f_cal_ul[m,:] = np.array([0]*L, np.complex64) 
                continue 
            calib_rx_dl[m] -= np.mean(calib_rx_dl[m])
            calib_rx_ul[m] -= np.mean(calib_rx_ul[m])
             
            best_peak_dl, _, _ = find_lts(calib_rx_dl[m], thresh=LTS_THRESH)
            best_peak_ul, _, _ = find_lts(calib_rx_ul[m], thresh=LTS_THRESH)
            dl_cal_offset[m] = 0 if not best_peak_dl else best_peak_dl - len(lts_sym) + cp_len 
            ul_cal_offset[m] = 0 if not best_peak_ul else best_peak_ul - len(lts_sym) + cp_len 
            if (dl_cal_offset[m] < 150 or ul_cal_offset[m] < 150):
                bad_frame = True
            lts_dn_1 = calib_rx_dl[m][dl_cal_offset[m]:dl_cal_offset[m]+fft_size]
            lts_dn_2 = calib_rx_dl[m][dl_cal_offset[m]+fft_size:dl_cal_offset[m]+2*fft_size]
            lts_up_1 = calib_rx_ul[m][ul_cal_offset[m]:ul_cal_offset[m]+fft_size]
            lts_up_2 = calib_rx_ul[m][ul_cal_offset[m]+fft_size:ul_cal_offset[m]+2*fft_size]

            rx_f_cal_dl1 = np.fft.fftshift(np.fft.fft(lts_dn_1, fft_size, 0), 0)
            rx_f_cal_dl2 = np.fft.fftshift(np.fft.fft(lts_dn_2, fft_size, 0), 0)
            rx_f_cal_ul1 = np.fft.fftshift(np.fft.fft(lts_up_1, fft_size, 0), 0)
            rx_f_cal_ul2 = np.fft.fftshift(np.fft.fft(lts_up_2, fft_size, 0), 0)
            rx_f_cal_dl[m, :] = (rx_f_cal_dl1 + rx_f_cal_dl2)/2
            rx_f_cal_ul[m, :] = (rx_f_cal_ul1 + rx_f_cal_ul2)/2
 
            rx_noise_dl = calib_rx_dl[m][:fft_size] # take prefix part as noise
            rx_noise_ul = calib_rx_ul[m][:fft_size] # take prefix part as noise
            noise_f_dl = np.fft.fftshift(np.fft.fft(rx_noise_dl, fft_size, 0), 0)
            noise_f_ul = np.fft.fftshift(np.fft.fft(rx_noise_ul, fft_size, 0), 0)
            signal_power_cal[2 * m] = np.mean(np.power(np.abs(rx_f_cal_dl), 2))
            signal_power_cal[2*m+1] = np.mean(np.power(np.abs(rx_f_cal_ul), 2))
            noise_power_cal[2 * m] = np.mean(np.power(np.abs(noise_f_dl), 2))
            noise_power_cal[2*m+1] = np.mean(np.power(np.abs(noise_f_ul), 2))

        cal_snr = 10*np.log10(signal_power_cal/noise_power_cal)
        cal_snr[2*ref_ant] = 100
        cal_snr[2*ref_ant+1] = 100
        cal_snr_min[cur_frame] = np.min(cal_snr) #min snr among all calibration pairs

        # processing uplink pilot received samples
        for k in range(K):
            for m in range(M):
                pilot_rx_ul[k][m] -= np.mean(pilot_rx_ul[k][m]) 
                best_peak, _, _ = find_lts(pilot_rx_ul[k][m], thresh=LTS_THRESH)
                ul_offset[m][k] = 0 if not best_peak else (best_peak - len(lts_sym) + cp_len) 
                if ul_offset[m][k] < 150:
                    bad_frame = True 
                    #print("No LTS in Uplink Pilot from Ant %d to User %d"%(m,k)) 
                lts_ul_1 = pilot_rx_ul[k][m][ul_offset[m][k]:ul_offset[m][k]+fft_size]
                lts_ul_2 = pilot_rx_ul[k][m][ul_offset[m][k]+fft_size:ul_offset[m][k]+2*fft_size]
                rx_f_ul1 = np.fft.fftshift(np.fft.fft(lts_ul_1, fft_size, 0), 0)
                rx_f_ul2 = np.fft.fftshift(np.fft.fft(lts_ul_2, fft_size, 0), 0)
                ul_csi_mat[k,m,:] = (rx_f_cal_ul1 + rx_f_cal_ul2)*lts_f/2 

        # processing downlink pilot received samples
        for m in range(M):
            for k in range(K):
                pilot_rx_dl[m][k] -= np.mean(pilot_rx_dl[m][k]) 
                best_peak, _, _ = find_lts(pilot_rx_dl[m][k], thresh=LTS_THRESH)
                dl_offset[m][k] = 0 if not best_peak else (best_peak - len(lts_sym) + cp_len)
                if dl_offset[m][k] < 150:
                    bad_frame = True 
                    #print("No LTS in Downlink Pilot from Ant %d to User %d"%(m,k)) 
                lts_dl_1 = pilot_rx_dl[m][k][dl_offset[m][k]:dl_offset[m][k]+fft_size]
                lts_dl_2 = pilot_rx_dl[m][k][dl_offset[m][k]+fft_size:dl_offset[m][k]+2*fft_size]
                rx_f_dl1 = np.fft.fftshift(np.fft.fft(lts_dl_1, fft_size, 0), 0)
                rx_f_dl2 = np.fft.fftshift(np.fft.fft(lts_dl_2, fft_size, 0), 0)
                dl_csi_mat[m,k,:] = (rx_f_cal_dl1 + rx_f_cal_dl2)*lts_f/2

        #for k in range(K):
        #    cl_rx_noise[k] -= np.mean(cl_rx_noise[k])
        #    offset = sym_samps//2 - fft_size
        #    dn1 = cl_rx_noise[k][offset:offset+fft_size]
        #    dn2 = cl_rx_noise[k][offset+fft_size:offset+2*fft_size]
        #    H1 = np.fft.fftshift(np.fft.fft(dn1, fft_size, 0), 0)
        #    H2 = np.fft.fftshift(np.fft.fft(dn2, fft_size, 0), 0)
        #    noise_dn = np.delete((H1+H2)/2,[0,1,2,3,4,5,32,59,60,61,62,63],0)
        #    noise_power[k,:] = np.power(np.abs(noise_dn), 2)
 
        #for k in range(K):
        #    calib_correction[k,:] = np.divide(dl_csi_mat[ref_ant,k,:], ul_csi_mat[k,ref_ant,:])
        

        #if bad_frame: 
        #    print("bad frame")
        #    continue

        for m in range(M):
            if ref_ant == m: 
                calib[m,:] = np.array([1]*L, np.complex64) #continue
            else:
                calib[m,:] = np.divide(rx_f_cal_dl[m,:], rx_f_cal_ul[m,:])
            if first: calib_first[m,:] = calib[m,:]

        first = False

        print("Calib Dn/Up Per Antenna Amplitudes")
        print(['{:.2f}'.format(x) for x in np.mean(np.abs(rx_f_cal_dl), axis=1)])
        print(['{:.2f}'.format(x) for x in np.mean(np.abs(rx_f_cal_ul), axis=1)])
        #print("")

        print("User 0 Pilot Dn/Up Per Antenna Amplitudes")
        print(['{:.2f}'.format(x) for x in np.mean(np.abs(dl_csi_mat), axis=2)[:,0]])
        print(['{:.2f}'.format(x) for x in np.mean(np.abs(ul_csi_mat), axis=2)[0,:]])
        if K > 1:
            print("User 1 Pilot Dn/Up Per Antenna Amplitudes")
            print(['{:.2f}'.format(x) for x in np.mean(np.abs(dl_csi_mat), axis=2)[:,1]])
            print(['{:.2f}'.format(x) for x in np.mean(np.abs(ul_csi_mat), axis=2)[1,:]])
        print("")

        # processing beamforming SNRs 
        for l in range(L):
            dl_csi_imp = np.matmul(ul_csi_mat[:, :, l], np.diag(calib_first[:, l]))
            dl_csi_imp_cur = np.matmul(ul_csi_mat[:, :, l], np.diag(calib[:, l]))
            dl_csi_err[:, :, l] = dl_csi_mat[:, :, l] - np.transpose(dl_csi_imp)
            dl_csi_err_cur[:, :, l] = dl_csi_mat[:, :, l] - np.transpose(dl_csi_imp_cur)
            w_zf_dl[:, :, l]      = np.linalg.pinv(dl_csi_imp)
            w_zf_dl_cur[:, :, l] = np.linalg.pinv(dl_csi_imp_cur)
            w_zf_ul[:, :, l]     = np.linalg.pinv(ul_csi_mat[:, :, l])
            w_conj_dl[:, :, l]      = np.transpose(np.conj(dl_csi_imp))
            w_conj_dl_cur[:, :, l] = np.transpose(np.conj(dl_csi_imp_cur))
            w_conj_ul[:, :, l]     = np.transpose(np.conj(ul_csi_mat[:, :, l]))

            # zeroforcing downlink SNR calc. with first calibration vector
            tx_sig[:, :, l] = np.matmul(w_zf_dl[:, :, l], np.diag(tx_data[:, l]))
            rx_sig[:, :, l] = np.matmul(np.transpose(dl_csi_mat[:, :, l]), tx_sig[:, :, l]) 
            rx_sig_abs = np.power(np.abs(rx_sig[:, :, l]), 2)
            rx_sig_power[0, :, l] = np.diag(rx_sig_abs)
            rx_sig_intf[0, :, l] = np.sum(rx_sig_abs, axis=1) - rx_sig_power[0, :, l]

            # zeroforcing downlink SNR calc. with current calibration vector
            tx_sig[:, :, l] = np.matmul(w_zf_dl_cur[:,:,l], np.diag(tx_data[:,l]))
            rx_sig[:, :, l] = np.matmul(np.transpose(dl_csi_mat[:, :, l]), tx_sig[:, :, l]) 
            rx_sig_abs = np.power(np.abs(rx_sig[:, :, l]), 2)
            rx_sig_power[1, :, l] = np.diag(rx_sig_abs)
            rx_sig_intf[1, :, l] = np.sum(rx_sig_abs, axis=1) - rx_sig_power[1, :, l]

            # zeroforcing downlink SNR calc. with no calibration
            tx_sig[:, :, l] = np.matmul(w_zf_ul[:, :, l], np.diag(tx_data[:, l]))
            rx_sig[:, :, l] = np.matmul(np.transpose(dl_csi_mat[:, :, l]), tx_sig[:, :, l]) 
            rx_sig_abs = np.power(np.abs(rx_sig[:, :, l]), 2)
            rx_sig_power[2, :, l] = np.diag(rx_sig_abs)
            rx_sig_intf[2, :, l] = np.sum(rx_sig_abs, axis=1) - rx_sig_power[2, :, l]

            # conjugate downlink SNR calc. with first calibration vector
            tx_sig[:, :, l] = np.matmul(w_conj_dl[:,:,l], np.diag(tx_data[:, l]))
            rx_sig[:, :, l] = np.matmul(np.transpose(dl_csi_mat[:, :, l]), tx_sig[:, :, l]) 
            rx_sig_abs = np.power(np.abs(rx_sig[:, :, l]), 2)
            rx_sig_power[3, :, l] = np.diag(rx_sig_abs)
            rx_sig_intf[3, :, l] = np.sum(rx_sig_abs, axis=1) - rx_sig_power[3, :, l]

            # conjugate downlink SNR calc. with current calibration vector
            tx_sig[:, :, l] = np.matmul(w_conj_dl_cur[:, :, l], np.diag(tx_data[:, l]))
            rx_sig[:, :, l] = np.matmul(np.transpose(dl_csi_mat[:, :, l]), tx_sig[:, :, l]) 
            rx_sig_abs = np.power(np.abs(rx_sig[:, :, l]), 2)
            rx_sig_power[4, :, l] = np.diag(rx_sig_abs)
            rx_sig_intf[4, :, l] = np.sum(rx_sig_abs, axis=1) - rx_sig_power[4, :, l]

            # conjugate downlink SNR calc. with no calibration
            tx_sig[:, :, l] = np.matmul(w_conj_ul[:, :, l], np.diag(tx_data[:, l]))
            rx_sig[:, :, l] = np.matmul(np.transpose(dl_csi_mat[:, :, l]), tx_sig[:, :, l]) 
            rx_sig_abs = np.power(np.abs(rx_sig[:, :, l]), 2)
            rx_sig_power[5, :, l] = np.diag(rx_sig_abs)
            rx_sig_intf[5, :, l] = np.sum(rx_sig_abs, axis=1) - rx_sig_power[5, :, l]

        for i in range(6):
            #sinr[i,f,:] = 10*np.log10(np.mean(rx_sig_power[i,:,:]/(rx_sig_intf[i,:,:]+noise_power), axis=1))
            sinr[i,cur_frame,:] = 10*np.log10(np.mean(rx_sig_power[i, :, :]/(rx_sig_intf[i, :, :]), axis=1))

        dl_csi_err_abs = np.abs(dl_csi_err)
        dl_csi_err_abs_mean = np.mean(dl_csi_err_abs, 0)
        dl_csi_err_cur_abs = np.abs(dl_csi_err_cur)
        dl_csi_err_cur_abs_mean = np.mean(dl_csi_err_cur_abs, 0)
        #dl_csi_err_diff = np.abs(dl_csi_err_cur_abs - dl_csi_err_abs)
        #dl_csi_err_mean_diff = np.abs(dl_csi_err_cur_abs_mean - dl_csi_err_abs_mean)

        print("downlink signal power (zf) at user 0 %f"%np.mean(rx_sig_power[0, 0, :]))
        print("downlink interf power (zf) at user 0 %f"%np.mean(rx_sig_intf[0, 0, :]))
        print("downlink signal power (cj) at user 0 %f"%np.mean(rx_sig_power[3, 0, :]))
        print("downlink interf power (cj) at user 0 %f"%np.mean(rx_sig_intf[3, 0, :]))
        print("")
        print("")

        cur_frame += 1
        if cur_frame >= TOT_FRAMES: break
        if cont_plotter:
            for m in range(M):
                for k in range(K):
                    lines10[m][k].set_ydata(dl_csi_err_cur_abs[m, k, :])
                    lines11[m][k].set_ydata(dl_csi_err_abs[m, k, :])
                    #lines12[m][k].set_ydata(dl_csi_err_diff[m, k, :])
            for k in range(K):
                linesMean10[k].set_ydata(dl_csi_err_cur_abs_mean if K == 1 else dl_csi_err_cur_abs_mean[k, :])
                linesMean11[k].set_ydata(dl_csi_err_abs_mean if K == 1 else dl_csi_err_abs_mean[k, :])
                #linesMean12[k].set_ydata(dl_csi_err_mean_diff if K == 1 else dl_csi_err_mean_diff[k, :])

            for m in range(M):
                if m == ref_ant:
                    lines20[m].set_ydata(np.real(wb_pilot1))
                    lines21[m].set_ydata(np.real(wb_pilot1))
                    continue
                lines20[m].set_ydata(np.real(calib_rx_dl[m]))
                lines21[m].set_ydata(np.real(calib_rx_ul[m]))
                lines24[m].set_data(dl_cal_offset[m], np.linspace(-1.0, 1.0, num=100))
                lines25[m].set_data(ul_cal_offset[m], np.linspace(-1.0, 1.0, num=100))

            for m in range(M):
                for k in range(K):
                    lines30[m][k].set_ydata(np.real(pilot_rx_dl[m][k]))
                    lines31[m][k].set_ydata(np.real(pilot_rx_ul[k][m]))
                    #lines34[m][k].set_data(dl_offset[m][k], np.linspace(-1.0, 1.0, num=100))
                    #lines35[m][k].set_data(ul_offset[m][k], np.linspace(-1.0, 1.0, num=100))

            fig1.canvas.draw()
            fig1.show()
            fig2.canvas.draw()
            fig2.show()
            fig3.canvas.draw()
            fig3.show()

    plt.close()
    plt.close()
    plt.close()
    tend = datetime.datetime.now()
    c = tend - tstart
    print("Elapsed %d (secs)"%c.total_seconds())
    #print("sinr")
    #print(sinr[:f,:])
    #print("cal_snr_min")
    #print(cal_snr_min)
    sio.savemat("sinr"+str(M)+"x"+str(K)+".mat", mdict={'sinr':sinr})
    #if cur_frame > 5:
    #    fig4 = [plt.figure(figsize=(20, 8), dpi=100) for k in range(K)]
    #    for k in range(K): 
    #        ax1 = fig4[k].add_subplot(1, 1, 1)
    #        ax1.set_xlim(0, cur_frame)
    #        ax1.set_ylim(-40,40)
    #        ax1.set_ylabel('sinr')
    #        ax1.plot(range(cur_frame), sinr[0,:cur_frame,k], label='ZF, 1-time cal, User %d'%(k))
    #        ax1.plot(range(cur_frame), sinr[1,:cur_frame,k], label='ZF, conti. cal, User %d'%(k)) 
    #        ax1.plot(range(cur_frame), sinr[2,:cur_frame,k], label='ZF, no cal(UL), User %d'%(k)) 
    #        ax1.legend(fontsize=10)
    #    fig5 = [plt.figure(figsize=(20, 8), dpi=100) for k in range(K)]
    #    for k in range(K): 
    #        ax2 = fig5[k].add_subplot(1, 1, 1)
    #        ax2.set_xlim(0, cur_frame)
    #        ax2.set_ylim(-40,40)
    #        ax2.set_ylabel('sinr')
    #        ax2.plot(range(cur_frame), sinr[3,:cur_frame,k], label='Conj, 1-time cal, User %d'%(k))
    #        ax2.plot(range(cur_frame), sinr[4,:cur_frame,k], label='Conj, conti. cal, User %d'%(k))
    #        ax2.plot(range(cur_frame), sinr[5,:cur_frame,k], label='Conj, no cal(UL), User %d'%(k))
    #        ax2.legend(fontsize=10)
    #    plt.show()

    # clear up fpga states 
    tdd_conf = {"tdd_enabled" : False}
    corr_conf = {"corr_enabled" : False}
    for sdr in csdrs:
        if not use_trig:
            sdr.writeSetting("CORR_CONFIG", json.dumps(corr_conf))
    for sdr in bsdrs+csdrs:
        sdr.writeSetting("TDD_CONFIG", json.dumps(tdd_conf))
        sdr.writeSetting("TDD_MODE", "false")
        sdr.writeSetting("RESET_DATA_LOGIC", "")

    # close streams and exit
    for i, sdr in enumerate(bsdrs):
        sdr.closeStream(rx_stream_ul[i])
    for i, sdr in enumerate(csdrs):
        sdr.closeStream(rx_stream_dl[i])

def main():
    parser = OptionParser()
    parser.add_option("--args", type="string", dest="args", help="arguments", default="")
    parser.add_option("--bnodes", type="string", dest="bnodes", help="file name containing serials on the base station", default="bs_serials.txt")
    parser.add_option("--cnodes", type="string", dest="cnodes", help="file name containing serials to be used as clients", default="client_serials.txt")
    parser.add_option("--hub", type="string", dest="hub", help="Hub node", default="")
    parser.add_option("--ref-ant", type="int", dest="ref_ant", help="Calibration reference antenna", default=0)
    parser.add_option("--ampl", type="float", dest="ampl", help="Amplitude coefficient for downCal/upCal", default=0.5)
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--freq", type="float", dest="freq", help="Optional Tx freq (Hz)", default=3.6e9)
    parser.add_option("--txgain", type="float", dest="txgain", help="Optional Tx gain (dB)", default=40.0)
    parser.add_option("--rxgain", type="float", dest="rxgain", help="Optional Rx gain (dB)", default=20.0)
    parser.add_option("--bw", type="float", dest="bw", help="Optional Tx filter bw (Hz)", default=10e6)
    parser.add_option("--cp", action="store_true", dest="cp", help="adds cyclic prefix to tx symbols", default=True)
    parser.add_option("--plotter", action="store_true", dest="plotter", help="continuously plots all signals and stats",default=False)
    parser.add_option("--numSamps", type="int", dest="numSamps", help="Number of samples in Symbol", default=400)
    parser.add_option("--prefix-length", type="int", dest="prefix_length", help="prefix padding length for beacon and pilot", default=82)
    parser.add_option("--postfix-length", type="int", dest="postfix_length", help="postfix padding length for beacon and pilot", default=68)
    parser.add_option("--tx-advance", type="int", dest="tx_advance", help="symbol advance for tx", default=2)
    parser.add_option("--corr-threshold", type="int", dest="threshold", help="Correlator Threshold Value", default=1)
    parser.add_option("--use-trig", action="store_true", dest="use_trig", help="uses chain triggers for synchronization",default=False)
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

    init(hub=options.hub,
         bnodes=bserials,
         cnodes=cserials,
         ref_ant=options.ref_ant,
         ampl=options.ampl,
         rate=options.rate,
         freq=options.freq,
         txgain=options.txgain,
         rxgain=options.rxgain,
         cp=options.cp,
         plotter=options.plotter,
         numSamps=options.numSamps,
         prefix_length=options.prefix_length,
         postfix_length=options.postfix_length,
         tx_advance=options.tx_advance,
         threshold=options.threshold,
         use_trig=options.use_trig)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
