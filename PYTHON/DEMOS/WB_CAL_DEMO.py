
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
#import preambles as Preambles
import matplotlib
from scipy.linalg import hadamard
import scipy.io as sio 
from array import array
matplotlib.rcParams.update({'font.size': 10})
import matplotlib.pyplot as plt
from matplotlib import animation
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

def init(hub, bnodes, cnodes, ref_ant, ampl, rate, freq, txgain, rxgain, cp, plotter, numSamps, prefix_length, postfix_length, tx_advance, threshold, use_trig):
    if hub != "": hub_dev = SoapySDR.Device(dict(driver="remote", serial = hub)) # device that triggers bnodes and ref_node
    bsdrs = [SoapySDR.Device(dict(driver="iris", serial = serial)) for serial in bnodes] # base station sdrs
    #ref_sdr = SoapySDR.Device(dict(driver="iris", serial = ref_node))

    csdrs = [SoapySDR.Device(dict(driver="iris", serial = serial)) for serial in cnodes] # client sdrs
    # assume trig_sdr is part of the master nodes
    trig_dev = None
    if hub != "":
        trig_dev = hub_dev
    else:
        trig_dev = bsdrs[0]

    #set params on both channels
    #for sdr in bsdrs+[ref_sdr]+csdrs:
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

        #for ch in [0, 1]:
        #    if calibrate:
        #        sdr.writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", 'SKLK')
        #        sdr.writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", '')

        sdr.writeRegister("IRIS30", RF_RST_REG, (1 << 29) | 0x1)
        sdr.writeRegister("IRIS30", RF_RST_REG, (1 << 29))
        sdr.writeRegister("IRIS30", RF_RST_REG, 0)
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
    lts_rep = numSamps//(ofdm_len)
    zeros = np.array([0]*(numSamps-ofdm_len))
    ltsSym, lts_f = generate_training_seq(preamble_type='lts', cp=cp_len, upsample=1)
    pilot = np.concatenate((ltsSym, zeros))

    upsample = 1
    preambles_bs = generate_training_seq(preamble_type='gold_ifft', seq_length=128, cp=0, upsample=upsample)
    preambles = preambles_bs[:,::upsample] #the correlators can run at lower rates, so we only need the downsampled signal.
    beacon = preambles[0,:]
    coe = cfloat2uint32(np.conj(beacon), order='IQ') # FPGA correlator takes coefficients in QI order
    pad1 = np.array([0]*(prefix_length), np.complex64) # to comprensate for front-end group delay
    pad2 = np.array([0]*(postfix_length), np.complex64) # to comprensate for rf path delay
    wbz = np.array([0]*(symSamp), np.complex64)
    bcnz = np.array([0]*(symSamp-prefix_length-len(beacon)), np.complex64)  
    beacon1 = np.concatenate([pad1,beacon*.5,bcnz])
    beacon2 = wbz  

    wb_pilot = 0.25 * pilot
    wbz = np.array([0]*(symSamp), np.complex64)
    wb_pilot1 = np.concatenate([pad1, wb_pilot, pad2])
    wb_pilot2 = wbz 

    L = fft_size - 12

    M = len(bsdrs)
    R = M - 1
    K = len(csdrs)
    print("R %d, M %d, K %d"%(R,M,K))
    # configure tdd mode
    guardSize = (len(csdrs)) % 2 + 1
    frameLen = len(csdrs) + len(bsdrs)*2 + 4 + guardSize
    lts_thresh = 0.8

    # BS
    ref_sdr = bsdrs.pop(ref_ant)
    for i,sdr in enumerate(bsdrs):
        #sched_main = "PG"+''.join("R"*(len(csdrs)+1))+''.join("G"*guardSize)+''.join("RG"*i)+"PG"+''.join("RG"*(len(bsdrs)-(i+1)))
        sched_main = "PG"+''.join("R"*(len(csdrs)))+"GR"+''.join("G"*guardSize)+''.join("GG"*i)+"PG"+''.join("GG"*(len(bsdrs)-(i+1)))#+"RG"
        print("BS node %d"%i)
        print("sched_main   %s"%(sched_main))
        bconf = {"tdd_enabled": True, "frame_mode": "free_running", "symbol_size" : symSamp, "dual_pilot" : True, "frames": [sched_main], "max_frame" : 1}
        sdr.writeSetting("TDD_CONFIG", json.dumps(bconf))
        sdr.writeSetting("TDD_MODE", "true")

    # Client
    for i,sdr in enumerate(csdrs):
        #sched_main = "GG"+''.join("G"*i)+"P"+''.join("G"*(len(csdrs)-i-1))+"R"+''.join("G"*(guardSize))+''.join("RG"*len(bsdrs))+''.join("G"*(frameLen-4-len(bsdrs)*2-len(csdrs)-guardSize))
        sched_main = "GG"+''.join("G"*i)+"P"+''.join("G"*(len(csdrs)-i-1))+"GR"+''.join("G"*(guardSize))+''.join("RG"*len(bsdrs))#+"RG"
        print("Client node %d"%i)
        print("sched_main   %s"%(sched_main))
        cconf = {"tdd_enabled": True, "frame_mode": "triggered", "symbol_size" : symSamp, "frames": [sched_main], "max_frame" : 0}
        sdr.writeSetting("TDD_CONFIG", json.dumps(cconf))
        sdr.writeSetting("TDD_MODE", "true")

    # REF
    print("Reference node")
    #sched_middle = ''.join("G"*(frameLen))
    #print("sched_middle %s"%(sched_middle))
    sched_main = "GG"+''.join("R"*len(csdrs))+"GP"+''.join("G"*guardSize)+''.join("RG"*len(bsdrs))#+"RG"
    print("sched_main   %s"%(sched_main))
    rconf = {"tdd_enabled": True, "frame_mode": "free_running", "symbol_size" : symSamp, "frames": [sched_main], "max_frame" : 1}
    ref_sdr.writeSetting("TDD_CONFIG", json.dumps(rconf))
    ref_sdr.writeSetting("TDD_MODE", "true")

    for sdr in bsdrs+csdrs+[ref_sdr]:
    #for sdr in bsdrs+csdrs:
        sdr.writeSetting("TX_SW_DELAY", str(30))

    if not use_trig:
        for sdr in csdrs:
            sdr.writeRegister("IRIS30", CORR_CONF, int("00004001", 16))  # enable the correlator, with zeros as inputs
            for i in range(128):
                sdr.writeRegister("ARGCOE", i*4, 0)
            time.sleep(0.1)
            sdr.writeRegister("IRIS30", CORR_RST, 0x1)  # reset corr
            sdr.writeRegister("IRIS30", CORR_RST, 0x0)  # unrst corr
            sdr.writeRegister("IRIS30", CORR_THRESHOLD, threshold)
            for i in range(128):
                sdr.writeRegister("ARGCOE", i*4, int(coe[i]))

            # DEV: ueTrigTime = 153 (prefix_length=0), CBRS: ueTrigTime = 235 (prefix_length=82), tx_advance=prefix_length,
            # corr delay is 17 cycles
            ueTrigTime = prefix_length + len(beacon) + postfix_length + 17 + postfix_length 
            sf_start = ueTrigTime // symSamp
            sp_start = ueTrigTime % symSamp
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
    nRadios = len(bsdrs)
    possible_dim.append(2**(np.ceil(np.log2(nRadios))))
    h_dim = min(possible_dim)
    hadamard_matrix = hadamard(h_dim)       #hadamard matrix : http://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.linalg.hadamard.html
    beacon_weights = hadamard_matrix[0:nRadios, 0:nRadios]
    print(beacon_weights)
    beacon_weights = beacon_weights.astype(np.uint32)

    replay_addr = 0
    for i, sdr in enumerate(bsdrs):
        sdr.writeRegisters("TX_RAM_A", replay_addr, cfloat2uint32(beacon1, order='IQ').tolist())
        sdr.writeRegisters("TX_RAM_A", replay_addr+2048, cfloat2uint32(np.concatenate((wb_pilot1,np.array([0]*(2048-len(wb_pilot1))))), order='IQ').tolist())
        sdr.writeRegisters("TX_RAM_B", replay_addr, cfloat2uint32(beacon2, order='IQ').tolist())
        #sdr.writeRegisters("TX_RAM_WGT_A", replay_addr, beacon_weights[i].tolist())

        #sdr.writeRegister("RFCORE", 156, int(nRadios))
        #sdr.writeRegister("RFCORE", 160, 1) # enable beamsweeping

    for sdr in csdrs:
        sdr.writeRegisters("TX_RAM_A", replay_addr, cfloat2uint32(wb_pilot1, order='IQ').tolist())
        sdr.writeRegisters("TX_RAM_B", replay_addr, cfloat2uint32(wbz, order='IQ').tolist())

    ref_sdr.writeRegisters("TX_RAM_A", replay_addr, cfloat2uint32(wb_pilot1, order='IQ').tolist())
    ref_sdr.writeRegisters("TX_RAM_B", replay_addr, cfloat2uint32(wbz, order='IQ').tolist())
    # Create streams
    bsdrs = bsdrs[:ref_ant]+[ref_sdr]+bsdrs[ref_ant:]
    rxStream_ul = [sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0, 1]) for sdr in bsdrs]
    rxStream_dl = [sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0, 1]) for sdr in csdrs]
    flags = 0
    for i, sdr in enumerate(bsdrs):
        sdr.activateStream(rxStream_ul[i], flags, 0)
    for i, sdr in enumerate(csdrs):
        sdr.activateStream(rxStream_dl[i], flags, 0)

    #rxStream_ref = ref_sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0, 1])  
    #ref_sdr.activateStream(rxStream_ref, flags, 0)

    frame = 1000 

    #calibRxA = [[np.array([0]*symSamp).astype(np.complex64) for r in range(R)] for r in range(R)] 
    #calibRxB = [[np.array([0]*symSamp).astype(np.complex64) for r in range(R)] for r in range(R)] 
    calibRxDnA = [np.array([0]*symSamp).astype(np.complex64) for m in range(M)]
    calibRxDnB = [np.array([0]*symSamp).astype(np.complex64) for m in range(M)]
    calibRxUpA = [np.array([0]*symSamp).astype(np.complex64) for m in range(M)]
    calibRxUpB = [np.array([0]*symSamp).astype(np.complex64) for m in range(M)]
    pilotRxDnA = [[np.array([0]*symSamp).astype(np.complex64) for r in range(K)] for m in range(M)]
    pilotRxDnB = [[np.array([0]*symSamp).astype(np.complex64) for r in range(K)] for m in range(M)]
    pilotRxUpA = [[np.array([0]*symSamp).astype(np.complex64) for m in range(M)] for s in range(K)]
    pilotRxUpB = [[np.array([0]*symSamp).astype(np.complex64) for mm in range(M)] for s in range(K)]
    quietRxDnA = [np.array([0]*symSamp).astype(np.complex64) for r in range(K)]
    quietRxDnB = [np.array([0]*symSamp).astype(np.complex64) for r in range(K)]
    quietRxUpA = [np.array([0]*symSamp).astype(np.complex64) for m in range(M)]
    quietRxUpB = [np.array([0]*symSamp).astype(np.complex64) for m in range(M)]

    offsetCalib = np.empty([M,M], np.int32)
    offsetCalibUp = np.array([0]*M, np.int32)
    offsetCalibDn = np.array([0]*M, np.int32)

    peaks0 = [np.array([0]*613, np.int32) for i in range(M)]
    peaks1 = [np.array([0]*613, np.int32) for i in range(M)]

    peaksUp = [np.array([0]*613, np.int32) for i in range(M)]
    peaksDn = [np.array([0]*613, np.int32) for i in range(M)]

    csiDn = np.empty([M,K,L], dtype=np.complex64) # MxKxL
    csiUp = np.empty([K,M,L], dtype=np.complex64) # KxMxL
    csiCalibDn = np.empty([M,L], dtype=np.complex64) # MxL
    csiCalibUp = np.empty([M,L], dtype=np.complex64) # MxL
    csiCalib = np.empty([M,M,L], dtype=np.complex64) # MxL

    wzdn = np.empty([M,K,L], dtype=np.complex64) # MxKxL
    wzdn_inst = np.empty([M,K,L], dtype=np.complex64) # MxKxL
    twzdn = np.empty([M,K,L], dtype=np.complex64) # MxKxL
    wjdn = np.empty([M,K,L], dtype=np.complex64) # MxKxL
    wjdn_inst = np.empty([M,K,L], dtype=np.complex64) # MxKxL
    twjdn = np.empty([M,K,L], dtype=np.complex64) # MxKxL

    w_err = np.empty([M,K,L], dtype=np.complex64) # MxKxL
    w_err_abs = np.empty([M,K,L], dtype=np.float64) # MxKxL
    w_err_abs_mean = np.empty([K,L], dtype=np.complex64) # KxL

    w_err_inst = np.empty([M,K,L], dtype=np.complex64) # MxKxL
    w_err_inst_abs = np.empty([M,K,L], dtype=np.float64) # MxKxL
    w_err_diff = np.empty([M,K,L], dtype=np.float64) # MxKxL
    w_err_inst_abs_mean = np.empty([K,L], dtype=np.complex64) # KxL
    w_err_mean_diff = np.empty([K,L], dtype=np.complex64) # KxL

    calib = np.empty((M, L)).astype(np.complex64)
    calib1st = np.empty((M,L)).astype(np.complex64)
    calibCorrection = np.empty((K, L)).astype(np.complex64)
    signal_power_cal = np.array([0]*2*M).astype(np.float64)
    noise_power_cal = np.array([0]*2*M).astype(np.float64)
    SNR_cal = np.array([0]*2*M).astype(np.float64)
    SNR_cal_min = np.array([0]*frame).astype(np.float64)

    tx_data = np.matlib.repmat(np.delete(lts_f, [0,1,2,3,4,5,32,59,60,61,62,63], 0), K, 1) # KxL
    tx_sig = np.empty([M,K,L], dtype=np.complex64) # KxL
    rx_sig = np.empty([K,K,L], dtype=np.complex64) # KxKxL
    rx_sig_power = np.empty([6,K,L], dtype=np.float64) # KxL
    rx_sig_intf = np.empty([6,K,L], dtype=np.float64) # KxL
    noise_power = np.empty([K,L], dtype=np.float64) # KxL
    SINR = np.zeros((6,frame,K), dtype=np.float64)
    first = True
    cont_plotter = plotter

    if cont_plotter:
        fig1, axes1= plt.subplots(nrows=M+1, ncols=3, figsize=(9,12))
        axes1[0,0].set_title('Implicit vs Explicit Error (inst)')
        axes1[0,1].set_title('Implicit vs Explicit Error')
        for m in range(M+1):
            axes1[m,0].set_xlim(0,L)
            axes1[m,0].set_ylim(0,10)
            axes1[m,0].set_ylabel('Ant %d'%m)
            axes1[m,0].legend(fontsize=10)
            axes1[m,1].set_xlim(0,L)
            axes1[m,1].set_ylim(0,10)
            axes1[m,1].legend(fontsize=10)
            axes1[m,2].set_xlim(0,L)
            axes1[m,2].set_ylim(0,1)
            axes1[m,2].legend(fontsize=10)
        axes1[M,0].set_ylabel('Mean')
        lines10 = [[axes1[m,0].plot(range(L), w_err_inst_abs[m,k,:], label='Inst Err Ant %d User %d'%(m,k))[0] for k in range(K)] for m in range(M)]
        linesMean10 = [axes1[M,0].plot(range(L), w_err_inst_abs_mean[k,:], label='Inst Mean Err User %d'%k)[0] for k in range(K)]
        lines11 = [[axes1[m,1].plot(range(L), w_err_abs[m,k,:], label='Err Ant %d User %d'%(m,k))[0] for k in range(K)] for m in range(M)]
        lines12 = [[axes1[m,2].plot(range(L), w_err_diff[m,k,:], label='Err Ant %d User %d'%(m,k))[0] for k in range(K)] for m in range(M)]
        linesMean11 = [axes1[M,1].plot(range(L), w_err_abs_mean[k,:], label='Mean Err User %d'%k)[0] for k in range(K)]
        linesMean12 = [axes1[M,2].plot(range(L), w_err_mean_diff[k,:], label='Mean Err User %d'%k)[0] for k in range(K)]
        fig1.show()
        fig2, axes2 = plt.subplots(nrows=M, ncols=2, figsize=(9,12))
        axes2[0,0].set_title('Calibration Downlink')
        axes2[0,1].set_title('Calibration Uplink')
        for m in range(M):
            axes2[m,0].set_xlim(0, symSamp)
            axes2[m,0].set_ylim(-1,1)
            axes2[m,0].set_ylabel('Ant %d'%m)
            axes2[m,0].legend(fontsize=10)
            axes2[m,1].set_xlim(0, symSamp)
            axes2[m,1].set_ylim(-1,1)
            axes2[m,1].legend(fontsize=10)
        #lines20 = [axes2[m,0].plot(range(symSamp), np.real(calibRxA[m][ref_ant]), label='Calib Ant %d->Ant %d'%(m,ref_ant))[0] for m in range(M)] 
        #lines21 = [axes2[m,1].plot(range(symSamp), np.real(calibRxA[ref_ant][m]), label='Calib Ant %d->Ant %d'%(ref_ant,m))[0] for m in range(M)] 
        lines20 = [axes2[m,0].plot(range(symSamp), calibRxDnA[m][:symSamp], label='Offset Calib Dn %d'%m)[0] for m in range(M)]
        lines21 = [axes2[m,1].plot(range(symSamp), calibRxUpA[m][:symSamp], label='Offset Calib Up %d'%m)[0] for m in range(M)]
        lines24 = [axes2[m,0].plot(range(symSamp), calibRxDnA[m][:symSamp])[0] for m in range(M)]
        lines25 = [axes2[m,1].plot(range(symSamp), calibRxUpA[m][:symSamp])[0] for m in range(M)]
        fig2.show()

        fig3, axes3 = plt.subplots(nrows=M, ncols=2, figsize=(9,12))
        axes3[0,0].set_title('Pilot Downlink')
        axes3[0,1].set_title('Pilot Uplink')
        for m in range(M):
            axes3[m,0].set_xlim(0,symSamp)
            axes3[m,0].set_ylim(-1,1)
            axes3[m,0].set_ylabel('Ant %d'%m)
            axes3[m,0].legend(fontsize=10)
            axes3[m,1].set_xlim(0,symSamp)
            axes3[m,1].set_ylim(-1,1)
            axes3[m,1].legend(fontsize=10)
        lines30 = [[axes3[m,0].plot(range(symSamp), np.real(pilotRxDnA[m][k]), label='Pilot Ant %d User %d Dn'%(m,k))[0] for k in range(K)] for m in range(M)]
        lines31 = [[axes3[m,1].plot(range(symSamp), np.real(pilotRxUpA[k][m]), label='Pilot Ant %d User %d Up'%(m,k))[0] for k in range(K)] for m in range(M)]
        #lines32 = [axes3[m,0].plot(range(symSamp), peaksDn[m][:symSamp], label='Offset Pilot Dn User %d Ant %d'%(K-1,m))[0] for m in range(M)]
        #lines33 = [axes3[m,1].plot(range(symSamp), peaksUp[m][:symSamp], label='Offset Pilot Up User %d Ant %d'%(K-1,m))[0] for m in range(M)]
        fig3.show()

    f = 0
    signal.signal(signal.SIGINT, signal_handler)
    tstart = datetime.datetime.now()
    while(running):
        # disarm correlator in the clients
        if not use_trig:    
            for i, sdr in enumerate(csdrs):
                sdr.writeRegister("IRIS30", CORR_CONF, int("00004001", 16))  # enable the correlator, with inputs from adc

        #print("New Trigger!")
        bad_read = False
        bad_frame = False
        # arm correlator in the clients
        if not use_trig:    
            for i, sdr in enumerate(csdrs):
                sdr.writeRegister("IRIS30", CORR_CONF, int("00004011", 16))  # enable the correlator, with inputs from adc

        trig_dev.writeSetting("TRIGGER_GEN", "")

        # collect uplink pilots
        for k in range(K):
            if bad_read: break
            for m in range(M):
                sr = bsdrs[m].readStream(rxStream_ul[m], [pilotRxUpA[k][m], pilotRxUpB[k][m]], symSamp)
                if sr.ret < symSamp:
                    print("PilotUP: k: %d, m %d ret %d"%(k,m,sr.ret))
                    bad_read = True

        for m in range(M):
            if m == ref_ant: continue
            if bad_read: break
            sr = bsdrs[m].readStream(rxStream_ul[m], [calibRxUpA[m], calibRxUpB[m]], symSamp)
            if sr.ret < symSamp:
                print("PilotUP: k: %d, m %d ret %d"%(k,m,sr.ret))
                bad_read = True
        for k in range(K):
            if bad_read: break
            sr = csdrs[k].readStream(rxStream_dl[k], [pilotRxDnA[ref_ant][k], pilotRxDnB[ref_ant][k]], symSamp)
            if sr.ret < symSamp:
                print("PilotDn: m: %d, k %d ret %d"%(ref_ant,k,sr.ret))
                bad_read = True
        #print("Done with reading uplink pilots")

        # collect downlink/reciprocity pilots from antenna m
        for m in range(M):
            if m == ref_ant: continue
            if bad_read: break
            for k in range(K):
                sr = csdrs[k].readStream(rxStream_dl[k], [pilotRxDnA[m][k], pilotRxDnB[m][k]], symSamp)
                if sr.ret < symSamp:
                    print("PilotDn: m: %d, k %d ret %d"%(m,k,sr.ret))
                    bad_read = True
            sr = bsdrs[ref_ant].readStream(rxStream_ul[ref_ant], [calibRxDnA[m], calibRxDnB[m]], symSamp)
            if sr.ret < symSamp:
                bad_read = True

        ## collect noise 
        #for m in range(M):
        #    sr = bsdrs[m].readStream(rxStream_ul[m], [quietRxUpA[m], quietRxUpB[m]], symSamp)
        #    if sr.ret < symSamp:
        #        bad_read = True
        #for k in range(K):
        #    sr = csdrs[k].readStream(rxStream_dl[k], [quietRxDnA[k], quietRxDnB[k]], symSamp)
        #    if sr.ret < symSamp:
        #        bad_read = True
        ##print("Done with reading noise")

        if bad_read: 
            print("bad read")
            continue
        #print("Done with readings")

        for m in range(M):
            print("frame %d, antenna %d"%(f,m))
            if ref_ant == m: 
                calib[m,:] = np.array([1]*L, np.complex64) #continue
                csiCalibDn[m,:] = np.array([0]*L, np.complex64) 
                csiCalibUp[m,:] = np.array([0]*L, np.complex64) 
                continue 
            calibRxDnA[m] -= np.mean(calibRxDnA[m])
            calibRxUpA[m] -= np.mean(calibRxUpA[m])
             
            a0, b0, peaks0[m] = find_lts(calibRxDnA[m], thresh=lts_thresh)
            a1, b1, peaks1[m] = find_lts(calibRxUpA[m], thresh=lts_thresh)
            #print("Dn: %d, Up: %d"%(a0, a1))
            offsetCalibDn[m] = 0 if not a0 else a0 - len(ltsSym) + cp_len 
            offsetCalibUp[m] = 0 if not a1 else a1 - len(ltsSym) + cp_len 
            if (offsetCalibDn[m] < 150 or offsetCalibUp[m] < 150): bad_frame = True
            dn_m1 = calibRxDnA[m][offsetCalibDn[m]:offsetCalibDn[m]+fft_size]
            dn_m2 = calibRxDnA[m][offsetCalibDn[m]+fft_size:offsetCalibDn[m]+2*fft_size]
            up_m1 = calibRxUpA[m][offsetCalibUp[m]:offsetCalibUp[m]+fft_size]
            up_m2 = calibRxUpA[m][offsetCalibUp[m]+fft_size:offsetCalibUp[m]+2*fft_size]

            Hdn1 = np.fft.fftshift(np.fft.fft(dn_m1, fft_size, 0), 0)
            Hdn2 = np.fft.fftshift(np.fft.fft(dn_m2, fft_size, 0), 0)
            Hup1 = np.fft.fftshift(np.fft.fft(up_m1, fft_size, 0), 0)
            Hup2 = np.fft.fftshift(np.fft.fft(up_m2, fft_size, 0), 0)
            csiCalibDn[m,:] = np.delete((Hdn1+Hdn2)*lts_f/2, [0,1,2,3,4,5,32,59,60,61,62,63]) #remove zero subcarriers
            csiCalibUp[m,:] = np.delete((Hup1+Hup2)*lts_f/2, [0,1,2,3,4,5,32,59,60,61,62,63]) #remove zero subcarriers
            
            dn_n = calibRxDnA[m][:fft_size]
            up_n = calibRxUpA[m][:fft_size]
            Hdnn = np.fft.fftshift(np.fft.fft(dn_n, fft_size, 0), 0)
            Hupn = np.fft.fftshift(np.fft.fft(up_n, fft_size, 0), 0)
            signal_power_cal[2*m] = np.mean(np.power((np.abs(Hdn1)+np.abs(Hdn2))/2,2))
            signal_power_cal[2*m+1] = np.mean(np.power((np.abs(Hup1)+np.abs(Hup2))/2,2))
            noise_power_cal[2*m] = np.mean(np.power(np.abs(Hdnn),2))
            noise_power_cal[2*m+1] = np.mean(np.power(np.abs(Hupn),2))

        SNR_cal = 10*np.log10(signal_power_cal/noise_power_cal)
        SNR_cal[2*ref_ant] = 100
        SNR_cal[2*ref_ant+1] = 100
        SNR_cal_min[f] = np.min(SNR_cal)

        for k in range(K):
            for m in range(M):
                pilotRxUpA[k][m] -= np.mean(pilotRxUpA[k][m]) 
                a2, b2, peaksUp[m] = find_lts(pilotRxUpA[k][m], thresh=lts_thresh)
                offsetPilotUp = 0 if not a2 else (a2 - len(ltsSym) + cp_len) 
                if offsetPilotUp < 150: bad_frame = True #print("No LTS in Uplink Pilot from Ant %d to User %d"%(m,k)) 
                up1 = pilotRxUpA[k][m][offsetPilotUp:offsetPilotUp+fft_size]
                up2 = pilotRxUpA[k][m][offsetPilotUp+fft_size:offsetPilotUp+2*fft_size]
                H1 = np.fft.fftshift(np.fft.fft(up1, fft_size, 0), 0)
                H2 = np.fft.fftshift(np.fft.fft(up2, fft_size, 0), 0)
                #csiUp[k,m,:] = H*lts.lts_freq 
                csiUp[k,m,:] = np.delete((H1+H2)*lts_f/2,[0,1,2,3,4,5,32,59,60,61,62,63],0)

        for m in range(M):
            for k in range(K):
                pilotRxDnA[m][k] -= np.mean(pilotRxDnA[m][k]) 
                a2, b2, peaksDn[m] = find_lts(pilotRxDnA[m][k], thresh=lts_thresh)
                offsetPilotDn = 0 if not a2 else (a2 - len(ltsSym) + cp_len)
                if offsetPilotDn < 150: bad_frame = True #print("No LTS in Downlink Pilot from Ant %d to User %d"%(m,k)) 
                dn1 = pilotRxDnA[m][k][offsetPilotDn:offsetPilotDn+fft_size]
                dn2 = pilotRxDnA[m][k][offsetPilotDn+fft_size:offsetPilotDn+2*fft_size]
                H1 = np.fft.fftshift(np.fft.fft(dn1, fft_size, 0), 0)
                H2 = np.fft.fftshift(np.fft.fft(dn2, fft_size, 0), 0)
                #csiDn[m,k,:] = H*lts.lts_freq 
                csiDn[m,k,:] = np.delete((H1+H2)*lts_f/2,[0,1,2,3,4,5,32,59,60,61,62,63],0)

        #for k in range(K):
        #    quietRxDnA[k] -= np.mean(quietRxDnA[k])
        #    offset = symSamp//2 - fft_size
        #    dn1 = quietRxDnA[k][offset:offset+fft_size]
        #    dn2 = quietRxDnA[k][offset+fft_size:offset+2*fft_size]
        #    H1 = np.fft.fftshift(np.fft.fft(dn1, fft_size, 0), 0)
        #    H2 = np.fft.fftshift(np.fft.fft(dn2, fft_size, 0), 0)
        #    noise_dn = np.delete((H1+H2)/2,[0,1,2,3,4,5,32,59,60,61,62,63],0)
        #    noise_power[k,:] = np.power(np.abs(noise_dn), 2)
 
        #for k in range(K):
        #    calibCorrection[k,:] = np.divide(csiDn[ref_ant,k,:], csiUp[k,ref_ant,:])
        

        #if bad_frame: 
        #    print("bad frame")
        #    continue

        for m in range(M):
            if ref_ant == m: 
                calib[m,:] = np.array([1]*L, np.complex64) #continue
            else:
                calib[m,:] = np.divide(csiCalibDn[m,:], csiCalibUp[m,:])
            if first: calib1st[m,:] = calib[m,:]

        first = False

        print("Calib Dn/Up Per Antenna Amplitudes")
        print(['{:.2f}'.format(x) for x in np.mean(np.abs(csiCalibDn), axis=1)])
        print(['{:.2f}'.format(x) for x in np.mean(np.abs(csiCalibUp), axis=1)])
        #print("")

        print("User 0 Pilot Dn/Up Per Antenna Amplitudes")
        print(['{:.2f}'.format(x) for x in np.mean(np.abs(csiDn), axis=2)[:,0]])
        print(['{:.2f}'.format(x) for x in np.mean(np.abs(csiUp), axis=2)[0,:]])
        if K > 1:
            print("User 1 Pilot Dn/Up Per Antenna Amplitudes")
            print(['{:.2f}'.format(x) for x in np.mean(np.abs(csiDn), axis=2)[:,1]])
            print(['{:.2f}'.format(x) for x in np.mean(np.abs(csiUp), axis=2)[1,:]])
        print("")
        print("")

        for l in range(L):
            wcsi = np.matmul(csiUp[:,:,l], np.diag(calib1st[:,l]))
            wcsi_inst = np.matmul(csiUp[:,:,l], np.diag(calib[:,l]))   
            w_err[:,:,l] = csiDn[:,:,l] - np.transpose(wcsi)
            w_err_inst[:,:,l] = csiDn[:,:,l] - np.transpose(wcsi_inst)
            #w_err[:,:,l] = csiDn[:,:,l] - np.transpose(np.matmul(np.matmul(np.diag(calibCorrection[:,l]), csiUp[:,:,l]), np.diag(calib1st[:,l])))
            #w_err_inst[:,:,l] = csiDn[:,:,l] - np.transpose(np.matmul(np.matmul(np.diag(calibCorrection[:,l]), csiUp[:,:,l]), np.diag(calib[:,l])))
            wzdn[:,:,l]      = np.linalg.pinv(wcsi)
            wzdn_inst[:,:,l] = np.linalg.pinv(wcsi_inst)
            twzdn[:,:,l]     = np.linalg.pinv(csiUp[:,:,l]) #np.transpose(np.linalg.pinv(csiDn[:,:,l]))
            wjdn[:,:,l]      = np.transpose(np.conj(wcsi))
            wjdn_inst[:,:,l] = np.transpose(np.conj(wcsi_inst))
            twjdn[:,:,l]     = np.transpose(np.conj(csiUp[:,:,l])) #np.conj(csiDn[:,:,l])

            tx_sig[:, :, l] = np.matmul(wzdn[:,:,l], np.diag(tx_data[:,l]))
            rx_sig[:, :, l] = np.matmul(np.transpose(csiDn[:,:,l]), tx_sig[:,:,l]) 
            rx_sig_abs = np.power(np.abs(rx_sig[:,:,l]), 2)
            rx_sig_power[0, :, l] = np.diag(rx_sig_abs)
            rx_sig_intf[0, :, l] = np.sum(rx_sig_abs, axis=1) - rx_sig_power[0, :, l]

            tx_sig[:, :, l] = np.matmul(wzdn_inst[:,:,l], np.diag(tx_data[:,l]))
            rx_sig[:, :, l] = np.matmul(np.transpose(csiDn[:,:,l]), tx_sig[:,:,l]) 
            rx_sig_abs = np.power(np.abs(rx_sig[:,:,l]), 2)
            rx_sig_power[1, :, l] = np.diag(rx_sig_abs)
            rx_sig_intf[1, :, l] = np.sum(rx_sig_abs, axis=1) - rx_sig_power[1, :, l]

            tx_sig[:, :, l] = np.matmul(twzdn[:,:,l], np.diag(tx_data[:,l]))
            rx_sig[:, :, l] = np.matmul(np.transpose(csiDn[:,:,l]), tx_sig[:,:,l]) 
            rx_sig_abs = np.power(np.abs(rx_sig[:,:,l]), 2)
            rx_sig_power[2, :, l] = np.diag(rx_sig_abs)
            rx_sig_intf[2, :, l] = np.sum(rx_sig_abs, axis=1) - rx_sig_power[2, :, l]

            tx_sig[:, :, l] = np.matmul(wjdn[:,:,l], np.diag(tx_data[:,l]))
            rx_sig[:, :, l] = np.matmul(np.transpose(csiDn[:,:,l]), tx_sig[:,:,l]) 
            rx_sig_abs = np.power(np.abs(rx_sig[:,:,l]), 2)
            rx_sig_power[3, :, l] = np.diag(rx_sig_abs)
            rx_sig_intf[3, :, l] = np.sum(rx_sig_abs, axis=1) - rx_sig_power[3, :, l]

            tx_sig[:, :, l] = np.matmul(wjdn_inst[:,:,l], np.diag(tx_data[:,l]))
            rx_sig[:, :, l] = np.matmul(np.transpose(csiDn[:,:,l]), tx_sig[:,:,l]) 
            rx_sig_abs = np.power(np.abs(rx_sig[:,:,l]), 2)
            rx_sig_power[4, :, l] = np.diag(rx_sig_abs)
            rx_sig_intf[4, :, l] = np.sum(rx_sig_abs, axis=1) - rx_sig_power[4, :, l]

            tx_sig[:, :, l] = np.matmul(twjdn[:,:,l], np.diag(tx_data[:,l]))
            rx_sig[:, :, l] = np.matmul(np.transpose(csiDn[:,:,l]), tx_sig[:,:,l]) 
            rx_sig_abs = np.power(np.abs(rx_sig[:,:,l]), 2)
            rx_sig_power[5, :, l] = np.diag(rx_sig_abs)
            rx_sig_intf[5, :, l] = np.sum(rx_sig_abs, axis=1) - rx_sig_power[5, :, l]

        print("signal power at user 0 %f"%np.mean(rx_sig_power[0,0,:]))
        print("intfer power at user 0 %f"%np.mean(rx_sig_intf[0,0,:]))
        for i in range(6):
            #SINR[i,f,:] = 10*np.log10(np.mean(rx_sig_power[i,:,:]/(rx_sig_intf[i,:,:]+noise_power), axis=1))
            SINR[i,f,:] = 10*np.log10(np.mean(rx_sig_power[i,:,:]/(rx_sig_intf[i,:,:]), axis=1))
        print(SINR[:,f,:])
        w_err_abs = np.abs(w_err)
        w_err_abs_mean = np.mean(w_err_abs, 0) # KxL
        w_err_inst_abs = np.abs(w_err_inst)
        w_err_inst_abs_mean = np.mean(w_err_inst_abs, 0) # KxL
        #print (w_err_abs_mean)

        f += 1
        if f >= frame: break
        if cont_plotter:
            for m in range(M):
                for k in range(K):
                    lines10[m][k].set_ydata(w_err_inst_abs[m,k,:])
                    lines11[m][k].set_ydata(w_err_abs[m,k,:])
                    lines12[m][k].set_ydata(np.abs(w_err_inst_abs[m,k,:]-w_err_abs[m,k,:]))
            for k in range(K):
                linesMean10[k].set_ydata(w_err_inst_abs_mean if K == 1 else w_err_inst_abs_mean[k,:])
                linesMean11[k].set_ydata(w_err_abs_mean if K == 1 else w_err_abs_mean[k,:])
                linesMean12[k].set_ydata(np.abs(w_err_inst_abs_mean-w_err_abs_mean) if K == 1 else np.abs(w_err_inst_abs_mean[k,:]-w_err_abs_mean[k,:]))

            for m in range(M):
                if m == ref_ant:
                    lines20[m].set_ydata(np.real(wb_pilot1))
                    lines21[m].set_ydata(np.real(wb_pilot1))
                    continue
                #lines20[m].set_ydata(np.real(calibRxA[m][ref_ant]))
                #lines21[m].set_ydata(np.real(calibRxA[ref_ant][m]))
                lines20[m].set_ydata(np.real(calibRxDnA[m]))
                lines21[m].set_ydata(np.real(calibRxUpA[m]))
                #lines22[m].set_ydata(peaks0[m][:symSamp]/np.max(peaks0[m][:symSamp]))
                #lines23[m].set_ydata(peaks1[m][:symSamp]/np.max(peaks1[m][:symSamp]))
                lines24[m].set_data(offsetCalibDn[m], np.linspace(-1.0, 1.0, num=100))
                lines25[m].set_data(offsetCalibUp[m], np.linspace(-1.0, 1.0, num=100))

            for m in range(M):
                for k in range(K):
                    lines30[m][k].set_ydata(np.real(pilotRxDnA[m][k]))
                    lines31[m][k].set_ydata(np.real(pilotRxUpA[k][m]))
                #lines32[m].set_ydata(peaksDn[m][:symSamp]/np.max(peaksDn[m][:symSamp]))
                #lines33[m].set_ydata(peaksUp[m][:symSamp]/np.max(peaksUp[m][:symSamp]))

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
    print("ran for %d"%c.total_seconds())
    #print("SINR")
    #print(SINR[:f,:])
    #print("SNR_cal_min")
    #print(SNR_cal_min)
    scipy.io.savemat("SINR"+str(M)+"x"+str(K)+".mat", mdict={'SINR':SINR})
    if f > 5:
        fig4 = [plt.figure(figsize=(20, 8), dpi=100) for k in range(K)]
        for k in range(K): 
            ax1 = fig4[k].add_subplot(1, 1, 1)
            ax1.set_xlim(0, f)
            ax1.set_ylim(-40,40)
            ax1.set_ylabel('SINR')
            ax1.plot(range(f), SINR[0,:f,k], label='ZF, 1-time cal, User %d'%(k))
            ax1.plot(range(f), SINR[1,:f,k], label='ZF, conti. cal, User %d'%(k)) 
            ax1.plot(range(f), SINR[2,:f,k], label='ZF, no cal(UL), User %d'%(k)) 
            ax1.legend(fontsize=10)
        fig5 = [plt.figure(figsize=(20, 8), dpi=100) for k in range(K)]
        for k in range(K): 
            ax2 = fig5[k].add_subplot(1, 1, 1)
            ax2.set_xlim(0, f)
            ax2.set_ylim(-40,40)
            ax2.set_ylabel('SINR')
            ax2.plot(range(f), SINR[3,:f,k], label='Conj, 1-time cal, User %d'%(k))
            ax2.plot(range(f), SINR[4,:f,k], label='Conj, conti. cal, User %d'%(k))
            ax2.plot(range(f), SINR[5,:f,k], label='Conj, no cal(UL), User %d'%(k))
            ax2.legend(fontsize=10)
        plt.show()
    #for sdr in bsdrs+[ref_sdr]+csdrs:
    for sdr in bsdrs+csdrs:
        sdr.writeRegister("IRIS30", RF_RST_REG, (1<<29)| 0x1)
        sdr.writeRegister("IRIS30", RF_RST_REG, (1<<29))
        sdr.writeRegister("IRIS30", RF_RST_REG, 0)
        for i in range(frameLen):
            sdr.writeRegister("RFCORE", SCH_ADDR_REG, i) # subframe 0
            sdr.writeRegister("RFCORE", SCH_MODE_REG, 0) # 01 replay
        sdr.writeRegister("RFCORE", TDD_CONF_REG, 0)

    #ref_sdr.closeStream(rxStream_recip_dl)
    for i,sdr in enumerate(bsdrs):
        sdr.closeStream(rxStream_ul[i])
    for i,sdr in enumerate(csdrs):
        sdr.closeStream(rxStream_dl[i])

def main():
    parser = OptionParser()
    parser.add_option("--args", type="string", dest="args", help="arguments", default="")
    parser.add_option("--bnodes", type="string", dest="bnodes", help="file name containing serials on the base station", default="bs_serials.txt")
    parser.add_option("--cnodes", type="string", dest="cnodes", help="file name containing serials to be used as clients", default="client_serials.txt")
    parser.add_option("--hub", type="string", dest="hub", help="Hub node", default="")
    parser.add_option("--ref-ant", type="int", dest="ref_ant", help="Calibration reference antenna", default=0)
    parser.add_option("--ampl", type="float", dest="ampl", help="Amplitude coefficient for downCal/upCal", default=5.0)
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--freq", type="float", dest="freq", help="Optional Tx freq (Hz)", default=3.6e9)
    parser.add_option("--txgain", type="float", dest="txgain", help="Optional Tx gain (dB)", default=20.0)
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
	plotter=options.plotter,
	numSamps=options.numSamps,
	prefix_length=options.prefix_length,
	postfix_length=options.postfix_length,
	tx_advance=options.tx_advance,
	threshold=options.threshold,
	use_trig=options.use_trig
    )

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
#    except Exception as e:
#	print e
#	exit()

