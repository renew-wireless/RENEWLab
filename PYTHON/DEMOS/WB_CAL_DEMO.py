#!/usr/bin/env python3
"""
 WB_CAL_DEMO.py

  Wideband calibration demo:
  It performs Argos calibration method among anntennas
  within a base station, and plots the magnitude and phase
  of the calibration vector on 4 select subcarriers for
  all antennas.

 Example:
    python3 WB_CAL_DEMO.py

---------------------------------------------------------------------
 Copyright (c) 2018-2019, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import signal
import time
from optparse import OptionParser
import sys
import collections
import matplotlib
import matplotlib.pyplot as plt
matplotlib.rcParams.update({'font.size': 10})
import numpy as np
import SoapySDR
from SoapySDR import *
sys.path.append('../IrisUtils/')
from find_lts import *
from sklearn.linear_model import LinearRegression

plt.style.use('ggplot')  # customize your plots style

RF_RST_REG = 48
CORR_CONF = 60
CORR_RST = 64
CORR_THRESHOLD = 92
TDD_CONF_REG = 120
SCH_ADDR_REG = 136
SCH_MODE_REG = 140

NUM_BUFFER_SAMPS = 3000
LTS_THRESH = 0.8
RUNNING = True


def signal_handler(signum, frame):
    global RUNNING
    RUNNING = False


def init(hub, bnodes, ref_ant, ampl, rate,
        freq, txgain, rxgain, cyc_prefix, num_samps,
        prefix_length, postfix_length, take_duration,
        both_channels, plotter):
    """main function initializing all radios and performing calibration"""

    if hub != "":
        hub_dev = SoapySDR.Device(dict(driver="remote", serial=hub))
    bsdrs = [SoapySDR.Device(dict(driver="iris", serial=serial)) for serial in bnodes]

    ant = 2 if both_channels else 1
    num_sdrs = len(bsdrs)
    num_ants = num_sdrs * ant

    # assume trig_sdr is part of the master nodes
    trig_dev = None
    if hub != "":
        trig_dev = hub_dev
    else:
        trig_dev = bsdrs[0]

    #set params on both channels
    for sdr in bsdrs:
        info = sdr.getHardwareInfo()
        print("%s settings on device" % (info["frontend"]))
        for ch in [0, 1]:
            sdr.setBandwidth(SOAPY_SDR_TX, ch, 2.5*rate)
            sdr.setBandwidth(SOAPY_SDR_RX, ch, 2.5*rate)
            sdr.setSampleRate(SOAPY_SDR_TX, ch, rate)
            sdr.setSampleRate(SOAPY_SDR_RX, ch, rate)
            sdr.setFrequency(SOAPY_SDR_TX, ch, 'RF', freq-.75*rate)
            sdr.setFrequency(SOAPY_SDR_RX, ch, 'RF', freq-.75*rate)
            sdr.setFrequency(SOAPY_SDR_TX, ch, 'BB', .75*rate)
            sdr.setFrequency(SOAPY_SDR_RX, ch, 'BB', .75*rate)

            sdr.setGain(SOAPY_SDR_TX, ch, txgain)
            sdr.setGain(SOAPY_SDR_RX, ch, rxgain)

            sdr.setAntenna(SOAPY_SDR_RX, ch, "TRX")
            sdr.setDCOffsetMode(SOAPY_SDR_RX, ch, True)

            # Read initial gain settings
            read_lna = sdr.getGain(SOAPY_SDR_RX, 0, 'LNA')
            read_tia = sdr.getGain(SOAPY_SDR_RX, 0, 'TIA')
            read_pga = sdr.getGain(SOAPY_SDR_RX, 0, 'PGA')
            print("INITIAL GAIN - LNA: {}, \t TIA:{}, \t PGA:{}".format(read_lna, read_tia, read_pga))

        #for ch in [0, 1]:
        #    if calibrate:
        #        sdr.writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", 'SKLK')
        #        sdr.writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", '')

        sdr.writeSetting("RESET_DATA_LOGIC", "")
        if not both_channels:
            sdr.writeSetting(SOAPY_SDR_RX, 1, 'ENABLE_CHANNEL', 'false')
            sdr.writeSetting(SOAPY_SDR_TX, 1, 'ENABLE_CHANNEL', 'false')

    trig_dev.writeSetting("SYNC_DELAYS", "")

    sym_samps = num_samps + prefix_length + postfix_length
    print("num_samps = %d" % sym_samps)

    fft_size = 64
    cp_len = 32 if cyc_prefix else 0
    lts_sym, _ = generate_training_seq(preamble_type='lts', cp=cp_len, upsample=1)
    ofdm_len = len(lts_sym)
    zeros = np.array([0]*(num_samps-ofdm_len))
    pilot = np.concatenate((lts_sym, zeros)).astype(np.complex64)
    wb_pilot = ampl * pilot
    wbz = np.array([0]*(sym_samps), np.complex64)
    pad1 = np.array([0]*(prefix_length), np.complex64) # to comprensate for front-end group delay
    pad2 = np.array([0]*(postfix_length), np.complex64) # to comprensate for rf path delay
    wb_pilot_pad = np.concatenate([pad1, wb_pilot, pad2]).astype(np.complex64)
    pilot_subcarriers = [7, 21, 43, 57]
    pilot_sc_num = len(pilot_subcarriers)

    # Create streams
    tx_streams = [sdr.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, [0, 1]) for sdr in bsdrs]
    rx_streams = [sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0, 1]) for sdr in bsdrs]
    for r,sdr in enumerate(bsdrs):
        sdr.activateStream(tx_streams[r])

    rx_samps = [[np.empty(sym_samps).astype(np.complex64) for r in range(num_ants)] for t in range(num_ants)]
    rx_f = [[np.empty(fft_size).astype(np.complex64) for r in range(num_ants)] for t in range(num_ants)]
    calib_mag = [np.empty(fft_size).astype(np.complex64) for r in range(num_ants)]
    calib_ang = [np.empty(fft_size).astype(np.complex64) for r in range(num_ants)]
    est_calib_mag = [np.empty(fft_size).astype(np.complex64) for r in range(num_ants)]
    est_calib_ang = [np.empty(fft_size).astype(np.complex64) for r in range(num_ants)]
    mag_buffer = [[collections.deque(maxlen=NUM_BUFFER_SAMPS) for i in range(pilot_sc_num)] for j in range(num_ants)]
    ang_buffer = [[collections.deque(maxlen=NUM_BUFFER_SAMPS) for i in range(len(pilot_subcarriers))] for j in range(num_ants)]
    dummy = np.empty(sym_samps).astype(np.complex64)

    fig1, axes1 = plt.subplots(nrows=num_ants, ncols=2, figsize=(12,8))
    axes1[0,0].set_title('Reciprocity Calibration Magnitude')
    axes1[0,1].set_title('Reciprocity Calibration Phase')
    for m in range(num_ants):
        axes1[m,0].set_xlim(0, NUM_BUFFER_SAMPS)
        axes1[m,1].set_xlim(0, NUM_BUFFER_SAMPS)
        axes1[m,0].set_ylim(0,5)
        axes1[m,1].set_ylim(-np.pi,np.pi)
        if m == ref_ant:
            axes1[m,0].set_ylabel('Ant %d (ref)'%(m))
        else:
            axes1[m,0].set_ylabel('Ant %d'%(m))

    lines10 = [[axes1[m,0].plot(range(NUM_BUFFER_SAMPS), np.zeros(NUM_BUFFER_SAMPS), label='SC %d'%(pilot_subcarriers[p]))[0] for p in range(pilot_sc_num)] for m in range(num_ants)]
    lines11 = [[axes1[m,1].plot(range(NUM_BUFFER_SAMPS), np.zeros(NUM_BUFFER_SAMPS), label='SC %d'%(pilot_subcarriers[p]))[0] for p in range(pilot_sc_num)] for m in range(num_ants)]
    for m in range(num_ants):
        for l in range(2):
            axes1[m,l].legend(fontsize=10)
    fig1.show()

    fig3, axes3 = plt.subplots(nrows=num_ants, ncols=2, figsize=(12,8))
    axes3[0,0].set_title('Reciprocity Calibration Magnitude')
    axes3[0,1].set_title('Reciprocity Calibration Phase')
    for m in range(num_ants):
        axes3[m,0].set_xlim(-8, 72)
        axes3[m,1].set_xlim(-8, 72)
        axes3[m,0].set_ylim(0,5)
        axes3[m,1].set_ylim(-np.pi,np.pi)
        if m == ref_ant:
            axes3[m,0].set_ylabel('Ant %d (ref)'%(m))
        else:
            axes3[m,0].set_ylabel('Ant %d'%(m))

    lines300 = [axes3[m,0].plot(range(64), np.zeros(64), label='Measured Mag')[0] for m in range(num_ants)]
    lines301 = [axes3[m,0].plot(range(64), np.zeros(64), label='Estimated Mag')[0] for m in range(num_ants)]
    lines310 = [axes3[m,1].plot(range(64), np.zeros(64), label='Measured Ang')[0] for m in range(num_ants)]
    lines311 = [axes3[m,1].plot(range(64), np.zeros(64), label='Estimated Ang')[0] for m in range(num_ants)]
    for m in range(num_ants):
        for l in range(2):
            axes3[m,l].legend(fontsize=10)
    fig3.show()

    if plotter:
        fig2, axes2 = plt.subplots(nrows=num_ants, ncols=num_ants, figsize=(12,12))
        for m in range(num_ants):
            for l in range(num_ants):
                axes2[m,l].set_xlim(0, sym_samps)
                axes2[m,l].set_ylim(-1,1)
                axes2[m,l].set_ylabel('Tx Ant %d, Rx Ant %d'%(m,l))
                axes2[m,l].legend(fontsize=10)

        lines20 = [[axes2[m,l].plot(range(sym_samps), np.real(wbz), label='Pilot TxAnt %d RxAnt %d (real)'%(m,l))[0] for l in range(num_ants)] for m in range(num_ants)]
        lines21 = [[axes2[m,l].plot(range(sym_samps), np.imag(wbz), label='Pilot TxAnt %d RxAnt %d (imag)'%(m,l))[0] for l in range(num_ants)] for m in range(num_ants)]
        lines22 = [[axes2[m,l].plot(range(sym_samps), sym_samps*[LTS_THRESH])[0] for m in range(num_ants)] for l in range(num_ants)] 
        fig2.show()

    first = True
    take = 0
    prev_take = take
    cur_mean_ang = np.ndarray(shape=(num_ants, pilot_sc_num), dtype=float)
    cur_mean_mag = np.ndarray(shape=(num_ants, pilot_sc_num), dtype=float)
    prev_mean_ang = np.ndarray(shape=(num_ants, pilot_sc_num), dtype=float)
    prev_mean_mag = np.ndarray(shape=(num_ants, pilot_sc_num), dtype=float)
    first_mean_ang = np.ndarray(shape=(num_ants, pilot_sc_num), dtype=float)
    first_mean_mag = np.ndarray(shape=(num_ants, pilot_sc_num), dtype=float)

    signal.signal(signal.SIGINT, signal_handler)
    begin = time.time()
    prev_time = begin
    while RUNNING:
        for d in range(num_sdrs):
            ref_sdr = bsdrs[d]
            flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST

            # transmit pilot from node m
            sr = ref_sdr.writeStream(tx_streams[d], [wb_pilot_pad, wbz], sym_samps, flags)
            if sr.ret == -1:
                print("bad write")
            
            flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST
            for r,sdr in enumerate(bsdrs):
                if r != d: 
                    sdr.activateStream(rx_streams[r], flags, 0, sym_samps)

            trig_dev.writeSetting("TRIGGER_GEN", "")
            
            for r,sdr in enumerate(bsdrs):
                if r != d:
                    sr = sdr.readStream(rx_streams[r], [rx_samps[d][r], dummy], sym_samps, timeoutUs=int(1e6))
                    if sr.ret != sym_samps:
                        print("bad read %d"%sr.ret)

        bad_data = False
        for m in range(num_ants):
            if bad_data: break
            for p in range(num_ants):
                if m != p:
                    rx_samps[m][p] -= np.mean(rx_samps[m][p])
                    best_peak, _, _ = find_lts(rx_samps[m][p], thresh=LTS_THRESH, flip=True)
                    offset = 0 if not best_peak else best_peak - len(lts_sym) + cp_len
                    if offset < 150:
                        print("bad data, skip")
                        bad_data = True
                        #break
                    rx_m_p = rx_samps[m][p]
                    rx_f1 = np.fft.fft(rx_m_p[offset:offset+fft_size], fft_size, 0)
                    rx_f2 = np.fft.fft(rx_m_p[offset+fft_size:offset+2*fft_size], fft_size, 0)
                    rx_f[m][p] = (rx_f1+rx_f2)/2

                    if plotter:
                        lines20[m][p].set_ydata(np.real(rx_samps[m][p]))
                        lines21[m][p].set_ydata(np.imag(rx_samps[m][p]))
                        lines22[m][p].set_data(offset, np.linspace(-1.0, 1.0, num=100))

        if bad_data:
            continue

        mag_model = LinearRegression()
        ang_model = LinearRegression()
        for m in range(num_ants):
            if m == ref_ant:
                calib_mag[m] = np.ones(fft_size)
                calib_ang[m] = np.zeros(fft_size)
            else: 
                calib_mag[m] = np.divide(np.abs(rx_f[m][ref_ant]), np.abs(rx_f[ref_ant][m]))
                calib_ang[m] = np.angle(rx_f[m][ref_ant]*np.conj(rx_f[ref_ant][m]))
            for c in range(pilot_sc_num):
                s = pilot_subcarriers[c]
                mag_buffer[m][c].append(calib_mag[m][s])
                ang_buffer[m][c].append(calib_ang[m][s])
            x = np.asarray(pilot_subcarriers).reshape((-1,1))
            y_mag = calib_mag[m][pilot_subcarriers]
            y_ang = calib_ang[m][pilot_subcarriers]
            mag_model.fit(x, y_mag)
            ang_model.fit(x, y_ang)
            for c in range(64):
                est_calib_mag[m][c] = mag_model.intercept_ + mag_model.coef_ * c
                est_calib_ang[m][c] = ang_model.intercept_ + ang_model.coef_ * c

        for m in range(num_ants):
            for p in range(pilot_sc_num):
                lines10[m][p].set_data(range(len(mag_buffer[m][p])), mag_buffer[m][p])
                lines11[m][p].set_data(range(len(mag_buffer[m][p])), ang_buffer[m][p])
            lines300[m].set_ydata(calib_mag[m])
            lines301[m].set_ydata(est_calib_mag[m])
            lines310[m].set_ydata(calib_ang[m])
            lines311[m].set_ydata(est_calib_ang[m])

        fig1.canvas.draw()
        fig1.show()
        fig3.canvas.draw()
        fig3.show()
        if plotter:
            fig2.canvas.draw()
            fig2.show()

        cur_time = time.time()
        if (cur_time - prev_time > take_duration):
            take_num = take - prev_take
            for m in range(num_ants):
                for p in range(pilot_sc_num):
                    mag_buffer_list = list(mag_buffer[m][p])
                    ang_buffer_list = list(ang_buffer[m][p])
                    mag_buffer_shot = mag_buffer_list[-take_num:]
                    ang_buffer_shot = ang_buffer_list[-take_num:]
                    cur_mean_mag[m,p] = np.mean(mag_buffer_shot)
                    cur_mean_ang[m,p] = np.mean(ang_buffer_shot)
            if first:
                first_mean_mag = cur_mean_mag.copy()
                first_mean_ang = cur_mean_ang.copy()
                first = False

            mag_drift_last = cur_mean_mag - prev_mean_mag
            ang_drift_last = np.unwrap(cur_mean_ang - prev_mean_ang)
            mag_drift_start = cur_mean_mag - first_mean_mag
            ang_drift_start = np.unwrap(cur_mean_ang - first_mean_ang)
            print("%d total takes, %d new takes, %f secs elapsed:"%(take, take_num, cur_time-begin))
            print("Mag drift from last take:")
            print(mag_drift_last[1:,:])
            print("")
            print("Ang drift from last take:")
            print(ang_drift_last[1:,:])
            print("")
            print("Mag drift from first take:")
            print(mag_drift_start[1:,:])
            print("")
            print("Ang drift from first take:")
            print(ang_drift_start[1:,:])
            print("")
            print("")
            print("")
            prev_time = cur_time
            prev_take = take
            prev_mean_mag = cur_mean_mag.copy()
            prev_mean_ang = cur_mean_ang.copy()
        take += 1

    for r,sdr in enumerate(bsdrs):
        sdr.closeStream(tx_streams[r])
        sdr.closeStream(rx_streams[r])


def main():
    parser = OptionParser()
    parser.add_option("--bnodes", type="string", dest="bnodes", help="file name containing serials on the base station", default="../IrisUtils/data_in/bs_serials.txt")
    parser.add_option("--hub", type="string", dest="hub", help="Hub node", default="")
    parser.add_option("--ref-ant", type="int", dest="ref_ant", help="Calibration reference antenna", default=0)
    parser.add_option("--ampl", type="float", dest="ampl", help="Pilot amplitude scaling coefficient", default=0.5)
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--freq", type="float", dest="freq", help="Optional Tx freq (Hz)", default=2.5e9)
    parser.add_option("--txgain", type="float", dest="txgain", help="Optional Tx gain (dB) w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]", default=30.0)
    parser.add_option("--rxgain", type="float", dest="rxgain", help="Optional Rx gain (dB) w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]", default=30.0)
    parser.add_option("--cp", action="store_true", dest="cyc_prefix", help="adds cyclic prefix to tx symbols", default=True)
    parser.add_option("--num-samps", type="int", dest="num_samps", help="Number of samples in Symbol", default=400)
    parser.add_option("--prefix-length", type="int", dest="prefix_length", help="prefix padding length for beacon and pilot", default=100)
    parser.add_option("--postfix-length", type="int", dest="postfix_length", help="postfix padding length for beacon and pilot", default=100)
    parser.add_option("--take-duration", type="float", dest="take_duration", help="measurement single take duration", default=5.0)
    parser.add_option("--both-channels", action="store_true", dest="both_channels", help="transmit from both channels",default=False)
    parser.add_option("--plot-samps", action="store_true", dest="plotter", help="plots all rx signals",default=False)
    (options, args) = parser.parse_args()

    bserials = []
    with open(options.bnodes, "r") as f:
        for line in f.read().split():
            if line[0] != '#':
                bserials.append(line)
            else:
                continue      

    init(
        hub=options.hub,
        bnodes=bserials,
        ref_ant=options.ref_ant,
        ampl=options.ampl,
        rate=options.rate,
        freq=options.freq,
        txgain=options.txgain,
        rxgain=options.rxgain,
        cyc_prefix=options.cyc_prefix,
        num_samps=options.num_samps,
        prefix_length=options.prefix_length,
        postfix_length=options.postfix_length,
        take_duration=options.take_duration,
        both_channels=options.both_channels,
        plotter=options.plotter
    )


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()

