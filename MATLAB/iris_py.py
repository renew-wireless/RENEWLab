#!/usr/bin/python
# -*- coding: UTF-8 -*-
""" 
        Author(s): C. Nicolas Barati nicobarati@rice.edu 
                Rahman Doost-Mohamamdy: doost@rice.edu
                Oscar Bejarano: obejarano@rice.edu

---------------------------------------------------------------------
        Copyright (c) 2018-2019, Rice University
        RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
from argparse import ArgumentParser
import SoapySDR
from SoapySDR import *  # SOAPY_SDR_ constants
from optparse import OptionParser
import numpy as np
import time
import os
import math
import json
import threading
import matplotlib.pyplot as plt
import hub_py
from hub_py import *

# Global variables
corr_threshold = 1

#preambles_bs = generate_training_seq(preamble_type='gold_ifft', seq_length=128, cp=0, upsample=1)
#beacon = preambles_bs[0, :]*.5
#np.savetxt('beacon.txt', np.column_stack([beacon.real, beacon.imag]))

# Read beacon from file
bcn_real, bcn_imag = np.loadtxt('beacon.txt', unpack=True)
beacon = bcn_real + 1j * bcn_imag


#Generate LTS seq:
def gen_lts(seq_length=128, cp=0, upsample=1):
    # Generate 802.11 LTS preamble
    lts_freq = np.array([
        0, 0, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -
        1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 0, 1, -1, -1, 1,
        1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, 0, 0, 0, 0, 0])
    up_zeros = np.zeros(len(lts_freq) // 2 * (upsample - 1))
    lts_freq_up = np.concatenate((up_zeros, lts_freq, up_zeros))
    signal = np.fft.ifft(np.fft.ifftshift(lts_freq_up))
    # Now affix the cyclic prefix
    # could use tile...
    sequence = np.concatenate((signal[len(signal) - cp:], signal, signal))
    return sequence, lts_freq


def cfloat2uint32(arr, order='IQ'):
    """
    Convert floating point iq values to uint32 (FPGA format)
    ARGS:
    - arr: Data array
    - order: Whether it is IQ or QI
    RETURNS:
    - corresponding uint32 value
    """
    arr_i = (np.real(arr) * 32767).astype(np.uint16)
    arr_q = (np.imag(arr) * 32767).astype(np.uint16)
    if order == 'IQ':
        return np.bitwise_or(arr_q, np.left_shift(arr_i.astype(np.uint32), 16))
    else:
        return np.bitwise_or(arr_i, np.left_shift(arr_q.astype(np.uint32), 16))


#######################################
####### SDR Class:              #######
#######################################

class Iris_py:
        '''
                Iris python class. To act as a middle layer between Matlab classes and SoapySDRs
        '''

        def __init__(self,
              serial_id=None,
              tx_freq=None,
              rx_freq=None,
              tx_gain=None,
              rx_gain=None,
              bw=None,
              sample_rate=None,
              n_samp=None,                              # Total number of samples, including zero-pads
              both_channels=False,
              agc_en=False,
              ):

                if serial_id is not None:
                        self.sdr = SoapySDR.Device(
                            dict(driver="iris", timeout="1000000", serial=serial_id))
                        self.serial_id = serial_id
                else:
                        self.sdr = None

                self.sample_rate = sample_rate

                if n_samp is not None:
                    self.n_samp = int(n_samp)

                self.agc_en = agc_en
                self.both_channels = both_channels
                self.max_frames = 1

                ### Setup channel rates, ports, gains, and filters ###
                info = self.sdr.getHardwareInfo()
                for chan in [0, 1]:

                        #Tx:
                        if sample_rate is not None:
                                self.sdr.setSampleRate(SOAPY_SDR_TX, chan, sample_rate)
                        if bw is not None:
                                self.sdr.setBandwidth(SOAPY_SDR_TX, chan, bw)
                        else:
                                self.sdr.setBandwidth(SOAPY_SDR_TX, chan, 2.5*sample_rate)
                        if tx_gain is not None:
                                self.sdr.setGain(SOAPY_SDR_TX, chan, min(tx_gain, 81.0))
                        if tx_freq is not None:
                                self.sdr.setFrequency(SOAPY_SDR_TX, chan, 'RF', tx_freq - .75*sample_rate)
                                self.sdr.setFrequency(SOAPY_SDR_TX, chan, 'BB', .75*sample_rate)

                        #print("Set TX frequency to %f" % self.sdr.getFrequency(SOAPY_SDR_TX, chan))
                        #self.sdr.setAntenna(SOAPY_SDR_TX, chan, "TRX")
                        #self.sdr.setGain(SOAPY_SDR_TX, chan, 'ATTN', -6)

                        #Rx:
                        if sample_rate is not None:
                                self.sdr.setSampleRate(SOAPY_SDR_RX, chan, sample_rate)
                        if bw is not None:
                                self.sdr.setBandwidth(SOAPY_SDR_RX, chan, bw)
                        else:
                                self.sdr.setBandwidth(SOAPY_SDR_RX, chan, 2.5*sample_rate)
                        if rx_gain is not None:
                                self.sdr.setGain(SOAPY_SDR_RX, chan, rx_gain)
                        if rx_freq is not None:
                                self.sdr.setFrequency(SOAPY_SDR_RX, chan, 'RF', rx_freq - .75*sample_rate)
                                self.sdr.setFrequency(SOAPY_SDR_RX, chan, 'BB', .75*sample_rate)

                        self.sdr.setAntenna(SOAPY_SDR_RX, chan, "TRX")

                        if self.agc_en:
                                self.sdr.setGain(SOAPY_SDR_RX, chan, 100)  # high gain value
                        else:
                                self.sdr.setGain(SOAPY_SDR_RX, chan, rx_gain)

                        self.sdr.setDCOffsetMode(SOAPY_SDR_RX, chan, True)

                self.tx_stream = None  # Burst mode

                self.sdr.writeSetting("RESET_DATA_LOGIC", "")

                if not self.both_channels:
                        self.sdr.writeSetting(SOAPY_SDR_RX, 1, 'ENABLE_CHANNEL', 'false')
                        self.sdr.writeSetting(SOAPY_SDR_TX, 1, 'ENABLE_CHANNEL', 'false')

        # Set trigger:
        def set_trigger(self):
                self.sdr.writeSetting("TRIGGER_GEN", "")

        def set_corr(self):
                '''enable the correlator, with inputs from adc'''
                self.sdr.writeSetting("CORR_START", "A")

        def unset_corr(self):
                corr_conf = {"corr_enabled": False}
                self.sdr.writeRegister("IRIS30", 60, 
                    self.sdr.readRegister("IRIS30", 60) & 0xFFFE)

        def config_sdr_tdd(self, is_bs=True, tdd_sched="G", prefix_len=0, max_frames=1):
                '''Configure the TDD schedule and functionality when unchained. Set up the correlator.'''
                global corr_threshold, beacon
                coe = cfloat2uint32(np.conj(beacon), order='QI')
                self.tdd_sched = tdd_sched
                self.max_frames = int(max_frames)
                if bool(is_bs):
                        conf_str = {"tdd_enabled": True,
                            "frame_mode": "free_running",
                            "symbol_size": self.n_samp,
                            "max_frame": max_frames,
                            "beacon_start": prefix_len,
                            "beacon_stop": prefix_len + len(beacon),
                            "frames": [self.tdd_sched]}
                        self.sdr.writeSetting("TDD_CONFIG", json.dumps(conf_str))
                        print("TDD schedule of BS node {}: {}".format(self.serial_id, tdd_sched))
                else:
                        conf_str = {"tdd_enabled": True,
                            "frame_mode": "triggered",
                            "symbol_size": self.n_samp,
                            "frames": [self.tdd_sched]}
                        self.sdr.writeSetting("TDD_CONFIG", json.dumps(conf_str))

                        #Correlator stuff:
                        corr_conf = {"corr_enabled": True,
                                     "corr_threshold": corr_threshold}
                        self.sdr.writeSetting(
                            "CORR_CONFIG", json.dumps(corr_conf))
                        if coe is not None:
                                self.sdr.writeRegisters(
                                    "CORR_COE", 0, coe.tolist())
                        else:
                                print("No coe was passed into config_sdr_tdd() \n")

                        # DEV: ueTrigTime = 153 (prefix_length=0), CBRS: ueTrigTime = 235 (prefix_length=82), tx_advance=prefix_length, corr delay is 17 cycles
                        ueTrigTime = len(beacon) + 200
                        sf_start = int(ueTrigTime//(self.n_samp))
                        sp_start = int(ueTrigTime % (self.n_samp))
                        print("config_sdr_tdd: UE starting symbol and sample count (%d, %d)" %
                              (sf_start, sp_start))
                        # make sure to set this after TDD mode is enabled "writeSetting("TDD_CONFIG", ..."
                        self.sdr.setHardwareTime(SoapySDR.ticksToTimeNs(
                            (sf_start << 16) | sp_start, self.sample_rate), "TRIGGER")
                        print("TDD schedule of UE node {}: {}".format(self.serial_id, tdd_sched))

                self.sdr.writeSetting("TX_SW_DELAY", str(30))
                self.sdr.writeSetting("TDD_MODE", "true")

        def config_gain_ctrl(self):
                tpc_conf = {"tpc_enabled": False}
                self.sdr.writeSetting("TPC_CONFIG", json.dumps(tpc_conf))
                agc_conf = {"agc_enabled": self.agc_en}
                self.sdr.writeSetting("AGC_CONFIG", json.dumps(agc_conf))

        def sync_delays(self):
                # NB: This should be called for the BS
                self.sdr.writeSetting("SYNC_DELAYS", "")

        # Setup and activate RX streams
        def setup_stream_rx(self):
                print("Setting up RX stream \n")
                self.rx_stream = self.sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0, 1])

        def activate_stream_rx(self, flags=0):
                r1 = self.sdr.activateStream(self.rx_stream, flags, 0)
                if r1 < 0:
                    print("Problem activating stream\n")

        def burn_beacon(self):
                '''Write beacon to the FPGA ram'''
                buf_a = cfloat2uint32(beacon, order='QI')
                beacon_weights = [[1, 1], [1, 1]]
                self.sdr.writeRegisters("BEACON_RAM", 0, buf_a.tolist())
                self.sdr.writeRegisters(
                    "BEACON_RAM_WGT_A", 0, beacon_weights[0])
                self.sdr.writeRegisters(
                    "BEACON_RAM_WGT_B", 0, beacon_weights[1])
                self.sdr.writeSetting("BEACON_START", str(2))

        # Write data to FPGA RAM
        def burn_data(self, data_r, data_i=None, replay_addr=0):
                '''Write data to FPGA RAM. A pilot for example. Need to compose a complex vector out of data_r and data_i'''

                if data_i is not None:
                    # hack for matlab...
                    data = np.asarray(data_r, dtype=np.complex64) + 1.j * \
                        np.asarray(data_i, dtype=np.complex64)

                buf_a = cfloat2uint32(data, order='QI')
                print("burnt data on to node {} FPGA RAM".format(self.serial_id))

                self.sdr.writeRegisters("TX_RAM_A", replay_addr, buf_a.tolist())
                #self.sdr.writeRegisters("TX_RAM_B", replay_addr, buf_b.tolist() )

        def recv_stream_tdd(self):
                '''Read an incoming stream.'''
                max_frames = int(self.max_frames)
                in_len = int(self.n_samp)
                wave_rx_a = np.zeros((in_len), dtype=np.complex64)
                wave_rx_b = np.zeros((in_len), dtype=np.complex64)
                rx_frames_a = np.zeros((in_len*max_frames), dtype=np.complex64)

                n_R = self.tdd_sched.count("R")  # How many Read frames in the tdd schedule
                print("n_samp is: %d  \n" % self.n_samp)

                for m in range(max_frames):
                    for k in range(n_R):
                        r1 = self.sdr.readStream(
                            self.rx_stream, [wave_rx_a, wave_rx_b], int(self.n_samp))
                        print("reading stream: ({})".format(r1))
                    rx_frames_a[m*in_len: (m*in_len + in_len)] = wave_rx_a

                return(rx_frames_a)

        def close(self):
                '''Cleanup streams. Rest SDRs'''
                print("Cleaning up streams")
                if self.sdr is not None:
                        if self.tx_stream is not None:
                                self.sdr.deactivateStream(self.tx_stream)
                                self.sdr.closeStream(self.tx_stream)
                                self.tx_stream = None

                        if self.rx_stream is not None:
                                self.sdr.deactivateStream(self.rx_stream)
                                self.sdr.closeStream(self.rx_stream)
                                self.rx_stream = None

                        self.sdr.writeSetting("RESET_DATA_LOGIC", "")
                        tdd_conf = {"tdd_enabled": False}
                        self.sdr.writeSetting(
                            "TDD_CONFIG", json.dumps(tdd_conf))
                        agc_conf = {"agc_enabled": False}
                        self.sdr.writeSetting(
                            "AGC_CONFIG", json.dumps(agc_conf))
                        self.sdr = None  # still doesn't release handle... you have to kill python.

                print("Done!")



#Used only in case of testing without Matlab:
if __name__ == '__main__':

        parser = ArgumentParser()
        parser.add_argument("--serial-id1", type=str, dest="serial_id1",
                            help="BS SDR Serial Number, e.g., RF3E000XXX", default=None)
        parser.add_argument("--serial-id2", type=str, dest="serial_id2",
                            help="UE SDR Serial Number, e.g., RF3E000XXX", default=None)
        parser.add_argument("--hub-serial", type=str, dest="hub_serial",
                            help="Hub Serial Number, e.g., FH4XXXXXXX", default=None)
        parser.add_argument("--rate", type=float, dest="rate",
                            help="Sample rate", default=5e6)
        parser.add_argument("--txGain", type=float, dest="txGain",
                            help="Optional Tx gain (dB)", default=40)
        parser.add_argument("--rxGain", type=float, dest="rxGain",
                            help="Optional Rx gain (dB)", default=20)
        parser.add_argument("--freq", type=float, dest="freq",
                            help="Optional Tx freq (Hz)", default=3.6e9)
        parser.add_argument("--bw", type=float, dest="bw",
                            help="Optional filter bw (Hz)", default=None)
        args = parser.parse_args()

        siso_bs = Iris_py(
                serial_id=args.serial_id1,
                sample_rate=args.rate,
                tx_freq=args.freq,
                rx_freq=args.freq,
                bw=args.bw,
                tx_gain=args.txGain,
                rx_gain=args.rxGain,
                n_samp=1024,
        )

        siso_ue = Iris_py(
                serial_id=args.serial_id2,
                sample_rate=args.rate,
                tx_freq=args.freq,
                rx_freq=args.freq,
                bw=args.bw,
                tx_gain=args.txGain,
                rx_gain=args.rxGain,
                n_samp=1024,
        )

        hub = None
        if args.hub_serial is not None:
            hub = Hub_py(args.hub_serial)

        print(siso_ue.sample_rate)
        #Generate signal to send
        nsamps = siso_bs.n_samp
        upsample = 1
        ampl = 1
        nsamps_pad = 82
        n_sym_samp = nsamps + 2*nsamps_pad - 14

        ltsSym, lts_f = gen_lts(cp=32, upsample=1)

        # to comprensate for front-end group delay
        pad1 = np.zeros((nsamps_pad), np.complex64)
        # to comprensate for rf path delay
        pad2 = np.zeros((nsamps_pad-14), np.complex64)

        wb_pilot = np.tile(ltsSym, nsamps//len(ltsSym)).astype(np.complex64)*.5
        wbz = np.zeros((n_sym_samp), dtype=np.complex64)
        wb_pilot1 = np.concatenate([pad1, wb_pilot, pad2])
        wb_pilot2 = wbz  # wb_pilot1 if both_channels else wbz

        print("len_beacon: ")
        print(len(beacon))

        wb_pilot1_r = np.real(wb_pilot1)
        wb_pilot1_i = np.imag(wb_pilot1)

        siso_ue.config_gain_ctrl()

        siso_ue.setup_stream_rx()
        siso_bs.setup_stream_rx()

        siso_bs.config_sdr_tdd(tdd_sched="BGGGGGRG", prefix_len=nsamps_pad)
        siso_ue.config_sdr_tdd(tdd_sched="GGGGGGPG", is_bs=False)

        siso_bs.burn_beacon()
        siso_ue.burn_data(wb_pilot1_r, wb_pilot1_i)

        siso_bs.activate_stream_rx()

        siso_ue.set_corr()
        if hub is not None:
            hub.set_trigger()
        else:
            siso_bs.set_trigger()

        wave_rx_a_bs_mn = siso_bs.recv_stream_tdd()

        freq = args.freq
        print("printing number of frames")
        print("BS 0x%X" % SoapySDR.timeNsToTicks(
            siso_bs.sdr.getHardwareTime(""), freq))
        print("UE 0x%X" % SoapySDR.timeNsToTicks(
            siso_ue.sdr.getHardwareTime(""), freq))

        siso_ue.close()
        siso_bs.close()

        #Test:
        plt.close('all')
        fig = plt.figure(figsize=(20, 8), dpi=100)
        ax1 = fig.add_subplot(2, 1, 1)
        ax2 = fig.add_subplot(2, 1, 2)
        ax1.plot(np.real(wb_pilot), label='pilot i')
        ax1.plot(np.imag(wb_pilot), label='pilot q')
        ax2.plot(np.real(wave_rx_a_bs_mn), label='rx data i')
        ax2.plot(np.imag(wave_rx_a_bs_mn), label='rx data q')
        plt.show()
