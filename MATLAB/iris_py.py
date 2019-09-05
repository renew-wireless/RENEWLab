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
#These are needed in case one needs to run the driver on its own.
# Make sure it is ran from the Python toolbox folder. 
sys.path.append('../PYTHON/IrisUtils/')
sys.path.append('../')
sys.path.append('../IrisUtils/data_in/')

from argparse import ArgumentParser
import SoapySDR
from SoapySDR import * #SOAPY_SDR_ constants
from optparse import OptionParser
import numpy as np
import time
import os
import math
import json
import threading
from generate_sequence import *
import matplotlib.pyplot as plt
from type_conv import *


# CORRTHRESHOLDING REGS
CORR_THRESHOLD = 92
CORR_RST = 64
CORR_CONF = 60

# TDD Register Set
RF_RST_REG = 48
TDD_CONF_REG = 120
SCH_ADDR_REG = 136
SCH_MODE_REG = 140
TX_GAIN_CTRL = 88


# Packet Detect Register Set 
FPGA_IRIS030_WR_PKT_DET_THRESH = 288
FPGA_IRIS030_WR_PKT_DET_NUM_SAMPS = 292
FPGA_IRIS030_WR_PKT_DET_ENABLE = 296
FPGA_IRIS030_WR_PKT_DET_NEW_FRAME = 300

# Global variables
tx_advance = 68
corr_threshold = 1

preambles_bs = generate_training_seq(preamble_type='gold_ifft', seq_length=128, cp=0, upsample=1)
beacon = preambles_bs[0, :]*.5

#######################################				
#######	SDR Class:		#######
#######################################

class Iris_py:
	'''
		Iris python class. To act as a middle layer between Matlab classes and SoapySDRs
	'''
	def __init__(self,
		serial_id = None,
		tx_freq = None,
		rx_freq = None,
		tx_gain = None,
		rx_gain = None,
		bw = None,
		sample_rate = None,
		n_samp = None,				# Total number of samples, including zero-pads
		n_zpad_samp = 150,			# Total number of samples used for zero-padding in prefix and postfix
                max_frames = 10,                        # Number of frames TXed: How many times the schedule will be repeated
		both_channels = False,
		agc_en = False,
	):

		if serial_id is not None: 
			self.sdr = SoapySDR.Device(dict(driver="iris", serial = serial_id))
		else:
			self.sdr = None
		
		self.sample_rate = sample_rate
		
		if n_samp is not None: self.n_samp = int(n_samp)
		
		self.agc_en = agc_en
		self.both_channels = both_channels
		self.n_zpad_samp = int(n_zpad_samp)
                self.max_frames = int(max_frames)

		# PACKET DETECT SETUP
		self.sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_THRESH, 0)
		self.sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NUM_SAMPS, 5)
		self.sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_ENABLE, 0)
		self.sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NEW_FRAME, 0)

		### Setup channel rates, ports, gains, and filters ###
		info = self.sdr.getHardwareInfo()
		for chan in [0, 1]:
			
			#Tx:
			if sample_rate is not None:
				self.sdr.setSampleRate(SOAPY_SDR_TX, chan, sample_rate)
			if bw is not None:
				self.sdr.setBandwidth(SOAPY_SDR_TX, chan, bw)
			if tx_gain is not None:
				self.sdr.setGain(SOAPY_SDR_TX, chan, tx_gain) 
			if tx_freq is not None:
				self.sdr.setFrequency(SOAPY_SDR_TX, chan, 'RF', tx_freq - .75*sample_rate)
				self.sdr.setFrequency(SOAPY_SDR_TX, chan, 'BB', .75*sample_rate)
			
			#print("Set TX frequency to %f" % self.sdr.getFrequency(SOAPY_SDR_TX, chan))
			#self.sdr.setAntenna(SOAPY_SDR_TX, chan, "TRX")
			self.sdr.setGain(SOAPY_SDR_TX, chan, 'IAMP', 0) #[0,12]
			self.sdr.setGain(SOAPY_SDR_TX, chan, 'PAD', tx_gain) #[-52,0]
			
			#Rx:
			if sample_rate is not None:
				self.sdr.setSampleRate(SOAPY_SDR_RX, chan, sample_rate)
			if bw is not None:
				self.sdr.setBandwidth(SOAPY_SDR_RX, chan, bw)
			if rx_gain is not None:
				self.sdr.setGain(SOAPY_SDR_RX, chan, rx_gain)
			if rx_freq is not None:
				self.sdr.setFrequency(SOAPY_SDR_RX, chan, 'RF', rx_freq - .75*sample_rate)
				self.sdr.setFrequency(SOAPY_SDR_RX, chan, 'BB', .75*sample_rate)
			
			self.sdr.setAntenna(SOAPY_SDR_RX, chan, "TRX")

			if self.agc_en:
				self.sdr.setGain(SOAPY_SDR_RX, chan, 'LNA', 30)       # [0,30]
				self.sdr.setGain(SOAPY_SDR_RX, chan, 'TIA', 12)       # [0,12]
				self.sdr.setGain(SOAPY_SDR_RX, chan, 'PGA', 19)       # [-12,19]
			else:
				self.sdr.setGain(SOAPY_SDR_RX, chan, 'LNA', rx_gain) #[0,30]
				self.sdr.setGain(SOAPY_SDR_RX, chan, 'TIA', 0) #[0,12]
				self.sdr.setGain(SOAPY_SDR_RX, chan, 'PGA', 0) #[-12,19]

			self.sdr.setDCOffsetMode(SOAPY_SDR_RX, chan, True)
				
			if ("CBRS" in info["frontend"]):
				#Tx:
				self.sdr.setGain(SOAPY_SDR_TX, chan, 'ATTN', 0) #[-18,0] by 3
				self.sdr.setGain(SOAPY_SDR_TX, chan, 'PA1', 15) #[0|15]
				self.sdr.setGain(SOAPY_SDR_TX, chan, 'PA2', 0) #[0|15]
				self.sdr.setGain(SOAPY_SDR_TX, chan, 'PA3', 30) #[0|30]
            	#Rx:
				self.sdr.setGain(SOAPY_SDR_RX, chan, 'ATTN', 0) #[-18,0]
				self.sdr.setGain(SOAPY_SDR_RX, chan, 'LNA1', 30) #[0,33]
				self.sdr.setGain(SOAPY_SDR_RX, chan, 'LNA2', 17) #[0,17]

		self.tx_stream = None # Burst mode

		self.sdr.writeRegister("IRIS30", RF_RST_REG, (1<<29) | 0x1)
		self.sdr.writeRegister("IRIS30", RF_RST_REG, (1<<29))
		self.sdr.writeRegister("IRIS30", RF_RST_REG, 0)

		if not self.both_channels:
			self.sdr.writeSetting(SOAPY_SDR_RX, 1, 'ENABLE_CHANNEL', 'false')
			self.sdr.writeSetting(SOAPY_SDR_TX, 1, 'ENABLE_CHANNEL', 'false')


    # Set trigger:
	def set_trigger(self, trig = False):
		if bool(trig) is True:
			self.sdr.writeSetting("TRIGGER_GEN", "")
        
	def set_corr(self):
		'''enable the correlator, with inputs from adc'''
		self.sdr.writeRegister("IRIS30", CORR_CONF, int("00004011", 16)) 


    # Write SDR configuration (chained mode):
	def config_sdr_tdd_chained(self, tdd_sched = None):
		'''Configure the TDD schedule and functionality when chained.'''
		if tdd_sched is not None:
			self.tdd_sched = tdd_sched
		else: self.tdd_sched = "G"
		print(tdd_sched)
		conf_str = {"tdd_enabled": True, "frame_mode": "free_running", "symbol_size" : self.n_samp, "max_frame": 1, "frames": [self.tdd_sched]}
		self.sdr.writeSetting("TDD_CONFIG", json.dumps(conf_str))
		self.sdr.writeSetting("TDD_MODE", "true")
		self.sdr.writeSetting("TX_SW_DELAY", str(30))

	def config_sdr_tdd(self, tdd_sched = None, is_bs = True):
		'''Configure the TDD schedule and functionality when unchained. Set up the correlator.'''
		global tx_advance, corr_threshold, beacon
		len_beacon_zpad = len(beacon) + self.n_zpad_samp
		coe = cfloat2uint32(np.conj(beacon), order='QI') 
		if tdd_sched is not None:
			self.tdd_sched = tdd_sched
		else: self.tdd_sched = "G"
		print(tdd_sched)
	        max_frames = self.max_frames
		if bool(is_bs):
			conf_str = {"tdd_enabled": True, "frame_mode": "free_running", "symbol_size" : self.n_samp, "max_frame": max_frames, "frames": [self.tdd_sched]}
			self.sdr.writeSetting("TDD_CONFIG", json.dumps(conf_str))
		else:
			conf_str = {"tdd_enabled": True, "frame_mode": "triggered", "symbol_size" : self.n_samp, "frames": [self.tdd_sched]}
			self.sdr.writeSetting("TDD_CONFIG", json.dumps(conf_str))
			
			#Correlator stuff:
			self.sdr.writeRegister("IRIS30", CORR_CONF, int("00004001", 16)) # enable the correlator, with zeros as inputs 
			for i in range(128):
				self.sdr.writeRegister("ARGCOE", i*4, 0)
			time.sleep(0.1)
			self.sdr.writeRegister("IRIS30", CORR_RST, 0x1) # reset corr
			self.sdr.writeRegister("IRIS30", CORR_RST, 0x0) # unrst corr
			self.sdr.writeRegister("IRIS30", CORR_THRESHOLD, int(corr_threshold)) 
			if coe is not None:
				for i in range(128):
					self.sdr.writeRegister( "ARGCOE", i*4, int(coe[i]))
			else:
				print("No coe was passed into config_sdr_tdd() \n")

			# DEV: ueTrigTime = 153 (prefix_length=0), CBRS: ueTrigTime = 235 (prefix_length=82), tx_advance=prefix_length, corr delay is 17 cycles
			ueTrigTime =  17 + tx_advance +len_beacon_zpad 
			sf_start = int(ueTrigTime//(self.n_samp))	
			sp_start = int(ueTrigTime%(self.n_samp))
			print("config_sdr_tdd: UE starting symbol and sample count (%d, %d)" % (sf_start, sp_start))
			self.sdr.setHardwareTime(SoapySDR.ticksToTimeNs((sf_start<<16) | sp_start, self.sample_rate),"TRIGGER") # make sure to set this after TDD mode is enabled "writeSetting("TDD_CONFIG", ..."
		
		self.sdr.writeSetting("TX_SW_DELAY", str(30))
		self.sdr.writeSetting("TDD_MODE", "true")


	
	def tx_gain_ctrl(self):
		self.sdr.writeRegister("IRIS30", TX_GAIN_CTRL, 0) #NB: what does this do? needs to be called on the UE 


	def sync_delays(self, is_bs = True):
		'''Synchronise delays. If is_bs = True, the BS sets the trigger off, else the UE resets the correlator.'''
		if bool(is_bs):
			self.sdr.writeSetting("SYNC_DELAYS", "")		#NB: This should be called for the BS
		#else:
			#self.sdr.writeRegister("ARGCOR", CORR_RST, 0x1) #NB: In UL, this should be called for the UE


    # Setup and activate RX streams
	def setup_stream_rx(self):
		print("Setting up RX stream \n")
		self.rx_stream = self.sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0, 1])


	def activate_stream_rx(self, flags = 0):
		r1 = self.sdr.activateStream(self.rx_stream, flags, 0)
		if r1<0: print("Problem activating stream\n")
	

	def config_beacon(self, prefix_len = 0):
		'''Zero-pad beacon to aling with the frame length'''
		global beacon
		len_beacon = len(beacon)
		n_bz = self.n_samp - prefix_len - len_beacon
		pad1 = np.zeros( (prefix_len), np.complex64)
		bcnz = np.zeros( (n_bz), np.complex64)    
		beacon1 = np.concatenate([pad1,beacon,bcnz])
		return beacon1
	
	def burn_beacon(self, prefix_len = 82):
		'''Write beacon to the FPGA ram'''
		py_prefix_len = int(prefix_len)
		beacon1 = self.config_beacon(py_prefix_len)
		beacon1_r = np.real(beacon1)
		beacon1_i = np.imag(beacon1)
		self.burn_data(beacon1_r, beacon1_i)


    # Write data to FPGA RAM
	def burn_data(self, data_r, data_i = None, replay_addr = 0):
		'''Write data to FPGA RAM. A pilot for example. Need to compose a complex vector out of data_r and data_i'''
    	
		if data_i is not None: data = np.asarray(data_r, dtype=np.complex64) + 1.j*np.asarray(data_i, dtype=np.complex64)  #hack for matlab...

		buf_a = cfloat2uint32(data, order='QI')

		#buf_b = buf_a - buf_a

		self.sdr.writeRegisters("TX_RAM_A", replay_addr, buf_a.tolist() )
		#self.sdr.writeRegisters("TX_RAM_B", replay_addr, buf_b.tolist() )

	def recv_stream_tdd(self):
		'''Read an incoming stream.'''
                max_frames = int(self.max_frames)
		in_len  =  int(self.n_samp)
		wave_rx_a = np.zeros((in_len), dtype=np.complex64)
		wave_rx_b = np.zeros((in_len), dtype=np.complex64)
                rx_frames_a = np.zeros((in_len*max_frames), dtype=np.complex64)

		n_R = self.tdd_sched.count("R")         #How many Read frames in the tdd schedule
		print("n_samp is: %d  \n"%self.n_samp)

                for m in range(max_frames):
                    for k in range(n_R):
		        r1 = self.sdr.readStream(self.rx_stream, [wave_rx_a, wave_rx_b], int(self.n_samp))
		        print("reading stream: ({})".format(r1))
                    rx_frames_a[m*in_len : (m*in_len + in_len)] = wave_rx_a

		print("SDR {} ".format(SoapySDR.timeNsToTicks(self.sdr.getHardwareTime(""), self.sample_rate)))
		#print("recv_stream_tdd: wave_rx_a: \n")
		return( rx_frames_a )

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
			
			self.sdr.writeRegister("IRIS30", RF_RST_REG, (1<<29)| 0x1)
			self.sdr.writeRegister("IRIS30", RF_RST_REG, (1<<29))
			self.sdr.writeRegister("IRIS30", RF_RST_REG, 0)
			for i in range(8):
				self.sdr.writeRegister("RFCORE", SCH_ADDR_REG, i) # subframe 0
				self.sdr.writeRegister("RFCORE", SCH_MODE_REG, 0) # 01 replay
				self.sdr.writeRegister("RFCORE", TDD_CONF_REG, 0)
			self.sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_THRESH, 0)
			self.sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NUM_SAMPS, 5)
			self.sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_ENABLE, 0)
			self.sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NEW_FRAME, 0)
			self.sdr = None #still doesn't release handle... you have to kill python.
				

		print("Done!")
    


#Used only in case of testing without Matlab:
if __name__ == '__main__':

	parser = ArgumentParser()
	parser.add_argument("--serial_id", type=str, dest="serial_id", help="TX SDR Serial Number, e.g., 00001", default=None)
	parser.add_argument("--rate", type=float, dest="rate", help="Sample rate", default= 5e6)
	parser.add_argument("--txGain", type=float, dest="txGain", help="Optional Tx gain (dB)", default=40)
	parser.add_argument("--rxGain", type=float, dest="rxGain", help="Optional Rx gain (dB)", default=20)
	parser.add_argument("--freq", type=float, dest="freq", help="Optional Tx freq (Hz)", default=3.6e9)
	parser.add_argument("--bw", type=float, dest="bw", help="Optional filter bw (Hz)", default=None)
	args = parser.parse_args()
	
	siso_bs = Iris_py(
		serial_id = "0339",
		sample_rate = args.rate,
		tx_freq = args.freq,
		rx_freq = args.freq,
		bw = args.bw,
		tx_gain = args.txGain,
		rx_gain = args.rxGain,
		n_samp = 1024,
	)

	siso_ue = Iris_py(
		serial_id = "RF3C000045",
		sample_rate = args.rate,
		tx_freq = args.freq,
		rx_freq = args.freq,
		bw = args.bw,
		tx_gain = args.txGain,
		rx_gain = args.rxGain,
		n_samp = 1024,
	)
	
	print(siso_ue.sample_rate)
	#Generate signal to send
	nsamps = siso_bs.n_samp
	upsample = 1
	ampl = 1
	nsamps_pad = 82
	n_sym_samp = nsamps + 2*nsamps_pad - 14

	ltsSym, lts_f = generate_training_seq(preamble_type='lts', cp=32, upsample=1)
	
	pad1 = np.zeros((nsamps_pad), np.complex64)			# to comprensate for front-end group delay
	pad2 = np.zeros((nsamps_pad-14), np.complex64)		# to comprensate for rf path delay

	wb_pilot = np.tile(ltsSym,nsamps//len(ltsSym)).astype(np.complex64)*.5
	wbz = np.zeros((n_sym_samp), dtype=np.complex64)
	wb_pilot1 = np.concatenate([pad1,wb_pilot,pad2])  
	wb_pilot2 = wbz #wb_pilot1 if both_channels else wbz

	#bcnz = np.zeros((n_sym_samp-nsamps_pad-len(beacon)), dtype=np.complex64)  
	#beacon1 = np.concatenate([pad1,beacon,bcnz])

	#beacon1 = siso_bs.config_beacon(nsamps_pad)
	#if mimo: beacon2 = wbz #beacon1 if both_channels else wbz
	beacon1 = siso_bs.config_beacon(nsamps_pad)
	beacon1_r = np.real(beacon1)
	beacon1_i = np.imag(beacon1)
	
	
	print("len_beacon: ")
	print(len(beacon))

	wb_pilot1_r = np.real(wb_pilot1)
	wb_pilot1_i = np.imag(wb_pilot1)
	
	siso_ue.tx_gain_ctrl()
	siso_ue.sync_delays(False)
	
	siso_ue.setup_stream_rx()
	siso_bs.setup_stream_rx()

	siso_bs.config_sdr_tdd(tdd_sched = "PGGGGGRG")
	siso_ue.config_sdr_tdd(tdd_sched = "GGGGGGPG", is_bs = False)

	siso_bs.burn_data(beacon1_r, beacon1_i)
	#siso_bs.burn_beacon()
	siso_ue.burn_data(wb_pilot1_r, wb_pilot1_i)

	siso_bs.activate_stream_rx()

	siso_ue.set_corr()
	siso_bs.set_trigger(True)

	wave_rx_a_bs_mn =  siso_bs.recv_stream_tdd()

	freq = 3.6e9
	print("printing number of frames")
	print("BS 0x%X" % SoapySDR.timeNsToTicks(siso_bs.sdr.getHardwareTime(""), freq))
	print("UE 0x%X" % SoapySDR.timeNsToTicks(siso_ue.sdr.getHardwareTime(""), freq))

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


	


