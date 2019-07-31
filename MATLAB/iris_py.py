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

#Beacon:
beacon = np.array([-0.56463587-0.56463587j ,0.46699509+0.34623110j ,0.87693578-0.35418823j ,0.54079849+0.34757638j ,-0.48144832+0.00891686j ,-0.88476783-0.00866229j,
		0.33639774+0.37122476j ,-0.43609348-0.42793053j ,-0.26278743+0.41920760j ,0.69103312+0.17364220j ,-0.25535262+0.24096492j ,0.11774132+0.22344504j,
		0.46892625-0.37271422j ,0.77644444+0.42836890j ,-0.14834122+0.40922493j ,-0.13464923+0.03079897j ,-0.26617187+0.53234375j ,0.13412920+0.73846632j,
		0.13357399+0.19951832j ,0.15594807-0.13213991j ,-0.05784707+0.18199626j ,0.39676210+0.38991654j ,0.04760613+0.22615194j ,0.01414329-0.40837204j,
		0.41560003+0.33933204j ,0.12632199-0.68115342j ,-0.33603117-0.12019706j ,-0.56691819-0.40438867j ,-0.20043480-0.18971694j ,0.55602646+0.00865902j,
		0.24340886+0.11038142j ,-0.16611233-0.90827960j ,0.79049021-0.18821196j ,-0.42025912+0.31633607j ,-0.38651145+0.92221844j ,-0.14808364+0.82918876j,
		-0.27662534-0.25830117j ,-0.74715126+0.01612155j ,0.59089267-0.14366682j ,-0.75451213+0.18401834j ,-0.33933204-0.23919682j ,0.36646086+0.42271641j,
		-0.57852495-0.06262089j ,0.10015667+0.35279569j ,-0.34719938+0.64131898j ,0.35134000-0.82117832j ,0.73830807-0.38437498j ,-0.37431011-0.33556038j,
		-0.53234375+0.26617187j ,-0.33714586-0.00221796j ,0.01215767-0.44250035j ,-0.39932099-0.15740044j ,-0.38716090-0.72959810j ,0.27705255+0.26336509j,
		0.44698530+0.25250778j ,-0.16857521-0.08888090j ,0.60894567+0.26278743j ,-0.04652265+0.64291787j ,0.21421923+0.32680589j ,0.01422996+0.19918670j,
		0.87569416-0.08036269j ,-0.28046992-0.09114221j ,0.64841086+0.68264091j ,0.06317055-0.73030668j ,-0.03764239-0.03764239j ,-0.73030668+0.06317055j,
		0.68264091+0.64841086j ,-0.09114221-0.28046992j ,-0.08036269+0.87569416j ,0.19918670+0.01422996j ,0.32680589+0.21421923j ,0.64291787-0.04652265j,
		0.26278743+0.60894567j ,-0.08888090-0.16857521j ,0.25250778+0.44698530j ,0.26336509+0.27705255j ,-0.72959810-0.38716090j ,-0.15740044-0.39932099j,
		-0.44250035+0.01215767j ,-0.00221796-0.33714586j ,0.26617187-0.53234375j ,-0.33556038-0.37431011j ,-0.38437498+0.73830807j ,-0.82117832+0.35134000j,
		0.64131898-0.34719938j ,0.35279569+0.10015667j ,-0.06262089-0.57852495j ,0.42271641+0.36646086j ,-0.23919682-0.33933204j ,0.18401834-0.75451213j,
		-0.14366682+0.59089267j ,0.01612155-0.74715126j ,-0.25830117-0.27662534j ,0.82918876-0.14808364j ,0.92221844-0.38651145j ,0.31633607-0.42025912j,
		-0.18821196+0.79049021j ,-0.90827960-0.16611233j ,0.11038142+0.24340886j ,0.00865902+0.55602646j ,-0.18971694-0.20043480j ,-0.40438867-0.56691819j,
		-0.12019706-0.33603117j ,-0.68115342+0.12632199j ,0.33933204+0.41560003j ,-0.40837204+0.01414329j ,0.22615194+0.04760613j ,0.38991654+0.39676210j,
		0.18199626-0.05784707j ,-0.13213991+0.15594807j ,0.19951832+0.13357399j ,0.73846632+0.13412920j ,0.53234375-0.26617187j ,0.03079897-0.13464923j,
		0.40922493-0.14834122j ,0.42836890+0.77644444j ,-0.37271422+0.46892625j ,0.22344504+0.11774132j ,0.24096492-0.25535262j ,0.17364220+0.69103312j,
		0.41920760-0.26278743j ,-0.42793053-0.43609348j ,0.37122476+0.33639774j ,-0.00866229-0.88476783j ,0.00891686-0.48144832j ,0.34757638+0.54079849j,
		-0.35418823+0.87693578j ,0.34623110+0.46699509j ,-0.56463587-0.56463587j ,0.46699509+0.34623110j ,0.87693578-0.35418823j ,0.54079849+0.34757638j,
		-0.48144832+0.00891686j ,-0.88476783-0.00866229j ,0.33639774+0.37122476j ,-0.43609348-0.42793053j ,-0.26278743+0.41920760j ,0.69103312+0.17364220j,
		-0.25535262+0.24096492j ,0.11774132+0.22344504j ,0.46892625-0.37271422j ,0.77644444+0.42836890j ,-0.14834122+0.40922493j ,-0.13464923+0.03079897j,
		-0.26617187+0.53234375j ,0.13412920+0.73846632j ,0.13357399+0.19951832j ,0.15594807-0.13213991j ,-0.05784707+0.18199626j ,0.39676210+0.38991654j,
		0.04760613+0.22615194j ,0.01414329-0.40837204j ,0.41560003+0.33933204j ,0.12632199-0.68115342j ,-0.33603117-0.12019706j ,-0.56691819-0.40438867j,
		-0.20043480-0.18971694j ,0.55602646+0.00865902j ,0.24340886+0.11038142j ,-0.16611233-0.90827960j ,0.79049021-0.18821196j ,-0.42025912+0.31633607j,
		-0.38651145+0.92221844j ,-0.14808364+0.82918876j ,-0.27662534-0.25830117j ,-0.74715126+0.01612155j ,0.59089267-0.14366682j ,-0.75451213+0.18401834j,
		-0.33933204-0.23919682j ,0.36646086+0.42271641j ,-0.57852495-0.06262089j ,0.10015667+0.35279569j ,-0.34719938+0.64131898j ,0.35134000-0.82117832j,
		0.73830807-0.38437498j ,-0.37431011-0.33556038j ,-0.53234375+0.26617187j ,-0.33714586-0.00221796j ,0.01215767-0.44250035j ,-0.39932099-0.15740044j,
		-0.38716090-0.72959810j ,0.27705255+0.26336509j ,0.44698530+0.25250778j ,-0.16857521-0.08888090j ,0.60894567+0.26278743j ,-0.04652265+0.64291787j,
		0.21421923+0.32680589j ,0.01422996+0.19918670j ,0.87569416-0.08036269j ,-0.28046992-0.09114221j ,0.64841086+0.68264091j ,0.06317055-0.73030668j,
		-0.03764239-0.03764239j ,-0.73030668+0.06317055j ,0.68264091+0.64841086j ,-0.09114221-0.28046992j ,-0.08036269+0.87569416j ,0.19918670+0.01422996j,
		0.32680589+0.21421923j ,0.64291787-0.04652265j ,0.26278743+0.60894567j ,-0.08888090-0.16857521j ,0.25250778+0.44698530j ,0.26336509+0.27705255j,
		-0.72959810-0.38716090j ,-0.15740044-0.39932099j ,-0.44250035+0.01215767j ,-0.00221796-0.33714586j ,0.26617187-0.53234375j ,-0.33556038-0.37431011j,
		-0.38437498+0.73830807j ,-0.82117832+0.35134000j ,0.64131898-0.34719938j ,0.35279569+0.10015667j ,-0.06262089-0.57852495j ,0.42271641+0.36646086j,
		-0.23919682-0.33933204j ,0.18401834-0.75451213j ,-0.14366682+0.59089267j ,0.01612155-0.74715126j ,-0.25830117-0.27662534j ,0.82918876-0.14808364j,
		0.92221844-0.38651145j ,0.31633607-0.42025912j ,-0.18821196+0.79049021j ,-0.90827960-0.16611233j ,0.11038142+0.24340886j ,0.00865902+0.55602646j,
		-0.18971694-0.20043480j ,-0.40438867-0.56691819j ,-0.12019706-0.33603117j ,-0.68115342+0.12632199j ,0.33933204+0.41560003j ,-0.40837204+0.01414329j,
		0.22615194+0.04760613j ,0.38991654+0.39676210j ,0.18199626-0.05784707j ,-0.13213991+0.15594807j ,0.19951832+0.13357399j ,0.73846632+0.13412920j,
		0.53234375-0.26617187j ,0.03079897-0.13464923j ,0.40922493-0.14834122j ,0.42836890+0.77644444j ,-0.37271422+0.46892625j ,0.22344504+0.11774132j,
		0.24096492-0.25535262j ,0.17364220+0.69103312j ,0.41920760-0.26278743j ,-0.42793053-0.43609348j , 0.37122476+0.33639774j ,-0.00866229-0.88476783j,
		0.00891686-0.48144832j ,0.34757638+0.54079849j ,-0.35418823+0.87693578j ,0.34623110+0.46699509j], dtype=np.complex64)


#######################################				
#######		Help functions:		#######
#######################################


def cfloat2uint32(arr, order='IQ'):
		arr_i = (np.real(arr) * 32767).astype(np.uint16)
		arr_q = (np.imag(arr) * 32767).astype(np.uint16)
		if order == 'IQ':
			return np.bitwise_or(arr_q ,np.left_shift(arr_i.astype(np.uint32), 16))
		else:
			return np.bitwise_or(arr_i ,np.left_shift(arr_q.astype(np.uint32), 16))
	
def uint32tocfloat(arr, order='IQ'):
	arr_hi = ((np.right_shift(arr, 16).astype(np.int16)) /32768.0)
	arr_lo = (np.bitwise_and(arr, 0xFFFF).astype(np.int16)) /32768.0
	if order == 'IQ':
		return (arr_hi + 1j*arr_lo).astype(np.complex64)
	else:
		return (arr_lo + 1j*arr_hi).astype(np.complex64)


#######################################				
#######			SDR Class:		#######
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
			
			print("Set TX frequency to %f" % self.sdr.getFrequency(SOAPY_SDR_TX, chan))
			#self.sdr.setAntenna(SOAPY_SDR_TX, chan, "TRX")
			self.sdr.setGain(SOAPY_SDR_TX, chan, 'IAMP', 12) #[0,12]
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
		conf_str = {"tdd_enabled": True, "trigger_out": True, "wait_trigger": True, "symbol_size" : self.n_samp, "max_frame": 1, "frames": [self.tdd_sched]}
		self.sdr.writeSetting("TDD_CONFIG", json.dumps(conf_str))
		self.sdr.writeSetting("TDD_MODE", "true")
		self.sdr.writeSetting("TX_SW_DELAY", str(30))

	def config_sdr_tdd(self, tdd_sched = None, is_bs = True, trigger_out = False):
		'''Configure the TDD schedule and functionality when unchained. Set up the correlator.'''
		global tx_advance, corr_threshold, beacon
		len_beacon_zpad = len(beacon) + self.n_zpad_samp
		coe = cfloat2uint32(np.conj(beacon), order='QI') 
		if tdd_sched is not None:
			self.tdd_sched = tdd_sched
		else: self.tdd_sched = "G"
		print(tdd_sched)
		
		if bool(is_bs):
			conf_str = {"tdd_enabled": True, "trigger_out": False, "symbol_size" : self.n_samp, "max_frame": 0, "frames": [self.tdd_sched]}
			self.sdr.writeSetting("TDD_CONFIG", json.dumps(conf_str))
			self.sdr.writeSetting("TX_SW_DELAY", str(30))
		else:
			conf_str = {"tdd_enabled": True, "trigger_out": not trigger_out, "wait_trigger": True, "symbol_size" : self.n_samp, "frames": [self.tdd_sched]}
			self.sdr.writeSetting("TDD_CONFIG", json.dumps(conf_str))
			self.sdr.writeSetting("TX_SW_DELAY", str(30))
			
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
		self.rx_stream = self.sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1],  dict(WIRE=SOAPY_SDR_CS16))


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

		buf_a = cfloat2uint32(data)
		#buf_b = buf_a - buf_a

		self.sdr.writeRegisters("TX_RAM_A", replay_addr, buf_a.tolist() )
		#self.sdr.writeRegisters("TX_RAM_B", replay_addr, buf_b.tolist() )

	def recv_stream_tdd(self):
		'''Read an incoming stream.'''
		in_len  =  int(self.n_samp)
		wave_rx_a = np.zeros((in_len), dtype=np.uint32)
		wave_rx_b = np.zeros((in_len), dtype=np.uint32)

		n_R = self.tdd_sched.count("R")         #How many Read frames in the tdd schedule
		print("n_samp is: %d  \n"%self.n_samp)
		for k in range(n_R):
			r1 = self.sdr.readStream(self.rx_stream, [wave_rx_a, wave_rx_b], int(self.n_samp))
			print("reading stream: ({})".format(r1))

		print("SDR {} ".format(SoapySDR.timeNsToTicks(self.sdr.getHardwareTime(""), self.sample_rate)))
		#print("recv_stream_tdd: wave_rx_a: \n")
		return( uint32tocfloat(wave_rx_a) )

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
			for i in range(4):
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
		serial_id = "0328",
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
	
	print("len_beacon: ")
	print(len(beacon))

	wb_pilot1_r = np.real(wb_pilot1)
	wb_pilot1_i = np.imag(wb_pilot1)
	
	siso_ue.tx_gain_ctrl()
	siso_ue.sync_delays(False)
	
	siso_ue.setup_stream_rx()
	siso_bs.setup_stream_rx()

	siso_bs.config_sdr_tdd(tdd_sched = "PGGGGGRG")
	siso_ue.config_sdr_tdd(tdd_sched = "GGGGGGPG", is_bs = False, trigger_out = False)

	#siso_bs.burn_data(beacon1_r, beacon1_i)
	siso_bs.burn_beacon()
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


	


