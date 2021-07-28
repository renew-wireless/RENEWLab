

#### Imports ####
import sys
sys.path.append('../IrisUtils/')
import os
import argparse
import SoapySDR
from SoapySDR import *  # SOAPY_SDR_contents
import time
from datetime import datetime
import collections
from digital_rssi import *
from find_lts import *
from fft_power import *
from bandpower import *
from type_conv import *
from init_fncs import *
from generate_sequence import *
import csv
import numpy as np

### Constants ###
TX_LO = 50
TX_HI = 81
TX_STEP = 1
RX_LO = 50
RX_HI = 81
RX_STEP = 1
#################

## User input? Hardcode boundaries? Don't want to destroy a board by accident haha
# TX_LO = input("Enter LOWER bound for TX gain: ")
# TX_HI = input("Enter UPPER bound for TX gain: ")
# TX_STEP = input("Enter step size for TX gain: ")
# RX_LO = input("Enter LOWER bound for RX gain: ")
# RX_HI = input("Enter UPPER bound for RX gain: ")
# RX_STEP = input("Enter step size for RX gain: ")

def command_line():
	parser = argparse.ArgumentParser()
	parser.add_argument("-f", "--freq",
		type = float,
		dest = "freq",
		default = 3.6e9,
		help = "Irises operate in Hz"
				"Default: %(default)s")

	parser.add_argument("--ref_serial",
		type = str,
		dest = "ref_serial",
		default = "",
		help = "The reference Iris's serial number. "
				"Default: %(default)s"
				"If no Iris specified, will choose first Iris "
				"as reference node.")
	
	parser.add_argument("--rm-from-other-serials",
		nargs = "+",
		type = str,
		default = [],
		help = "List of Iris serials to be removed from test list "
				"Default: %(default)s")
	
	parser.add_argument("--tx_lo",
		type = int,
		dest = "TX_LO",
		default = 50,
		help = "Default lower bound on transmitter gain. "
				"Default: %(default)s")
	
	parser.add_argument("--tx_hi",
		type = int,
		dest = "TX_HI",
		default = 81,
		help = "Upper bound on transmitter gain. "
				"Default: %(default)s")
	
	parser.add_argument("--tx_step",
		type = int,
		dest = "TX_STEP",
		default = 1,
		help = "Step size for each increment of TX gain. "
				"Default: %(default)s")
	
	parser.add_argument("--rx_low",
		type = int,
		dest = "RX_LOW",
		default = 81,
		help = "Lower bound on receiver gain "
				"Default: %(default)s")
	
	parser.add_argument("--rx_hi",
		type = int,
		dest = "RX_HI",
		default = 81,
		help = "Upper bound on receiver gain. "
				"Default: %(default)s")

	parser.add_argument("--rx_step",
		type = int,
		dest = "RX_STEP",
		default = 1,
		help = "Step size for each increment of RX gain. "
				"Default: %(default)s")

	parser.add_argument("--sdr-log-level",
		type = int,
		dest = "sdr_log_level",
		default = 3,
		help = "Logging depth of Iris Sdr error/info/warning. "
				"Default: %(default)s")


def start_irises(other_serials, ref_serial, rm_from_oth_serials):
    '''
    Start up for all irises.
	Args:
	- other_serials: The serials to receive TX
	- ref_serial: Main serial that transmits from
	- rm_from_oth_serials: specified serials to be excluded from (eventual) gain tester

    Returns:
    Sdrs of other irises(?), serials of other irises, frontends, sdr of ref iris, serial of ref iris
  
  	'''
    #Enumerate devices on network
    handles = SoapySDR.Device.enumerate({"remote:timeout": "250000"}) #Stuff after enumerate?
    if not other_serials:
        #Instantiate all devices on network
        sdrs = SoapySDR.Device(handles)
    else:
        other_serials.append(ref_serial)
        sdrs = [SoapySDR.Device(dict(driver="iris", serial=s)) for s in other_serials]

    #Remove instances that aren't an iris
    other_sdrs = list(sdrs)
    for sdr in sdrs:
        if "Iris" not in sdr.getHardwareInfo()["revision"]:
            other_sdrs.remove(sdr)

    #Find all Iris serials and frontend types
    other_serials = [sdr.getHardwareInfo()["serial"] for sdr in other_sdrs]
    other_frontends = []
    #Iterate thru and find CBRS among sdrs
    for sdr in other_sdrs:
        if "CBRS" in sdr.getHardwareInfo()["frontend"]:
            other_frontends.append("CBRS")
        else:
            other_frontends.append("")
    
    #Find ref iris and remove from other list
    if ref_serial == "":
        ref_serial = other_serials[0]
        ref_sdr = other_sdrs.pop(other_serials.index(ref_serial))

    #"Decide on other list" but why do this?
    for serial in rm_from_oth_serials:
        other_sdrs.pop(other_serials.index(serial))
        other_serials.pop(other_serials.index(serial))
        other_frontends.pop(other_serials.index(serial))

    #Make a tuple....
    other_sdrs = tuple(other_sdrs)

    return other_sdrs, other_serials, other_frontends, ref_sdr, ref_serial

#Generate pilots
#Not actually sure how many of these parameters I'll need in the end
def generate_pilots(rate, ampl, wave_freq, bb_freq, num_samps):
    '''
    Generate the pilots to be transmitted/received.
	Args:
	- rate:
	- ampl:
	- wave_freq:
	- bb_freq:
	- num_samps:

	Returns:
	- pilot1_ui32
	- pilot2-ui32
    '''

    #Generate TX signal
    tx_signal = np.empty(num_samps).astype(np.complex64)
    #Why reassigning variable?
    wbz = tx_signal
    
    #Generate Pilots
    #WiFi LTS Signal
    ltsSym, lts_f = generate_training_seq(preamble_type='lts', cp=32, upsample =1)
    tx_signal = np.tile(ltsSym, num_samps//len(ltsSym)).astype(np.complex64) * ampl

    #Float to fixed point
    pilot1_ui32 = cfloat2uint32(tx_signal, order = "QI")
    pilot2_ui32 = cfloat2uint32(wbz)

    return pilot1_ui32, pilot2_ui32

def send_tx_rx(sdr_tx, sdr_rx, tx_gain, rx_gain, freq, tx_ant,
                      samp_rate=5e6, fft_size=1024, rx_num_samps=1024,
                      wait_trigger=False, tx_amp=1, bb_freq=0, wave_freq=None,
                      tx_num_samps=2**10, lo_tone=False):
	'''
	Use SoapySDR to send and receive pilots. Most code is 
	from iris_health_monitor.py
	Args: 
	- sdr_tx - sdr for transmitting
	- sdr_rx - sdr for receiving
	- tx_gain
	- rx_gain
	- freq - the frequency of ...?
	- tx_ant - transmitter antenna to use
	Returns:
	- start: Start time for TX/RX
	- stop: End time for TX/RX
	- lts_pks0: List of lts signals received from first buffer (?)
	- lts_pks1: List of lts signals received from second buffer
	'''
	#Transmit
	info = sdr_tx.getHardwareInfo()

	#Set Iris RF parameters
	tx_channel = [0]
	if tx_ant == "B":
		tx_channel = [1]
	elif tx_ant == "AB":
		tx_channel = [0, 1]
	amp_fixed = int(tx_amp * (1 << 13)) #Why do this this way? (Bitwise op.)

	for ch in tx_channel:
		sdr_tx.writeSetting(SOAPY_SDR_TX, ch, "ENABLE_CHANNEL", "true")
		sdr_tx.setBandwidth(SOAPY_SDR_TX, ch, 2.5 * samp_rate)
		sdr_tx.setSampleRate(SOAPY_SDR_TX, ch, samp_rate)
		#Math for both of these setFrequency? Do I need to change
		#for using pilots instead of sine?
		sdr_tx.setFrequency(SOAPY_SDR_TX, ch, "RF", freq - 0.75 * samp_rate)
		sdr_tx.setFrequency(SOAPY_SDR_TX, ch, "BB", 0.75 * samp_rate)
		sdr_tx.setFrequency(SOAPY_SDR_TX, ch, "TRX")

		#Do I need this lo_tone stuff?
		if lo_tone:
			sdr_tx.writeSetting(SOAPY_SDR_TX, ch, "TSP_TSG_CONST", str(amp_fixed))
			sdr_tx.writeSetting(SOAPY_SDR_TX, ch, "TX_ENB_OVERRIDE", "true")

		#Do I need this?
		if "CBRS" in info["frontend"]:
			sdr_tx.setGain(SOAPY_SDR_TX, ch, tx_gain)
		else:
			#No CBRS board gains, only changing LMS7 gains
			sdr_tx.setGain(SOAPY_SDR_TX, ch, "PAD", tx_gain) #[0:1:42]
			sdr_tx.setGain(SOAPY_SDR_TX, ch, "IAMP", 0) #[-12:1:3]
		#I don't know what those brackets mean

		#Generate pilots
		pilot1_ui32, pilot2_ui32 = generate_pilots(samp_rate, tx_amp, wave_freq, bb_freq, tx_num_samps)

		#Setup to transmit
		if not lo_tone:
			replay_addr = 0
			if tx_ant == "A":
				sdr_tx.writeRegisters("TX_RAM_A", replay_addr, pilot1_ui32.tolist())
			elif tx_ant == "B":
				sdr_tx.writeRegisters("TX_RAM_B", replay_addr, pilot1_ui32.tolist())
			elif tx_ant == "AB":
				sdr_tx.writeRegisters("TX_RAM_A", replay_addr, pilot1_ui32.tolist())
				sdr_tx.writeRegisters("TX_RAM_B", replay_addr, pilot1_ui32.tolist())
			#Continuous TX
			sdr_tx.writeSetting("TX_REPLAY", str(tx_num_samps))
	
	#Receive
	start = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") #WHAT is this

	#Set parameters on both RF channels
	info = sdr_rx.getHardwareInfo() #Better to assign to dif variable that's not 'info'? See line 435
	for ch in [0, 1]:
		#Same question as before.. does any of this need tochange
		#for pilots instead of sine now?
		sdr_rx.setBandwidth(SOAPY_SDR_RX, ch, 2.5 * samp_rate)
		sdr_rx.setFrequency(SOAPY_SDR_RX, ch, freq)
		sdr_rx.setSampleRate(SOAPY_SDR_RX, ch, samp_rate)
		sdr_rx.setAntenna(SOAPY_SDR_RX, ch, "TRX")
		sdr_rx.setDCOffsetMode(SOAPY_SDR_RX, ch, True)

		#Not sure what this does
       	if "CBRS" in info["frontend"]:
           sdr_rx.setGain(SOAPY_SDR_RX, ch, rx_gain)
       	else:
           # No CBRS board gains, only changing LMS7 gains
           sdr_rx.setGain(SOAPY_SDR_RX, ch, "LNA", rx_gain)  # [0:1:30]
           sdr_rx.setGain(SOAPY_SDR_RX, ch, "TIA", 0)  # [0, 3, 9, 12]
           sdr_rx.setGain(SOAPY_SDR_RX, ch, "PGA", -10)  # [-12:1:19]
	sdr_rx.writeRegister("RFCORE", 120, 0)

	#Setup RX Stream
	rx_stream = sdr_rx.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0,1])

	#RSSi read setup
	setUpDigitalRssimode(sdr_rx)

	#Read samples into buffers 
	samps_rx = [np.zeros(rx_num_samps, np.complex64), 
			np.zeros(rx_num_samps, np.complex64)] 
	buffer0 = samps_rx[0] #RF Chain 1
	buffer1 = samps_rx[1] #RF Chain 2

	flags = SOAPY_SDR_END_BURST #What is this?
	if wait_trigger: 
		flags |= SOAPY_SDR_WAIT_TRIGGER #What is this
	sdr_rx.activateStream(rx_stream, flags, 0, buffer0.size)
	sr = sdr_rx.readStream(rx_stream, [buffer0, buffer1], buffer0.size)
	if sr.ret != buffer0.size:
		print("Read RX burst of %d, requested %d" % (sr.ret, buffer0.size))

	#Remove DC
	for i in [0, 1]:
		samps_rx[i] -= np.mean(samps_rx[i])

	lts_thresh = 0.8
	#Looked back at docstring for this function on github, think what I'm doing
	#With lts_pks is okay
	best_pk0, lts_pks0, lts_corr0 = find_lts(samps_rx[0], thresh=lts_thresh, flip=True)
	best_pk1, lts_pks1, lts_corr1 = find_lts(samps_rx[1], thresh=lts_thresh, flip=True)
	'''
	Inputs:
		iq: IQ samples
		thresh: threshold to detect peak
		us: upsampling factor, needed for generate_training_seq() function
		cp: cyclic prefix
		flip: Flag to specify order or LTS sequence.
		lts_seq: if transmitted lts sequence is provided, use it, otherwise generate it
	Returns:
		best_pk: highest LTS peak,
		lts_pks: the list of all detected LTSs, and
		lts_corr: the correlated signal, multiplied by itself delayed by 1/2 an LTS
	'''

	#Check if LTS found
	if not best_pk0:
		print("No LTS found...")

	#Stop RX and TX
	sdr_rx.deactivateStream(rx_stream)
	sdr_rx.closeStream(rx_stream)
	sdr_tx.writeSetting("TX_REPLAY", "")
	for ch in tx_channel:
		sdr_tx.writeSetting(SOAPY_SDR_TX, ch, "ENABLE_CHANNEL", "false")

	stop = datetime.now()

	return start, stop, lts_pks0, lts_pks1

def gain_tester(freq_t, other_sdrs_t, other_serials_t, ref_serial_t, rm_from_oth_serials_t, sdr_log_level_t):
	'''
	For each increment of the RX gain, runs through all increments of TX gains and records how many pilots received.
	Limits on start/stop/step of each increment can be changed for the respective constant at the top of code
	Args:
		- freq_t (to distinguish from other vars used)
		- other_sdrs_t
		- other_serials_t
		- ref_serial_t
		- rm_from_oth_serials_t
		- sdr_log_level_t
	Returns:
		- Print to console of TX/RX combos that received the greatest # of LTS signals
	'''
	results_dict = collections.defaultdict(int)
	test_start = datetime.now() #Do I even need this?
	SoapySDR.SoapySDR_setLogLevel(sdr_log_level_t) #What is this?
	best_pks0 = np.NINF
	best_pks1 = np.NINF

	#Prep ref Iris and other Irises
	other_sdrs, other_serials, other_frontends, ref_sdr, ref_serial = start_irises(other_serials_t, ref_serial_t, rm_from_oth_serials_t)
	
	#Iterate through all serials
	for pos in range(len(other_serials)):
		#Start with one RX gain, then try all possible TX gains before increasing the RX gain
		for rx_gain in range(RX_LO, RX_HI, RX_STEP):
			#One ant at a time, or both simultaneously?
			for tx_ant in ["A", "B"]:
				for tx_gain in range(TX_LO, TX_HI, TX_STEP):
					start, stop, test_pks0, test_pks1 = send_tx_rx(other_sdrs_t[pos], ref_sdr, tx_gain, rx_gain, freq_t, tx_ant)
					#What happens if no LTS signal found?
					#Don't care about list of LTS's, only the # of them found
					num_pks0 = len(test_pks0)
					num_pks1 = len(test_pks1) #And keep results from buffers separate
					peak_tuple = tuple(num_pks0, num_pks1)
					gain_tuple = tuple(rx_gain, tx_gain)
					#Store all results here
					results_dict[gain_tuple] = peak_tuple

					#Store what was the best number of pilots recorded
					if num_pks0 > best_pks0:
						best_pks0 = num_pks0
					if num_pks1 > best_pks1:
						best_pks1 = num_pks1

	
	
	#Find and store the best results in this dictionary
	#Key: RX gain; key val: TX gain
	best_list = list()
	for gain_pairs, pk_pairs in results_dict.items():
		if pk_pairs[0] == best_pks0 or pk_pairs[1] == best_pks1:
			best_list.append(gain_pairs)

	#Tell the user stuff
	print("=+=+=+=+=+=+=+=+=+=+=")
	print("Recommended gains:")
	for pairs in best_list:
		print("RX gain: " + pairs[0] + "//" + "TX gain: " + pairs[1])
		print("-------------------------")
	print("=+=+=+=+=+=+=+=+=+=+=")
	
	return None

### Main ###

if __name__ == "__main__":
	args = command_line()

	gain_tester(freq_t=args.freq,
	other_sdrs_t = args.
	
	
	
	)
