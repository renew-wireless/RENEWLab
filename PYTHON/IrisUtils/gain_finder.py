'''
  gain_finder.py

  Run to send pilots from one or several Irises and receive at one reference Iris;
  Will send pilots at each increment of TX/RX gain specified and record which
  gain settings record the most pilots, then tell user best combo of gains

Ex: 
	-Use all available Irises to transmit pilot (one by one)
		python3 gain_finder.py
		Will choose first Iris discovered as "ref_serial" (Iris that will receive pilots)
	-Specify Irises to transmit
		python3 gain_finder.py -o "RF3E000354" "RF3E000276" "RF3E000216"
		Will choose RF3E000354 as ref_serial to receive and other two Irises will transmit
	-Specify Irises to transmit and Iris to receive
		python3 gain_finder.py -o "RF3E000354" "RF3E000276" "RF3E000216" --ref_serial="RF3E000068"

  Can also input Irises to exclude from the script, lowest/highest (inclusive) gains to be tested,
  increment size between each change in TX/RX gain

  Will automatically plot results on a heatmap

'''


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
import matplotlib
import matplotlib.pyplot as plt


def c_line():
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

	parser.add_argument("-o", "--other_serials",
						nargs = "+",
						type = str,
						dest = "other_serials",
						default=[],
						help = "List of Irises to be tested for gains. Default: all "
								"Irises on network. Default: %(default)s ex. "
								"-o= 'RF3E000287 RF3E000300'")
	
	parser.add_argument("-rm", "--rm_from_oth_serials", 
		nargs = "+",
		type = str,
		default = [],
		help = "List of Iris serials to be removed from test list "
				"Default: %(default)s")
	
	parser.add_argument("--tx_lo",
		type = int,
		dest = "tx_lo",
		default = 50,
		help = "Default lower bound on transmitter gain. "
				"Default: %(default)s")
	
	parser.add_argument("--tx_hi",
		type = int,
		dest = "tx_hi",
		default = 80,
		help = "Upper bound on transmitter gain. "
				"Default: %(default)s")
	
	parser.add_argument("--tx_step",
		type = int,
		dest = "tx_step",
		default = 1,
		help = "Step size for each increment of TX gain. "
				"Default: %(default)s")
	
	parser.add_argument("--rx_lo",
		type = int,
		dest = "rx_lo",
		default = 50,
		help = "Lower bound on receiver gain "
				"Default: %(default)s")
	
	parser.add_argument("--rx_hi",
		type = int,
		dest = "rx_hi",
		default = 80,
		help = "Upper bound on receiver gain. "
				"Default: %(default)s")

	parser.add_argument("--rx_step",
		type = int,
		dest = "rx_step",
		default = 1,
		help = "Step size for each increment of RX gain. "
				"Default: %(default)s")

	parser.add_argument("--sdr_log_level",
		type = int,
		dest = "sdr_log_level",
		default = 3,
		help = "Logging depth of Iris Sdr error/info/warning. "
				"Default: %(default)s")
	return parser.parse_args()

def start_irises(other_serials, ref_serial, rm_from_oth_serials):
    '''
    Start up for all irises.
	Args:
	- other_serials: The Irises serials to send TX; 
	- ref_serial: Main serial that receives pilots
	- rm_from_oth_serials: specified serials to be excluded from gain tester
    Returns:
    - other_sdrs: SoapySdr objects of other_serials (will transmit)
	- other_serials: unchanged (list)
	- other_frontends: list containing frontends of serials, in order
	- ref_sdr: SoapySdr object of reference Iris's serial
	- ref_serial: Serial for ref Iris (will receive)
  	'''
    #Enumerate devices on network
    handles = SoapySDR.Device.enumerate({"remote:timeout": "250000"})
    if not other_serials: #User has not specified Irises to transmit, so use all available Irises on the network
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

    #If user specified any Irises to be excluded, remove them here
    for serial in rm_from_oth_serials:
        other_sdrs.pop(other_serials.index(serial))
        other_serials.pop(other_serials.index(serial))
        other_frontends.pop(other_serials.index(serial))
	###############
    other_serials.remove(ref_serial) #Remove ref_serial to avoid indexing error when running gain_tester (more serials than there are Irises to test)
	############
    # other_sdrs = tuple(other_sdrs)
    # print("--------")
    # print("other sdrs: ", other_sdrs)
    # print("other serials: ", other_serials)
    # print("other frontends: ", other_frontends)
    # print("ref sdr: ", ref_sdr)
    # print("ref serial: ", ref_serial)
    # print("--------")
    return other_sdrs, other_serials, other_frontends, ref_sdr, ref_serial

#Generate pilots
def generate_pilots(rate, ampl, wave_freq, bb_freq, num_samps):
    '''
    Generate the pilots to be transmitted/received.
	Args:
	- rate
	- ampl
	- wave_freq
	- bb_freq
	- num_samps

	Returns:
	- pilot1_ui32
	- pilot2-ui32
    '''

    #Generate TX signal
    tx_signal = np.empty(num_samps).astype(np.complex64)
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
	- sdr_tx - Soapysdr for transmitting
	- sdr_rx - Soapysdr for receiving
	- tx_gain
	- rx_gain
	- freq
	- tx_ant - transmitter antenna to use
	Returns:
	- start: Start time for TX/RX
	- stop: End time for TX/RX
	- lts_pks0: List of lts signals received from first buffer
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
	amp_fixed = int(tx_amp * (1 << 13))

	for ch in tx_channel:
		sdr_tx.writeSetting(SOAPY_SDR_TX, ch, "ENABLE_CHANNEL", "true")
		sdr_tx.setBandwidth(SOAPY_SDR_TX, ch, 2.5 * samp_rate)
		sdr_tx.setSampleRate(SOAPY_SDR_TX, ch, samp_rate)
		sdr_tx.setFrequency(SOAPY_SDR_TX, ch, "RF", freq - 0.75 * samp_rate)
		sdr_tx.setFrequency(SOAPY_SDR_TX, ch, "BB", 0.75 * samp_rate)
		sdr_tx.setAntenna(SOAPY_SDR_TX, ch, "TRX")

		if lo_tone:
			sdr_tx.writeSetting(SOAPY_SDR_TX, ch, "TSP_TSG_CONST", str(amp_fixed))
			sdr_tx.writeSetting(SOAPY_SDR_TX, ch, "TX_ENB_OVERRIDE", "true")

		if "CBRS" in info["frontend"]:
			sdr_tx.setGain(SOAPY_SDR_TX, ch, tx_gain)
		else:
			#No CBRS board gains, only changing LMS7 gains
			sdr_tx.setGain(SOAPY_SDR_TX, ch, "PAD", tx_gain) #[0:1:42]
			sdr_tx.setGain(SOAPY_SDR_TX, ch, "IAMP", 0) #[-12:1:3]

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
	start = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

	#Set parameters on both RF channels
	info = sdr_rx.getHardwareInfo()
	for ch in [0, 1]:
		sdr_rx.setBandwidth(SOAPY_SDR_RX, ch, 2.5 * samp_rate)
		sdr_rx.setFrequency(SOAPY_SDR_RX, ch, freq)
		sdr_rx.setSampleRate(SOAPY_SDR_RX, ch, samp_rate)
		sdr_rx.setAntenna(SOAPY_SDR_RX, ch, "TRX")
		sdr_rx.setDCOffsetMode(SOAPY_SDR_RX, ch, True)
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
	setUpDigitalRssiMode(sdr_rx)

	#Read samples into buffers 
	samps_rx = [np.zeros(rx_num_samps, np.complex64), 
			np.zeros(rx_num_samps, np.complex64)] 
	buffer0 = samps_rx[0] #RF Chain 1
	buffer1 = samps_rx[1] #RF Chain 2

	flags = SOAPY_SDR_END_BURST
	if wait_trigger: 
		flags |= SOAPY_SDR_WAIT_TRIGGER
	sdr_rx.activateStream(rx_stream, flags, 0, buffer0.size)
	sr = sdr_rx.readStream(rx_stream, [buffer0, buffer1], buffer0.size)
	if sr.ret != buffer0.size:
		print("Read RX burst of %d, requested %d" % (sr.ret, buffer0.size))

	#Remove DC
	for i in [0, 1]:
		samps_rx[i] -= np.mean(samps_rx[i])

	lts_thresh = 0.8
	best_pk0, lts_pks0, lts_corr0 = find_lts(samps_rx[0], thresh=lts_thresh, flip=True)
	best_pk1, lts_pks1, lts_corr1 = find_lts(samps_rx[1], thresh=lts_thresh, flip=True)
	'''
	find_lts.py docstring ▼

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
	#print("start: ", start)
	#print("stop: ", stop)
	print("1st lts: ", lts_pks0)
	print("2nd lts: ", lts_pks1)
	#print("type lts0: ", type(lts_pks0))
	return start, stop, lts_pks0, lts_pks1

def heatmap(data, row_labels, col_labels, ax=None,
            cbar_kw={}, cbarlabel="", **kwargs):
    """
	Code from matplotlib website "Annotated Heatmap"
    Create a heatmap from a numpy array and two lists of labels.

    Parameters
    ----------
    data
        A 2D numpy array of shape (N, M).
    row_labels
        A list or array of length N with the labels for the rows.
    col_labels
        A list or array of length M with the labels for the columns.
    ax
        A `matplotlib.axes.Axes` instance to which the heatmap is plotted.  If
        not provided, use current axes or create a new one.  Optional.
    cbar_kw
        A dictionary with arguments to `matplotlib.Figure.colorbar`.  Optional.
    cbarlabel
        The label for the colorbar.  Optional.
    **kwargs
        All other arguments are forwarded to `imshow`.
    """

    if not ax:
        ax = plt.gca()

    # Plot the heatmap
    im = ax.imshow(data, **kwargs)

    # Create colorbar
    cbar = ax.figure.colorbar(im, ax=ax, **cbar_kw)
    cbar.ax.set_ylabel(cbarlabel, rotation=-90, va="bottom")

    # We want to show all ticks...
    ax.set_xticks(np.arange(data.shape[1]))
    ax.set_yticks(np.arange(data.shape[0]))
    # ... and label them with the respective list entries.
    ax.set_xticklabels(col_labels)
    ax.set_yticklabels(row_labels)

    # Let the horizontal axes labeling appear on top.
    ax.tick_params(top=True, bottom=False,
                   labeltop=True, labelbottom=False)

    # Rotate the tick labels and set their alignment.
    plt.setp(ax.get_xticklabels(), rotation=-30, ha="right",
             rotation_mode="anchor")

    # Turn spines off and create white grid.
    #ax.spines[:].set_visible(False)

    ax.set_xticks(np.arange(data.shape[1]+1)-.5, minor=True)
    ax.set_yticks(np.arange(data.shape[0]+1)-.5, minor=True)
    ax.grid(which="minor", color="w", linestyle='-', linewidth=3)
    ax.tick_params(which="minor", bottom=False, left=False)

    return im, cbar

def annotate_heatmap(im, data=None, valfmt="{x:.2f}",
                     textcolors=("black", "white"),
                     threshold=None, **textkw):
    """
	Code from matplotlib website example "Annotated Heatmap"
    A function to annotate a heatmap.

    Parameters
    ----------
    im
        The AxesImage to be labeled.
    data
        Data used to annotate.  If None, the image's data is used.  Optional.
    valfmt
        The format of the annotations inside the heatmap.  This should either
        use the string format method, e.g. "$ {x:.2f}", or be a
        `matplotlib.ticker.Formatter`.  Optional.
    textcolors
        A pair of colors.  The first is used for values below a threshold,
        the second for those above.  Optional.
    threshold
        Value in data units according to which the colors from textcolors are
        applied.  If None (the default) uses the middle of the colormap as
        separation.  Optional.
    **kwargs
        All other arguments are forwarded to each call to `text` used to create
        the text labels.
    """

    if not isinstance(data, (list, np.ndarray)):
        data = im.get_array()

    # Normalize the threshold to the images color range.
    if threshold is not None:
        threshold = im.norm(threshold)
    else:
        threshold = im.norm(data.max())/2.

    # Set default alignment to center, but allow it to be
    # overwritten by textkw.
    kw = dict(horizontalalignment="center",
              verticalalignment="center")
    kw.update(textkw)

    # Get the formatter in case a string is supplied
    if isinstance(valfmt, str):
        valfmt = matplotlib.ticker.StrMethodFormatter(valfmt)

    # Loop over the data and create a `Text` for each "pixel".
    # Change the text's color depending on the data.
    texts = []
    for i in range(data.shape[0]):
        for j in range(data.shape[1]):
            kw.update(color=textcolors[int(im.norm(data[i, j]) > threshold)])
            text = im.axes.text(j, i, valfmt(data[i, j], None), **kw)
            texts.append(text)

    return texts

def gain_plotter(ant_A=dict, ant_B=dict):
	'''
	Args:
	- ant_A: dict returned as results_A from gain_tester() // Has format of {'Radio Serial': {(RX, TX): Greatest # of pilots}}
	- ant_B: dict returned as results_B from gain_tester()

	Returns:
	- Heat map of ideal gains, separated by TX and RX

	'''
	#Set up the 2D array
	final_array_TX = []
	final_array_RX = []
	serials_labels = []
	array_A_TX = []
	array_A_RX = []
	array_B_TX = []
	array_B_RX = []
	ant_labels = ["A","B"]
	for serial, data in ant_A.items():
		serials_labels.append(serial)
		for gains in data.keys():
			array_A_TX.append(gains[1]) #TX
			array_A_RX.append(gains[0]) #RX

	for data in ant_B.values():
		for gains in data.keys():
			array_B_TX.append(gains[1]) #TX
			array_B_RX.append(gains[0]) #RX
	
	final_array_TX.append(array_A_TX)
	final_array_TX.append(array_B_TX)
	final_array_RX.append(array_A_RX)
	final_array_RX.append(array_B_RX)
	#Silly workaround
	final_array_TX = np.array(final_array_TX)
	final_array_RX = np.array(final_array_RX)

	#TX
	fig, ax = plt.subplots()
	im, cbar = heatmap(final_array_TX, ant_labels, serials_labels, ax=ax, cmap="BuPu", cbarlabel="Gain [dB]")
	texts = annotate_heatmap(im, textcolors=("black", "white"))
	ax.set_title("Ideal TX Gains")
	ax.set_ylabel("antenna")
	fig.tight_layout()
	#RX
	fig2, ax2 = plt.subplots()
	im2, cbar2 = heatmap(final_array_RX, ant_labels, serials_labels, ax=ax2, cmap="OrRd", cbarlabel="Gain [dB]")
	texts2 = annotate_heatmap(im2, textcolors=("black", "white"))
	ax2.set_title("Ideal RX Gains")
	ax2.set_ylabel("antenna")
	fig2.tight_layout()
	plt.show()

	return None

def gain_tester(freq, other_serials, ref_serial, rm_from_oth_serials, tx_lo, tx_hi,
				 tx_step, rx_lo, rx_hi, rx_step, sdr_log_level):
	'''
	For each increment of the RX gain, runs through all increments of TX gains and records how many pilots received.
	Limits on start/stop/step of each increment can be changed for the respective constant at the top of code
	Args:
		- freq
		- other_serials: string of serials to transmit
		- ref_serial: string of serial to receive
		- rm_from_oth_serials: string of serials to exclude from testing
		- tx_lo: Lower bound on transmit gain (inclusive) 
		- tx_hi: Higher bound on transmit gain (inclusive) 
		- tx_step: Increment size for each transmit gain
		- rx_lo: Lower bound on receiver gain (inclusive)
		- rx_hi: Higher bound on receive gain (inclusive)
		- rx_step: Increment size for each receiver gain
		- sdr_log_level (Params given same name as what could be passed to parser to overwrite)
	Returns:
		- Prints to console TX/RX combos that received the greatest # of LTS signals
		- results_dict: contains number of greatest pilots received for each gain combo, doesn't distinguish between Irises (will overwrite results for each BaseStation)
		- results_A: Contains best gain combo, along with pilots received for antenna A, organized by Iris serial
		- results_B: Contains best gain combo, along with pilots received for antenna B, organized by Iris serial
	'''
	#For both antennas
	results_A = collections.defaultdict(int)
	results_B = collections.defaultdict(int)
	results_dict = collections.defaultdict(int)
	test_start = datetime.now()
	SoapySDR.SoapySDR_setLogLevel(sdr_log_level)
	all_bestA = np.NINF
	all_bestB = np.NINF

	#Prep ref Iris and other Irises
	other_sdrs, other_serials, other_frontends, ref_sdr, ref_serial = start_irises(other_serials, ref_serial, rm_from_oth_serials)
	#Iterate through all serials
	for pos in range(len(other_serials)):
		print("Transmitting with Iris ", other_serials[pos], "...")
		best_pks0 = np.NINF
		best_pks1 = np.NINF
		best_gains0 = None
		best_gains1 = None
		#Start with one RX gain, then try all possible TX gains before increasing the RX gain
		for rx_gain in range(rx_lo, rx_hi+rx_step, rx_step):
			for tx_ant in ["A", "B"]:
				for tx_gain in range(tx_lo, tx_hi+tx_step, tx_step):
					start, stop, test_pks0, test_pks1 = send_tx_rx(other_sdrs[pos], ref_sdr, tx_gain, rx_gain, freq, tx_ant)
					#Don't care about list of LTS's, only the # of them found
					num_pks0 = test_pks0.size
					num_pks1 = test_pks1.size #And keep results from buffers separate
					peak_tuple = (num_pks0, num_pks1)
					gain_tuple = (rx_gain, tx_gain)
					#Store all results here
					results_dict[gain_tuple] = peak_tuple #e.g {(60, 70): (12, 24)} // {(RX, TX): (num pilots buffer 1: num pilots buffer 2)}	
					#Determine if the pilots received is better for antenna A
					if num_pks0 > best_pks0:
						best_gains0 = (rx_gain, tx_gain)
						best_pks0 = num_pks0
					#Determine if pilots received is better for antenna B
					if num_pks1 > best_pks1:
						best_gains1 = (rx_gain, tx_gain)
						best_pks1 = num_pks1
					
		#Store best gain combo for each antenna, along with number of pilots received for that antenna
		results_A[other_serials[pos]] = {best_gains0:best_pks0}
		results_B[other_serials[pos]] = {best_gains1:best_pks1}
	
	#print("results A:", results_A)
	#print("results B:", results_B)
	# best_list = []
	# print("all best A: ", all_bestA)
	# print("all best B: ", all_bestB)
	# for gain_pairs, pk_pairs in results_dict.items():
	# 	print("gain pair:", gain_pairs)
	# 	print("pk pair:", pk_pairs)
	# 	if pk_pairs[0] == all_bestA or pk_pairs[1] == all_bestB:
	# 		best_list.append(gain_pairs)
	## ISSUE WITH THIS:
	'''
	Currently this best_list does not distinguish between buffers, so it will not recommend (usually) more than one
	gain combo to the user because the previous results that had the most pilots was overwritten by one buffer or the other
	when stored into the dictionary
	'''
	# #Tell the user stuff
	# print("=+=+=+=+=+=+=+=+=+=+=")
	# print("Recommended gains:")
	# for pairs in best_list:
	# 	print("RX gain: ", pairs[0], "// TX gain: ", pairs[1])
	# 	print("-------------------------")
	# print("=+=+=+=+=+=+=+=+=+=+=")
	print("Done!")
	return results_dict, results_A, results_B


### Main ###

if __name__ == "__main__":
	args = c_line()

	#Test if start_irises functions properly
	#start_irises(other_serials = args.other_serials, ref_serial = args.ref_serial, rm_from_oth_serials = args.rm_from_oth_serials)

	#Once initialized, check that irises transmit and receive correctly
	#tsto_sdrs, tsto_serials, tsto_frontends, refo_sdr, refo_serial = start_irises(other_serials = args.other_serials, ref_serial = args.ref_serial, rm_from_oth_serials = args.rm_from_oth_serials)
	#send_tx_rx(tsto_sdrs, refo_sdr, tx_gain=65, rx_gain=81, freq= 3.6e9, tx_ant="A")

	#Now that Irises transmit and receive, test recording of LTS signals and print results back to user
	full_results, results_A, results_B = gain_tester(freq=args.freq,
	other_serials = args.other_serials,
	ref_serial = args.ref_serial,
	rm_from_oth_serials = args.rm_from_oth_serials,
	tx_lo = args.tx_lo,
	tx_hi = args.tx_hi,
	tx_step = args.tx_step,
	rx_lo = args.rx_lo,
	rx_hi = args.rx_hi,
	rx_step = args.rx_step,
	sdr_log_level = args.sdr_log_level)

	#Plot
	gain_plotter(results_A, results_B)

	#Test plotting
	#gain_plotter({'RF3E000087': {(65, 60): 26}, 'RF3E000084': {(65, 60): 27}, 'RF3E000580': {(60, 65): 27}, 'RF3E000474': {(50, 50): 38}},{'RF3E000087': {(55, 50): 27}, 'RF3E000084': {(55, 55): 30}, 'RF3E000580': {(50, 55): 30}, 'RF3E000474': {(50, 60): 34}})