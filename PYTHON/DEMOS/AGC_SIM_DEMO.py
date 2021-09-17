#!/usr/bin/python
"""
 AGC_SIM_DEMO.py

 Script designed to demonstrate the operation of the AGC State machine.
 This AGC runs on the host machine and therefore it implements a VERY
 coarse version of the AGC (non-real time). That is, only one gain
 adjustment occurs at each buffer read.
 It requires two Iris boards. One TX and one RX. The TX is continuously
 transmitting and we use the digital RSSI measurements obtained from the
 LMS7 in order to adapt the RX amplifiers. As of this moment, there is
 no way of synchronizing a received frame to the reading
 of the RSSI. Therefore, we need to keep the TX continuously
 sending a signal.
 NOTE: Iris boards don't need to be chained

 Basic Operation:
   1) Run SISO_TX.py or another TX script on TX Iris board
   2) Run this script (AGC_SIM_DEMO.py) on the RX Iris board.
      The AGC thread will trigger the AGC "maxNumFrames" number of times
      and will plot the received signal

 Usage example: python3 AGC_SIM_DEMO.py --serial="RF3C000034"

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 ---------------------------------------------------------------------
"""
import matplotlib
#matplotlib.use('GTK3Agg')

import sys
sys.path.append('../IrisUtils/')
sys.path.append('../IrisUtils/data_in/')
sys.path.append('../../deps/transitions/')
import SoapySDR
import collections
import logging
import time
import threading
import pickle
import scipy.signal
import matplotlib.pyplot as plt
from matplotlib import animation
#from SoapySDR import *              # SOAPY_SDR constants
from optparse import OptionParser
from digital_rssi import *
from fft_power import *
from find_lts import *
from agc_fsm_iris import *
from file_rdwr import *
from bandpower import *
from generate_sequence import *
matplotlib.rcParams.update({'font.size': 10})
from MyFuncAnimation import *


#########################################
#            Global Parameters          #
#########################################
agc_fsm = None
threadT = None
IQmag = None
sdr = None
rxStream = None
sampsRx = None
timeScale = None
rxChan = 0
fft_size = 1024
rate = 5e6
num_samps = 2**12
freqScale = np.arange(-rate // 2, rate // 2, rate // fft_size)
# freqScale = np.arange(-rate / 2 / 1e6, rate / 2 / 1e6, rate / fft_size / 1e6)[:fft_size]
numBufferSamps = 1000
rssiPwrBuffer = collections.deque(maxlen=numBufferSamps)
timePwrBuffer = collections.deque(maxlen=numBufferSamps)
freqPwrBuffer = collections.deque(maxlen=numBufferSamps)
noisPwrBuffer = collections.deque(maxlen=numBufferSamps)


########################################
#               LOGGER                 #
########################################
logging.basicConfig(filename='./data_out/debug_AGC_NON-REAL-TIME_DEMO.log',
                    level=logging.DEBUG,
                    format='[%(levelname)s] (%(threadName)-10s) %(asctime)s %(message)s',
                    )


#########################################
#             Create Plots              #
#########################################
# Figure 1
fig = plt.figure(figsize=(20, 8), dpi=100)
fig.subplots_adjust(hspace=.5, top=.85)
# Subplot1
ax1 = fig.add_subplot(4, 1, 1)
ax1.grid(True)
ax1.set_title('Waveform capture')
ax1.set_ylabel('Amplitude (units)')
line1, = ax1.plot([], [], label='AI', animated=True)
line2, = ax1.plot([], [], label='AQ', animated=True)
ax1.set_ylim(-1, 1)
ax1.set_xlim(0, num_samps)
ax1.legend(fontsize=10)
# Subplot2
ax2 = fig.add_subplot(4, 1, 2)
ax2.grid(True)
ax2.set_xlabel('Frequency (Hz)')
ax2.set_ylabel('PSD [V**2/Hz]')
line3, = ax2.plot([], [], animated=True)
ax2.set_ylim(-130, 0)
ax2.set_xlim(freqScale[0], freqScale[-1])
ax2.legend(fontsize=10)
# Subplot3
ax3 = fig.add_subplot(4, 1, 3)
ax3.grid(True)
ax3.set_xlabel('Real-Time Samples')
ax3.set_ylabel('Power (dB)')
line4, = ax3.plot([], [], label='Digital RSSI', animated=True)
line5, = ax3.plot([], [], label='TimeDomain Sig Pwr', animated=True)
line6, = ax3.plot([], [], label='FreqDomain Sig Pwr', animated=True, linestyle='dashed')
line7, = ax3.plot([], [], label='Noise Floor', animated=True)
ax3.set_ylim(-100, 10)
ax3.set_xlim(0, numBufferSamps*1.5)
ax3.legend(fontsize=10)
# Subplot 4
ax4 = fig.add_subplot(4, 1, 4)
ax4.grid(True)
ax4.set_xlabel('Sample Index')
ax4.set_ylabel('Correlation Peaks')
line8, = ax4.plot([], [], label='Corr I ChA', animated=True)
ax4.set_ylim(-10, 10)
ax4.set_xlim(0, num_samps)
ax4.legend(fontsize=10)

countMegd = 0


#########################################
#              Functions                #
#########################################
def init():
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    line4.set_data([], [])
    line5.set_data([], [])
    line6.set_data([], [])
    line7.set_data([], [])
    line8.set_data([], [])
    return line1, line2, line3, line4, line5, line6, line7, line8


def agc_thread():
    global agc_fsm, sdr, IQmag
    logging.debug("Run State Machine Test, MAX OUT GAINS")
    # Allow for the rest of the RX process to finish up (arbitrary wait, logs show 3 sec is enough...)
    time.sleep(5)

    # Emulate multiple transmissions
    maxNumFrames = 5
    for iterIdx in range(0, maxNumFrames):
        # Reset Gains
        print("FRAME RECEPTION: {}/{}".format(iterIdx, maxNumFrames-1))
        agc_fsm.numSatSamps = 0             # IMPORTANT: Reset
        agc_fsm.satDetectedFlag = False     # IMPORTANT: Reset
        agc_fsm.satDetGoodFlag = False      # IMPORTANT: Reset
        agc_fsm.enableAGC()                 # IMPORTANT: Re-enable

        ch = 0
        # Arbitrary - start off with some "decent" initial fixed gain
        rxgain_init = 90
        sdr.setGain(SOAPY_SDR_RX, ch, rxgain_init)
        time.sleep(0.1)     # Let gain settle
        time.sleep(5)       # Wait for "demo" purposes
        print(" ********** AGC KICKS IN ********** ")
        # DO NOT CHANGE RANGE IN THIS FOR LOOP UNLESS YOU CHANGE THE NUMBER OF SAMPLES
        # USED FOR FINE/COARSE ADJUSTMENT (max 16 samples plus 3 more that should be used for fine tuning)
        for i in range(0, 20):
            agc_avg = 3
            rssi, PWRdBm = getDigitalRSSI(sdr, agc_avg)
            agc_fsm.currentSample = i
            agc_fsm.rssi[i] = PWRdBm  # 30-50 good range for this sim
            # print("sample: " + str(i) + " RSSI: " + str(agc_fsm.rssi[i]))
            logging.debug("READ NEW SAMPLE: {} \t RSSI: {}".format(i, PWRdBm))

            # Check if Saturation
            if agc_fsm.rssi[i] > agc_fsm.rssiSatThresh:
                agc_fsm.numSatSamps += 1
            if agc_fsm.numSatSamps >= agc_fsm.numSampAboveThresh:
                agc_fsm.satDetectedFlag = True
                agc_fsm.numSatSamps = 0
            # Check if number of samples threshold has been reached, set saturation detection good flag
            if i == agc_fsm.numSampThresh - 1:
                logging.debug("REACHED SAMPLE THRESHOLD")
                agc_fsm.satDetGoodFlag = True

            # We've read a new sample
            agc_fsm.newSample()

        time.sleep(10)  # Wait for "demo" purposes
    print("*********************** DONE WITH AGC THREAD ***********************")


def animate(i):
    global sdr, rxStream, timeScale, sampsRx, fft_size, rate, num_samps, agc_fsm, IQmag, countMegd
    # read samples into this buffer
    buff0 = sampsRx[0]  # RF Chain 1
    buff1 = sampsRx[1]  # RF Chain 2 (Ignore for now)

    sdr.activateStream(rxStream,
        SOAPY_SDR_END_BURST,        # flags
        0,                          # timeNs (dont care unless using SOAPY_SDR_HAS_TIME)
        buff0.size)                 # numElems - this is the burst size
    sr = sdr.readStream(rxStream, [buff0, buff1], buff0.size)

    # Re-assign
    rxSignal = sampsRx[0]

    # dc removal
    sampsRx[rxChan] -= np.mean(sampsRx[rxChan])
    lts_thresh = 0.8
    a, b, peaks = find_lts(sampsRx[0], thresh=lts_thresh)
    write_to_file("./data_out/rx0", sampsRx[0])

    # Magnitude of IQ Samples - Real and imaginary
    I = np.real(rxSignal)
    Q = np.imag(rxSignal)
    IQmag = np.mean(np.sqrt(I**2 + Q**2))

    # DIGITAL RSSI
    agc_avg = 7  # average over X number of samples. See LMS7 datasheet
    rssi, PWRdBm = getDigitalRSSI(sdr, agc_avg)
    PWRdB = PWRdBm - 30

    # TIME DOMAIN POWER
    sigRms = np.sqrt(np.mean(rxSignal * np.conj(rxSignal)))
    sigPwr = np.real(sigRms) ** 2
    sigPwr_dB = 10 * np.log10(sigPwr)
    sigPwr_dBm = 10 * np.log10(sigPwr / 1e-3)

    # POWER FFT
    # powerBins = log_power_fft(rxSignal, fft_size)
    # noiseFloor, _ = log_power_fft_analysis(powerBins, rate)
    f1, powerBins, noiseFloor, pks = fft_power(rxSignal, rate, num_bins=None, peak=1.0, scaling='density', peak_thresh=20)
    fftPower = bandpower(rxSignal, rate, 0, rate / 2)
    fftPower_dB = 10 * np.log10(fftPower)
    fftPower_dBm = 10 * np.log10(fftPower / 1e-3)

    # Circular buffer - continuously display data
    rssiPwrBuffer.append(PWRdBm)
    timePwrBuffer.append(sigPwr_dB)
    freqPwrBuffer.append(fftPower_dB)
    noisPwrBuffer.append(noiseFloor)

    # PRINT VALUES
    # 50kHz sinusoid
    print("DigitalRSSI: {} \t DigitalPwr: {} ".format(rssi, PWRdBm))

    line1.set_data(range(buff0.size), np.real(rxSignal))
    line2.set_data(range(buff0.size), np.imag(rxSignal))
    line3.set_data(f1, powerBins)
    line4.set_data(range(len(rssiPwrBuffer)), rssiPwrBuffer)
    line5.set_data(range(len(timePwrBuffer)), timePwrBuffer)
    line6.set_data(range(len(freqPwrBuffer)), freqPwrBuffer)
    line7.set_data(range(len(noisPwrBuffer)), noisPwrBuffer)
    line8.set_data(range(buff0.size), np.real(peaks[:buff0.size]))

    return line1, line2, line3, line4, line5, line6, line7, line8


def rxsamples_app(args, srl, freq, bw, rxgain, clockRate, out):
    global sdr, rxStream, timeScale, sampsRx, freqScale, rate, num_samps, fft_size, threadT, agc_fsm
    sdr = SoapySDR.Device(dict(serial=srl))
    info = sdr.getHardwareInfo()
    ch = rxChan

    # RSSI read setup
    setUpDigitalRssiMode(sdr)

    # Instantiate AGC FSM and enable
    agc_fsm = AutomaticGainControl(sdr, ch)
    # Enable AGC and trigger AGC state machine
    threadT = threading.Thread(target=agc_thread)  # , args=(sampsRx,))

    # set clock rate first
    if clockRate is None:
        sdr.setMasterClockRate(rate*8)
    else:
        sdr.setMasterClockRate(clockRate)
    if bw is not None:
        sdr.setBandwidth(SOAPY_SDR_RX, ch, bw)
    # set params on both channels

    # sdr.setBandwidth(SOAPY_SDR_RX, ch, 10e6)
    sdr.setSampleRate(SOAPY_SDR_RX, ch, rate)
    sdr.setFrequency(SOAPY_SDR_RX, ch, "RF", freq)
    sdr.setFrequency(SOAPY_SDR_RX, ch, "BB", 0)  # don't use cordic
    sdr.setGain(SOAPY_SDR_RX, ch, rxgain)        # w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
    sdr.setAntenna(SOAPY_SDR_RX, ch, "TRX")
    sdr.setDCOffsetMode(SOAPY_SDR_RX, ch, True)
    sdr.setHardwareTime(0)
    print("Set Frequency to %f" % sdr.getFrequency(SOAPY_SDR_RX, 0))

    # setup rxStreaming
    # request an rx burst as an example
    # repeat activateStream and readStream() for each burst needed
    sdr.writeSetting(SOAPY_SDR_RX, 0, 'CALIBRATE', 'SKLK')
    rxStream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0], {})

    # cleanup rxStream
    # print("Cleanup rxStreams")
    # sdr.deactivateStream(rxStream)
    # sdr.closeStream(rxStream)
    step = 1
    print("numSamps %d " % num_samps)
    timeScale = np.arange(0, num_samps*step, num_samps)
    sampsRx = [np.zeros(num_samps, np.complex64), np.zeros(num_samps, np.complex64)]

    # Start AGC thread
    threadT.start()

    #anim = animation.FuncAnimation(fig, animate, init_func=init,
    #                           frames=100, interval=100, blit=True)
    anim = MyFuncAnimation(fig, animate, init_func=init,
                                frames=100, interval=100, blit=True)

    plt.show()
    if out is not None:
        fig.savefig(out)
        plt.close(fig)


#########################################
#                  Main                 #
#########################################
def main():
    parser = OptionParser()
    parser.add_option("--args",      type="string", dest="args",      help="Device factory arguments",   default="")
    parser.add_option("--freq",      type="float",  dest="freq",      help="Optional Rx freq (Hz)",      default=2.5e9)
    parser.add_option("--bw",        type="float",  dest="bw",        help="Optional Tx filter bw (Hz)", default=30e6)
    parser.add_option("--rxgain",    type="float",  dest="rxgain",    help="Optional Rx gain (dB)",      default=90.0)
    parser.add_option("--clockRate", type="float",  dest="clockRate", help="Optional clock rate (Hz)",   default=80e6)
    parser.add_option("--out",       type="string", dest="out",       help="Path to output image file",  default=None)
    parser.add_option("--serial",    type="string", dest="serial",    help="Serial number of the device",default="")
    (options, args) = parser.parse_args()

    rxsamples_app(
        args=options.args,
        srl=options.serial,
        freq=options.freq,
        bw=options.bw,
        rxgain=options.rxgain,
        clockRate=options.clockRate,
        out=options.out,
    )


if __name__ == '__main__': 
    main()
