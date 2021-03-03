#!usr/bin/python3
"""
iris_health_monitor.py


Description: 

    Monitor the power levels and temperature of all Iris modules
    detectable on the network.

1.  Find all Iris modules on the network.

2.  TX from a Golden Iris module on channel A, loop through all other
    Iris modules and auto adjust rxgain until the RX power is in range.
    If within the rxgain range the RX power cannot be adjusted in range,
    fail the test. Otherwise, pass. Both channel A & B shall be scanned.

    Tx from the golden Iris. If no golden Iris is picked in the command,
    the first discovered Iris shall be designated as the golden Iris
    module. Tx on the specified channel. Loop through all other Iris
    modules' channel A & B, rx and find the best rxgain for each channel
    by increasing rxgain with gain-step.

        if using CBRS, rxgain range is
            [3:1:105] at 2.5 GHz
            [3:1:102] at 3.6 GHz
        if using only Dev Board, rxgain range is
            [0:1:30]

3.  RX from a Golden Iris module on channel A, loop through all other
    Iris modules and auto adjust txgain until the RX power is in range.
    If within the txgain range the RX power cannot be adjusted in range,
    fail the test. Otherwise, pass. Both channel A & B shall be scanned.

    Rx from the golden Iris channel A. if no golden Iris is picked in
    the command, the first discovered Iris shall be designated as the
    golen Iris module. Loop through all other iris modules' channel A &
    B, tx and find the best txgain for each channel by increasing txgain
    with gain-step.

        if using CBRS, txgain range is
            [16:1:93] at 2.5 GHz
            [15:1:102] at 3.6 GHz
        if using only Dev Board, txgain range is
            [0:1:42]


Usage Examples: 

1.  If no serial number is specified, the first detected Iris serial
    number shall shall be used as the golden unit. All other Iris
    modules on the network shall be scanned.

        python3 IRIS_HEALTH_MONITOR.py

2.  If the golden unit serial number is specified, all other Iris
    modules on the network shall be scanned.

        python3 IRIS_HEALTH_MONITOR.py --serial=RF3E000392

3.  If both the golden unit and a list of UUT Iris modules are specified

    pyton3 IRIS_HEALTH_MONITOR.py -s RF3E000392 -u RF3E000295 RF3E000300

4.  The expected output shall be both on Terminal and be saved in a log.
    The contents are timestampts, serial numbers, rxgains, txgains, rx
    power, temperature, and PASS/FAIL.
    TODO: Save to a database instead of a log file.

------------------------------------------------------------
Copyright © 2020 Rice University
RENEW OPEN SOURCE LICENSE http://renew-wireless.org/license
------------------------------------------------------------
"""


########################################################################
#                           Import Modules                             #
########################################################################
import sys
sys.path.append('../IrisUtils/')

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
import csv
import matplotlib.pyplot as plt


########################################################################
#                             Functions                                #
########################################################################

def comamnd_line():
    parser = argparse.ArgumentParser()
    parser.add_argument("--golden-serial",
                        type=str,
                        dest="golden_serial",
                        default="",
                        help="The golden Iris's serial number. "
                             "Default: %(default)s")
    parser.add_argument("--golden-tx-gain",
                        type=int,
                        dest="golden_tx_gain",
                        default=60,
                        help="The golden Iris's transmit gain. "
                             "Default: %(default)s")
    parser.add_argument("--golden-ant",
                        type=str,
                        dest="golden_ant",
                        default="A",
                        help="The golden Iris's transmit channel/antenna. "
                             "Default: %(default)s")
    parser.add_argument("--golden-rx-gain",
                        type=int,
                        dest="golden_rx_gain",
                        default=60,
                        help="The golden Iris's receive gain. "
                             "Default: %(default)s")
    parser.add_argument("--gain-step",
                        type=int,
                        dest="gain_step",
                        default=1,
                        help="UUT Iris tx/rx gain adjustment step size. "
                             "Resolution: 1. Default: %(default)s")
    parser.add_argument("-f", "--freq",
                        type=float,
                        dest="freq",
                        default=3.6e9,
                        help="All Irises' RF operation frequency in Hz. "
                             "Default: %(default)s")
    parser.add_argument("--rx-power-limit",
                        type=float,
                        dest="rx_power_limit",
                        default=-18.0,
                        help="The passing threshold for Iris's received power."
                             "Default: %(default)s")
    parser.add_argument("-u", "--uut-serials",
                        nargs="+",
                        type=str,
                        dest="uut_serials",
                        default=[],
                        help="A list of Irises to be tested. Default: all "
                             "Irises on network. Default: %(default)s. e.g. "
                             "-u RF3E000295 RF3E000300")
    parser.add_argument("--rm-from-uut-serials",
                        nargs="+",
                        type=str,
                        dest="rm_from_uut_serials",
                        default=[],
                        help="A list of Iris serials to be removed from the "
                             "test UUT list. Default: %(default)s. e.g. "
                             "-u RF3E000241 RF3E000068")
    parser.add_argument("-p", "--enable-detailed-logs",
                        action="store_true",
                        dest="en_detailed_logs",
                        default=False,
                        help="Show detailed logs for each gain step. "
                             "Default: %(default)s")
    parser.add_argument("-t", "--test-tx-only",
                        action="store_true",
                        dest="test_tx_only",
                        default=False,
                        help="If specified, UUT receivers will not be tested. "
                             "Only UUT transmitters will be tested.")
    parser.add_argument("-r", "--test-rx-only",
                        action="store_true",
                        dest="test_rx_only",
                        default=False,
                        help="If specified, UUT transmitter will not be tested"
                             ". Only UUT receivers will be tested.")
    parser.add_argument("--enable-logging",
                        action="store_true",
                        dest="en_logging",
                        default=False,
                        help="If specified, two files shall be saved: "
                             "./data_out/rx_test_DATETIME.csv and "
                             "./data_out/tx_test_DATETIME.csv",
                        )
    parser.add_argument("--sdr-log-level",
                        type=int,
                        dest="sdr_log_level",
                        default=3,
                        help="Logging depth of Iris SDR error/info/warning. "
                             "Default: %(default)s",
                        )
    # SOAPY_SDR_FATAL    = 1, // fatal error. terminate. the highest
    # SOAPY_SDR_CRITICAL = 2, // critical error. might fail to run
    # SOAPY_SDR_ERROR    = 3, // error. operation not successful
    # SOAPY_SDR_WARNING  = 4, // warning. unexpected result
    # SOAPY_SDR_NOTICE   = 5, // notice, just an information
    # SOAPY_SDR_INFO     = 6, // info and warning, operation compmletion
    # SOAPY_SDR_DEBUG    = 7, // debugging message plus INFO and warning
    # SOAPY_SDR_TRACE    = 8, // tracing message. the lowest priority
    # SOAPY_SDR_SSI      = 9, // streaming status ie Underflow, Overflow

    return parser.parse_args()


def prepare_irises(uut_serials, golden_serial, rm_from_uut_serials):

    handles = SoapySDR.Device.enumerate({"remote:timeout": "250000"})
    if not uut_serials:
        sdrs = SoapySDR.Device(handles)  # Instantiate all devices on the network.
    else:
        uut_serials.append(golden_serial)
        sdrs = [SoapySDR.Device(dict(driver="iris", serial=s))
                for s in uut_serials]

    # Remove non-Iris instances, e.g. hub serial.
    uut_sdrs = list(sdrs)
    for sdr in sdrs:
        if "Iris" not in sdr.getHardwareInfo()["revision"]:
            uut_sdrs.remove(sdr)

    # Find all Iris serials and frontend types.
    uut_serials = [sdr.getHardwareInfo()["serial"] for sdr in uut_sdrs]
    uut_frontends = []
    for sdr in uut_sdrs:
        if "CBRS" in sdr.getHardwareInfo()["frontend"]:
            uut_frontends.append("CBRS")
        else:
            uut_frontends.append("")

    # Pick the golden Iris and remove it from the UUT list.
    if golden_serial == "":
        golden_serial = uut_serials[0]
    index = uut_serials.index(golden_serial)
    golden_sdr = uut_sdrs.pop(index)
    uut_serials.pop(index)

    # Decide on the UUT list.
    for serial in rm_from_uut_serials:
        index = uut_serials.index(serial)
        uut_sdrs.pop(index)
        uut_serials.pop(index)
        uut_frontends.pop(index)

    uut_sdrs = tuple(uut_sdrs)

    return uut_sdrs, uut_serials, uut_frontends, golden_sdr, golden_serial


def set_gain_limits(frontend):
    if frontend == "CBRS":
        if args.freq == 3.6e9:  # Iris module's tx/rx gain limits
            tx_lo = 50  # 15
            tx_hi = 90  # 102
            rx_lo = 50  # 3
            rx_hi = 90  # 102
        else:  # args.freq == 2.5e9
            tx_lo = 50  # 16
            tx_hi = 90  # 93
            rx_lo = 50  # 3
            rx_hi = 90  # 105
    else:  # Dev Board
        tx_lo = 0  # 0
        tx_hi = 42  # 42
        rx_lo = 0  # 0
        rx_hi = 30  # 30

    return tx_lo, tx_hi, rx_lo, rx_hi


def disp_params(freq, rx_pwr_limit, golden_serial, golden_ant, golden_tx_gain,
                golden_rx_gain, gain_step, uut_serials, pf_rx, pf_tx,
                en_logging, detailed_logs, log_name_rx, log_name_tx,
                test_tx_only, test_rx_only):
    print("\n============ Common Irises Parameters ============")
    print("RF frequency (Hz):         {}".format(freq))
    print("RX power limit:            {}".format(rx_pwr_limit))
    print("============== Golden Iris Parameters ==============")
    print("Serial number:             {}".format(golden_serial))
    print("Antenna:                   {}".format(golden_ant))
    print("TX gain:                   {}".format(golden_tx_gain))
    print("RX gain:                   {}".format(golden_rx_gain))
    print("====== Unit Under Test (UUT) Irises Parameters =====")
    print("Gain adjustment step size: {}".format(gain_step))
    print("List of UUT Iris modules:  {}".format(uut_serials))
    print("=============== Final Test Results =================")
    if test_rx_only or (not test_tx_only and not test_rx_only):
        print("Receiver test:             ", "Passed" if pf_rx else "Failed")
        if en_logging:
            print("Receiver test log:         {}".format(log_name_rx))
        if detailed_logs:
            print("Receiver test plot:        {}".
                  format(log_name_rx.replace("csv", "png")))
    if test_tx_only or (not test_tx_only and not test_rx_only):
        print("Transmitter test:          ", "Passed" if pf_tx else "Failed")
        if en_logging:
            print("Transmitter test log:      {}".format(log_name_tx))
        if detailed_logs:
            print("Transmitter test plot:     {}".
                  format(log_name_tx.replace("csv", "png")))


# Receiver log
def process_rx_log(log_name_rx, rx_thresh):
    rx_axis_x0, rx_axis_x1, rx_axis_y0, rx_axis_y1, rx_noise_floor0, \
    rx_noise_floor1, axis_x0_buff, axis_x1_buff, axis_y0_buff, axis_y1_buff,\
    axis_noise0_buff, axis_noise1_buff, rx_serials = \
        [], [], [], [], [], [], [], [], [], [], [], [], [""]
    with open(log_name_rx, mode="r") as log:
        log_reader = csv.DictReader(log, delimiter=",")
        uut_iris = ""
        for row in log_reader:
            if uut_iris != row["UUT_Iris"]:
                # For the first entry.
                uut_iris = row["UUT_Iris"]
                rx_serials.append(uut_iris)
                # Each sublist consists of a x/y axis pair.
                rx_axis_x0.append(axis_x0_buff)
                rx_axis_x1.append(axis_x1_buff)
                rx_axis_y0.append(axis_y0_buff)
                rx_axis_y1.append(axis_y1_buff)
                rx_noise_floor0.append(axis_noise0_buff)
                rx_noise_floor1.append(axis_noise1_buff)
                # Start a new list for each Iris's each channel.
                reached_thresh0, reached_thresh1 = False, False
                axis_x0_buff = [int(row["UUT_Gain"])]
                axis_x1_buff = [int(row["UUT_Gain"])]
                axis_y0_buff = [float(row["fftPwr_dB_A"])]
                axis_y1_buff = [float(row["fftPwr_dB_B"])]
                axis_noise0_buff = [float(row["Noise_Floor_A"])]
                axis_noise1_buff = [float(row["Noise_Floor_B"])]
            else:
                # For the rest of the entries.
                if not reached_thresh0:
                    if float(row["fftPwr_dB_A"]) > rx_thresh:
                        reached_thresh0 = True
                    axis_x0_buff.append(int(row["UUT_Gain"]))
                    axis_y0_buff.append(float(row["fftPwr_dB_A"]))
                    axis_noise0_buff.append(float(row["Noise_Floor_A"]))

                if not reached_thresh1:
                    if float(row["fftPwr_dB_B"]) > rx_thresh:
                        reached_thresh1 = True
                    axis_x1_buff.append(int(row["UUT_Gain"]))
                    axis_y1_buff.append(float(row["fftPwr_dB_B"]))
                    axis_noise1_buff.append(float(row["Noise_Floor_B"]))
                else:
                    reached_thresh1 = True

    rx_axis_x0.append(axis_x0_buff)
    rx_axis_x1.append(axis_x1_buff)
    rx_axis_y0.append(axis_y0_buff)
    rx_axis_y1.append(axis_y1_buff)
    rx_noise_floor0.append(axis_noise0_buff)
    rx_noise_floor1.append(axis_noise1_buff)

    return rx_axis_x0, rx_axis_x1, rx_axis_y0, rx_axis_y1, rx_noise_floor0, \
           rx_noise_floor1, rx_serials


# Transmitter log
def process_tx_log(log_name_tx):
    tx_axis_x0, tx_axis_x1, tx_axis_y0, tx_axis_y1, tx_noise_floor0, \
    tx_noise_floor1, axis_x0_buff, axis_x1_buff, axis_y0_buff, axis_y1_buff, \
    axis_noise0_buff, axis_noise1_buff, tx_serials = \
        [], [], [], [], [], [], [], [], [], [], [], [], [""]
    with open(log_name_tx, mode="r") as log:
        log_reader = csv.DictReader(log, delimiter=",")
        uut_ch = ""
        tx_axis_x0.append(axis_x0_buff)
        tx_axis_y0.append(axis_y0_buff)
        tx_noise_floor0.append(axis_noise0_buff)
        for row in log_reader:
            if uut_ch != row["UUT_Ch"]:
                if row["UUT_Ch"] == "A":
                    tx_axis_x1.append(axis_x1_buff)
                    tx_axis_y1.append(axis_y1_buff)
                    tx_noise_floor1.append(axis_noise1_buff)

                    axis_x0_buff = [float(row["UUT_Gain"])]
                    if row["Ant"] == "A":
                        axis_y0_buff = [float(row["fftPwr_dB_A"])]
                        axis_noise0_buff = [float(row["Noise_Floor_A"])]
                    else:
                        axis_y0_buff = [float(row["fftPwr_dB_B"])]
                        axis_noise0_buff = [float(row["Noise_Floor_B"])]
                    tx_serials.append(row["UUT_Iris"])
                else:
                    tx_axis_x0.append(axis_x0_buff)
                    tx_axis_y0.append(axis_y0_buff)
                    tx_noise_floor0.append(axis_noise0_buff)

                    axis_x1_buff = [float(row["UUT_Gain"])]
                    if row["Ant"] == "A":
                        axis_y1_buff = [float(row["fftPwr_dB_A"])]
                        axis_noise1_buff = [float(row["Noise_Floor_A"])]
                    else:
                        axis_y1_buff = [float(row["fftPwr_dB_B"])]
                        axis_noise1_buff = [float(row["Noise_Floor_B"])]
                    tx_serials.append(row["UUT_Iris"])

                # Start a new list for each Iris's each channel.
                uut_ch = row["UUT_Ch"]
            else:
                if uut_ch == "A":
                    axis_x0_buff.append(float(row["UUT_Gain"]))
                    if row["Ant"] == "A":
                        axis_y0_buff.append(float(row["fftPwr_dB_A"]))
                        axis_noise0_buff.append(float(row["Noise_Floor_A"]))
                    else:
                        axis_y0_buff.append(float(row["fftPwr_dB_B"]))
                        axis_noise0_buff.append(float(row["Noise_Floor_B"]))
                else:
                    axis_x1_buff.append(float(row["UUT_Gain"]))
                    if row["Ant"] == "A":
                        axis_y1_buff.append(float(row["fftPwr_dB_A"]))
                        axis_noise1_buff.append(float(row["Noise_Floor_A"]))
                    else:
                        axis_y1_buff.append(float(row["fftPwr_dB_B"]))
                        axis_noise1_buff.append(float(row["Noise_Floor_B"]))

    if uut_ch == "A":
        tx_axis_x0.append(axis_x0_buff)
        tx_axis_y0.append(axis_y0_buff)
        tx_noise_floor0.append(axis_noise0_buff)
    else:
        tx_axis_x1.append(axis_x1_buff)
        tx_axis_y1.append(axis_y1_buff)
        tx_noise_floor1.append(axis_noise1_buff)

    tx_serials = list(dict.fromkeys(tx_serials))

    return tx_axis_x0, tx_axis_x1, tx_axis_y0, tx_axis_y1, tx_noise_floor0,\
           tx_noise_floor1, tx_serials


def plotter(rx_axis_x0, rx_axis_x1, rx_axis_y0, rx_axis_y1, rx_noise_floor0,
            rx_noise_floor1, rx_serials, tx_axis_x0, tx_axis_x1, tx_axis_y0,
            tx_axis_y1, tx_noise_floor0, tx_noise_floor1, tx_serials,
            test_tx_only, test_rx_only, pf_rx, pf_tx, golden_serial,
            golden_ant, golden_tx_gain, golden_rx_gain, test_datetime,
            rx_thresh, rx_lo, rx_hi, tx_lo, tx_hi):
    axis_y_lo, axis_y_hi = -180, 0

    # Receiver plot
    if test_rx_only or (not test_rx_only and not test_tx_only):
        # fig, axs = plt.subplots(2, 1, constrained_layout=True)
        fig, axs = plt.subplots()
        for i in range(1, len(rx_axis_x0)):
            axs.plot(rx_axis_x0[i], rx_axis_y0[i], label=rx_serials[i] + "-A",
                     marker="*")
            axs.plot(rx_axis_x0[i], rx_noise_floor0[i], linestyle=":",
                     marker="*", label=rx_serials[i] + "-noise-A")
        for i in range(1, len(rx_axis_x1)):
            axs.plot(rx_axis_x1[i], rx_axis_y1[i], label=rx_serials[i] + "-B",
                     linestyle="--", marker=".")
            axs.plot(rx_axis_x1[i], rx_noise_floor1[i], linestyle=":",
                     marker=".", label=rx_serials[i] + "-noise-B")

        axs.set(xlabel="rx_gain (dB)", ylabel="FFT Power from UUTs")
        axs.legend(loc="lower right", ncol=2, fontsize="xx-small")
        axs.axhline(rx_thresh, c="red", ls="--", lw=3, label="PF Threshold")
        axs.grid()
        axs.set_xlim([rx_lo, rx_hi])
        axs.set_ylim([axis_y_lo, axis_y_hi])
        pf = "Passed" if pf_rx else "Failed"
        title = "Iris Health Monitor: RX Test - " + pf + " (" + \
                test_datetime.strftime("%Y/%m/%d %H:%M:%S") + \
                ")\nGolden Iris Params: Serial " + golden_serial + \
                ", TX Ant " + golden_ant + ", TX Gain " + str(golden_tx_gain)
        fig.suptitle(title, fontsize=12, fontweight="bold")
        fig_name = "./data_out/rx_test_" + \
                   test_datetime.strftime("%Y-%m-%d_%H-%M-%S") + ".png"
        fig.savefig(fig_name)

    # Transmitter plot
    if test_tx_only or (not test_rx_only and not test_tx_only):
        fig, axs = plt.subplots()
        for i in range(1, len(tx_axis_x0)):
            axs.plot(tx_axis_x0[i], tx_axis_y0[i], label=tx_serials[i] + "-A",
                     marker="*")
            axs.plot(tx_axis_x0[i], tx_noise_floor0[i], linestyle=":",
                     label=tx_serials[i] + "-noise-A", marker="*")
        for i in range(1, len(tx_axis_x1)):
            axs.plot(tx_axis_x1[i], tx_axis_y1[i], label=tx_serials[i] + "-B",
                     linestyle="--", marker=".")
            axs.plot(tx_axis_x1[i], tx_noise_floor1[i], linestyle=":",
                     marker=".", label=tx_serials[i] + "-noise-B")

        axs.set(xlabel="tx_gain (dB)", ylabel="FFT Power from Golden Unit")
        axs.legend(loc="lower right", ncol=2, fontsize="xx-small")
        axs.axhline(rx_thresh, c="red", ls="--", lw=3, label="PF Threshold")
        axs.grid()
        axs.set_xlim([tx_lo, tx_hi])
        axs.set_ylim([axis_y_lo, axis_y_hi])
        pf = "Passed" if pf_tx else "Failed"
        title = "Iris Health Monitor: TX Test - " + pf + " (" + \
                test_datetime.strftime("%Y/%m/%d %H:%M:%S") + \
                ")\nGolden Iris Params: Serial " + golden_serial + \
                ", RX Ant " + golden_ant + ", RX Gain " + str(golden_rx_gain)
        fig.suptitle(title, fontsize=12, fontweight="bold")
        fig_name = "./data_out/tx_test_" + \
                   test_datetime.strftime("%Y-%m-%d_%H-%M-%S") + ".png"
        fig.savefig(fig_name)
        # plt.close(fig)

    plt.show()


def generate_sine(tx_num_samps, rate, wave_freq, bb_freq, amp):
    # Generate TX signal.
    tx_sig = np.empty(tx_num_samps).astype(np.complex64)
    wbz = tx_sig

    # Generate sine wave.
    ts = 1 / rate
    if wave_freq is None:
        wave_freq = rate / 20
    x = 20
    tx_num_samps = int(x * rate / wave_freq)  # x period of samples
    s_freq = wave_freq
    s_time_vals = np.array(np.arange(0, tx_num_samps)).transpose() * ts
    tx_sig = np.exp(s_time_vals * 1j * 2 * np.pi * s_freq).\
                 astype(np.complex64) * amp
    if bb_freq > 0:
        # Use with cordic.
        tx_sig = np.array([0] * tx_num_samps, np.complex64)
        tx_sig += .1

    # Convert float to fixed point.
    pilot1_ui32 = cfloat2uint32(tx_sig, order="QI")
    pilot2_ui32 = cfloat2uint32(wbz)
    return pilot1_ui32, pilot2_ui32


def print_header(en_logging, log_writer):
    print("PF\tUUT_Iris\tfftPwr_dB_A\tfftPwr_dB_B\tNoise_Floor_A\t"
          "Noise_Floor_B\tUUT_Ch\tUUT_Gain\tUUT_Gain_Lo\tUUT_Gain_Hi\tGolden:"
          " Iris\tAnt\tGain\tPF_Thresh\tTX_LMS7_Temp\tTX_Zynq_Temp\t"
          "RX_Power_dBFS\tRX_Start\t\t\tRX_Stop")


def print_results(pf, uut_serial, fft_pwr_db_1, fft_pwr_db_2, noise_floor_1,
                  noise_floor_2, uut_ch, uut_gain, uut_gain_lo, uut_gain_hi,
                  golden_serial, golden_ant, golden_gain, rx_pwr_limit,
                  lms7_temp, zynq_temp, pwr_dbfs, rx_start, rx_stop,
                  en_logging, log_writer):
    print("{pf}\t{uut_serial}\t{fft_pwr_db_1:.2f}\t\t{fft_pwr_db_2:.2f}\t\t"
          "{noise_floor_1:.2f}\t\t{noise_floor_2:.2f}\t\t{uut_ch}\t{uut_gain}\t\t"
          "{uut_gain_lo}\t\t{uut_gain_hi}\t\t{golden_serial}\t{golden_ant}\t"
          "{golden_gain:.2f}\t{rx_pwr_limit}\t\t{lms7_temp}\t{zynq_temp}\t\t"
          "{pwr_dbfs:.2f}\t\t{rx_start}\t{rx_stop}".
          format(pf=pf, uut_serial=uut_serial, fft_pwr_db_1=fft_pwr_db_1,
                 fft_pwr_db_2=fft_pwr_db_2, noise_floor_1=noise_floor_1,
                 noise_floor_2=noise_floor_2, uut_ch=uut_ch, uut_gain=uut_gain,
                 uut_gain_lo=uut_gain_lo, uut_gain_hi=uut_gain_hi,
                 golden_serial=golden_serial, golden_ant=golden_ant,
                 golden_gain=golden_gain, rx_pwr_limit=rx_pwr_limit,
                 lms7_temp=lms7_temp, zynq_temp=zynq_temp, pwr_dbfs=pwr_dbfs,
                 rx_start=rx_start, rx_stop=rx_stop))

    if en_logging:
        log_writer.writerow(
            [pf, uut_serial, fft_pwr_db_1, fft_pwr_db_2, noise_floor_1,
             noise_floor_2, uut_ch, uut_gain, uut_gain_lo, uut_gain_hi,
             golden_serial, golden_ant, golden_gain, rx_pwr_limit, lms7_temp,
             zynq_temp, pwr_dbfs, rx_start, rx_stop])


def test_a_tx_rx_pair(sdr_tx, sdr_rx, tx_gain, rx_gain, freq, tx_ant,
                      samp_rate=5e6, fft_size=1024, rx_num_samps=1024,
                      wait_trigger=False, tx_amp=1, bb_freq=0, wave_freq=None,
                      tx_num_samps=2**10, lo_tone=False):
    # 1. Transmit
    info = sdr_tx.getHardwareInfo()

    # 1.1 Set Iris RF parameters.
    if tx_ant == "A":    tx_channel = [0]
    elif tx_ant == "B":  tx_channel = [1]
    elif tx_ant == "AB": tx_channel = [0, 1]
    else:                tx_channel = [0]
    amp_fixed = int(tx_amp * (1 << 13))
    for ch in tx_channel:
        sdr_tx.writeSetting(SOAPY_SDR_TX, ch, "ENABLE_CHANNEL", "true")
        sdr_tx.setBandwidth(SOAPY_SDR_TX, ch, 2.5 * samp_rate)
        sdr_tx.setSampleRate(SOAPY_SDR_TX, ch, samp_rate)
        sdr_tx.setFrequency(SOAPY_SDR_TX, ch, "RF", freq - .75 * samp_rate)
        sdr_tx.setFrequency(SOAPY_SDR_TX, ch, "BB", .75 * samp_rate)
        sdr_tx.setAntenna(SOAPY_SDR_TX, ch, "TRX")

        if lo_tone:
            sdr_tx.writeSetting(SOAPY_SDR_TX, ch, "TSP_TSG_CONST",
                                str(amp_fixed))
            sdr_tx.writeSetting(SOAPY_SDR_TX, ch, "TX_ENB_OVERRIDE", "true")

        if "CBRS" in info["frontend"]:
            sdr_tx.setGain(SOAPY_SDR_TX, ch, tx_gain)
        else:
            # No CBRS board gains, only changing LMS7 gains
            sdr_tx.setGain(SOAPY_SDR_TX, ch, "PAD", tx_gain)  # [0:1:42]
            sdr_tx.setGain(SOAPY_SDR_TX, ch, "IAMP", 0)  # [-12:1:3]

    # 1.2 Generate a sine wave signal
    pilot1_ui32, pilot2_ui32 = generate_sine(tx_num_samps, samp_rate,
                                             wave_freq, bb_freq, tx_amp)
    # 1.3 Setup to transmit
    if not lo_tone:
        replay_addr = 0
        if tx_ant == "A":
            sdr_tx.writeRegisters("TX_RAM_A", replay_addr,
                                  pilot1_ui32.tolist())
        elif tx_ant == "B":
            sdr_tx.writeRegisters("TX_RAM_B", replay_addr,
                                  pilot1_ui32.tolist())
        elif tx_ant == "AB":
            sdr_tx.writeRegisters("TX_RAM_A", replay_addr,
                                  pilot1_ui32.tolist())
            sdr_tx.writeRegisters("TX_RAM_B", replay_addr,
                                  pilot1_ui32.tolist())
        # Continuous TX
        sdr_tx.writeSetting("TX_REPLAY", str(tx_num_samps))

    # 2. Receive
    rx_start = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    tx_lo, tx_hi, rx_lo, rx_hi = set_gain_limits(sdr_rx)

    # 2.1 Set params on both RF channels
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

    # 2.2 Setup RX stream
    rx_stream = sdr_rx.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0, 1])

    # 2.3 RSSI read setup
    setUpDigitalRssiMode(sdr_rx)

    # 2.4 Read samples into buffers
    samps_rx = [np.zeros(rx_num_samps, np.complex64),
               np.zeros(rx_num_samps, np.complex64)]
    buff0 = samps_rx[0]  # RF Chain 1
    buff1 = samps_rx[1]  # RF Chain 2

    flags = SOAPY_SDR_END_BURST
    if wait_trigger: flags |= SOAPY_SDR_WAIT_TRIGGER
    sdr_rx.activateStream(rx_stream,
                          flags,
                          0,  # timeNs: Use only if SOAPY_SDR_HAS_TIME.
                          buff0.size)  # numElems = burst size
    sr = sdr_rx.readStream(rx_stream, [buff0, buff1], buff0.size)
    if sr.ret != buff0.size:
       print("Read RX burst of %d, requested %d" % (sr.ret, buff0.size))

    # 2.5 Remove DC.
    for i in [0, 1]:
       samps_rx[i] -= np.mean(samps_rx[i])

    # 2.6 Calculate magnitude of IQ Samples (RX RF channel A).
    i = np.real(samps_rx[0])
    q = np.imag(samps_rx[0])
    iq_mag = np.mean(np.sqrt(i ** 2 + q ** 2))

    # 2.6 Retrieve RSSI measured from digital samples at the LMS7, and
    # convert to PWR in dBm.
    agc_avg = 0
    rssi, pwr_dbfs = getDigitalRSSI(sdr_rx, agc_avg)  # dBFS

    # 2.7 Compute the power of time domain signal.
    sig_rms = np.sqrt(np.mean(samps_rx[0] * np.conj(samps_rx[0])))
    sig_pwr = np.real(sig_rms) ** 2
    sig_pwr_db = 10 * np.log10(sig_pwr)
    sig_pwr_dbm = 10 * np.log10(sig_pwr / 1e-3)

    # 2.8 Compute Power of Frequency Domain Signal (FFT) for channal A
    f1, power_bins, noise_floor_1, peaks_1 = fft_power(samps_rx[0],
                                                     samp_rate,
                                                     num_bins=fft_size,
                                                     peak=1.0,
                                                     scaling="spectrum",
                                                     peak_thresh=20)
    fft_power_1 = bandpower(samps_rx[0], samp_rate, 0, samp_rate / 2)
    if fft_power_1 <= 0: fft_power_1 = 1e-15  # Remove warning
    fft_pwr_db_1 = 10 * np.log10(fft_power_1)
    fft_pwr_dbm_1 = 10 * np.log10(fft_power_1 / 1e-3)

    # 2.9 Compute Power of Frequency Domain Signal (FFT) for channal B
    f2, power_bins_2, noise_floor_2, peaks_2 = fft_power(samps_rx[1],
                                                         samp_rate,
                                                         num_bins=fft_size,
                                                         peak=1.0,
                                                         scaling="spectrum",
                                                         peak_thresh=20)
    fft_power_2 = bandpower(samps_rx[1], samp_rate, 0, samp_rate / 2)
    if fft_power_2 <= 0:
       fft_power_2 = 1e-15  # Remove warning.
    fft_pwr_db_2 = 10 * np.log10(fft_power_2)
    fft_pwr_dbm_2 = 10 * np.log10(fft_power_2 / 1e-3)

    # 2.10 Retrieve RSSI computed in the FPGA.
    rssi_fpga = int(sdr_rx.readRegister("IRIS30",
                                        FPGA_IRIS030_RD_MEASURED_RSSI))
    # Vrms = Vpeak/sqrt(2) (In Volts) - 16 bit value
    vrms_fpga = (rssi_fpga / 2.0 ** 16) * (1 / np.sqrt(2.0))
    # 50 Ohms load (PWRrms in Watts)
    pwr_rms_fpga = (vrms_fpga ** 2.0) / 50.0
    # P(dBm)=10*log10(Prms/1mW)  OR  P(dBm)=10*log10(Prms)+30
    pwr_dbm_fpga = 10.0 * np.log10(pwr_rms_fpga) + 30

    # 3. Stop RX and TX.
    sdr_rx.deactivateStream(rx_stream)
    sdr_rx.closeStream(rx_stream)
    sdr_tx.writeSetting("TX_REPLAY", "")
    for ch in tx_channel:
        sdr_tx.writeSetting(SOAPY_SDR_TX, ch, "ENABLE_CHANNEL", "false")

    # 4. Read chip temperature.
    lms7_temp = sdr_tx.readSensor("LMS7_TEMP")
    zynq_temp = sdr_tx.readSensor("ZYNQ_TEMP")

    rx_stop = datetime.now()

    return rx_start, rx_stop, fft_pwr_db_1, fft_pwr_db_2, pwr_dbfs, lms7_temp,\
           zynq_temp, noise_floor_1, noise_floor_2


def test_receiver(gain_step, freq, uut_sdrs, uut_serials, uut_frontends,
                  golden_sdr, golden_serial, golden_tx_gain, golden_ant,
                  rx_pwr_limit, detailed_logs, en_logging, log_writer):

    # Headers for logs.
    print("\n\nTest Iris receivers: The golden Iris transmits at a fixed gain "
          "{} while each antenna of each UUT Iris sweeps through RX gains and "
          "measures the received power. Stop the sweep when the RX power "
          "passes {} (Pass the test) or the sweep is outside of the RX gain "
          "range (Fail the test).\n\n".format(golden_tx_gain, rx_pwr_limit))
    print_header(en_logging, log_writer)

    # Transmit from the golden Iris and receive on all UUT Iris modules.
    pf = True
    for i in range(len(uut_serials)):
        tx_lo, tx_hi, rx_lo, rx_hi = set_gain_limits(uut_frontends[i])
        ch_1_logged, ch_2_logged, logged = False, False, False
        for rx_gain in range(rx_lo, rx_hi+1, gain_step):
            rx_start, rx_stop, fft_pwr_db_1, fft_pwr_db_2, pwr_dbfs, \
            lms7_temp, zynq_temp, noise_floor_1, noise_floor_2 = \
                test_a_tx_rx_pair(golden_sdr, uut_sdrs[i], golden_tx_gain,
                                  rx_gain, freq, golden_ant)

            if (fft_pwr_db_1>=rx_pwr_limit) and (not ch_1_logged):
                ch_1_logged, logged = True, True
                print_results("Passed", uut_serials[i], fft_pwr_db_1,
                              fft_pwr_db_2, noise_floor_1, noise_floor_2, "A",
                              rx_gain, rx_lo, rx_hi, golden_serial, golden_ant,
                              golden_tx_gain, rx_pwr_limit, lms7_temp,
                              zynq_temp, pwr_dbfs, rx_start, rx_stop,
                              en_logging, log_writer)
            if (fft_pwr_db_2>=rx_pwr_limit) and (not ch_2_logged):
                ch_2_logged, logged = True, True
                print_results("Passed", uut_serials[i], fft_pwr_db_1,
                              fft_pwr_db_2, noise_floor_1, noise_floor_2, "B",
                              rx_gain, rx_lo, rx_hi, golden_serial, golden_ant,
                              golden_tx_gain, rx_pwr_limit, lms7_temp,
                              zynq_temp, pwr_dbfs, rx_start, rx_stop,
                              en_logging, log_writer)
            if (fft_pwr_db_1>=rx_pwr_limit) and (fft_pwr_db_2>=rx_pwr_limit):
                break
            if detailed_logs and not logged:
                print_results("", uut_serials[i], fft_pwr_db_1, fft_pwr_db_2,
                              noise_floor_1, noise_floor_2, "AB", rx_gain,
                              rx_lo, rx_hi, golden_serial, golden_ant,
                              golden_tx_gain, rx_pwr_limit, lms7_temp,
                              zynq_temp, pwr_dbfs, rx_start, rx_stop,
                              en_logging, log_writer)
            logged = False


        if not ch_1_logged:
            pf = pf & False
            print_results("Failed", uut_serials[i], fft_pwr_db_1, fft_pwr_db_2,
                          noise_floor_1, noise_floor_2, "A", rx_gain, rx_lo,
                          rx_hi, golden_serial, golden_ant, golden_tx_gain,
                          rx_pwr_limit, lms7_temp, zynq_temp, pwr_dbfs,
                          rx_start, rx_stop, en_logging, log_writer)
        if not ch_2_logged:
            pf = pf & False
            print_results("Failed", uut_serials[i], fft_pwr_db_1, fft_pwr_db_2,
                          noise_floor_1, noise_floor_2, "B", rx_gain, rx_lo,
                          rx_hi, golden_serial, golden_ant, golden_tx_gain,
                          rx_pwr_limit, lms7_temp, zynq_temp, pwr_dbfs,
                          rx_start, rx_stop, en_logging, log_writer)

    return pf, rx_lo, rx_hi


def test_transmitter(gain_step, freq, uut_sdrs, uut_serials, uut_frontends,
                     golden_sdr, golden_serial, golden_rx_gain, golden_ant,
                     rx_pwr_limit, detailed_logs, en_logging, log_writer):

    # Headers for logs.
    print("\n\nTest Iris transmitter: The golden Iris receives at a fixed gain"
          " {} while each antenna of each UUT Iris sweeps through TX gains and"
          " the golden Iris measures the received power. Stop the sweep when "
          "the RX power passes {} (Pass the test) or the sweep is outside of "
          "the RX gain range (Fail the test). \n\n".
          format(golden_rx_gain, rx_pwr_limit))
    print_header(en_logging, log_writer)

    # Transmit from all UUT Irises with incremental gains one by one
    # from channel A & B. Receive on the golden Iris with a fixed
    # receive gain.
    pf = True
    for i in range(len(uut_serials)):
        tx_lo, tx_hi, rx_lo, rx_hi = set_gain_limits(uut_frontends[i])
        for ch in [0, 1]:
            if ch == 0: tx_ant = "A"
            else:       tx_ant = "B"
            ch_1_logged, ch_2_logged, logged = False, False, False
            for tx_gain in range(tx_lo, tx_hi+1, gain_step):
                rx_start, rx_stop, fft_pwr_db_1, fft_pwr_db_2, pwr_dbfs, \
                lms7_temp, zynq_temp, noise_floor_1, noise_floor_2 = \
                    test_a_tx_rx_pair(uut_sdrs[i], golden_sdr, tx_gain,
                                      golden_rx_gain, freq, tx_ant)

                if golden_ant=="A" and fft_pwr_db_1>=rx_pwr_limit:
                    ch_1_logged, logged = True, True
                    print_results("Passed", uut_serials[i], fft_pwr_db_1, 0,
                                  noise_floor_1, 0, tx_ant, tx_gain, tx_lo,
                                  tx_hi, golden_serial, golden_ant,
                                  golden_rx_gain, rx_pwr_limit, lms7_temp,
                                  zynq_temp, pwr_dbfs, rx_start, rx_stop,
                                  en_logging, log_writer)
                    break
                if golden_ant=="B" and fft_pwr_db_2>=rx_pwr_limit:
                    ch_2_logged, logged = True, True
                    print_results("Passed", uut_serials[i], 0, fft_pwr_db_2, 0,
                                  noise_floor_2, tx_ant, tx_gain, tx_lo, tx_hi,
                                  golden_serial, golden_ant, golden_rx_gain,
                                  rx_pwr_limit, lms7_temp, zynq_temp, pwr_dbfs,
                                  rx_start, rx_stop, en_logging, log_writer)
                    break

                if detailed_logs and not logged:
                    if golden_ant=="A":
                        print_results("", uut_serials[i], fft_pwr_db_1, 0,
                                      noise_floor_1, 0, tx_ant, tx_gain, tx_lo,
                                      tx_hi, golden_serial, golden_ant,
                                      golden_rx_gain, rx_pwr_limit, lms7_temp,
                                      zynq_temp, pwr_dbfs, rx_start, rx_stop,
                                      en_logging, log_writer)
                    if golden_ant=="B":
                        print_results("", uut_serials[i], 0, fft_pwr_db_2, 0,
                                      noise_floor_2, tx_ant, tx_gain, tx_lo,
                                      tx_hi, golden_serial, golden_ant,
                                      golden_rx_gain, rx_pwr_limit, lms7_temp,
                                      zynq_temp, pwr_dbfs, rx_start, rx_stop,
                                      en_logging, log_writer)
                logged = False

            if golden_ant=="A" and not ch_1_logged:
                pf = pf & False
                print_results("Failed", uut_serials[i], fft_pwr_db_1, 0,
                              noise_floor_1, 0, tx_ant, tx_gain, tx_lo, tx_hi,
                              golden_serial, golden_ant, golden_rx_gain,
                              rx_pwr_limit, lms7_temp, zynq_temp, pwr_dbfs,
                              rx_start, rx_stop, en_logging, log_writer)

            if golden_ant == "B" and not ch_2_logged:
                pf = pf & False
                print_results("Failed", uut_serials[i], 0, fft_pwr_db_2, 0,
                              noise_floor_2, tx_ant, tx_gain, tx_lo, tx_hi,
                              golden_serial, golden_ant, golden_rx_gain,
                              rx_pwr_limit, lms7_temp, zynq_temp, pwr_dbfs,
                              rx_start, rx_stop, en_logging, log_writer)

    return  pf, tx_lo, tx_hi


def iris_health_monitor(en_logging, freq, uut_serials, golden_serial, gain_step,
                        golden_tx_gain, golden_ant, golden_rx_gain,
                        rx_pwr_limit, rm_from_uut_serials, detailed_logs,
                        sdr_log_level, test_tx_only, test_rx_only):

    test_start = datetime.now()
    SoapySDR.SoapySDR_setLogLevel(sdr_log_level)

    # 1. Prepare the golden Iris and the UUT Iris serials.
    uut_sdrs, uut_serials, uut_frontends, golden_sdr, golden_serial = \
        prepare_irises(uut_serials, golden_serial, rm_from_uut_serials)
    if en_logging:
        test_datetime = datetime.now()
        log_fieldnames = ["PF", "UUT_Iris", "fftPwr_dB_A", "fftPwr_dB_B",
                          "Noise_Floor_A", "Noise_Floor_B", "UUT_Ch",
                          "UUT_Gain", "UUT_Gain_Lo", "UUT_Gain_Hi",
                          "Golden: Iris", "Ant", "Gain", "PF_Thresh",
                          "TX_LMS7_Temp", "TX_Zynq_Temp", "RX_Power_dBFS",
                          "RX_Start", "RX_Stop"]

    # 2. Receiver Test on all Irises: Transmit from the golden Iris
    # module. Receive on all Irises one by one on each channel A & B.
    if test_rx_only or (not test_tx_only and not test_rx_only):
        if en_logging:
            log_name_rx = "./data_out/rx_test_" + \
                          test_datetime.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
            log_ptr = open(log_name_rx, mode='w')
            log_writer = csv.writer(log_ptr, delimiter=',')
            log_writer.writerow(log_fieldnames)
        else:
            log_writer = ""

        pf_rx, rx_lo, rx_hi = test_receiver(gain_step=gain_step,
                              freq=freq,
                              uut_sdrs=uut_sdrs,
                              uut_serials=uut_serials,
                              uut_frontends=uut_frontends,
                              golden_sdr=golden_sdr,
                              golden_serial=golden_serial,
                              golden_tx_gain=golden_tx_gain,
                              golden_ant=golden_ant,
                              rx_pwr_limit=rx_pwr_limit,
                              detailed_logs=detailed_logs,
                              en_logging=en_logging,
                              log_writer=log_writer)
        print("\nReceiver test: ", "Passed" if pf_rx else "Failed")
        if en_logging:
            log_ptr.close()

        if detailed_logs and en_logging:
            rx_axis_x0, rx_axis_x1, rx_axis_y0, rx_axis_y1, rx_noise_floor0, \
            rx_noise_floor1, rx_serials = \
                process_rx_log(log_name_rx, rx_pwr_limit)

    rx_test_end = datetime.now()

    # 3. Transmitter test: Transmit from all Irises one by one on each
    # channel A & B. Receive on the golden Iris module.
    if test_tx_only or (not test_tx_only and not test_rx_only):
        if en_logging:
            log_name_tx = "./data_out/tx_test_" + \
                          test_datetime.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
            log_ptr = open(log_name_tx, mode='w')
            log_writer = csv.writer(log_ptr, delimiter=',')
            log_writer.writerow(log_fieldnames)
        else:
            log_writer = ""

        pf_tx, tx_lo, tx_hi = test_transmitter(gain_step=gain_step,
                                 freq=freq,
                                 uut_sdrs=uut_sdrs,
                                 uut_serials=uut_serials,
                                 uut_frontends=uut_frontends,
                                 golden_sdr=golden_sdr,
                                 golden_serial=golden_serial,
                                 golden_rx_gain=golden_rx_gain,
                                 golden_ant=golden_ant,
                                 rx_pwr_limit=rx_pwr_limit,
                                 detailed_logs=detailed_logs,
                                 en_logging=en_logging,
                                 log_writer=log_writer)
        if en_logging:
            log_ptr.close()

        if detailed_logs and en_logging:
            tx_axis_x0, tx_axis_x1, tx_axis_y0, tx_axis_y1, tx_noise_floor0,\
            tx_noise_floor1, tx_serials = \
                process_tx_log(log_name_tx)

    tx_test_end = datetime.now()

    # 4. Test Parameters:
    disp_params(freq=freq,
                rx_pwr_limit=rx_pwr_limit,
                golden_serial=golden_serial,
                golden_ant=golden_ant,
                golden_tx_gain=golden_tx_gain,
                golden_rx_gain=golden_rx_gain,
                gain_step=gain_step,
                uut_serials=uut_serials,
                pf_rx=pf_rx,
                pf_tx=pf_tx,
                en_logging=en_logging,
                detailed_logs=detailed_logs,
                log_name_rx=log_name_rx,
                log_name_tx=log_name_tx,
                test_tx_only=test_tx_only,
                test_rx_only=test_rx_only)
    test_end = datetime.now()
    print("==================== Test Times ====================")
    print("Receiver test time:        {}".format(rx_test_end - test_start))
    print("Transmitter test time:     {}".format(tx_test_end - rx_test_end))
    print("Total test time:           {}".format(test_end - test_start))
    print("\nSee the plots ...\n")

    if detailed_logs and en_logging:
        plotter(rx_axis_x0, rx_axis_x1, rx_axis_y0, rx_axis_y1,
                rx_noise_floor0, rx_noise_floor1, rx_serials, tx_axis_x0,
                tx_axis_x1, tx_axis_y0, tx_axis_y1, tx_noise_floor0,
                tx_noise_floor1, tx_serials, test_tx_only, test_rx_only, pf_rx,
                pf_tx, golden_serial, golden_ant, golden_tx_gain,
                golden_rx_gain, test_datetime, rx_pwr_limit, rx_lo, rx_hi,
                tx_lo, tx_hi)


########################################################################
#                                 Main                                 #
########################################################################

if __name__ == "__main__":
    args = comamnd_line()

    iris_health_monitor(
        en_logging=args.en_logging,
        freq=args.freq,
        uut_serials=args.uut_serials,
        golden_serial=args.golden_serial,
        gain_step=args.gain_step,
        golden_tx_gain=args.golden_tx_gain,
        golden_ant=args.golden_ant,
        golden_rx_gain=args.golden_rx_gain,
        rx_pwr_limit=args.rx_power_limit,
        rm_from_uut_serials=args.rm_from_uut_serials,
        detailed_logs=args.en_detailed_logs,
        sdr_log_level=args.sdr_log_level,
        test_tx_only=args.test_tx_only,
        test_rx_only=args.test_rx_only,
    )

