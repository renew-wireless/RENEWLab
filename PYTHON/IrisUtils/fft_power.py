#!/usr/bin/python3
"""
 fft_power.py

    Calculate an estimate of the power spectral density
    employing a Hann window and analyze bins for noise floor and peaks

    INPUT:
        samples     - Numpy array of complex samples
        samp_rate   - Sampling rate
        num_bins    - Take FFTs of this size and average their bins
        peak        - Maximum value of a sample (floats are usually 1.0)
        scaling     - Scaling type. 'density' for power spectrum density
                      (units: V**2/Hz) or 'spectrum' for power spectum
                      (units: V**2)
        peak_thresh - detect peak 'peak_thresh' dBs above the noise floor


    OUTPUT:
        freq        - Frequency index array
        sig_psd     - Array of FFT power bins
        nf          - Noise Floor (dB)
        peaks_found - List of tuples with tone freq and corresponding power


---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import numpy as np
import scipy.signal
import matplotlib.pyplot as plt
from peakdet import *


def fft_power(samples, samp_rate, num_bins=None, peak=1.0, scaling='density', peak_thresh=10):

    if num_bins is None:
        num_iters = 1
        num_bins = len(samples)     # use all samples
    else:
        num_iters = len(samples) // num_bins
        num_bins = num_bins

    # Create a list of bins
    bin_list = list()

    for iter in range(num_iters):

        # Current samples
        samps = samples[iter*num_bins:(iter+1)*num_bins]

        # Length of sample vector
        L = len(samps)

        # Generate window... Default to Hann window
        window = scipy.signal.hann(L)

        # Apply window to signal in time domain
        windowed_signal = np.multiply(window, samps/peak)

        # FFT
        sig_fft = np.fft.fft(windowed_signal)
        sig_psd = np.multiply(sig_fft, np.conjugate(sig_fft))               # abs(sig_fft)**2
        # sig_psd = sig_psd[0: L // 2 + 1]                                  # half of PS

        # Scaling factors
        if scaling == 'density':
            s_val = np.sum(window ** 2)
            sig_psd = sig_psd * (1 / (s_val * samp_rate))                   # multiply by 2 if consider only half of PS
        elif scaling == 'spectrum':
            s_val = window.sum() ** 2
            sig_psd = sig_psd * (1 / s_val)                   # multiply by 2 if consider only half of PS
        else:
            raise ValueError('Unknown scaling. Options: density/spectrum')

        # Clip nulls
        sig_psd = np.maximum(sig_psd, 1e-20)
        # Log scale
        sig_psd = 10 * np.log10(sig_psd)

        # freq index
        freq = np.arange((-samp_rate // 2), (samp_rate // 2), samp_rate/L)  # [-Fs / 2: samp_rate/L: Fs / 2]

        # Reorder bins
        idx = np.argsort(np.fft.fftfreq(L))
        sig_psd = sig_psd[idx]

        # Add bins to Bin List
        bin_list.append(np.exp(sig_psd))

    # Re-assign varialbe if num_bins argument is None
    if num_iters != 1:
        avg_bin_log = np.log(sum(bin_list) / num_bins)
        sig_psd = avg_bin_log

    # Get estimate of noise floor
    nf = np.mean(np.real(sig_psd))

    # Find Peaks
    peaks_found = list()
    # local max/min
    maxMat, _ = peakdet(sig_psd, 20)

    # Remove peaks below nf
    for idx, val in maxMat:
        if val < nf + peak_thresh:
            continue
        freq_loc = (samp_rate * idx) / len(sig_psd) - samp_rate / 2
        peaks_found.append((np.real(freq_loc), np.real(val)))

    return freq, sig_psd, nf, peaks_found


def main():

    # Generate sine wave + noise (2 Vrms sine wave at 100Hz)
    Fs = 10e3       # 10 kHz sampling rate
    freq = 100      # 100 Hz sine
    ampl = 2 * np.sqrt(2)
    noise_power = 0.001 * Fs / 2
    N = 1e5
    tvec = np.arange(N) / Fs
    signal = ampl * np.sin(2 * np.pi * freq * tvec)
    signal = signal + np.random.normal(scale=np.sqrt(noise_power), size=tvec.shape)

    fft_size = None

    # fft_power() function
    f1, powerBins, nf, peaks = fft_power(signal, Fs, num_bins=fft_size, scaling='density', peak_thresh=20)

    # built-in periodogram function
    # Density units V**2/Hz (assuming x measured in V)
    f2, Pxx_den = scipy.signal.periodogram(signal, Fs, nfft=fft_size, window='hann', return_onesided=False, scaling='density')
    # Spectrum units V**2 (assuming x measured in V)
    f3, Pxx_spc = scipy.signal.periodogram(signal, Fs, nfft=fft_size, window='hann', scaling='spectrum')

    print("Peak fft_power: {} \nPeak periodogram-density: {} \nPeak periodogram-spectrum: {}".
          format(np.real(powerBins.max()), (10*np.log10(Pxx_den)).max(), np.sqrt(Pxx_spc.max())))
    print("Noise Floor: {}".format(nf))
    print("Peaks: {}".format(peaks))

    x_val = [x[0] for x in peaks]
    y_val = [x[1] for x in peaks]

    # Plot
    fig = plt.figure(1)
    ax2 = fig.add_subplot(3, 1, 1)
    ax2.grid(True)
    ax2.plot(f1, powerBins)
    ax2.scatter(x_val, y_val, c='r', marker='x')
    plt.xlabel('frequency [Hz]')
    plt.ylabel('PSD [V**2/Hz]')
    ax3 = fig.add_subplot(3, 1, 2)
    ax3.grid(True)
    ax3.plot(f2, 10*np.log10(Pxx_den))
    plt.xlabel('frequency [Hz]')
    plt.ylabel('PSD [V**2/Hz]')
    ax4 = fig.add_subplot(3, 1, 3)
    ax4.grid(True)
    ax4.plot(f3, 10*np.log10(np.sqrt(Pxx_spc)))
    plt.xlabel('frequency [Hz]')
    plt.ylabel('PSD [V**2]')
    plt.show()


if __name__ == '__main__':
    main()