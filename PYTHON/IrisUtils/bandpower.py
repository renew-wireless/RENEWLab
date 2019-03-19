####################################################################################
# bandpower.py
#
# Alternative to Matlab's bandpower function
#
# Source: https://stackoverflow.com/a/44770189
#
####################################################################################

import numpy as np
import math
import scipy.signal


def bandpower(x, fs, fmin, fmax):
    """
        Computes average power in the input x
        :param x: Received Signal
        :param fs: Sampling Rate
        :param fmin: Frequency range (lower end)
        :param fmax: Frequency range (upper end)
        :return: Average power of input signal x
    """
    f, Pxx = scipy.signal.periodogram(x, fs=fs)
    ind_min = scipy.argmax(f > fmin) - 1
    ind_max = scipy.argmax(f > fmax) - 1
    return scipy.trapz(Pxx[ind_min: ind_max], f[ind_min: ind_max])
