# -*- coding: UTF-8 -*-

"""
 type-conv.py

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import numpy as np
import math


def uint32tocfloat(arr, order='IQ'):
    """
    Convert uint32 (FPGA format) to floating point iq values

    ARGS:
        - arr: Data array
        - order: Whether it is IQ or QI

    RETURNS:
        - corresponding floating point value
    """
    arr_hi = ((np.right_shift(arr, 16).astype(np.int16))/32768.0)
    arr_lo = (np.bitwise_and(arr, 0xFFFF).astype(np.int16))/32768.0
    if order == 'IQ':
        return (arr_hi + 1j*arr_lo).astype(np.complex64)
    else:
        return (arr_lo + 1j*arr_hi).astype(np.complex64)


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
