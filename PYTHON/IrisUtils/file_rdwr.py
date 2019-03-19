"""
 file_rdwr.py

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import pickle
import sys
import numpy as np


def write_to_file(name, arr, num_bits=12):
    """
    Save complex numpy array val to files prefixed with name in twos-complement binary format with num_bits.
    """
    fi = open(name+'.bin', 'wb')
    for a in arr:
        # fi.write(np.binary_repr(a,width=num_bits))
        pickle.dump(a, fi)
    fi.close()


def read_from_file(name, leng, offset, num_bits=12):
    """
    Read complex numpy array from file prefixed with name in twos-complement binary format with num_bits.
    """
    fi = open(name+'.bin', 'rb')
    for k in range(offset):
        pickle.load(fi)
    arr = np.array([0]*leng, np.uint32)
    for a in range(leng):
        # fi.write(np.binary_repr(a,width=num_bits))
        arr[a] = pickle.load(fi)
        # print("Sample: %d, batch: %d, frame: %d, subframe: %d, count %d" % (a, a/600, np.right_shift(arr[a], 16),
        # np.right_shift(np.bitwise_and(arr[a], 0xF000),12),np.bitwise_and(arr[a], 0xFFF)))
    fi.close()
    return arr
