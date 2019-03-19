#!/usr/bin/python3
"""
 find_lts.py

 Find LTS sequence

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import numpy as np
from generate_sequence import *
import matplotlib
import matplotlib.pyplot as plt


def find_lts(iq, thresh=0.8, us=1, cp=32):
	"""
		Find the indices of LTSs in the input "iq" signal (upsampled by a factor of "up").
		"thresh" sets sensitivity.

		Inputs:
			iq: IQ samples
			thresh: threshold to detect peak
			us: upsampling factor, needed for generate_training_seq() function
			cp: cyclic prefix

		Returns:
			best: highest LTS peak,
			lts_pks: the list of all detected LTSs, and
			lts_corr: the correlated signal, multiplied by itself delayed by 1/2 an LTS
	"""

	lts, lts_f = generate_training_seq(preamble_type='lts', cp=cp, upsample=us)
	lts_flip = np.flip(lts[-64:]) 	# lts contains 2.5 64-sample-LTS sequences, we need only one symbol
	lts_flip_conj = np.conjugate(lts_flip)
	lts_corr = np.abs(np.convolve(lts_flip_conj, np.sign(iq)))
	lts_pks = np.where(lts_corr > (thresh * np.max(lts_corr)))
	lts_pks = np.squeeze(lts_pks)
	x_vec, y_vec = np.meshgrid(lts_pks, lts_pks)
	second_peak_idx, y = np.where((y_vec - x_vec) == len(lts[-64:]))

	if not second_peak_idx.any():
		print("NO LTS FOUND!")
		best = []
	else:
		best = lts_pks[second_peak_idx[0]]  # Grab only the first packet we have received

	return best, lts_pks, lts_corr


if __name__ == '__main__':

	lts, lts_f = generate_training_seq(preamble_type='lts', cp=32, upsample=1)
	find_lts(lts)
