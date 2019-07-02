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
import scipy.io as sio  # For .mat format


def find_lts(iq, thresh=0.8, us=1, cp=32, flip=False):
	"""
		Find the indices of LTSs in the input "iq" signal (upsampled by a factor of "up").
		"thresh" sets sensitivity.

		Inputs:
			iq: IQ samples
			thresh: threshold to detect peak
			us: upsampling factor, needed for generate_training_seq() function
			cp: cyclic prefix
			flip: Flag to specify order or LTS sequence.

		Returns:
			best_pk: highest LTS peak,
			lts_pks: the list of all detected LTSs, and
			lts_corr: the correlated signal, multiplied by itself delayed by 1/2 an LTS
	"""
	debug = False

	lts, lts_f = generate_training_seq(preamble_type='lts', cp=cp, upsample=us)
	# lts contains 2.5 64-sample-LTS sequences, we need only one symbol
	lts_tmp = lts[-64:]

	if flip:
		lts_flip = lts_tmp[::-1]
	else:
		lts_flip = lts_tmp

	lts_flip_conj = np.conjugate(lts_flip)
	lts_corr = np.abs(np.convolve(lts_flip_conj, iq/abs(iq))) 	# Equivalent to Matlab's sign function (X/abs(X))
	lts_pks = np.where(lts_corr > (thresh * np.max(lts_corr)))
	lts_pks = np.squeeze(lts_pks)
	x_vec, y_vec = np.meshgrid(lts_pks, lts_pks)
	second_peak_idx, y = np.where((y_vec - x_vec) == len(lts[-64:]))

	# To save mat files
	# sio.savemat('rx_iq_pilot.mat', {'iq_pilot': iq})

	if not second_peak_idx.any():
		if debug:
			print("NO LTS FOUND!")
		best_pk = []
	else:
		best_pk = lts_pks[second_peak_idx[0]]  # Grab only the first packet we have received

	if debug:
		print("LTS: {}, BEST: {}".format(lts_pks, lts_pks[second_peak_idx]))
		fig = plt.figure()
		ax1 = fig.add_subplot(2, 1, 1)
		ax1.grid(True)
		ax1.plot(np.abs(iq))
		ax2 = fig.add_subplot(2, 1, 2)
		ax2.grid(True)
		ax2.plot(np.abs(lts_corr))
		ax2.scatter(lts_pks, 2 * np.ones(len(lts_pks)))
		plt.show()

	return best_pk, lts_pks, lts_corr


if __name__ == '__main__':

	lts, lts_f = generate_training_seq(preamble_type='lts', cp=32, upsample=1)
	find_lts(lts)
