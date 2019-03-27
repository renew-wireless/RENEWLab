#!/usr/bin/python3
"""
 plotter.py


---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
import struct
import numpy as np
import os
import math
import h5py
import time
import signal
import pdb
import datetime
import logging
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib import animation
from detect_peaks import detect_peaks

# Use GG plotting style
plt.style.use('ggplot')


class Plotter:

    def __init__(self, plot_vec, num_cl):
        self.num_plots = len(plot_vec)
        self.FIG_LEN = 2000              # captures 2 pilots + data from both users
        self.pilot_len = 550
        self.tx_data = np.zeros(100)
        self.rx_data = np.zeros(100)
        self.chan_est = []
        self.lts_corr = 0 * np.ones(100)
        self.pilot_thresh = 0
        self.rx_syms_mat = []
        self.corr = []
        self.user_params = []
        self.metadata = []
        self.num_sc = 64
        self.rx_H_est_plot = self.num_sc

        matplotlib.rcParams.update({'font.size': 10})
        self.fig = plt.figure(figsize=(10, 20), dpi=120)
        self.fig.subplots_adjust(hspace=.4, top=.97, bottom=.03)
        self.gs = gridspec.GridSpec(ncols=4, nrows=4)

        self.init_tx_signal()
        self.init_rx_signal()
        self.init_corr_peaks()
        self.init_frame_corr()
        self.init_constellation()
        self.init_channel_est()

    def ani_init(self):
        # Tx plot
        self.line_tx_sig.set_data([], [])
        # Rx plot
        self.line_rx_sig.set_data([], [])
        self.line_pilot1_start.set_data([], [])
        self.line_pilot2_start.set_data([], [])
        self.line_payload_start.set_data([], [])
        # Pilot correlation plot
        self.line_corr_pk.set_data([], [])
        self.line_corr_th.set_data([], [])
        # Sample correlation plot
        self.line_frame_corr.set_data([], [])
        # Constellation plot
        self.line_tx_syms.set_data([], [])
        self.line_rx_syms.set_data([], [])
        # Channel estimate plot
        self.line_chan_est_mag.set_data([], [])

    def set_data(self, tx, rx, chan_est, rx_H_est_plot, lts_corr, pilot_thresh, rx_syms_mat, corr, user_params, metadata):
        self.tx_data = tx
        self.rx_data = rx
        self.chan_est = chan_est
        self.rx_H_est_plot = rx_H_est_plot
        self.lts_corr = lts_corr
        self.pilot_thresh = pilot_thresh
        self.rx_syms_mat = rx_syms_mat
        self.corr = corr
        self.user_params = user_params
        self.metadata = metadata
        self.num_sc = metadata['FFT_SIZE']

    def ani_update(self, i):
        # TX
        x_tx = range(len(self.tx_data))
        y_tx = np.real(self.tx_data)
        self.line_tx_sig.set_data(x_tx, y_tx)

        # RX
        x_rx = range(len(self.rx_data))
        y_rx = np.real(self.rx_data)
        self.line_rx_sig.set_data(x_rx, y_rx)
        self.line_pilot1_start.set_data(68 * np.ones(100), np.linspace(-0.5, 0.5, num=100))
        self.line_pilot2_start.set_data(68+550 * np.ones(100), np.linspace(-0.5, 0.5, num=100))
        self.line_payload_start.set_data(68+550+550 * np.ones(100), np.linspace(-0.5, 0.5, num=100))

        # Pilot correlation plot
        x_rx = range(len(self.lts_corr))
        y_rx = self.lts_corr

        self.line_corr_pk.set_data(x_rx, y_rx)
        self.line_corr_th.set_data(np.linspace(0.0, len(self.lts_corr), num=100), self.pilot_thresh * np.ones(100))

        # Frame-to-Frame correlation plot
        self.line_frame_corr.set_data(range(len(self.corr)), self.corr)

        # Constellation plot
        self.line_tx_syms.set_data([], [])
        self.line_rx_syms.set_data(np.real(self.rx_syms_mat), np.imag(self.rx_syms_mat))

        # Channel estimate plot
        x_ax = (20 / self.num_sc) * np.array(range(-(self.num_sc // 2), (self.num_sc // 2)))
        self.line_chan_est_mag.set_data(x_ax, self.rx_H_est_plot)

    def animate(self):
        self.anim = animation.FuncAnimation(self.fig, self.ani_update, init_func=self.ani_init, interval=20, blit=False)
        plt.show()

    def init_tx_signal(self):
            ax = self.fig.add_subplot(self.gs[0, :])
            ax.grid(True)
            ax.set_title('TX Signal')
            ax.text(0.5, 1, '|', ha="center")
            ax.set_ylabel('Magnitude')
            ax.set_xlabel('Sample index')
            self.line_tx_sig, = ax.plot([], [], label='RFA', lw=2)
            ax.set_ylim(-0.5, 0.5)
            ax.set_xlim(0, self.FIG_LEN)
            ax.legend(fontsize=10)

    def init_rx_signal(self):
            ax = self.fig.add_subplot(self.gs[1, :])
            ax.grid(True)
            ax.set_title('RX Signal')
            ax.set_xlabel('Sample index')
            ax.set_ylabel('Magnitude')
            self.line_rx_sig, = ax.plot([], [], label='RFA')
            self.line_pilot1_start, = ax.plot([], [], '--k', label='Pilot 1 Start')  # markers
            self.line_pilot2_start, = ax.plot([], [], '--b', label='Pilot 2 Start')  # markers
            self.line_payload_start, = ax.plot([], [], '--g', label='Payload Start')  # markers
            ax.set_ylim(-0.6, 0.6)
            ax.set_xlim(0, self.FIG_LEN)
            ax.legend(fontsize=10)

    def init_corr_peaks(self):
            ax = self.fig.add_subplot(self.gs[2, 0:2])
            ax.grid(True)
            ax.set_title('Pilot Correlation Peaks')
            ax.set_xlabel('Sample index')
            ax.set_ylabel('')
            self.line_corr_pk, = ax.plot([], [], label='RFA')
            self.line_corr_th, = ax.plot([], [], '--b', label='Thresh')  # markers
            ax.set_ylim(0, 5)
            ax.set_xlim(0, self.pilot_len)
            ax.legend(fontsize=10)

    def init_frame_corr(self):
            ax = self.fig.add_subplot(self.gs[2, 2:4])
            ax.grid(True)
            ax.set_title('Frame-to-Frame Correlation')
            ax.set_xlabel('Frame index')
            ax.set_ylabel('')
            self.line_frame_corr, = ax.plot([], [], marker="o", color="g")
            ax.set_ylim([0, 1.1])
            ax.set_xlim(0, self.FIG_LEN)
            ax.legend(fontsize=10)

    def init_constellation(self):
            ax = self.fig.add_subplot(self.gs[3, 0:2])
            ax.grid(True)
            ax.set_title('TX/RX Constellation')
            ax.set_xlabel('')
            ax.set_ylabel('')
            self.line_tx_syms, = ax.plot([], [], 'ro', label='TXSym')
            self.line_rx_syms, = ax.plot([], [], 'bx', label='RXSym')
            ax.set_ylim(-1.5, 1.5)
            ax.set_xlim(-2.8, 2.8)
            ax.legend(fontsize=10)

    def init_channel_est(self):
            ax = self.fig.add_subplot(self.gs[3, 2:4])
            ax.grid(True)
            ax.set_title('Magnitude Channel Estimates')
            ax.set_xlabel('Baseband Freq.')
            ax.set_ylabel('')
            self.line_chan_est_mag, = ax.step([], [])
            ax.set_ylim(-0.1, 3)
            ax.set_xlim(-10, 10)
            ax.legend(fontsize=10)


if __name__ == '__main__':
    cc = plot_hdf('test.hdf5', 0)
    cc.plot_debug(frame=0, length=frame_length)
    cc.plot_corr(frame=0, length=frame_length)
    cc.plot_csi_new(length=frame_length, subcarrier=2)
