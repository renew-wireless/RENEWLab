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
        self.FIG_LEN = 1520              # captures 2 pilots + data from both users
        self.pilot_len = 550
        self.tx_data = np.zeros(100)
        self.rx_data = np.zeros(100)
        self.chan_est = []
        self.lts_corr = 0 * np.ones(100)
        self.pilot_thresh = 0
        self.rx_syms_mat = []
        self.corr = []
        self.data_syms = []
        self.user_params = []
        self.metadata = []
        self.num_sc = 64
        self.rx_H_est_plot = self.num_sc

        matplotlib.rcParams.update({'font.size': 10})
        self.fig = plt.figure(figsize=(10, 20), dpi=120)
        self.fig.subplots_adjust(hspace=.3, top=.97, bottom=.03, wspace=1)
        self.gs = gridspec.GridSpec(ncols=8, nrows=4)

        self.init_tx_signal()
        self.init_rx_signal()
        self.init_corr_peaks()
        self.init_frame_corr()
        self.init_constellation()
        self.init_channel_est()

    def ani_init(self):
        # Tx plot
        self.line_tx_sig.set_data([], [])

        self.line_tx_sig2.set_data([], [])

        # Rx plot
        self.line_rx_sig.set_data([], [])
        self.line_pilot1_start.set_data([], [])
        self.line_pilot2_start.set_data([], [])
        self.line_payload_start.set_data([], [])

        self.line_rx_sig2.set_data([], [])
        self.line_pilot1_start2.set_data([], [])
        self.line_pilot2_start2.set_data([], [])
        self.line_payload_start2.set_data([], [])

        # Pilot correlation plot
        self.line_corr_pk.set_data([], [])
        self.line_corr_th.set_data([], [])

        self.line_corr_pk2.set_data([], [])
        self.line_corr_th2.set_data([], [])

        # Sample correlation plot
        self.line_frame_corr.set_data([], [])

        self.line_frame_corr2.set_data([], [])

        # Constellation plot
        self.line_tx_syms.set_data([], [])
        self.line_rx_syms.set_data([], [])

        self.line_tx_syms2.set_data([], [])
        self.line_rx_syms2.set_data([], [])

        # Channel estimate plot
        self.line_chan_est_mag.set_data([], [])

        self.line_chan_est_mag2.set_data([], [])

    def set_data(self, tx, rx, chan_est, rx_H_est_plot, lts_corr, pilot_thresh,
                 rx_syms_mat, corr, data_syms, user_params, metadata):
        self.tx_data = tx
        self.rx_data = rx
        self.chan_est = chan_est
        self.rx_H_est_plot = rx_H_est_plot
        self.lts_corr = lts_corr
        self.pilot_thresh = pilot_thresh
        self.rx_syms_mat = rx_syms_mat
        self.corr = corr
        self.data_syms = data_syms
        self.user_params = user_params
        self.metadata = metadata
        self.num_sc = metadata['FFT_SIZE']

    def ani_update(self, i):
        # TX
        x_tx = range(len(self.tx_data))
        y_tx = np.real(self.tx_data)
        self.line_tx_sig.set_data(x_tx, y_tx)
        # TX2
        self.line_tx_sig2.set_data(x_tx, y_tx)


        # RX
        self.line_rx_sig.set_data(range(len(self.rx_data)), np.real(self.rx_data))
        self.line_pilot1_start.set_data(68 * np.ones(100), np.linspace(-0.5, 0.5, num=100))
        self.line_pilot2_start.set_data(68+550 * np.ones(100), np.linspace(-0.5, 0.5, num=100))
        self.line_payload_start.set_data(68+550+550 * np.ones(100), np.linspace(-0.5, 0.5, num=100))
        # RX2
        self.line_rx_sig2.set_data(range(len(self.rx_data)), np.real(self.rx_data))
        self.line_pilot1_start2.set_data(68 * np.ones(100), np.linspace(-0.5, 0.5, num=100))
        self.line_pilot2_start2.set_data(68+550 * np.ones(100), np.linspace(-0.5, 0.5, num=100))
        self.line_payload_start2.set_data(68+550+550 * np.ones(100), np.linspace(-0.5, 0.5, num=100))


        # Pilot correlation plot 1
        self.line_corr_pk.set_data(range(len(self.lts_corr)), self.lts_corr)
        self.line_corr_th.set_data(np.linspace(0.0, len(self.lts_corr), num=100), self.pilot_thresh * np.ones(100))
        # Pilot correlation plot 2
        self.line_corr_pk2.set_data(range(len(self.lts_corr)), self.lts_corr)
        self.line_corr_th2.set_data(np.linspace(0.0, len(self.lts_corr), num=100), self.pilot_thresh * np.ones(100))


        # Frame-to-Frame correlation plot 1
        self.line_frame_corr.set_data(range(len(self.corr)), self.corr)
        # Frame-to-Frame correlation plot 2
        self.line_frame_corr2.set_data(range(len(self.corr)), self.corr)


        # Constellation plot 1
        self.line_tx_syms.set_data(np.real(self.data_syms), np.imag(self.data_syms))
        self.line_rx_syms.set_data(np.real(self.rx_syms_mat), np.imag(self.rx_syms_mat))
        # Constellation plot 2
        self.line_tx_syms2.set_data(np.real(self.data_syms), np.imag(self.data_syms))
        self.line_rx_syms2.set_data(np.real(self.rx_syms_mat), np.imag(self.rx_syms_mat))


        # Channel estimate plot 1
        x_ax = (20 / self.num_sc) * np.array(range(-(self.num_sc // 2), (self.num_sc // 2)))
        self.line_chan_est_mag.set_data(x_ax, self.rx_H_est_plot)
        # Channel estimate plot 2
        self.line_chan_est_mag2.set_data(x_ax, self.rx_H_est_plot)

    def animate(self):
        self.anim = animation.FuncAnimation(self.fig, self.ani_update, init_func=self.ani_init, interval=20, blit=False, repeat=False)
        plt.show()

    def init_tx_signal(self):
            ax = self.fig.add_subplot(self.gs[0, 0:4])
            ax.grid(True)
            ax.set_title('TX Signal Client 1')
            ax.text(0.5, 1, '|', ha="center")
            ax.set_ylabel('Magnitude')
            ax.set_xlabel('Sample index')
            self.line_tx_sig, = ax.plot([], [], color='r', label='RFA', lw=2)
            ax.set_ylim(-0.30, 0.30)
            ax.set_xlim(0, self.FIG_LEN)
            ax.legend(fontsize=10)

            ax = self.fig.add_subplot(self.gs[0, 4:8])
            ax.grid(True)
            ax.set_title('TX Signal Client 2')
            ax.text(0.5, 1, '|', ha="center")
            ax.set_ylabel('Magnitude')
            ax.set_xlabel('Sample index')
            self.line_tx_sig2, = ax.plot([], [], color='b', label='RFA', lw=2)
            ax.set_ylim(-0.30, 0.30)
            ax.set_xlim(0, self.FIG_LEN)
            ax.legend(fontsize=10)

    def init_rx_signal(self):
            ax = self.fig.add_subplot(self.gs[1, 0:4])
            ax.grid(True)
            ax.set_title('RX Signal Client 1')
            ax.set_xlabel('Sample index')
            ax.set_ylabel('Magnitude')
            self.line_rx_sig, = ax.plot([], [], color='r', label='RFA')
            self.line_pilot1_start, = ax.plot([], [], '--k', label='Pilot 1 Start')  # markers
            self.line_pilot2_start, = ax.plot([], [], '--b', label='Pilot 2 Start')  # markers
            self.line_payload_start, = ax.plot([], [], '--g', label='Payload Start')  # markers
            ax.set_ylim(-0.6, 0.6)
            ax.set_xlim(0, self.FIG_LEN)
            ax.legend(fontsize=10)

            ax = self.fig.add_subplot(self.gs[1, 4:8])
            ax.grid(True)
            ax.set_title('RX Signal Client 2')
            ax.set_xlabel('Sample index')
            ax.set_ylabel('Magnitude')
            self.line_rx_sig2, = ax.plot([], [], color='b', label='RFA')
            self.line_pilot1_start2, = ax.plot([], [], '--k', label='Pilot 1 Start')  # markers
            self.line_pilot2_start2, = ax.plot([], [], '--b', label='Pilot 2 Start')  # markers
            self.line_payload_start2, = ax.plot([], [], '--g', label='Payload Start')  # markers
            ax.set_ylim(-0.6, 0.6)
            ax.set_xlim(0, self.FIG_LEN)
            ax.legend(fontsize=10)

    def init_corr_peaks(self):
            ax = self.fig.add_subplot(self.gs[2, 0:2])
            ax.grid(True)
            ax.set_title('Pilot Correlation Peaks Client 1')
            ax.set_xlabel('Sample index')
            ax.set_ylabel('')
            self.line_corr_pk, = ax.plot([], [], color='r', label='RFA')
            self.line_corr_th, = ax.plot([], [], '--b', label='Thresh')  # markers
            ax.set_ylim(0, 5)
            ax.set_xlim(0, self.pilot_len)
            ax.legend(fontsize=10)

            ax = self.fig.add_subplot(self.gs[2, 4:6])
            ax.grid(True)
            ax.set_title('Pilot Correlation Peaks Client 2')
            ax.set_xlabel('Sample index')
            ax.set_ylabel('')
            self.line_corr_pk2, = ax.plot([], [], color='b', label='RFA')
            self.line_corr_th2, = ax.plot([], [], '--r', label='Thresh')  # markers
            ax.set_ylim(0, 5)
            ax.set_xlim(0, self.pilot_len)
            ax.legend(fontsize=10)

    def init_frame_corr(self):
            ax = self.fig.add_subplot(self.gs[2, 2:4])
            ax.grid(True)
            ax.set_title('Frame-to-Frame Correlation Client 1')
            ax.set_xlabel('Frame index')
            ax.set_ylabel('')
            self.line_frame_corr, = ax.plot([], [], marker="o", color="r")
            ax.set_ylim([0, 1.1])
            ax.set_xlim(0, self.FIG_LEN)
            ax.legend(fontsize=10)

            ax = self.fig.add_subplot(self.gs[2, 6:8])
            ax.grid(True)
            ax.set_title('Frame-to-Frame Correlation Client 2')
            ax.set_xlabel('Frame index')
            ax.set_ylabel('')
            self.line_frame_corr2, = ax.plot([], [], marker="o", color="b")
            ax.set_ylim([0, 1.1])
            ax.set_xlim(0, self.FIG_LEN)
            ax.legend(fontsize=10)

    def init_constellation(self):
            ax = self.fig.add_subplot(self.gs[3, 0:2])
            ax.grid(True)
            ax.set_title('TX/RX Constellation Client 1')
            ax.set_xlabel('')
            ax.set_ylabel('')
            self.line_tx_syms, = ax.plot([], [], 'bo', label='TXSym')
            self.line_rx_syms, = ax.plot([], [], 'rx', label='RXSym')
            ax.set_ylim(-1.5, 1.5)
            ax.set_xlim(-2.8, 2.8)
            ax.legend(fontsize=10)

            ax = self.fig.add_subplot(self.gs[3, 4:6])
            ax.grid(True)
            ax.set_title('TX/RX Constellation Client 2')
            ax.set_xlabel('')
            ax.set_ylabel('')
            self.line_tx_syms2, = ax.plot([], [], 'ro', label='TXSym')
            self.line_rx_syms2, = ax.plot([], [], 'bx', label='RXSym')
            ax.set_ylim(-1.5, 1.5)
            ax.set_xlim(-2.8, 2.8)
            ax.legend(fontsize=10)

    def init_channel_est(self):
            ax = self.fig.add_subplot(self.gs[3, 2:4])
            ax.grid(True)
            ax.set_title('Magnitude Channel Estimates Client 1')
            ax.set_xlabel('Baseband Freq.')
            ax.set_ylabel('')
            self.line_chan_est_mag, = ax.step([], [], color='r')
            ax.set_ylim(0, 1.5)
            ax.set_xlim(-10, 10)
            ax.legend(fontsize=10)

            ax = self.fig.add_subplot(self.gs[3, 6:8])
            ax.grid(True)
            ax.set_title('Magnitude Channel Estimates Client 2')
            ax.set_xlabel('Baseband Freq.')
            ax.set_ylabel('')
            self.line_chan_est_mag2, = ax.step([], [], color='b')
            ax.set_ylim(0, 1.5)
            ax.set_xlim(-10, 10)
            ax.legend(fontsize=10)


if __name__ == '__main__':
    # TODO
    cc = plot_hdf('test.hdf5', 0)

