"""
 ofdm_plotter.py

    Hardcoded for two clients. Need to parameterize for variable
    number of clients

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
import signal
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib import animation

# Use GG plotting style
plt.style.use('ggplot')


class OFDMplotter:

    def __init__(self, num_cl):
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)

        self.anim = []
        self.num_cl = num_cl
        self.FIG_LEN = 1000              # captures 2 pilots + data from both users
        self.pilot_len = 640
        self.num_sc = 64
        self.ofdm_syms = 10
        self.tx_data = np.zeros(100)
        self.rx_data = np.zeros(100)
        self.chan_est = []
        self.rx_H_est_plot = self.num_sc
        self.lts_corr = 0 * np.ones(100)
        self.pilot_thresh = 0
        self.rx_syms_mat = []
        self.corr = []
        self.data_syms = []
        self.user_params = []
        self.metadata = []
        self.subframe_size = 640
        self.prefix_len = 160
        self.postfix_len = 160
        self.phase_err = np.zeros(10)
        self.evm = -999
        self.snr = -999
        self.demmel = -999

        self.tx_data2 = np.zeros(100)
        self.rx_data2 = np.zeros(100)
        self.chan_est2 = []
        self.rx_H_est_plot2 = self.num_sc
        self.lts_corr2 = 0 * np.ones(100)
        self.pilot_thresh2 = 0
        self.rx_syms_mat2 = []
        self.corr2 = []
        self.data_syms2 = []
        self.phase_err2 = np.zeros(10)
        self.evm2 = -999
        self.snr2 = -998

        self.frameIdx = 0

        matplotlib.rcParams.update({'font.size': 10})
        self.fig = plt.figure(figsize=(10, 20), dpi=120)
        self.fig.subplots_adjust(hspace=.3, top=.97, bottom=.03, wspace=1)
        self.gs = gridspec.GridSpec(ncols=8, nrows=4)

        self.init_tx_signal(self.FIG_LEN)
        self.init_rx_signal(self.FIG_LEN)
        self.init_phase_err(self.ofdm_syms)
        self.init_corr_peaks(self.pilot_len)
        self.init_frame_corr(self.FIG_LEN)
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
        #self.line_rx_sig2.set_data([], [])
        #self.line_pilot1_start2.set_data([], [])
        #self.line_pilot2_start2.set_data([], [])
        #self.line_payload_start2.set_data([], [])
        self.line_phase_err.set_data([], [])
        self.line_phase_err2.set_data([], [])

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

        # EVM
        self.evm_text.set_text([])
        self.evm_text2.set_text([])

        # Demmel
        self.demmel_text.set_text([])

    def set_data(self, frameIdx, num_cl, tx, rx, chan_est, rx_H_est_plot, lts_corr, pilot_thresh, rx_syms_mat, corr,
                 data_syms, user_params, metadata, phase_error, avg_evm, snr_from_evm, demmelNumber):
        self.num_cl = num_cl
        self.metadata = metadata
        self.frameIdx = frameIdx
        self.tx_data = tx[0]                       # tx[num clients][num samples]
        self.rx_data = rx                          # [numBsAnt, symLen]
        self.chan_est = chan_est[0]                # [numCl][fft size]
        self.rx_H_est_plot = rx_H_est_plot[0]      # rx_H_est_plot[num clients][fft_size]
        self.lts_corr = lts_corr[0, :]             # [numCl, sym_len+fft_size-1]
        self.pilot_thresh = pilot_thresh[0]        # [numCl, numBsAnt]
        self.rx_syms_mat = rx_syms_mat[0]
        self.corr = corr[:, 0]                     # [num frames, numCl]
        self.data_syms = data_syms[0, :]           # tx symbols [numClients, data length]
        self.user_params = user_params
        self.num_sc = int(metadata['FFT_SIZE'])
        self.subframe_size = int(metadata['SYMBOL_LEN_NO_PAD'])
        self.pilot_len = int(metadata['SYMBOL_LEN_NO_PAD'])
        self.prefix_len = int(metadata['PREFIX_LEN'])
        self.postfix_len = int(metadata['POSTFIX_LEN'])
        self.phase_err = phase_error[0]
        self.evm = avg_evm[0]
        self.snr = snr_from_evm[0]
        self.demmel = demmelNumber

        if self.num_cl > 1:
            self.tx_data2 = tx[1]
            self.rx_data2 = rx                          # plot same signal
            self.chan_est2 = chan_est[1]
            self.rx_H_est_plot2 = rx_H_est_plot[1]
            self.lts_corr2 = lts_corr[1, :]
            self.pilot_thresh2 = pilot_thresh[1]
            self.rx_syms_mat2 = rx_syms_mat[1]
            self.corr2 = corr[:, 1]
            self.data_syms2 = data_syms[1, :]
            self.phase_err2 = phase_error[1]
            self.evm2 = avg_evm[1]
            self.snr2 = snr_from_evm[1]

    def ani_update(self, i):
        # TX
        self.line_tx_sig.set_data(range(len(self.tx_data)), np.real(self.tx_data))
        # TX2
        self.line_tx_sig2.set_data(range(len(self.tx_data2)), np.real(self.tx_data2))

        # RX
        subframe_size = self.subframe_size  # 640
        prefix_len = self.prefix_len        # 160
        postfix_len = self.postfix_len      # 160
        self.line_rx_sig.set_data(range(len(self.rx_data)), np.real(self.rx_data))
        if self.num_cl > 1:
            self.line_pilot1_start.set_data(prefix_len * np.ones(100), np.linspace(-0.5, 0.5, num=100))
            self.line_pilot2_start.set_data((prefix_len + subframe_size + postfix_len + prefix_len) * np.ones(100), np.linspace(-0.5, 0.5, num=100))
            self.line_payload_start.set_data((prefix_len+subframe_size+postfix_len+prefix_len+subframe_size+postfix_len+prefix_len) * np.ones(100), np.linspace(-0.5, 0.5, num=100))
        else:
            self.line_pilot1_start.set_data(prefix_len * np.ones(100), np.linspace(-0.5, 0.5, num=100))
            self.line_pilot2_start.set_data((np.nan) * np.ones(100),np.linspace(-0.5, 0.5, num=100))
            self.line_payload_start.set_data(( prefix_len + subframe_size + postfix_len + prefix_len) * np.ones(100), np.linspace(-0.5, 0.5, num=100))

        # Phase error 1
        self.line_phase_err.set_data(range(len(self.phase_err)), self.phase_err)
        # Phase error 2
        self.line_phase_err2.set_data(range(len(self.phase_err2)), self.phase_err2)

        # Pilot correlation plot 1
        self.line_corr_pk.set_data(range(len(self.lts_corr)), self.lts_corr)
        self.line_corr_th.set_data(np.linspace(0.0, len(self.lts_corr), num=100), self.pilot_thresh * np.ones(100))
        # Pilot correlation plot 2
        self.line_corr_pk2.set_data(range(len(self.lts_corr2)), self.lts_corr2)
        self.line_corr_th2.set_data(np.linspace(0.0, len(self.lts_corr2), num=100), self.pilot_thresh2 * np.ones(100))

        # Frame-to-Frame correlation plot 1
        self.line_frame_corr.set_data(range(len(self.corr)), self.corr)
        # Frame-to-Frame correlation plot 2
        self.line_frame_corr2.set_data(range(len(self.corr2)), self.corr2)

        # Constellation plot 1
        self.line_tx_syms.set_data(np.real(self.data_syms), np.imag(self.data_syms))
        self.line_rx_syms.set_data(np.real(self.rx_syms_mat), np.imag(self.rx_syms_mat))
        # Constellation plot 2
        self.line_tx_syms2.set_data(np.real(self.data_syms2), np.imag(self.data_syms2))
        self.line_rx_syms2.set_data(np.real(self.rx_syms_mat2), np.imag(self.rx_syms_mat2))

        # Channel estimate plot 1
        x_ax = (20 / self.num_sc) * np.array(range(-(self.num_sc // 2), (self.num_sc // 2)))
        self.line_chan_est_mag.set_data(x_ax, self.rx_H_est_plot)
        # Channel estimate plot 2
        self.line_chan_est_mag2.set_data(x_ax, self.rx_H_est_plot2)

        # EVM
        self.evm_text.set_text('EVM:{0:.2f}% , SNR:{1:.2f}'.format(100*self.evm, self.snr))
        self.evm_text2.set_text('EVM:{0:.2f}% , SNR:{1:.2f}'.format(100*self.evm2, self.snr2))

        # Demmel
        self.demmel_text.set_text('Demmel Condition Number:{0:.2f}'.format(self.demmel))

    def animate(self):
        self.anim = animation.FuncAnimation(self.fig, self.ani_update, init_func=self.ani_init)
        try:
            plt.show()
        except AttributeError:
            sys.exit()

    def init_tx_signal(self, fig_len_):
        self.FIG_LEN = fig_len_
        ax = self.fig.add_subplot(self.gs[0, 0:4])
        ax.grid(True)
        ax.set_title('TX Signal Client 1', fontsize=10)
        ax.text(0.5, 1, '|', ha="center")
        ax.set_ylabel('Magnitude')
        ax.set_xlabel('Sample index')
        self.line_tx_sig, = ax.plot([], [], color='r', label='RFA', lw=2)
        ax.set_ylim(-1.00, 1.00)
        ax.set_xlim(0, self.FIG_LEN)
        ax.legend(fontsize=10)

        ax = self.fig.add_subplot(self.gs[0, 4:8])
        ax.grid(True)
        ax.set_title('TX Signal Client 2', fontsize=10)
        ax.text(0.5, 1, '|', ha="center")
        ax.set_ylabel('Magnitude')
        ax.set_xlabel('Sample index')
        self.line_tx_sig2, = ax.plot([], [], color='b', label='RFA', lw=2)
        ax.set_ylim(-1.00, 1.00)
        ax.set_xlim(0, self.FIG_LEN)
        ax.legend(fontsize=10)

    def update_tx_signal_fig(self, fig_len_):
        self.FIG_LEN = fig_len_
        ax = self.fig.add_subplot(self.gs[0, 0:4])
        ax.set_xlim(0, self.FIG_LEN)
        ax = self.fig.add_subplot(self.gs[0, 4:8])
        ax.set_xlim(0, self.FIG_LEN)

    def init_rx_signal(self, fig_len_):
        self.FIG_LEN = fig_len_
        ax = self.fig.add_subplot(self.gs[1, 0:4])
        ax.grid(True)
        ax.set_title('RX Signal at BS', fontsize=10)
        ax.set_xlabel('Sample index')
        ax.set_ylabel('Magnitude')
        self.line_rx_sig, = ax.plot([], [], color='r', label='RFA')
        self.line_pilot1_start, = ax.plot([], [], '--k', label='Pilot 1 Start')  # markers
        self.line_pilot2_start, = ax.plot([], [], '--b', label='Pilot 2 Start')  # markers
        self.line_payload_start, = ax.plot([], [], '--g', label='Payload Start')  # markers
        ax.set_ylim(-1, 1)
        ax.set_xlim(0, self.FIG_LEN)
        ax.legend(fontsize=10, loc='upper center', shadow=True, ncol=2)  # bbox_to_anchor=(0.5, 1.00),
        self.demmel_text = ax.text(200, -0.75, 'Demmel Condition Number:{0:.2f}'.format(self.demmel), style='italic', bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 10})

        # ax = self.fig.add_subplot(self.gs[1, 4:8])
        # ax.grid(True)
        # ax.set_title('RX Signal Client 2', fontsize=10)
        # ax.set_xlabel('Sample index')
        # ax.set_ylabel('Magnitude')
        # self.line_rx_sig2, = ax.plot([], [], color='b', label='RFA')
        # self.line_pilot1_start2, = ax.plot([], [], '--k', label='Pilot 1 Start')  # markers
        # self.line_pilot2_start2, = ax.plot([], [], '--b', label='Pilot 2 Start')  # markers
        # self.line_payload_start2, = ax.plot([], [], '--g', label='Payload Start')  # markers
        # ax.set_ylim(-1, 1)
        # ax.set_xlim(0, self.FIG_LEN)
        # ax.legend(fontsize=10, loc='upper center', shadow=True, ncol=4)

    def update_rx_signal_fig(self, fig_len_):
        self.FIG_LEN = fig_len_
        ax = self.fig.add_subplot(self.gs[1, 0:4])
        ax.set_xlim(0, self.FIG_LEN)
        # ax = self.fig.add_subplot(self.gs[1, 4:8])
        # ax.set_xlim(0, self.FIG_LEN)

    def init_phase_err(self, ofdm_syms_):
        self.ofdm_syms = ofdm_syms_
        ax = self.fig.add_subplot(self.gs[1, 4:8])
        ax.grid(True)
        ax.set_title('Phase Error', fontsize=10)
        ax.set_xlabel('OFDM Symbols')
        self.line_phase_err, = ax.plot([], [], color='b', label='Cl0')
        self.line_phase_err2, = ax.plot([], [], '--r', label='Cl1')
        ax.set_ylim(-3, 3)
        ax.set_xlim(0, self.ofdm_syms)
        ax.legend(fontsize=10)

    def update_phaser_err(self, ofdm_syms_):
        self.ofdm_syms = ofdm_syms_
        ax = self.fig.add_subplot(self.gs[1, 4:8])
        ax.grid(True)
        ax.set_xlim(0, self.ofdm_syms)

    def init_corr_peaks(self, pilot_len_):
        self.pilot_len = pilot_len_
        ax = self.fig.add_subplot(self.gs[2, 0:2])
        ax.grid(True)
        ax.set_title('Pilot Correlation Peaks Client 1', fontsize=10)
        ax.set_xlabel('Sample index')
        ax.set_ylabel('')
        self.line_corr_pk, = ax.plot([], [], color='r', label='RFA')
        self.line_corr_th, = ax.plot([], [], '--b', label='Thresh')  # markers
        ax.set_ylim(0, 50)
        ax.set_xlim(0, self.pilot_len)
        ax.legend(fontsize=10)

        ax = self.fig.add_subplot(self.gs[2, 4:6])
        ax.grid(True)
        ax.set_title('Pilot Correlation Peaks Client 2', fontsize=10)
        ax.set_xlabel('Sample index')
        ax.set_ylabel('')
        self.line_corr_pk2, = ax.plot([], [], color='b', label='RFA')
        self.line_corr_th2, = ax.plot([], [], '--r', label='Thresh')  # markers
        ax.set_ylim(0, 50)
        ax.set_xlim(0, self.pilot_len)
        ax.legend(fontsize=10)

    def update_corr_peaks(self, pilot_len_):
        self.pilot_len = pilot_len_
        ax = self.fig.add_subplot(self.gs[2, 0:2])
        ax.set_xlim(0, self.pilot_len)
        ax = self.fig.add_subplot(self.gs[2, 4:6])
        ax.set_xlim(0, self.pilot_len)

    def init_frame_corr(self, fig_len_):
        self.FIG_LEN = fig_len_
        ax = self.fig.add_subplot(self.gs[2, 2:4])
        ax.grid(True)
        ax.set_title('Frame-to-Frame Correlation Client 1', fontsize=10)
        ax.set_xlabel('Frame index')
        ax.set_ylabel('')
        self.line_frame_corr, = ax.plot([], [], marker="o", color="r")
        ax.set_ylim([0, 1.1])
        ax.set_xlim(0, self.FIG_LEN)
        #ax.legend(fontsize=10)

        ax = self.fig.add_subplot(self.gs[2, 6:8])
        ax.grid(True)
        ax.set_title('Frame-to-Frame Correlation Client 2', fontsize=10)
        ax.set_xlabel('Frame index')
        ax.set_ylabel('')
        self.line_frame_corr2, = ax.plot([], [], marker="o", color="b")
        ax.set_ylim([0, 1.1])
        ax.set_xlim(0, self.FIG_LEN)
        #ax.legend(fontsize=10)

    def update_frame_corr(self, fig_len_):
        self.FIG_LEN = fig_len_
        ax = self.fig.add_subplot(self.gs[2, 2:4])
        ax.set_xlim(0, self.FIG_LEN)
        ax = self.fig.add_subplot(self.gs[2, 6:8])
        ax.set_xlim(0, self.FIG_LEN)

    def init_constellation(self):
        ax = self.fig.add_subplot(self.gs[3, 0:2])
        ax.grid(True)
        ax.set_title('TX/RX Constellation Client 1', fontsize=10)
        ax.set_xlabel('')
        ax.set_ylabel('')
        self.line_tx_syms, = ax.plot([], [], 'bo', label='TXSym')
        self.line_rx_syms, = ax.plot([], [], 'rx', label='RXSym')
        self.evm_text = ax.text(-1.5, -1.5, 'EVM:{0:.2f}% SNR:{1:.2f}'.format(100 * self.evm, self.snr), style='italic')  # , bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 10})
        ax.set_ylim(-1.5, 1.5)
        ax.set_xlim(-1.5, 1.5)
        ax.legend(fontsize=10, loc='upper center')

        ax = self.fig.add_subplot(self.gs[3, 4:6])
        ax.grid(True)
        ax.set_title('TX/RX Constellation Client 2', fontsize=10)
        ax.set_xlabel('')
        ax.set_ylabel('')
        self.line_tx_syms2, = ax.plot([], [], 'ro', label='TXSym')
        self.line_rx_syms2, = ax.plot([], [], 'bx', label='RXSym')
        self.evm_text2 = ax.text(-1.5, -1.5, 'EVM:{0:.2f}% SNR:{1:.2f}'.format(100 * self.evm2, self.snr2), style='italic')  # , bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 10})
        ax.set_ylim(-1.5, 1.5)
        ax.set_xlim(-1.5, 1.5)
        ax.legend(fontsize=10, loc='upper center')

    def init_channel_est(self):
        ax = self.fig.add_subplot(self.gs[3, 2:4])
        ax.grid(True)
        ax.set_title('Magnitude Channel Estimates Client 1', fontsize=10)
        ax.set_xlabel('Baseband Freq.')
        ax.set_ylabel('')
        self.line_chan_est_mag, = ax.plot([], [], color='r')
        ax.set_ylim(-2, 10)
        ax.set_xlim(-10, 10)
        #ax.legend(fontsize=10)

        ax = self.fig.add_subplot(self.gs[3, 6:8])
        ax.grid(True)
        ax.set_title('Magnitude Channel Estimates Client 2', fontsize=10)
        ax.set_xlabel('Baseband Freq.')
        ax.set_ylabel('')
        self.line_chan_est_mag2, = ax.plot([], [], color='b')
        ax.set_ylim(-2, 10)
        ax.set_xlim(-10, 10)
        #ax.legend(fontsize=10)

    def signal_handler(sig):
        """
        SIGINT signal handler

        Input:
            None

        Output:
            None
        """
        print("SIG HANDLER!")
        print('Caught signal %d' % sig)
        sys.exit(0)
