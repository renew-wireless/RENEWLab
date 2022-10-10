#!/usr/bin/python
# -*- coding: UTF-8 -*-
from argparse import ArgumentParser
from optparse import OptionParser
import copy
import numpy as np
from numpy import matlib
import scipy.io as sio
import time
import iris_py
from iris_py import *
import matplotlib.pyplot as plt
from scipy.io import savemat

class MIMODriver:
    def __init__(self, hub_serial, bs_serials, ue_serials, ref_serials, rate,
            tx_freq, rx_freq, tx_gain, tx_gain_ue, rx_gain, bs_channels='A', ue_channels='A', beamsweep=False, agc_en=False, trig_offset=235):
        # Init Radios
        #bs_serials_str = ""
        #for c in bs_serials:
        #    bs_serials_str += c
        #ue_serials_str = ""
        #for c in ue_serials:
        #    ue_serials_str += c
        #bs_serials_list = bs_serials.split(",")
        #ue_serials_list = ue_serials.split(",")

        print("HUB: {}".format(hub_serial))
        print("BS NODES: {}".format(bs_serials))
        print("UE NODES: {}".format(ue_serials))
        print("REF NODES: {}".format(ref_serials))
        self.bs_obj = [Iris_py(sdr, tx_freq, rx_freq, tx_gain, rx_gain, None, rate, None, bs_channels, agc_en, trig_offset) for sdr in bs_serials]

        if np.isscalar(tx_gain_ue):
            self.ue_obj = [Iris_py(sdr, tx_freq, rx_freq, tx_gain_ue, rx_gain, None, rate, None, ue_channels, agc_en, trig_offset) for sdr in ue_serials]
        else:
            self.ue_obj = [Iris_py(sdr, tx_freq, rx_freq, tx_gain_ue[i], rx_gain, None, rate, None, ue_channels, agc_en, trig_offset) for i, sdr in enumerate(ue_serials)]

        self.hub = None
        if len(hub_serial) != 0:
            self.hub = Hub_py(hub_serial)

        self.ref_obj = None
        if len(ref_serials) != 0:
            self.ref_obj = [Iris_py(sdr, tx_freq, rx_freq, tx_gain, rx_gain, None, rate, None, 'A', agc_en, trig_offset) for sdr in ref_serials]
        else:
            self.n_refs = len(ref_serials)

        # Setup Radios and Streams
        if self.hub is not None:
            self.hub.sync_delays()
        else:
            self.bs_obj[0].sync_delays()

        [ue.config_gain_ctrl() for ue in self.ue_obj]

        [ue.setup_stream_rx() for ue in self.ue_obj]
        [bs.setup_stream_rx() for bs in self.bs_obj]
        [ref.setup_stream_rx() for ref in self.ref_obj]
        if beamsweep == True:
            [bs.burn_beacon() for bs in self.bs_obj]
            # also write beamweights
        else:
            self.bs_obj[0].burn_beacon()

        self.n_bs_chan = len(bs_channels)
        self.n_ue_chan = len(ue_channels)
        self.n_users = len(ue_serials) * self.n_ue_chan
        self.n_bs_antenna = len(bs_serials) * self.n_bs_chan
        self.n_bs_sdrs = len(bs_serials)


    def bs_trigger(self):
        if self.hub is not None:
            self.hub.set_trigger()
        else:
            self.bs_obj[0].set_trigger()

    def reset_frame(self):
        [ue.reset_hw_time() for ue in self.ue_obj]
        [bs.reset_hw_time() for bs in self.bs_obj]
        [ref.reset_hw_time() for ref in self.ref_obj]

    def close(self):
        [ue.close() for ue in self.ue_obj]
        [bs.close() for bs in self.bs_obj]
        [ref.close() for ref in self.ref_obj]

    def timing_cal(self, tx_data_mat_re, tx_data_mat_im, nsamps_pad=160, num_frames=1):
        '''Calibrate timing offset due to base station chain length difference'''
        tx_data_mat = tx_data_mat_re + 1j * tx_data_mat_im
        n_samps = tx_data_mat.shape[0]

        ### Build TDD Schedule ###
        ref_sched = 'GGPGG'
        bs_sched  = 'GGRGG'
        numRxSyms = bs_sched.count('R')

        ### Write TDD Schedule ###
        [bs.config_sdr_tdd(tdd_sched=str(bs_sched), nsamps=n_samps, prefix_len=nsamps_pad) for i, bs in enumerate(self.bs_obj)]
        [ref.config_sdr_tdd(is_bs=True, tdd_sched=str(ref_sched), nsamps=n_samps, prefix_len=nsamps_pad) for i, ref in enumerate(self.ref_obj)]   # ref node is considered BS node

        for i, ref in enumerate(self.ref_obj):
            if python_mode:
                ref.burn_data_complex(tx_data_mat[:, i])   # for python
            else:
                ref.burn_data_complex(tx_data_mat[:, i] if self.n_refs > 1 else tx_data_mat)    # for matlab

        [bs.activate_stream_rx() for bs in self.bs_obj]

        rx_data = np.empty((num_frames, self.n_bs_antenna, numRxSyms, n_samps), dtype=np.complex64)

        for frame in range(num_frames):
            self.bs_trigger()
            # Receive Data
            rx_data_frame = [bs.recv_stream_tdd() for bs in self.bs_obj]  # Returns dimensions (num bs nodes, num channels, num samples)
            #rx_data_frame = np.array(rx_data_frame)
            #print("Dimensions: {}, {}, {}".format(rx_data_frame.shape, rx_data_frame[0].shape, rx_data_frame[0][0].shape))
            rx_data[frame, :, :, :] = np.reshape(np.array(rx_data_frame[0][0]), (self.n_bs_antenna, numRxSyms, n_samps))

        good_frame_id = num_frames   # dummy
        self.reset_frame()
        #savemat("./testtest.mat", {'dataOBCH': rx_data, 'txdataOBCH': tx_data_mat, 'rxdataOBCH': rx_data_frame})
        return rx_data, good_frame_id, numRxSyms

    def txrx_uplink(self, tx_data_mat_re, tx_data_mat_im, num_frames, nsamps_pad=160, max_try=50, python_mode=False):
        tx_data_mat = tx_data_mat_re + 1j * tx_data_mat_im
        n_samps = tx_data_mat.shape[0]

        if len(tx_data_mat.shape) > 1:
            n_users = tx_data_mat.shape[1]
        else:
            n_users = 1
        if n_users != self.n_users:
            print("Input data size (dim 2) does not match number of UEs!")
            return
        if n_samps > 4096:
            print("Input data size (dim 1) exceeds pilot buffer size!")
            return

        ### Build TDD Schedule ###
        bs_sched_b = 'BG'   # Beacon node
        bs_sched   = 'GG'
        ue_sched_tmp = list(''.join([char*2*n_users for char in 'G']))
        ue_sched = []
        for ue_idx in range(n_users):
            # Individual Pilots (uplink)
            curr_str_bs = 'RG'
            bs_sched_b = bs_sched_b + curr_str_bs
            bs_sched = bs_sched + curr_str_bs
            tmp = copy.copy(ue_sched_tmp)
            tmp[2*ue_idx:2*ue_idx+2] = 'PG'
            tmpstr = 'GG' + ''.join([char for char in tmp])
            if n_users > 1:
                # Multi-user TX (uplink)
                tmpstr = tmpstr + 'PG'

            ue_sched.append(tmpstr)

        # Multi-user RX (uplink)
        if n_users > 1:
            bs_sched_b = bs_sched_b + 'RG'
            bs_sched = bs_sched + 'RG'

        numRxSyms = bs_sched.count('R')

        ### Write TDD Schedule ###
        [bs.config_sdr_tdd(tdd_sched=str(bs_sched_b) if i == 0 else str(bs_sched), nsamps=n_samps, prefix_len=nsamps_pad) for i, bs in enumerate(self.bs_obj)]
        [ue.config_sdr_tdd(is_bs=False, tdd_sched=str(ue_sched[i]), nsamps=n_samps, prefix_len=nsamps_pad) for i, ue in enumerate(self.ue_obj)]

        for i, ue in enumerate(self.ue_obj):
            if python_mode:
                ue.burn_data_complex(tx_data_mat[:, i])   # for python
            else:
                ue.burn_data_complex(tx_data_mat[:, i] if self.n_users > 1 else tx_data_mat)    # for matlab

        [bs.activate_stream_rx() for bs in self.bs_obj]
        [ue.set_corr() for ue in self.ue_obj]

        rx_data = np.empty((num_frames, self.n_bs_antenna, numRxSyms, n_samps), dtype=np.complex64)
        #rx_data_frame = np.empty((self.n_bs_antenna, n_samps), dtype=np.complex64)
        good_frame_id = 0
        #old_triggers = []
        #triggers = []
        #for u in range(self.n_users):
        #    old_triggers.append(0)

        for frame in range(num_frames):
            all_triggered = False
            good_signal = False
            amp = 0
            #if frame > 0:
            old_triggers = []
            triggers = []
            for u in range(self.n_users):
                old_triggers.append(0)
            for i in range(max_try):
                self.bs_trigger()
                # Receive Data
                rx_data_frame_tmp = [bs.recv_stream_tdd() for bs in self.bs_obj]  # Returns dimensions (num bs nodes, num channels, num samples)
                rx_data_frame = np.array(rx_data_frame_tmp)
                rx_data_frame = rx_data_frame[:,0:2]

                # Verify Data
                if self.n_bs_chan > 1:
                    both_channels = True
                else:
                    both_channels = False
                good_signal = self.verify_signal_quality(rx_data_frame_tmp, self.n_bs_sdrs, both_channels)

                #amp = np.mean(np.abs(rx_data_frame[0][0]))
                #amp = np.max(np.abs(rx_data_frame[0][0]))
                #good_signal = amp > 0.01
                
                triggers = [ue.sdr_gettriggers() for ue in self.ue_obj]
                new_triggers = []
                zip_triggers = zip(triggers, old_triggers)
                for triggers_i, old_triggers_i in zip_triggers:
                    new_triggers.append(triggers_i - old_triggers_i)
                print("triggers = {}, Amplitude = {}".format(new_triggers, amp))
                all_triggered = True
                for u in range(n_users):
                    if (new_triggers[u] == 0):
                        all_triggered = False
                if all_triggered and good_signal:
                    break

            print("frame = {}, tries = {}, all_triggred = {}, good_signal = {}, amp = {}".format(frame, i + 1, all_triggered, good_signal, amp))
            if all_triggered and good_signal:
                if n_users == 1 and self.n_bs_antenna == 1:
                    rx_data[good_frame_id, :, :, :] = np.reshape(np.array(rx_data_frame[0][0]), (self.n_bs_antenna, numRxSyms, n_samps))
                else:
                    #rx_data[good_frame_id, :, :, :] = np.reshape(np.array(rx_data_frame[j][]), (self.n_bs_antenna, numRxSyms, n_samps))
                    rx_data_frame_arr = np.array(rx_data_frame)
                    ant_cnt = 0
                    for j in range(self.n_bs_sdrs):
                        for k in range(self.n_bs_chan):
                            # Dimensions of rx_data: (num_good_frames, self.n_bs_antenna, numRxSyms, n_samps)
                            tmp = rx_data_frame_arr[j, k][:]
                            rx_data[good_frame_id, ant_cnt, :, :] = np.reshape(tmp, (numRxSyms, n_samps))
                            ant_cnt = ant_cnt + 1

                    

                good_frame_id = good_frame_id + 1
                #tmptmp = np.array(rx_data_frame[0])
                #plt.plot(np.abs(tmptmp))

        #plt.show()
        [ue.unset_corr() for ue in self.ue_obj]
        self.reset_frame()
        return rx_data, good_frame_id, numRxSyms


    def txrx_downlink(self, tx_data_mat_re, tx_data_mat_im, num_frames, nsamps_pad=160, max_try=30, python_mode=False):

        tx_data_mat = tx_data_mat_re + 1j * tx_data_mat_im

        ################################
        print(tx_data_mat.shape)
        print(len(tx_data_mat.shape))
        print(tx_data_mat.shape[0])
        ################################

        n_users = self.n_users
        n_bs_antenna = self.n_bs_antenna
        n_bs_nodes = self.n_bs_sdrs

        if len(tx_data_mat.shape) == 1:
            n_samps = tx_data_mat.shape[0]
        else:
            n_samps = tx_data_mat.shape[1]

        if n_samps > 4096:
            print("Input data size exceeds pilot buffer size!")
            return

        ### Build TDD Schedule ###
        # Each channel has its own separate ram. So you can write different data into them. 
        # When dual pilot is enabled, what happens is that we write 0 to the second half of RAM_A and to the first half of RAM_B. 
        # And that's how we get orthogonal pilots.
        # To send from both antennas, simply use 'PG', write data to both, and enable both channels.
        bs_sched_b = 'BGPG'   # Beacon node
        bs_sched   = 'GGPG'
        ue_sched_tmp = 'GGRG' #list(''.join([char*2*n_users for char in 'G']))
        ue_sched = []

        for ue_idx in range(n_users):
            ue_sched.append(ue_sched_tmp)

        # UE
        #tmp = copy.copy(ue_sched_tmp)
        #curr_str_bs = 'PG'
        #tmp[2*ue_idx:2*ue_idx+2] = 'RG'
        # BS
        #bs_sched_b = bs_sched_b + curr_str_bs
        #bs_sched = bs_sched + curr_str_bs
        # UE
        #tmpstr = 'GG' + ''.join([char for char in tmp])
        #ue_sched.append(tmpstr)

        print("Base Station Schedule (B)")
        print(bs_sched_b)
        print("Base Station Schedule")
        print(bs_sched)
        print("UE Schedule")
        print(ue_sched)

        numRxSyms = bs_sched.count('P')
        print("NumRxSyms: {}, n_samps: {}, n_users: {}, bs_sched_b: {}, bs_sched: {}, ue_sched: {}".format(numRxSyms,n_samps,n_users,bs_sched_b,bs_sched,ue_sched))

        if self.n_bs_chan > 1:
            dual_pilot = True
        else:
            dual_pilot = False

        ### Write TDD Schedule ###
        [bs.config_sdr_tdd(tdd_sched=str(bs_sched_b) if i == 0 else str(bs_sched), nsamps=n_samps, prefix_len=nsamps_pad, dualpilot=dual_pilot) for i, bs in enumerate(self.bs_obj)]
        [ue.config_sdr_tdd(is_bs=False, tdd_sched=str(ue_sched[i]), nsamps=n_samps, prefix_len=nsamps_pad) for i, ue in enumerate(self.ue_obj)]

        for i, bs in enumerate(self.bs_obj):
            bs.burn_data_complex(tx_data_mat[i, :] if n_bs_antenna > 1 else tx_data_mat)

        [ue.activate_stream_rx() for ue in self.ue_obj]
        [ue.set_corr() for ue in self.ue_obj]

        rx_data = np.empty((num_frames, n_users, numRxSyms, n_samps), dtype=np.complex64)
        good_frame_id = 0

        for frame in range(num_frames):
            all_triggered = False
            good_signal = False
            amp = 0
            #if frame > 0:
            old_triggers = []
            triggers = []
            for u in range(self.n_users):
                old_triggers.append(0)
            for i in range(max_try):
                self.bs_trigger()
                # Receive Data
                rx_data_frame_tmp = [ue.recv_stream_tdd() for ue in self.ue_obj]
                rx_data_frame = np.array(rx_data_frame_tmp)
                rx_data_frame = rx_data_frame[:,0:2]

                #if i == 0:
                #    savemat("./testtest.mat", {'rxdataOBCH': rx_data_frame})

                # Verify Data
                if self.n_ue_chan > 1:
                    both_channels = True
                else:
                    both_channels = False
                good_signal = self.verify_signal_quality(rx_data_frame_tmp, self.n_users, both_channels)

                amp = np.mean(np.abs(rx_data_frame[0][0]))
                #good_signal = amp > 0.001
                triggers = [ue.sdr_gettriggers() for ue in self.ue_obj]
                new_triggers = []
                zip_triggers = zip(triggers, old_triggers)
                for triggers_i, old_triggers_i in zip_triggers:
                    new_triggers.append(triggers_i - old_triggers_i)
                print("triggers = {}, Amplitude = {}".format(new_triggers, amp))
                all_triggered = True
                for u in range(n_users):
                    if (new_triggers[u] == 0):
                        all_triggered = False
                if all_triggered and good_signal:
                    break

            print("frame = {}, tries = {}, all_triggred = {}, good_signal = {}, amp = {}".format(frame, i + 1, all_triggered, good_signal, amp))
            if all_triggered and good_signal:
                if n_users == 1 and n_bs_antenna == 1:
                    rx_data[good_frame_id, :, :, :] = np.reshape(np.array(rx_data_frame[0][0]), (n_bs_antenna, numRxSyms, n_samps))
                else:
                    rx_data[good_frame_id, :, :, :] = np.reshape(np.array(rx_data_frame[0][0]), (n_users, numRxSyms, n_samps))  # FIXME: Will require changes for multiple users (similar to uplink but for UEs)
                good_frame_id = good_frame_id + 1
                #tmptmp = np.array(rx_data_frame[0])
                #plt.plot(np.abs(tmptmp))

        #plt.show()
        [ue.unset_corr() for ue in self.ue_obj]
        self.reset_frame()

        return rx_data, good_frame_id, numRxSyms


    def txrx_dl_sound(self, tx_data_mat_re, tx_data_mat_im, num_frames, nsamps_pad=160, max_try=30, python_mode=False):
        tx_data_mat = tx_data_mat_re + 1j * tx_data_mat_im
        n_samps = tx_data_mat.shape[0]

        if len(tx_data_mat.shape) > 1:
            n_users = tx_data_mat.shape[1]
        else:
            n_users = 1
        if n_users != self.n_users:
            print("Input data size (dim 2) does not match number of UEs!")
            return
        if n_samps > 4096:
            print("Input data size (dim 1) exceeds pilot buffer size!")
            return

        ### Build TDD Schedule ###
        bs_sched_tmp = list(''.join([char*2*(self.n_bs_sdrs) for char in 'G']))
        ue_sched_tmp = list(''.join([char*2*(self.n_bs_sdrs) for char in 'G']))
        tmp_ue = copy.copy(ue_sched_tmp)

        bs_sched = []
        ue_sched = []

        if self.n_bs_chan == 1:
            curr_str_bs = 'PG'
            curr_str_ue = 'RG'
        elif self.n_bs_chan == 2:
            curr_str_bs = 'PP'
            curr_str_ue = 'RR'
        else:
            print("Invalid number of RF channels (must be 1 or 2)!")
            return

        print("Base Station Schedule")
        for bs_idx in range(self.n_bs_sdrs):
            # Individual Pilots (downlink)
            tmp_bs = copy.copy(bs_sched_tmp)
            tmp_bs[2*bs_idx:2*bs_idx+2] = curr_str_bs
            tmp_ue[2*bs_idx:2*bs_idx+2] = curr_str_ue

            if bs_idx == 0:
                tmp_bs = 'BG' + ''.join([char for char in tmp_bs])
            else:
                tmp_bs = 'GG' + ''.join([char for char in tmp_bs])

            print(tmp_bs)
            bs_sched.append(tmp_bs)

        tmp_ue = 'GG' + ''.join([char for char in tmp_ue])
        print("UE Schedule")
        print(tmp_ue)
        ue_sched.append(tmp_ue)
        numRxSyms = ue_sched[0].count('R')
        print("NumRxSyms: {}, n_samps: {}, n_users: {}, bs_sched: {}, ue_sched: {}".format(numRxSyms,n_samps,n_users,bs_sched,ue_sched))

        ### Write TDD Schedule ###
        [bs.config_sdr_tdd(tdd_sched=str(bs_sched[i]), nsamps=n_samps, prefix_len=nsamps_pad) for i, bs in enumerate(self.bs_obj)]
        [ue.config_sdr_tdd(is_bs=False, tdd_sched=str(ue_sched[i]), nsamps=n_samps, prefix_len=nsamps_pad) for i, ue in enumerate(self.ue_obj)]

        for i, bs in enumerate(self.bs_obj):
            if python_mode:
                bs.burn_data_complex(tx_data_mat[0])  # For python
            else:
                bs.burn_data_complex(tx_data_mat)     # for matlab


        [ue.activate_stream_rx() for ue in self.ue_obj]
        [ue.set_corr() for ue in self.ue_obj]

        rx_data = np.empty((num_frames, self.n_users, numRxSyms, n_samps), dtype=np.complex64)
        good_frame_id = 0

        for frame in range(num_frames):
            all_triggered = False
            good_signal = False
            amp = 0
            #if frame > 0:
            old_triggers = []
            triggers = []
            for u in range(self.n_users):
                old_triggers.append(0)

            for i in range(max_try):
                new_triggers = []
                for u in range(self.n_users):
                    new_triggers.append(0)

                self.bs_trigger()
                # Receive Data
                rx_data_frame_tmp = [ue.recv_stream_tdd() for ue in self.ue_obj]
                rx_data_frame = np.array(rx_data_frame_tmp)
                rx_data_frame = rx_data_frame[:,0:2]

                #if i == 0:
                #    savemat("./testtest.mat", {'rxdataOBCH': rx_data_frame})

                # Verify Data
                if self.n_ue_chan > 1:
                    both_channels = True
                else:
                    both_channels = False
                good_signal = self.verify_signal_quality(rx_data_frame_tmp, self.n_users, both_channels)

                amp = np.mean(np.abs(rx_data_frame[0][0]))
                #good_signal = amp > 0.001
                triggers = [ue.sdr_gettriggers() for ue in self.ue_obj]

                all_triggered = True
                for u in range(self.n_users):
                    new_triggers[u] = triggers[u] - old_triggers[u]
                    old_triggers[u] = triggers[u]
                    if (new_triggers[u] == 0):
                        all_triggered = False
                print("Try: {}, Total Triggers: {}, New Trigs: {}".format(i,triggers,new_triggers))
                if all_triggered and good_signal:
                    break;

            print("frame = {}, tries = {}, all_triggred = {}, good_signal = {}, amp = {}".format(frame, i + 1, all_triggered, good_signal, amp))
            if all_triggered and good_signal:
                #print("ARRAY SIZE: {}, {}, {}".format(rx_data_frame.size, (rx_data_frame[0]).size, (rx_data_frame[0][0]).size))
                if n_users == 1 and self.n_bs_antenna == 1:
                    rx_data[good_frame_id, :, :, :] = np.reshape(np.array(rx_data_frame[0][0]), (self.n_users, numRxSyms, n_samps))
                else:
                    rx_data[good_frame_id, :, :, :] = np.reshape(np.array(rx_data_frame[0][0]), (self.n_users, numRxSyms, n_samps))  # FIXME: Will require changes for multiple users and multiple antennas
                good_frame_id = good_frame_id + 1
                #tmptmp = np.array(rx_data_frame[0])
                #plt.plot(np.abs(tmptmp))

        #plt.show()
        [ue.unset_corr() for ue in self.ue_obj]
        self.reset_frame()

        return rx_data, good_frame_id, numRxSyms


    def txrx_refnode(self, tx_data_mat_re, tx_data_mat_im, num_frames, bs_sched, ue_sched, nsamps_pad=160, max_try=20, python_mode=False):
        print("[PYTHON DRIVER] TX RX REF NODE SYNC: VIA HUB...")
        tx_data_mat = tx_data_mat_re + 1j * tx_data_mat_im
        n_samps = tx_data_mat.shape[0]

        n_users = 1
        numRxSyms = bs_sched.count('R')

        ### Write TDD Schedule ###
        [bs.config_sdr_tdd(tdd_sched=str(bs_sched), nsamps=n_samps, prefix_len=nsamps_pad) for i, bs in enumerate(self.bs_obj)]
        [ue.config_sdr_tdd(is_bs=True, tdd_sched=str(ue_sched), nsamps=n_samps, prefix_len=nsamps_pad) for i, ue in enumerate(self.ue_obj)]   # ref node is considered BS node

        for i, ue in enumerate(self.ue_obj):
            if python_mode:
                ue.burn_data_complex(tx_data_mat[:, i])   # for python
            else:
                ue.burn_data_complex(tx_data_mat[:, i] if self.n_users > 1 else tx_data_mat)    # for matlab

        [bs.activate_stream_rx() for bs in self.bs_obj]
        #[ue.set_corr() for ue in self.ue_obj]

        rx_data = np.empty((num_frames, self.n_bs_antenna, numRxSyms, n_samps), dtype=np.complex64)

        for frame in range(num_frames):
            self.bs_trigger()
            # Receive Data
            rx_data_frame = [bs.recv_stream_tdd() for bs in self.bs_obj]  # Returns dimensions (num bs nodes, num channels, num samples)
            #rx_data_frame = np.array(rx_data_frame)
            #print("Dimensions: {}, {}, {}".format(rx_data_frame.shape, rx_data_frame[0].shape, rx_data_frame[0][0].shape))
            rx_data[frame, :, :, :] = np.reshape(np.array(rx_data_frame[0][0]), (self.n_bs_antenna, numRxSyms, n_samps))
 
        good_frame_id = num_frames   # dummy
        self.reset_frame()
        #savemat("./testtest.mat", {'dataOBCH': rx_data, 'txdataOBCH': tx_data_mat, 'rxdataOBCH': rx_data_frame})
        return rx_data, good_frame_id, numRxSyms


    def set_opt_gains(self, num_frames):
        print("Testing gain combinations to find best settings...")
        n_samps = 4096
        [bs.config_sdr_tdd(tdd_sched="BGGG" if i == 0 else "GGGG", nsamps=n_samps, prefix_len=0) for i, bs in enumerate(self.bs_obj)] 
        [ue.config_sdr_tdd(is_bs=False, tdd_sched="GGGG", nsamps=n_samps) for ue in self.ue_obj]

        [ue.activate_stream_rx() for ue in self.ue_obj]
        [ue.set_corr() for ue in self.ue_obj]

        txgainvec = range(50,81,2)
        rxgainvec = range(50,81,2)
        txg = -999
        rxg = -999
        current_max = 0
        valid = False
        # trig_cnt_arr = np.zeros((len(txgainvec), len(rxgainvec), self.n_users))
        
        old_triggers = np.zeros((1, self.n_users))
        for rxg_cnt, rxgain in enumerate(rxgainvec):
            # Set RX gain at BS
            [bs.sdr_setrxgain(rxgain) for bs in self.bs_obj]

            for txg_cnt, txgain in enumerate(txgainvec):
                print("Set Gains to = TX {}, RX {}".format(txgain, rxgain))
                # Set TX gain at UE
                [ue.sdr_settxgain(txgain) for ue in self.ue_obj]
                time.sleep(0.10)

                new_triggers = np.zeros((1, self.n_users))
                for frame in range(num_frames):
                    self.bs_trigger()
                    triggers = np.array([ue.sdr_gettriggers() for ue in self.ue_obj])
                    # print("frame = {}, triggers = {}".format(frame, triggers))

                new_triggers = triggers - old_triggers
                old_triggers = triggers
                # trig_cnt_arr[txg_cnt, rxg_cnt, :] = triggers

                if not any(element == 0 for element in new_triggers) and sum(new_triggers) >= current_max:
                    # All UEs got beacon and the new sum of triggers is higher than previous iter
                    current_max = sum(new_triggers)
                    txg = txgain
                    rxg = rxgain
                    valid = True
                    print("MAX = {}, GAINS[TX/RX] = {}/{}".format(current_max, txg, rxg))

        [ue.unset_corr() for ue in self.ue_obj]
        self.reset_frame()
        return txg, rxg, current_max, valid

    def update_sdr_params(self, param, param_val, is_bs):
        # Update params on all boards, might want to create two separate functions, one for BS SDRs and one for UE SDRs
        if param == 1:
            if is_bs:
                print("UPDATE TX GAIN DOWNLINK!!")
                [bs.sdr_settxgain(param_val) for bs in self.bs_obj]
            else:
                [ue.sdr_settxgain(param_val) for ue in self.ue_obj]
        elif param == 2:
            if is_bs:
                [bs.sdr_setrxgain(param_val) for bs in self.bs_obj]
            else:
                [ue.sdr_setrxgain(param_val) for ue in self.ue_obj]
        else:
            print("Invalid Parameter: No action taken!")
        return


    def verify_signal_quality(self, rx_data_frame, n_sdrs, both_channels):
        # Verify Data
        low_signal_a = False
        low_signal_b = False
        good_signal = True
        for isdr in range(n_sdrs):
            low_signal_a = rx_data_frame[isdr][2]   # Second output
            low_signal_b = rx_data_frame[isdr][3]
            print("LOW SIGNAL A {}, LOW SIGNAL B {}, BOTH CHAN? {}".format(low_signal_a, low_signal_b, both_channels))
            # If the sounding pilot from any BS antenna is low, count as bad and re-try
            if low_signal_a is True or (low_signal_b is True and both_channels is True):
                print("Bad Signal At Board {}".format(isdr))
                good_signal = False
                break

        return good_signal


#########################################
#                  Main                 #
#########################################
def main():
    parser = OptionParser()
    parser.add_option("--hub", type="string", dest="hub", help="serial number of the hub device", default="FH4B000019")
    parser.add_option("--bs-serials", type="string", dest="bs_serials", help="serial numbers of the BS devices", default='RF3E000654,RF3E000458') #default='RF3E000146,RF3E000356,RF3E000546')
    parser.add_option("--ue-serials", type="string", dest="ue_serials", help="serial numbers of the UE devices", default='RF3E000760')
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--freq", type="float", dest="freq", help="Tx freq (Hz). POWDER users must set to 3.6e9", default=3.6e9)
    parser.add_option("--tx-gain", type="float", dest="tx_gain", help="Optional Tx gain (dB)", default=81.0)
    parser.add_option("--tx-gain-ue", type="float", dest="tx_gain_ue", help="Optional Tx gain (dB)", default=[81.0, 75.0])
    parser.add_option("--rx-gain", type="float", dest="rx_gain", help="Optional Rx gain (dB)", default=65.0)
    (options, args) = parser.parse_args()
    mimo = MIMODriver(
        hub_serial=options.hub,
        bs_serials=options.bs_serials.split(','),
        ue_serials=options.ue_serials.split(','),
        rate=options.rate,
        tx_freq=options.freq,
        rx_freq=options.freq,
        tx_gain=options.tx_gain,
        tx_gain_ue=options.tx_gain_ue,
        rx_gain=options.rx_gain
    )
    nsamps = 1024
    nsamps_pad = 82
    n_sym_samp = nsamps + 2*nsamps_pad - 14
    
    ltsSym, lts_f = gen_lts(cp=32, upsample=1)
    
    # to comprensate for front-end group delay
    pad1 = np.zeros((nsamps_pad), np.complex64)
    # to comprensate for rf path delay
    pad2 = np.zeros((nsamps_pad-14), np.complex64)
    
    wb_pilot = np.tile(ltsSym, nsamps//len(ltsSym)).astype(np.complex64)*.5
    wbz = np.zeros((n_sym_samp), dtype=np.complex64)
    wb_pilot1 = np.concatenate([pad1, wb_pilot, pad2])
    wb_pilot1 = np.transpose(np.matlib.repmat(wb_pilot1, len(options.ue_serials.split(',')), 1))
    wb_pilot2 = wbz  # wb_pilot1 if both_channels else wbz

    test_uplink = False
    if test_uplink:
        ul_rx_data, n_ul_good, numRxSyms = mimo.txrx_uplink(np.real(wb_pilot1), np.imag(wb_pilot1), 5, len(pad2), python_mode=True)
        print("Uplink Rx Num {}, ShapeRxData: {}, NumRsyms: {}".format(n_ul_good, ul_rx_data.shape, numRxSyms))

    test_downlink = False
    if test_downlink:
        dl_rx_data, n_dl_good, numRxSyms = mimo.txrx_downlink(np.real(wb_pilot1), np.imag(wb_pilot1), 1, len(pad2), python_mode=True)
        print("Downlink Rx Num {}".format(n_dl_good))

    test_sounding = True
    if test_sounding:
        snd_rx_data, n_snd_good, numRxSyms = mimo.txrx_dl_sound(np.real(wb_pilot1), np.imag(wb_pilot1), 1, len(pad2), python_mode=True)
        print("Sounding (Downlink) Rx Num {}".format(n_snd_good))

    test_hubSync = False
    if test_hubSync:
        snd_rx_data, n_snd_good, numRxSyms = mimo.txrx_refnode(np.real(wb_pilot1), np.imag(wb_pilot1), 3, "GGGGGRG", "GGGGGPG", len(pad2), python_mode=True)
        print("Sounding (Downlink) Rx Num {}".format(n_snd_good))

    # 10 frames
    #[txg, rxg, max_val, is_valid] = mimo.set_opt_gains(10)
    #print("BEST GAIN COMBO: TX {} / RX {} with MAX # BEACONS: {}".format(txg,rxg,max_val))
    # Test gain change
    #mimo.update_sdr_params('txgain', 30)
    #mimo.update_sdr_params('rxgain', 30)
    mimo.close()

if __name__ == '__main__':
    main()
