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

class MIMODriver:
    def __init__(self, hub_serial, bs_serials, ue_serials, rate,
            tx_freq, rx_freq, tx_gain, rx_gain, bs_channels='A', ue_channels='A', beamsweep=False):
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
        self.bs_obj = [Iris_py(sdr, tx_freq, rx_freq, tx_gain, rx_gain, None, rate, None, bs_channels) for sdr in bs_serials]
        self.ue_obj = [Iris_py(sdr, tx_freq, rx_freq, tx_gain, rx_gain, None, rate, None, ue_channels) for sdr in ue_serials]
        self.hub = None
        if len(hub_serial) != 0:
            self.hub = Hub_py(hub_serial)

        # Setup Radios and Streams
        if self.hub is not None:
            self.hub.sync_delays()
        else:
            self.bs_obj[0].sync_delays()

        [ue.config_gain_ctrl() for ue in self.ue_obj]

        [ue.setup_stream_rx() for ue in self.ue_obj]
        [bs.setup_stream_rx() for bs in self.bs_obj]

        if beamsweep == True:
            [bs.burn_beacon() for bs in self.bs_obj]
            # also write beamweights
        else:
            self.bs_obj[0].burn_beacon()

        self.n_bs_chan = len(bs_channels)
        self.n_ue_chan = len(ue_channels)
        self.n_users = len(ue_serials) * self.n_ue_chan
        self.n_bs_antenna = len(bs_serials) * self.n_bs_chan

    def bs_trigger(self):
        if self.hub is not None:
            self.hub.set_trigger()
        else:
            self.bs_obj[0].set_trigger()

    def reset_frame(self):
        [ue.reset_hw_time() for ue in self.ue_obj]
        [bs.reset_hw_time() for bs in self.bs_obj]

    def close(self):
        [ue.close() for ue in self.ue_obj]
        [bs.close() for bs in self.bs_obj]

    def txrx_uplink(self, tx_data_mat_re, tx_data_mat_im, num_frames, nsamps_pad=160, max_try=10):
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
        [ue.config_sdr_tdd(is_bs=False, tdd_sched=str(ue_sched[i]), nsamps=n_samps) for i, ue in enumerate(self.ue_obj)]

        for i, ue in enumerate(self.ue_obj):
            ue.burn_data_complex(tx_data_mat[:, i] if self.n_users > 1 else tx_data_mat)    # for matlab
            #ue.burn_data_complex(tx_data_mat[:, i])   # for python

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
                rx_data_frame = [bs.recv_stream_tdd() for bs in self.bs_obj]
                amp = np.mean(np.abs(rx_data_frame[0]))
                good_signal = amp > 0.001
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
                    rx_data[good_frame_id, :, :, :] = np.reshape(np.array(rx_data_frame[0]), (self.n_bs_antenna, numRxSyms, n_samps))
                else:
                    rx_data[good_frame_id, :, :, :] = np.reshape(np.array(rx_data_frame), (self.n_bs_antenna, numRxSyms, n_samps))
                good_frame_id = good_frame_id + 1
                #tmptmp = np.array(rx_data_frame[0])
                #plt.plot(np.abs(tmptmp))

        #plt.show()
        [ue.unset_corr() for ue in self.ue_obj]
        self.reset_frame()
        return rx_data, good_frame_id, numRxSyms


    def txrx_downlink(self, tx_data_mat_re, tx_data_mat_im, num_frames, nsamps_pad=160, max_try=10):
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
            # Individual Pilots (downlink)
            curr_str_bs = 'PG'
            bs_sched_b = bs_sched_b + curr_str_bs
            bs_sched = bs_sched + curr_str_bs
            tmp = copy.copy(ue_sched_tmp)
            tmp[2*ue_idx:2*ue_idx+2] = 'RG'
            tmpstr = 'GG' + ''.join([char for char in tmp])
            if n_users > 1:
                # Multi-user TX (downlink)
                tmpstr = tmpstr + 'RG'

            ue_sched.append(tmpstr)

        # Multi-user RX (downlink)
        if n_users > 1:
            bs_sched_b = bs_sched_b + 'PG'
            bs_sched = bs_sched + 'PG'

        numRxSyms = bs_sched.count('P')
        print("NumRxSyms: {}, n_samps: {}, n_users: {}, bs_sched_b: {}, bs_sched: {}, ue_sched: {}".format(numRxSyms,n_samps,n_users,bs_sched_b,bs_sched,ue_sched))

        ### Write TDD Schedule ###
        [bs.config_sdr_tdd(tdd_sched=str(bs_sched_b) if i == 0 else str(bs_sched), nsamps=n_samps, prefix_len=nsamps_pad) for i, bs in enumerate(self.bs_obj)]
        [ue.config_sdr_tdd(is_bs=False, tdd_sched=str(ue_sched[i]), nsamps=n_samps, prefix_len=nsamps_pad) for i, ue in enumerate(self.ue_obj)]

        for i, bs in enumerate(self.bs_obj):
            bs.burn_data_complex(tx_data_mat[:, i] if self.n_bs_antenna > 1 else tx_data_mat)

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
                self.bs_trigger()
                # Receive Data
                rx_data_frame = [ue.recv_stream_tdd() for ue in self.ue_obj]
                amp = np.mean(np.abs(rx_data_frame[0]))
                good_signal = amp > 0.001
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
                    rx_data[good_frame_id, :, :, :] = np.reshape(np.array(rx_data_frame[0]), (self.n_bs_antenna, numRxSyms, n_samps))
                else:
                    rx_data[good_frame_id, :, :, :] = np.reshape(np.array(rx_data_frame), (self.n_bs_antenna, numRxSyms, n_samps))
                good_frame_id = good_frame_id + 1
                #tmptmp = np.array(rx_data_frame[0])
                #plt.plot(np.abs(tmptmp))

        #plt.show()
        [ue.unset_corr() for ue in self.ue_obj]
        self.reset_frame()

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

    def update_sdr_params(self, param, param_val):
        # Update params on all boards, might want to create two separate functions, one for BS SDRs and one for UE SDRs
        if param == 'txgain':
            [ue.sdr_settxgain(param_val) for ue in self.ue_obj]
            [bs.sdr_settxgain(param_val) for bs in self.bs_obj]
        elif param == 'rxgain':
            [ue.sdr_setrxgain(param_val) for ue in self.ue_obj]
            [bs.sdr_setrxgain(param_val) for bs in self.bs_obj]
        else:
            print("Invalid Parameter: No action taken!")
        return


#########################################
#                  Main                 #
#########################################
def main():
    parser = OptionParser()
    parser.add_option("--hub", type="string", dest="hub", help="serial number of the hub device", default="FH4B000019")
    parser.add_option("--bs-serials", type="string", dest="bs_serials", help="serial numbers of the BS devices", default='RF3E000146')
    parser.add_option("--ue-serials", type="string", dest="ue_serials", help="serial numbers of the UE devices", default='RF3D000016')
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--freq", type="float", dest="freq", help="Tx freq (Hz). POWDER users must set to 3.6e9", default=3.6e9)
    parser.add_option("--tx-gain", type="float", dest="tx_gain", help="Optional Tx gain (dB)", default=81.0)
    parser.add_option("--rx-gain", type="float", dest="rx_gain", help="Optional Rx gain (dB)", default=60.0)
    (options, args) = parser.parse_args()
    mimo = MIMODriver(
        hub_serial=options.hub,
        bs_serials=options.bs_serials.split(','),
        ue_serials=options.ue_serials.split(','),
        rate=options.rate,
        tx_freq=options.freq,
        rx_freq=options.freq,
        tx_gain=options.tx_gain,
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

    #ul_rx_data, n_ul_good, numRxSyms = mimo.txrx_uplink(np.real(wb_pilot1), np.imag(wb_pilot1), 5, len(pad2))
    #print("Uplink Rx Num {}, ShapeRxData: {}, NumRsyms: {}".format(n_ul_good, ul_rx_data.shape, numRxSyms))

    dl_rx_data, n_dl_good, numRxSyms = mimo.txrx_downlink(np.real(wb_pilot1), np.imag(wb_pilot1), 1, len(pad2))
    print("Downlink Rx Num {}".format(n_dl_good))

    # 10 frames
    #[txg, rxg, max_val, is_valid] = mimo.set_opt_gains(10)
    #print("BEST GAIN COMBO: TX {} / RX {} with MAX # BEACONS: {}".format(txg,rxg,max_val))
    # Test gain change
    #mimo.update_sdr_params('txgain', 30)
    #mimo.update_sdr_params('rxgain', 30)
    mimo.close()

if __name__ == '__main__':
    main()
