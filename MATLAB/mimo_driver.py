#!/usr/bin/python
# -*- coding: UTF-8 -*-
from argparse import ArgumentParser
from optparse import OptionParser
import scipy.io as sio
import iris_py
from iris_py import *

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
        print(hub_serial)
        print(bs_serials)
        print(ue_serials)
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

    def txrx_uplink(self, tx_data_mat, num_frames, nsamps_pad=160, max_try=10):
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
        [bs.config_sdr_tdd(tdd_sched="BGRG" if i == 0 else "GGRG", nsamps=n_samps, prefix_len=nsamps_pad) for i, bs in enumerate(self.bs_obj)] 
        [ue.config_sdr_tdd(is_bs=False, tdd_sched="GGPG", nsamps=n_samps) for ue in self.ue_obj]

        for i, ue in enumerate(self.ue_obj):
            ue.burn_data_complex(tx_data_mat[:, i] if self.n_users > 1 else tx_data_mat)

        [bs.activate_stream_rx() for bs in self.bs_obj]
        [ue.set_corr() for ue in self.ue_obj]

        rx_data = np.empty((num_frames, self.n_bs_antenna, n_samps), dtype=np.complex64)
        rx_data_frame = np.empty((self.n_bs_antenna, n_samps), dtype=np.complex64)
        good_frame_id = 0
        old_triggers = []
        triggers = []
        for u in range(self.n_users):
            old_triggers.append(0)

        for frame in range(num_frames):
            all_triggered = True
            good_signal = True
            amp = 0
            if frame > 0:
                old_triggers = triggers
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
                rx_data[good_frame_id] = rx_data_frame
                good_frame_id = good_frame_id + 1
        [ue.unset_corr() for ue in self.ue_obj]
        self.reset_frame()
        return rx_data, good_frame_id

    def txrx_downlink(self, tx_data_mat, num_frames, nsamps_pad=160, max_try=10):
        n_samps = tx_data_mat.shape[0]
        if len(tx_data_mat.shape) > 1:
            n_bs_ant = tx_data_mat.shape[1]
        else:
            n_bs_ant = 1
        if n_bs_ant != self.n_bs_antenna:
            print("Input data size (dim 2) does not match number of BS antennas!")
            return
        if n_samps > 4096:
            print("Input data size (dim 1) exceeds pilot buffer size!")
            return
        [bs.config_sdr_tdd(tdd_sched="BGPG" if i == 0 else "GGPG", nsamps=n_samps, prefix_len=nsamps_pad) for i, bs in enumerate(self.bs_obj)] 
        [ue.config_sdr_tdd(is_bs=False, tdd_sched="GGRG", nsamps=n_samps) for ue in self.ue_obj]

        for i, bs in enumerate(self.bs_obj):
            bs.burn_data_complex(tx_data_mat[:, i] if self.n_bs_antenna > 1 else tx_data_mat)

        [ue.activate_stream_rx() for ue in self.ue_obj]
        [ue.set_corr() for ue in self.ue_obj]

        rx_data = np.empty((num_frames, self.n_users, n_samps), dtype=np.complex64)
        rx_data_frame = np.empty((self.n_users, n_samps), dtype=np.complex64)
        good_frame_id = 0
        old_triggers = []
        triggers = []
        for u in range(self.n_users):
            old_triggers.append(0)

        for frame in range(num_frames):
            all_triggered = True
            good_signal = True
            amp = 0
            if frame > 0:
                old_triggers = triggers
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
                for u in range(self.n_users):
                    if (new_triggers[u] == 0):
                        all_triggered = False
                if all_triggered and good_signal:
                    break

            print("frame = {}, tries = {}, all_triggred = {}, good_signal = {}, amp = {}".format(frame, i + 1, all_triggered, good_signal, amp))
            if all_triggered and good_signal:
                rx_data[good_frame_id] = rx_data_frame
                good_frame_id = good_frame_id + 1
        [ue.unset_corr() for ue in self.ue_obj]
        self.reset_frame()
        return rx_data, good_frame_id


#########################################
#                  Main                 #
#########################################
def main():
    parser = OptionParser()
    parser.add_option("--hub", type="string", dest="hub", help="serial number of the hub device", default="")
    parser.add_option("--bs-serials", type="string", dest="bs_serials", help="serial numbers of the BS devices", default="RF3E000385")
    parser.add_option("--ue-serials", type="string", dest="ue_serials", help="serial numbers of the UE devices", default="RF3E000027")
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--freq", type="float", dest="freq", help="Tx freq (Hz). POWDER users must set to 3.6e9", default=3.6e9)
    parser.add_option("--tx-gain", type="float", dest="tx_gain", help="Optional Tx gain (dB)", default=80.0)
    parser.add_option("--rx-gain", type="float", dest="rx_gain", help="Optional Rx gain (dB)", default=65.0)
    (options, args) = parser.parse_args()
    mimo = MIMODriver(
        hub_serial=options.hub,
        bs_serials=options.bs_serials.split(","),
        ue_serials=options.ue_serials.split(","),
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
    wb_pilot2 = wbz  # wb_pilot1 if both_channels else wbz
    ul_rx_data, n_ul_good = mimo.txrx_uplink(wb_pilot1, 1)
    print("Uplink Rx Num %d"%n_ul_good)
    dl_rx_data, n_dl_good = mimo.txrx_downlink(wb_pilot1, 1)
    print("Downlink Rx Num %d"%n_dl_good)
    mimo.close()

if __name__ == '__main__':
    main()
