import scipy.io as sio
import iris_py
from iris_py import *

def mimo(hub_serial, bs_serials, ue_serials, rate,
         freq, txgain, rxgain, both_channels, num_frames, matfile, rx_freq_mat_file):
    tx_data_dict = sio.loadmat(matfile)
    tx_data = tx_data_dict['tx_vec_iris']
    print(tx_data.shape)
    n_samps = tx_data.shape[0]
    n_users = tx_data.shape[1]
    if n_users != len(ue_serials):
        print("Input data size does not match number of UEs!")
        return
    nsamps_pad = 0
    for i in range(n_samps):
        if tx_data[i, 0] != 0:
            nsamps_pad = i
            break
    n_bs_ant = len(bs_serials)
    print("data length = {}, pad = {}, n_users = {}".format(n_samps, nsamps_pad, n_users))

    
    freq_sweep_num = 1
    freq_sweep_en = False
    if len(rx_freq_mat_file) > 0:
        rx_freq_dict = sio.loadmat(rx_freq_mat_file)
        rx_freq = rx_freq_dict['cfo_freq']
        freq_sweep_en = True
        if rx_freq.shape[0] == n_bs_ant:
           freq_sweep_num = rx_freq.shape[1]
        elif rx_freq.shape[1] == n_bs_ant:
           freq_sweep_num = rx_freq.shape[0]
        else:
            print("Freq sweep data size does not match number of number of BS antennas!")
            return


    # Init Radios
    bs_obj = [Iris_py(sdr, freq, freq, txgain, rxgain, None, rate, n_samps, both_channels) for sdr in bs_serials]
    ue_obj = [Iris_py(sdr, freq, freq, txgain, rxgain, None, rate, n_samps, both_channels) for sdr in ue_serials]
    hub = None
    if len(hub_serial) != 0:
        hub = Hub_py(hub_serial)

    # Setup Radios and Streams
    if hub is not None:
        hub.sync_delays()
    else:
        bs_obj[0].sync_delays()

    [ue.config_gain_ctrl() for ue in ue_obj]

    [ue.setup_stream_rx() for ue in ue_obj]
    [bs.setup_stream_rx() for bs in bs_obj]

    [bs.config_sdr_tdd(tdd_sched="BGGGGGRG" if i == 0 else "GGGGGGRG", prefix_len=nsamps_pad) for i, bs in enumerate(bs_obj)] 
    [ue.config_sdr_tdd(tdd_sched="GGGGGGPG", is_bs=False) for ue in ue_obj]

    #[bs.burn_beacon() for bs in bs_obj]
    bs_obj[0].burn_beacon()
    for i, ue in enumerate(ue_obj):
        ue.burn_data(np.real(tx_data[:, i]), np.imag(tx_data[:, i]))

    [bs.activate_stream_rx() for bs in bs_obj]
    [ue.set_corr() for ue in ue_obj]

    for f in range(freq_sweep_num):
        if freq_sweep_en:
            [bs.sdr_setrxfreq(rx_freq[f][n]) for n, bs in enumerate(bs_obj)]
        rx_data = np.empty((num_frames, n_bs_ant, n_samps), dtype=np.complex64)

        max_try = 10
        old_triggers = []
        triggers = []
        for u in range(n_users):
            old_triggers.append(0)

        for frame in range(num_frames):
            all_triggered = True
            good_signal = True
            amp = 0
            if frame > 0:
                old_triggers = triggers
            for i in range(max_try):
                # Receive Data
                if hub is not None:
                    hub.set_trigger()
                else:
                    bs_obj[0].set_trigger()

                rx_data[frame] = [bs.recv_stream_tdd() for bs in bs_obj]
                amp = np.mean(np.abs(rx_data[frame][0]))
                good_signal = amp > 0.001
                triggers = [ue.sdr_gettriggers() for ue in ue_obj]
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

        # Save Received Data
        if freq_sweep_en:
            sio.savemat('rx_data'+str(n_bs_ant)+'x'+str(n_users)+'f'+str(f)+'.mat', {'rx_vec_iris':rx_data})
        else:
            sio.savemat('rx_data'+str(n_bs_ant)+'x'+str(n_users)+'.mat', {'rx_vec_iris':rx_data})

    # Terminate Radios
    [ue.close() for ue in ue_obj]
    [bs.close() for bs in bs_obj]



#########################################
#                  Main                 #
#########################################
def main():
    parser = OptionParser()
    parser.add_option("--hub", type="string", dest="hub", help="serial number of the hub device", default="")
    parser.add_option("--bs-serials", type="string", dest="bs_serials", help="serial numbers of the BS devices", default="RF3E000143,RF3E000160,RF3E000025,RF3E000034")
    parser.add_option("--ue-serials", type="string", dest="ue_serials", help="serial numbers of the UE devices", default="RF3E000164,RF3E000241")
    parser.add_option("--mat-file", type="string", dest="mat_file", help="MAT file including transmit data for UEs", default="RF3E000164,RF3E000241")
    parser.add_option("--num-frames", type="int", dest="num_frames", help="Number of frames to collect", default=1)
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--freq", type="float", dest="freq", help="Tx freq (Hz). POWDER users must set to 3.6e9", default=3.6e9)
    parser.add_option("--freq-mat", type="string", dest="freq_mat", help="MAT files of the list of rx frequencies", default="")
    parser.add_option("--txgain", type="float", dest="txgain", help="Optional Tx gain (dB)", default=80.0)
    parser.add_option("--rxgain", type="float", dest="rxgain", help="Optional Rx gain (dB)", default=70.0)
    parser.add_option("--both-channels", action="store_true", dest="both_channels", help="transmit from both channels",default=False)
    (options, args) = parser.parse_args()
    mimo(
        hub_serial=options.hub,
        bs_serials=options.bs_serials.split(','),
        ue_serials=options.ue_serials.split(','),
        rate=options.rate,
        freq=options.freq,
        txgain=options.txgain,
        rxgain=options.rxgain,
        both_channels=options.both_channels,
        num_frames=options.num_frames,
        matfile=options.mat_file,
        rx_freq_mat_file=options.freq_mat
    )

if __name__ == '__main__':
    main()
 

