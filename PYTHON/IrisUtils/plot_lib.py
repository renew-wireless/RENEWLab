import numpy as np
import matplotlib
import matplotlib.pyplot as plt

# Plots IQ samples in one single frame as well as all frames for
# selected cell,  BS ant, and user
def plot_iq_samps(samps, frm_st, frame_i, cells, users, ants, data_str="Pilots"):
    # Samps Dimensions: (Frame, Cell, User, Antenna, Sample)
    for i in cells:
        for j in users:
            for k in ants:
                cell_i = i
                user_i = j
                ant_i = k
                fig, axes = plt.subplots(nrows=2, ncols=1, squeeze=False, figsize=(10, 8))
                axes[0, 0].set_title(data_str + " IQ - Cell %d - Antenna %d - User %d"%(cell_i, ant_i, user_i))
                axes[0, 0].set_ylabel('Frame %d (IQ)' %( (frame_i + frm_st)) )
                axes[0, 0].plot(np.real(samps[frame_i, cell_i, user_i, ant_i, :]))
                axes[0, 0].plot(np.imag(samps[frame_i, cell_i, user_i, ant_i, :]))

                axes[1, 0].set_ylabel('All Frames (IQ)')
                axes[1, 0].plot(np.real(samps[:, cell_i, user_i, ant_i, :]).flatten())
                axes[1, 0].plot(np.imag(samps[:, cell_i, user_i, ant_i, :]).flatten())

def plot_csi(csi, corr, bs_nodes, good_frames, frame_i, cell_i, user_i, subcarrier_i, offset, data_str="Uplink"):
    fig, axes = plt.subplots(nrows=3, ncols=1, squeeze=False, figsize=(10, 8))
    axes[0, 0].set_title(data_str + " Pilot CSI Stats Across Frames- Cell %d - User %d - Subcarrier %d" % (cell_i, user_i, subcarrier_i))
    axes[0, 0].set_ylabel('Magnitude')
    for i in range(csi.shape[2]):
        axes[0, 0].plot(np.abs(csi[:, user_i, i, subcarrier_i]).flatten(), label="ant %d" % bs_nodes[i])
    axes[0, 0].legend(loc='lower right', frameon=False)
    axes[0, 0].set_xlabel('Frame')

    axes[1, 0].set_ylabel('Phase')
    for i in range(csi.shape[2]):
        axes[1, 0].plot(np.angle(csi[:, user_i, i, subcarrier_i]).flatten(), label="ant %d" % bs_nodes[i])
    axes[1, 0].legend(loc='lower right', frameon=False)
    axes[1, 0].set_ylim(-np.pi, np.pi)
    axes[1, 0].set_xlabel('Frame')

    axes[2, 0].set_ylabel('Correlation with Frame %d' % frame_i)
    axes[2, 0].set_ylim([0, 1.1])
    axes[2, 0].set_title('Cell %d offset %d' % (0, offset))
    for u in range(corr.shape[1]):
        axes[2, 0].plot(corr[good_frames, u], label="user %d"%u)
    axes[2, 0].legend(loc='lower right', frameon=False)
    axes[2, 0].set_xlabel('Frame')

def plot_calib(calib_mat, bs_nodes, frame_i, ant_i, subcarrier_i):
    fig, axes = plt.subplots(nrows=4, ncols=1, squeeze=False, figsize=(10, 8))
    axes[0, 0].set_title('Reciprocity Calibration Factor Across Frames - Cell 0 - Subcarrier %d' % subcarrier_i)

    axes[0, 0].set_ylabel('Magtinute (ant %d)' % (ant_i))
    axes[0, 0].plot(np.abs(calib_mat[:, ant_i, subcarrier_i]).flatten(), label='')
    axes[0, 0].set_xlabel('Frame')
    axes[0, 0].legend(frameon=False)

    axes[1, 0].set_ylabel('Phase (ant %d)' % (ant_i))
    axes[1, 0].plot(np.angle(calib_mat[:, ant_i, subcarrier_i]).flatten())
    axes[1, 0].set_ylim(-np.pi, np.pi)
    axes[1, 0].set_xlabel('Frame')
    axes[1, 0].legend(frameon=False)
    axes[1, 0].grid()

    axes[2, 0].set_ylabel('Magnitude')
    for i in range(calib_mat.shape[1]):
        axes[2, 0].plot(np.abs(calib_mat[:, i, subcarrier_i]).flatten(), label="ant %d" % bs_nodes[i])
    axes[2, 0].set_xlabel('Frame')
    axes[2, 0].legend(loc='lower right', frameon=False)

    axes[3, 0].set_ylabel('Phase')
    for i in range(calib_mat.shape[1]):
        axes[3, 0].plot(np.angle(calib_mat[:, i, subcarrier_i]).flatten(), label="ant %d" % bs_nodes[i])
    axes[3, 0].set_xlabel('Frame')
    axes[3, 0].set_ylim(-np.pi, np.pi)
    axes[3, 0].legend(loc='lower right', frameon=False)
    axes[3, 0].grid()

    fig, axes = plt.subplots(nrows=4, ncols=1, squeeze=False, figsize=(10, 8))
    axes[0, 0].set_title('Reciprocity Calibration Factor Across Subcarriers - Cell 0 - Frame %d' % frame_i)
    axes[0, 0].set_ylabel('Magnitude ant %d' % (ant_i))
    axes[0, 0].plot(np.abs(calib_mat[frame_i, ant_i, :]).flatten())
    axes[0, 0].set_xlabel('Subcarrier')

    axes[1, 0].set_ylabel('Phase ant %d' % (ant_i))
    axes[1, 0].plot(np.angle(calib_mat[frame_i, ant_i, :]).flatten())
    axes[1, 0].set_ylim(-np.pi, np.pi)
    axes[1, 0].set_xlabel('Subcarrier')

    axes[2, 0].set_ylabel('Magnitude')
    for i in range(calib_mat.shape[1]):
        axes[2, 0].plot(np.abs(calib_mat[frame_i, i, :]).flatten(), label="ant %d" % bs_nodes[i])
    axes[2, 0].set_xlabel('Subcarrier')
    axes[2, 0].legend(loc='lower right', frameon=False)

    axes[3, 0].set_ylabel('Phase')
    for i in range(calib_mat.shape[1]):
        axes[3, 0].plot(np.angle(calib_mat[frame_i, i, :]).flatten(), label="ant %d" % bs_nodes[i])
    axes[3, 0].set_xlabel('Subcarrier')
    axes[3, 0].set_ylim(-np.pi, np.pi)
    axes[3, 0].legend(loc='lower right', frameon=False)

def plot_constellation_stats(evm, evm_snr, ul_data, txdata, frame_i, cell_i, ul_slot_i, data_str = "Uplink"):
    n_users = ul_data.shape[1]
    plt_x_len = int(np.ceil(np.sqrt(n_users)))
    plt_y_len = int(np.ceil(n_users / plt_x_len))
    fig5, axes5 = plt.subplots(nrows=plt_y_len, ncols=plt_x_len, squeeze=False, figsize=(10, 8))
    fig5.suptitle(data_str+" User Constellations (ZF) - Frame %d - Cell %d - UL SF %d" % (frame_i, cell_i, ul_slot_i))
    fig6, axes6 = plt.subplots(nrows=2, ncols=1, squeeze=False, figsize=(10, 8))
    fig6.suptitle('Uplink EVM/SNR - Cell %d - UL SF %d' % (cell_i, ul_slot_i))
    axes6[0, 0].set_ylabel('EVM (%)')
    axes6[1, 0].set_ylabel('EVM-SNR (dB)')
    axes6[0, 0].set_xlabel('Frame Number')
    for i in range(n_users):
        y_i = int(i // plt_x_len)
        x_i = i % plt_x_len
        axes5[y_i, x_i].set_title('User %d'%(i))
        axes5[y_i, x_i].scatter(np.real(ul_data[frame_i, i, ul_slot_i, :]), np.imag(ul_data[frame_i, i, ul_slot_i, :]))
        axes5[y_i, x_i].scatter(np.real(txdata[frame_i, i, ul_slot_i, :]), np.imag(txdata[frame_i, i, ul_slot_i, :]))

        axes6[0, 0].plot(range(ul_data.shape[0]), 100 * evm[:, i], label='User %d'%(i))
        axes6[1, 0].plot(range(ul_data.shape[0]), evm_snr[:, i], label='User %d'%(i))
    axes6[0, 0].legend(loc='upper right', frameon=False)
