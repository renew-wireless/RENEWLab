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

