import numpy as np
from scipy import signal

def extract_pilots_data(cmpx_pilots, lts_seq_conj, k_lts, n_lts, frame):
    # normalized matched filter
    a = 1
    l_lts_fc = len(lts_seq_conj)
    unos = np.ones(l_lts_fc)
    v0 = signal.lfilter(lts_seq_conj, a, cmpx_pilots, axis=2)
    v1 = signal.lfilter(unos, a, (abs(cmpx_pilots)**2), axis=2)
    #supress invalid floating point operation error
    np.seterr(invalid='ignore')
    m_filt = (np.abs(v0)**2)/v1

    # clean up nan samples: replace nan with -1
    nan_indices = np.argwhere(np.isnan(m_filt))
    m_filt[np.isnan(m_filt)] = -0.5  # the only negative value in m_filt

    #if write_to_file:
    #    # write the nan_indices into a file
    #    np.savetxt("nan_indices.txt", nan_indices, fmt='%i')

    #if debug:
    #    print("Shape of truncated complex pilots: {} , l_lts_fc = {}, v0.shape = {}, v1.shape = {}, m_filt.shape = {}".
    #          format(cmpx_pilots.shape, l_lts_fc, v0.shape, v1.shape, m_filt.shape))

    rho_max = np.amax(m_filt, axis=2)         # maximum peak per SF per antenna
    rho_min = np.amin(m_filt, axis=2)        # minimum peak per SF per antenna
    ipos = np.argmax(m_filt, axis=2)          # positons of the max peaks
    sf_start = ipos - l_lts_fc + 1             # start of every received SF
    # get rid of negative indices in case of an incorrect peak
    sf_start = np.where(sf_start < 0, 0, sf_start)

    # get the pilot samples from the cmpx_pilots array and reshape for k_lts LTS pilots:
    n_ue = cmpx_pilots.shape[0]         # no. of UEs
    n_ant = cmpx_pilots.shape[1]        # no. of BS antennas
    pilots_rx_t = np.empty(
        [n_ue, n_ant, k_lts * n_lts], dtype='complex64')

    for k in range(n_ue):
        for l in range(n_ant):
            pilots_rx_t[k, l, :] = cmpx_pilots[k, l, sf_start[k, l]:  sf_start[k, l] + (k_lts * n_lts)]

    return frame, pilots_rx_t, m_filt, sf_start


