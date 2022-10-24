from optparse import OptionParser
import sys
import numpy as np
from hdf5_lib import *
import matplotlib.pyplot as plt
import scipy.stats as sps
from distfit import distfit
from fitter import Fitter

def csi_analysis(csi, ref_subcarr, ci_alpha=0.01, monitor_dist=False):

    print("\nStarting CSI Analysis...")
    plt.close("all")

    userCSI = np.mean(csi,axis=1)
    userCSI_ant_subcarr = np.reshape(userCSI, (userCSI.shape[0]*userCSI.shape[1], userCSI.shape[2], userCSI.shape[3]))
    userCSI_subcarr = np.mean(userCSI_ant_subcarr,axis=1)

    csi_subcarr_mag_mean = np.mean(np.abs(userCSI_subcarr),axis=0)
    csi_subcarr_mag_median = np.median(np.abs(userCSI_subcarr),axis=0)
    csi_subcarr_mag_var = np.var(np.abs(userCSI_subcarr),axis=0)
    csi_subcarr_mag_skew = sps.skew(np.abs(userCSI_subcarr),axis=0,bias=True)
    csi_subcarr_mag_kurtosis = sps.kurtosis(np.abs(userCSI_subcarr),axis=0,bias=True)
    csi_subcarr_ph_mean = np.mean(np.angle(userCSI_subcarr)/np.pi,axis=0)
    csi_subcarr_ph_median = np.median(np.angle(userCSI_subcarr)/np.pi,axis=0)
    csi_subcarr_ph_var = np.var(np.angle(userCSI_subcarr)/np.pi,axis=0)
    csi_subcarr_ph_skew = sps.skew(np.angle(userCSI_subcarr)/np.pi,axis=0,bias=True)
    csi_subcarr_ph_kurtosis = sps.kurtosis(np.angle(userCSI_subcarr)/np.pi,axis=0,bias=True)

    if (ref_subcarr > -1 and ref_subcarr < 52):
        userCSI_ref = userCSI_subcarr[:,ref_subcarr]
        ref_sc_pr = True
    else:
        ref_sc_pr = False
        ref_subcarr = 26
        userCSI_center_subcarrs = userCSI_subcarr[:,25:27]
        userCSI_ref = np.mean(userCSI_center_subcarrs,axis=1)

    print("\n** Raw Data Statistical Parameters **")
    print('=====================================================================================')
    print ("%-15s%-15s%-15s%-15s%-15s%-15s"%('Channel','Mean','Median','Var', 'Skew', 'Kurtosis'))
    print('-------------------------------------------------------------------------------------')
    print("%-15s%-15.6f%-15.6f%-15.6f%-15.6f%-15.6f"%('Magnitude',csi_subcarr_mag_mean[ref_subcarr],csi_subcarr_mag_median[ref_subcarr],csi_subcarr_mag_var[ref_subcarr],csi_subcarr_mag_skew[ref_subcarr],csi_subcarr_mag_kurtosis[ref_subcarr]))
    print("%-15s%-15.6f%-15.6f%-15.6f%-15.6f%-15.6f"%('Phase(x\u03C0)',csi_subcarr_ph_mean[ref_subcarr],csi_subcarr_ph_median[ref_subcarr],csi_subcarr_ph_var[ref_subcarr],csi_subcarr_ph_skew[ref_subcarr],csi_subcarr_ph_kurtosis[ref_subcarr]))
    print('-------------------------------------------------------------------------------------')
    if ref_sc_pr:
        print("Data subcarrier to analyze: {}".format(ref_subcarr))
    else:
        print("Data subcarrier to analyze: {} (central subcarrier)".format(ref_subcarr))
    print("Phase values are normalized to PI (\u03C0)")
    print('=====================================================================================')

    plt.rcParams['font.size'] = '14'
    plot_figures = True

    if (plot_figures):
        print("\n** Visualizing Raw Data**")
        print('====================================================================')
        if (monitor_dist):
            plt.rcParams['font.size'] = '10'
            # Histograms
            plt.figure(figsize=(65, 12))
            print("Monitoring magnitude distribution over all subcarriers")
            for i in range(52):
                plt.subplot(4,13,i+1)
                subcar_idx = i
                plt.hist(np.absolute(userCSI_subcarr[:,subcar_idx]), 100, density=True, alpha=0.6,label='Subcarrier %i'%subcar_idx)
                plt.legend(loc="upper right",fontsize=8)
            plt.suptitle("Probability Distribution of the Magnitude over all Subcarriers",fontsize=20)

            plt.figure(figsize=(65, 12))
            print("Monitoring phase distribution over all subcarriers")
            for i in range(52):
                plt.subplot(4,13,i+1)
                subcar_idx = i
                plt.hist(np.angle(userCSI_subcarr[:,subcar_idx])/np.pi, 100, density=True, alpha=0.6,label='Subcarrier %i'%subcar_idx)
                plt.legend(loc="upper right",fontsize=8)
            plt.suptitle("Probability Distribution of the Phase over all Subcarriers",fontsize=20)

        plt.rcParams['font.size'] = '14'
        # Mean and median
        plt.figure(figsize=(10, 10))
        print("Visualizing magnitude and phase values over all subcarriers")
        n_samp = int(userCSI_subcarr.shape[0])
        n_step = 20
        plt.subplot(2,1,1)
        for i in range(1,n_samp,n_step):
            plt.plot(np.arange(52)+1,np.abs(userCSI_subcarr[i,:]),'.',color='b',alpha=0.8)
        plt.plot(np.arange(52)+1,np.abs(userCSI_subcarr[0,:]),'.',color='b',alpha=0.8,label='Magnitude')
        plt.plot(np.arange(52)+1,csi_subcarr_mag_median,linewidth=3,linestyle='-',color='r',label='Median')
        #plt.plot(np.arange(52)+1,np.sqrt(csi_subcarr_abs_var),'--',linewidth=3,color='m',label='Standard Deviation')
        plt.legend(loc="upper right")
        plt.ylabel("Magnitude")
        plt.subplot(2,1,2)
        for i in range(1,n_samp,n_step):
            plt.plot(np.arange(52)+1,np.angle(userCSI_subcarr[i,:])/np.pi,'.',color='c',alpha=0.8)
        plt.plot(np.arange(52)+1,np.angle(userCSI_subcarr[0,:])/np.pi,'.',color='c',alpha=0.8,label='Phase')
        plt.plot(np.arange(52)+1,csi_subcarr_ph_median,linewidth=3,linestyle='-',color='r',label='Median')
        #plt.plot(np.arange(52)+1,np.sqrt(csi_subcarr_angle_var),'--',linewidth=3,color='m',label='Standard Deviation')
        plt.legend(loc="upper right",fontsize=14)
        #plt.title("Absolute Variance per Subcarrier",fontsize=18)
        plt.ylim([-1.5, 1.5])
        plt.ylabel("Phase (X $\pi$)",fontsize=18)
        plt.xlabel("Subcarrier Index",fontsize=18)
        plt.suptitle("Magnitude and Phase Values",fontsize=18)

        # Phase
        plt.figure(figsize=(18, 6))
        print("Visualizing distributions of the reference subcarrier")
        plt.subplot(1,2,1)
        bins_no = 800
        plt.hist(np.abs(userCSI_ref),bins_no,density=True,alpha=0.75,label='Raw Data')
        plt.legend(loc="upper right",fontsize=16)
        plt.xlabel("Magnitude",fontsize=16)
        plt.ylabel("Probability Density",fontsize=16)
        plt.subplot(1,2,2)
        bins_no_ph = 800
        count, bins_ph, ignored = plt.hist(np.angle(userCSI_ref)/np.pi,bins_no_ph,density=True,alpha=0.75,label='Raw Data')
        unif_dist = bins_ph*0 + 0.5
        plt.plot(bins_ph[0:bins_no_ph],unif_dist[0:bins_no_ph],'--',color='r',linewidth=4,label='Uniform')
        plt.legend(loc="upper right",fontsize=16)
        plt.xlabel("Phase (X $\pi$)",fontsize=16)
        plt.ylabel("Probability Density",fontsize=16)
        plt.suptitle("Reference Subcarrier #{} Raw Data Distributions".format(ref_subcarr),fontsize=18)

        plt.figure(figsize=(16,8))
        print("Visualizing magnitude/phase mean and variance over all subcarriers")
        plt.subplot(2,2,1)
        plt.plot(np.arange(52)+1,csi_subcarr_mag_mean,c='b',marker='o',label='Magnitude Mean')
        plt.ylabel('Magnitude')
        plt.legend(fontsize=12, loc='upper right')
        plt.subplot(2,2,2)
        plt.plot(np.arange(52)+1,np.sqrt(csi_subcarr_mag_var),'--',c='b',marker='o',label='Magnitude Standard Deviation')
        plt.legend(fontsize=12, loc='upper right')
        plt.subplot(2,2,3)
        plt.plot(np.arange(52)+1,csi_subcarr_ph_mean,c='c',marker='o',label='Phase Mean')
        plt.legend(fontsize=12, loc='upper right')
        plt.xlabel("Subcarrier Index")
        plt.ylabel("Phase(X $\pi$)")
        plt.subplot(2,2,4)
        plt.plot(np.arange(52)+1,np.sqrt(csi_subcarr_ph_var),'--',c='c',marker='o',label='Phase Standard Deviation')
        plt.xlabel("Subcarrier Index")
        plt.legend(fontsize=12, loc='upper right')
        print('====================================================================')
        print("Done!")
        #plt.show()

    csi_dist_fitting = True
    if(csi_dist_fitting):
        mag_dist_set = ['lognorm', 'expon', 'nakagami', 'rice', 'rayleigh', 'norm', 'chi2', 'gamma']
        ph_dist_set = ['uniform', 'beta', 'bradford', 'semicircular', 'nakagami']

        print("\n\n** Distribution fitting with RSS minimization **")
        print('=============================================================================================')
        # Magnitude
        print("Distribution fitting for magnitude")
        print('---------------------------------------------------------------------------------------------')
        dist = distfit(todf=True, distr=mag_dist_set, alpha=ci_alpha, bins=400)
        mag_results = dist.fit_transform(np.abs(userCSI_ref))
        dist.plot()
        dist.plot_summary()
        print("\nSummary:")
        print(dist.summary)
        # Phase
        print('---------------------------------------------------------------------------------------------')
        print("Distribution fitting for phase")
        print('---------------------------------------------------------------------------------------------')
        dist = distfit(todf=True, distr=ph_dist_set, alpha=ci_alpha, bins=400)
        phase_results = dist.fit_transform(np.angle(userCSI_ref)/np.pi)
        dist.plot()
        plt.ylim([0,1])
        dist.plot_summary()
        print("\nSummary:")
        print(dist.summary)
        #plt.show()
        print('=============================================================================================')
        print("Done!")

        print("\n\n** Distribution fitting using Fitter library **")
        print('==========================================================================================')
        # Magnitude
        mag_dist = Fitter(np.abs(userCSI_ref), bins=400, distributions=mag_dist_set)
        mag_dist.fit()
        best = mag_dist.get_best(method='sumsquare_error')
        for best_dist, params in best.items():
            print("Best distribution for magnitude: {}".format(best_dist))
            print("Parameters:")
            for param_type, param_val in params.items():
                print("%-15s%-15.6f"%(param_type, param_val))
        plt.figure(figsize=(10,8))
        mag_sum = mag_dist.summary(Nbest=4)
        print("\nSummary:")
        print(mag_sum)
        plt.grid(False)
        plt.ylabel("Probability Density")
        plt.xlabel("Magnitude")
        plt.title("Best Distribution: {}".format(best_dist))
        print('------------------------------------------------------------------------------------------')
        # Phase
        ph_dist = Fitter(np.angle(userCSI_ref)/np.pi, bins=400, distributions=ph_dist_set)
        ph_dist.fit()
        best = ph_dist.get_best(method='sumsquare_error')
        for best_dist, params in best.items():
            print("Best distribution for phase: {}".format(best_dist))
            print("Parameters:")
            for param_type, param_val in params.items():
                print("%-15s%-15.6f"%(param_type, param_val))
        plt.figure(figsize=(10,8))
        ph_sum = ph_dist.summary(Nbest=3)
        print("\nSummary:")
        print(ph_sum)
        plt.grid(False)
        plt.ylabel("Probability Density")
        plt.xlabel("Phase (X $\pi$)")
        plt.title("Best Distribution: {}".format(best_dist))
        #plt.show()
        print('==========================================================================================')
        print("Done!")

        print("\nPlotting...")
        plt.show()   



def main():
    parser = OptionParser()
    parser.add_option("--alpha", type="float", dest="ci_alpha", help="Confidence Interval Indicator", default=0.01)
    parser.add_option("--ref-subcarr", type="int", dest="ref_subcarr", help="Reference subcarrier to plot", default=-1)
    parser.add_option("--monitor-dist", action="store_true", dest="monitor_dist", help="Monitor channel distribution for all non-zero subcarriers", default=False)
    parser.add_option("--signal-offset", type="int", dest="signal_offset", help="signal offset from the start of the time-domain symbols", default=-1)
    parser.add_option("--n-frames", type="int", dest="n_frames", help="Number of frames to inspect", default=-1)
    parser.add_option("--sub-sample", type="int", dest="sub_sample", help="Sub sample rate", default=1)
    parser.add_option("--frame-start", type="int", dest="fr_strt", help="Starting frame. Must have set n_frames first and make sure fr_strt is within boundaries ", default=0)
    (options, args) = parser.parse_args()

    ci_alpha = options.ci_alpha
    ref_subcarr = options.ref_subcarr
    monitor_dist = options.monitor_dist
    n_frames = options.n_frames
    offset = options.signal_offset
    fr_strt = options.fr_strt
    sub_sample = options.sub_sample

    path = sys.argv[1]
    num_datasets = int(sys.argv[2])
    print("Opening {} datasets...\n".format(num_datasets))
    pilot_samples_vec = []
    for i in range(num_datasets):
        filename_temp = path + sys.argv[i+3]
        hdf5_temp = hdf5_lib(filename_temp, n_frames, fr_strt, sub_sample)
        if (i==0):
            pilot_samples_vec = hdf5_temp.pilot_samples
            hdf5 = hdf5_temp
        else:
            pilot_samples_vec = np.append(pilot_samples_vec,hdf5_temp.pilot_samples,0)

    metadata = hdf5.metadata
    num_pilots = int(metadata['PILOT_NUM'])
    chunk_size = 10000
    samps_per_slot = int(metadata['SLOT_SAMP_LEN'])
    fft_size = int(metadata['FFT_SIZE'])
    prefix_len = int(metadata['PREFIX_LEN'])
    postfix_len = int(metadata['POSTFIX_LEN'])
    z_padding = prefix_len + postfix_len
    if offset < 0: # if no offset is given use prefix from HDF5
        offset = int(prefix_len)
    cp = int(metadata['CP_LEN'])
    ofdm_pilot_f = np.array(metadata['OFDM_PILOT_F'])
    pilot_samples_vec = pilot_samples_vec[:, 0, :, :, :]

    if n_frames < 0 or n_frames == 0:
        print("Processing the whole datasets.")
        n_frames = pilot_samples_vec.shape[0]

    if (ref_subcarr < 0):
        ref_subcarr_print = 26
    else:
        ref_subcarr_print = ref_subcarr
    print("\n>> number of frames = {}, reference subcarrier = {}, alpha = {}, monitoring all subcarriers = {} <<\n".format(n_frames, ref_subcarr_print, ci_alpha, monitor_dist))

    pilot_samples_vec = pilot_samples_vec[fr_strt:fr_strt+n_frames,:,:,:]
    print("PILOT SAMPLES MATRIX SHAPE:")
    print(np.shape(pilot_samples_vec))

    csi,_ = hdf5_lib.samps2csi_large(pilot_samples_vec, num_pilots, chunk_size, samps_per_slot, fft_size=fft_size,
                                            offset=offset, bound=z_padding, cp=cp, pilot_f=ofdm_pilot_f)
    print("CSI generated!")
    print("CSI MATRIX SHAPE:")
    print(np.shape(csi))

    csi_analysis(csi=csi, ref_subcarr=ref_subcarr, ci_alpha=ci_alpha, monitor_dist=monitor_dist)


if __name__ == '__main__':
    main()
    
    
