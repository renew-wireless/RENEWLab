#!/usr/bin/python3
"""

---------------------------------------------------------------------
 Copyright © 2018-2020. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
from matplotlib.ticker import PercentFormatter
from matplotlib import pyplot
from find_lts import *
from channel_analysis import *
from hdf5_lib import *


def process_round1_collection():

    # Honors: (2,8) 2 UEs, 8 locations
    mean_snr_honors = [[2.81, 18.61, 0, 2.56, 1.81, np.nan, 2.62, 20.2], [0.74, 17.58, 0, 0.78, 0.5, np.nan, 0.73, 17.95]]
    # USTAR: (2, 2, 8) 2 UEs, 2 BS Ant, 8 locations
    mean_snr_ustar = [[[18.65, 16.61, 14.85, 18.88, 0, np.nan, 0, 23.88], [2.66, 0.97, 0.47, 1.27, 0, np.nan, 0, 3.24]],
                      [[10.54, 4.69, 15.2, 18.38, 0, np.nan, 0, 6.87], [0.29, 0.04, 0.4, 1.1, 0, np.nan, 0, 0.11]]]
    mean_snr_honors = np.array(mean_snr_honors)
    mean_snr_ustar = np.array(mean_snr_ustar)

    n_ue = 2
    c = []
    fig, axes = plt.subplots(n_ue, 1)
    for ue, ax in enumerate(axes):
        c.append(ax.imshow(mean_snr_ustar[ue, :, :], vmin=0, vmax=25, cmap='Blues',
                                        interpolation='nearest',
                                        #extent=[n_frm_st, n_frm_end, n_ant, 0],
                                        aspect="auto"))
        ax.set_title('UE {}'.format(ue))
        ax.set_ylabel('BSAntenna #')
        ax.set_xlabel('Location')
        ax.set_xticks(np.arange(0, 8, 1), ['1', '2', '3', '4', '5', '6', '7', '8'])
        ax.set_yticks(np.arange(0, 2, 1), ['0', '1'])
        ax.grid(which='minor', color='0.75', linestyle='-', linewidth=0.05)
        for (j,i),label in np.ndenumerate(mean_snr_ustar[ue,:,:]):
            if label > 14.5:
                ax.text(i,j,label,ha='center',va='center', color='white')
            else:
                ax.text(i,j,label,ha='center',va='center', color='black')

    cbar = plt.colorbar(c[-1], ax=axes.ravel().tolist(), orientation='horizontal', label='Averge SNR (dB)')
    fig.suptitle('USTAR')
    plt.draw()

    
    plt.figure()
    #plt.imshow(mean_snr_honors, interpolation='nearest', cmap='Blues', origin='lower', aspect="auto")  
    plt.imshow(mean_snr_honors, interpolation='nearest', cmap='Blues', aspect="auto")  
    plt.xticks(range(8), ['1', '2', '3', '4', '5', '6', '7', '8'])
    plt.yticks(range(2))
    #ax.set_xticklabels(['1', '2', '3', '4', '5', '6', '7', '8'])
    #plt.set_yticklabels(['0', '1'])
    plt.grid(which='minor', color='0.75', linestyle='-', linewidth=0.05)
    for (j,i),label in np.ndenumerate(mean_snr_honors):
        if label > 15:
            plt.text(i,j,label,ha='center',va='center', color='white')
        else:
            plt.text(i,j,label,ha='center',va='center', color='black')

    plt.title('Honors')
    plt.xlabel('Location')
    plt.ylabel('UE')
    cbar = plt.colorbar(orientation='horizontal', label='Averge SNR (dB)')
    plt.draw()
    plt.show()
    
    return




    snr_h1, pilot_h1 = np.load('./data_powder/honors_loc1.npy')
    snr_h2, pilot_h2 = np.load('./data_powder/honors_loc2.npy')
    snr_h3, pilot_h3 = np.load('./data_powder/honors_loc3.npy')
    snr_h4, pilot_h4 = np.load('./data_powder/honors_loc4.npy')
    snr_h5, pilot_h5 = np.load('./data_powder/honors_loc5.npy')
    snr_h7, pilot_h7 = np.load('./data_powder/honors_loc7.npy')
    snr_h8, pilot_h8 = np.load('./data_powder/honors_loc8.npy')
    
    snr_u1, pilot_u1 = np.load('./data_powder/ustar_loc1.npy')
    snr_u2, pilot_u2 = np.load('./data_powder/ustar_loc2.npy')
    snr_u3, pilot_u3 = np.load('./data_powder/ustar_loc3.npy')
    snr_u4, pilot_u4 = np.load('./data_powder/ustar_loc4.npy')
    snr_u5, pilot_u5 = np.load('./data_powder/ustar_loc5.npy')
    snr_u7, pilot_u7 = np.load('./data_powder/ustar_loc7.npy')
    snr_u8, pilot_u8 = np.load('./data_powder/ustar_loc8.npy')

    # SNR
    snr_arr = [snr_h1, snr_u1, snr_h2, snr_u2, snr_h3, snr_u3, 
                    snr_h4, snr_u4, snr_h5, snr_u5, snr_h7, snr_u7, snr_h8, snr_u8]

    for idx in range(len(snr_arr)//2):
        print('Iter: {}, HONORS: {}, USTAR: {}'.format(idx, 2*idx, 2*idx+1))

        h = snr_arr[2*idx]
        u = snr_arr[2*idx+1]
        h_snr = np.reshape(h, (1, h.shape[0]*h.shape[1]*h.shape[2]*h.shape[3]))
        u_snr = np.reshape(u, (1, u.shape[0]*u.shape[1]*u.shape[2]*u.shape[3]))

        h_max_snr = np.max(h_snr)
        u_max_snr = np.max(u_snr)
        # print("Location {}: Honors Max SNR {}, USTAR Max SNR {}".format(idx+1, h_max_snr, u_max_snr))

        jvec = np.transpose(np.hstack((h_snr, u_snr)))
        minVal = min(jvec)
        maxVal = max(jvec)
        n_bins = 100
        bins = np.linspace(minVal, maxVal, n_bins)

        if 0:
            # HISTOGRAMS
            fig, ax1 = plt.subplots()
            ax2 = ax1.twinx()
            ax1.hist(h_snr[0], np.squeeze(bins), alpha=0.5, color = "lightblue", label='honors')
            ax2.hist(u_snr[0], np.squeeze(bins), alpha=0.5, color = "lightcoral", label='ustar')
            lines, labels = ax1.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax2.legend(lines + lines2, labels + labels2, loc=0)
            ax1.set_xlabel('SNR')
            ax1.set_ylabel('Honors')
            ax2.set_ylabel('USTAR')

        # SNR CURVES
        # Shape of h and u: snr[frameIdx, cellIdx, ueIdx, bsAntIdx]
        # USTAR: single device on rooftop and single UE transmitting uplink (2 antennas in each)
        u_snr_0_0 = np.reshape(u[:, 0, 0, 0], (1, h.shape[0]))
        u_snr_0_1 = np.reshape(u[:, 0, 0, 1], (1, h.shape[0]))
        u_snr_1_0 = np.reshape(u[:, 0, 1, 0], (1, h.shape[0]))
        u_snr_1_1 = np.reshape(u[:, 0, 1, 1], (1, h.shape[0]))

        # Honors: one Faros base station. 54 antennas so we need average across antennas
        h_mean = np.mean(h, axis=3)
        h_snr_0 = np.reshape(h_mean[:, 0, 0], (1, u.shape[0]))
        h_snr_1 = np.reshape(h_mean[:, 0, 1], (1, u.shape[0]))

        fig, (ax1, ax2) = plt.subplots(2)
        fig.suptitle('SNR Across Frames')
        ax1.plot(u_snr_0_0[0], label='USTAR UE0, ANT0')
        ax1.plot(u_snr_0_1[0], label='USTAR UE0, ANT1')
        ax1.plot(u_snr_1_0[0], label='USTAR UE1, ANT0')
        ax1.plot(u_snr_1_1[0], label='USTAR UE1, ANT1')

        ax2.plot(h_snr_0[0], label='Honors UE0 (AVG BS ANT)')
        ax2.plot(h_snr_1[0], label='Honors UE1 (AVG BS ANT)')

        lines, labels = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines, labels, loc=0)
        ax2.legend(lines2, labels2, loc=0)

        ax1.set_xlabel('Number of Frames')
        ax2.set_xlabel('Number of Frames')
        ax1.set_ylabel('SNR (dB)')
        ax2.set_ylabel('SNR (dB)')
        #plt.gca().set_xlim(left=-10)
        plt.draw()

        if idx == 6:
            snr_print_u0 = np.mean(u_snr_0_0[0][24:1070])
            snr_print_u1 = np.mean(u_snr_0_1[0][24:1070])
            snr_print_u2 = np.mean(u_snr_1_0[0][24:1070])
            snr_print_u3 = np.mean(u_snr_1_1[0][24:1070])
            snr_print_h0 = np.mean(h_snr_0[0][15:454])
            snr_print_h1 = np.mean(h_snr_1[0][15:454])
            print("SNR USTAR: {}, {}, {}, {}".format(snr_print_u0,snr_print_u1,snr_print_u2,snr_print_u3))
            print("SNR HONORS: {}, {} ".format(snr_print_h0, snr_print_h1))

    plt.show()
    return

    # PILOTS
    pilot_arr = [pilot_h1, pilot_u1, pilot_h2, pilot_u2, pilot_h3, pilot_u3, 
                    pilot_h4, pilot_u4, pilot_h5, pilot_u5, pilot_h7, pilot_u7, pilot_h8, pilot_u8]
    
    for idx in range(len(pilot_arr)//2):
        print('Iter: {}, HONORS: {}, USTAR: {}'.format(idx, 2*idx, 2*idx+1))

        h = pilot_arr[2*idx]
        u = pilot_arr[2*idx+1]
        h_pilot = np.reshape(h, (1, h.shape[0]*h.shape[1]*h.shape[2]*h.shape[3]))
        u_pilot = np.reshape(u, (1, u.shape[0]*u.shape[1]*u.shape[2]*u.shape[3]))

        jvec = np.transpose(np.hstack((h_pilot, u_pilot)))
        minVal = min(jvec)
        maxVal = max(jvec)
        n_bins = 20
        bins = np.linspace(minVal, maxVal, n_bins)

        fig, ax1 = plt.subplots()
        ax2 = ax1.twinx()
        ax1.hist(h_pilot[0], np.squeeze(bins), alpha=0.5, color = "lightblue", label='honors')
        ax2.hist(u_pilot[0], np.squeeze(bins), alpha=0.5, color = "lightcoral", label='ustar')
        lines, labels = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax2.legend(lines + lines2, labels + labels2, loc=0)
        ax1.set_xlabel('% RX Pilots')
        ax1.set_ylabel('Honors')
        ax2.set_ylabel('USTAR')

        #plt.gca().set_xlim(left=-10)
        #plt.draw()

    plt.show()
    return




def process_round2_collection():

    if 0:
        # Data collected on August 10th, 2022
        snr_h1 = np.load('./data_powder/data_samps_honors_loc1.npy')
        snr_h2 = np.load('./data_powder/data_samps_honors_loc2.npy')
        snr_h3 = np.load('./data_powder/data_samps_honors_loc3.npy')
        snr_h4 = np.load('./data_powder/data_samps_honors_loc4.npy')
        snr_h8 = np.load('./data_powder/data_samps_honors_loc8.npy')

        snr_u1 = np.load('./data_powder/data_samps_ustar_loc1.npy')
        snr_u2 = np.load('./data_powder/data_samps_ustar_loc2.npy')
        #snr_u2 = np.load('./data_powder/data_samps_ustar_loc2_rx70.npy')
        snr_u3 = np.load('./data_powder/data_samps_ustar_loc3.npy')
        snr_u4 = np.load('./data_powder/data_samps_ustar_loc4.npy')
        snr_u8 = np.load('./data_powder/data_samps_ustar_loc8.npy')

        #snr_h1, pilot_h1 = np.load('./data_powder/data_samps_honors_loc1.npy')
        ##snr_h1, pilot_h1 = np.load('./data_powder/data_samps_honors_loc1_v2.npy')
        #snr_h2, pilot_h2 = np.load('./data_powder/data_samps_honors_loc2.npy')
        #snr_h3, pilot_h3 = np.load('./data_powder/data_samps_honors_loc3.npy')
        #snr_h4, pilot_h4 = np.load('./data_powder/data_samps_honors_loc4.npy')
        #snr_h8, pilot_h8 = np.load('./data_powder/data_samps_honors_loc8.npy')

        #snr_u1, pilot_u1 = np.load('./data_powder/data_samps_ustar_loc1.npy')
        #snr_u2, pilot_u2 = np.load('./data_powder/data_samps_ustar_loc2.npy')
        ##snr_u2, pilot_u2 = np.load('./data_powder/data_samps_ustar_loc2_rx70.npy')
        #snr_u3, pilot_u3 = np.load('./data_powder/data_samps_ustar_loc3.npy')
        #snr_u4, pilot_u4 = np.load('./data_powder/data_samps_ustar_loc4.npy')
        #snr_u8, pilot_u8 = np.load('./data_powder/data_samps_ustar_loc8.npy')


        # SNR
        snr_arr = [snr_h1, snr_u1, snr_h2, snr_u2, snr_h3, snr_u3, 
                        snr_h4, snr_u4, snr_h8, snr_u8]

        for idx in range(len(snr_arr)//2):
            print('Iter: {}, HONORS: {}, USTAR: {}'.format(idx, 2*idx, 2*idx+1))

            h = snr_arr[2*idx]
            u = snr_arr[2*idx+1]
            h_snr = np.reshape(h, (1, h.shape[0]*h.shape[1]*h.shape[2]))
            u_snr = np.reshape(u, (1, u.shape[0]*u.shape[1]*u.shape[2]))

            h_max_snr = np.max(h_snr)
            u_max_snr = np.max(u_snr)
            # print("Location {}: Honors Max SNR {}, USTAR Max SNR {}".format(idx+1, h_max_snr, u_max_snr))

            jvec = np.transpose(np.hstack((h_snr, u_snr)))
            minVal = min(jvec)
            maxVal = max(jvec)
            n_bins = 100
            bins = np.linspace(minVal, maxVal, n_bins)

            if 0:
                # HISTOGRAMS
                fig, ax1 = plt.subplots()
                ax2 = ax1.twinx()
                ax1.hist(h_snr[0], np.squeeze(bins), alpha=0.5, color = "lightblue", label='honors')
                ax2.hist(u_snr[0], np.squeeze(bins), alpha=0.5, color = "lightcoral", label='ustar')
                lines, labels = ax1.get_legend_handles_labels()
                lines2, labels2 = ax2.get_legend_handles_labels()
                ax2.legend(lines + lines2, labels + labels2, loc=0)
                ax1.set_xlabel('SNR')
                ax1.set_ylabel('Honors')
                ax2.set_ylabel('USTAR')

            # SNR CURVES
            # Shape of h and u: snr[frameIdx, cellIdx, ueIdx, bsAntIdx]
            # USTAR: single device on rooftop and single UE transmitting uplink (2 antennas in each)
            u_snr_0_0 = np.reshape(u[:, 0, 0], (1, h.shape[0]))
            u_snr_0_1 = np.reshape(u[:, 0, 1], (1, h.shape[0]))
            u_snr_1_0 = np.reshape(u[:, 1, 0], (1, h.shape[0]))
            u_snr_1_1 = np.reshape(u[:, 1, 1], (1, h.shape[0]))

            # Honors: one Faros base station. 54 antennas so we need average across antennas
            h_mean = np.mean(h, axis=2)
            h_snr_0 = np.reshape(h_mean[:, 0], (1, u.shape[0]))
            h_snr_1 = np.reshape(h_mean[:, 1], (1, u.shape[0]))

            fig, (ax1, ax2) = plt.subplots(2)
            fig.suptitle('SNR Across Frames')
            ax1.plot(u_snr_0_0[0], label='USTAR UE0, ANT0')
            ax1.plot(u_snr_0_1[0], label='USTAR UE0, ANT1')
            ax1.plot(u_snr_1_0[0], label='USTAR UE1, ANT0')
            ax1.plot(u_snr_1_1[0], label='USTAR UE1, ANT1')

            ax2.plot(h_snr_0[0], label='Honors UE0 (AVG BS ANT)')
            ax2.plot(h_snr_1[0], label='Honors UE1 (AVG BS ANT)')

            lines, labels = ax1.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax1.legend(lines, labels, loc=0)
            ax2.legend(lines2, labels2, loc=0)

            ax1.set_xlabel('Number of Frames')
            ax2.set_xlabel('Number of Frames')
            ax1.set_ylabel('SNR (dB)')
            ax2.set_ylabel('SNR (dB)')
            #plt.gca().set_xlim(left=-10)
            
            #plt.draw()

            if idx == 3:
                start = 550
                end = 2000
            else:
                start = 0
                end = 2000

            snr_print_u0 = np.mean(u_snr_0_0[0][start:end])
            snr_print_u1 = np.mean(u_snr_0_1[0][start:end])
            snr_print_u2 = np.mean(u_snr_1_0[0][start:end])
            snr_print_u3 = np.mean(u_snr_1_1[0][start:end])
            snr_print_h0 = np.mean(h_snr_0[0][start:end])
            snr_print_h1 = np.mean(h_snr_1[0][start:end])
            print("MEAN SNR USTAR: {}, {}, {}, {}".format(snr_print_u0,snr_print_u1,snr_print_u2,snr_print_u3))
            print("MEAN SNR HONORS: {}, {} ".format(snr_print_h0, snr_print_h1))
            print("")

            snr_print_u0 = np.max(u_snr_0_0[0][start:end])
            snr_print_u1 = np.max(u_snr_0_1[0][start:end])
            snr_print_u2 = np.max(u_snr_1_0[0][start:end])
            snr_print_u3 = np.max(u_snr_1_1[0][start:end])
            snr_print_h0 = np.max(h_snr_0[0][start:end])
            snr_print_h1 = np.max(h_snr_1[0][start:end])
            #print("MAX SNR USTAR: {}, {}, {}, {}".format(snr_print_u0,snr_print_u1,snr_print_u2,snr_print_u3))
            #print("MAX SNR HONORS: {}, {} ".format(snr_print_h0, snr_print_h1))
            print("")

        plt.show()


    # USE HARD CODED VALUES (FROM SECOND ROUND)
    # Honors: (2,8) 2 UEs, 8 locations
    mean_snr_honors = [[2.86, 6.68, np.nan, 9.76, np.nan, 9.34, np.nan, 13.64], [0.78, 6.79, np.nan, 5.06, np.nan, 6.81, np.nan, 10.85]]
    # USTAR: (2, 2, 8) 2 UEs, 2 BS Ant, 8 locations
    mean_snr_ustar = [[[13.48, 4.41, np.nan, 21.58, np.nan, 7.21, np.nan, 12.32], [11.63, 8.69, np.nan, 21.26, np.nan, 8.49, np.nan, 4.34]],
                      [[6.85, 5.15, np.nan, 17.99, np.nan, 4.48, np.nan, 5.12], [16.07, 12.26, np.nan, 18.35, np.nan, 5.54, np.nan, 12.75]]]
    mean_snr_honors = np.array(mean_snr_honors)
    mean_snr_ustar = np.array(mean_snr_ustar)


    n_ue = 2
    c = []
    fig, axes = plt.subplots(n_ue, 1)
    for ue, ax in enumerate(axes):
        c.append(ax.imshow(mean_snr_ustar[ue, :, :], vmin=0, vmax=25, cmap='Blues',
                                        interpolation='nearest',
                                        #extent=[n_frm_st, n_frm_end, n_ant, 0],
                                        aspect="auto"))
        ax.set_title('UE {}'.format(ue))
        ax.set_ylabel('BSAntenna #')
        ax.set_xlabel('Location')
        #ax.set_xticks(np.arange(0, 8, 1), ['1', '2', '3', '4', '5', '6', '7', '8'])
        #ax.set_yticks(np.arange(0, 2, 1), ['0', '1'])
        ax.set_xticks(np.arange(0, 8, 1))
        ax.set_yticks(np.arange(0, 2, 1))
        ax.set_xticklabels(['1', '2', '3', '4', '5', '6', '7', '8'])
        ax.set_yticklabels(['0', '1'])
        ax.grid(which='minor', color='0.75', linestyle='-', linewidth=0.05)
        for (j,i),label in np.ndenumerate(mean_snr_ustar[ue,:,:]):
            if label > 13.5:
                ax.text(i,j,label,ha='center',va='center', color='white')
            else:
                ax.text(i,j,label,ha='center',va='center', color='black')

    cbar = plt.colorbar(c[-1], ax=axes.ravel().tolist(), orientation='horizontal', label='Averge SNR (dB)')
    fig.suptitle('USTAR')
    plt.draw()

    
    plt.figure()
    #plt.imshow(mean_snr_honors, interpolation='nearest', cmap='Blues', origin='lower', aspect="auto")  
    plt.imshow(mean_snr_honors, interpolation='nearest', cmap='Blues', aspect="auto")  
    plt.xticks(range(8), ['1', '2', '3', '4', '5', '6', '7', '8'])
    plt.yticks(range(2))
    #ax.set_xticklabels(['1', '2', '3', '4', '5', '6', '7', '8'])
    #plt.set_yticklabels(['0', '1'])
    plt.grid(which='minor', color='0.75', linestyle='-', linewidth=0.05)
    for (j,i),label in np.ndenumerate(mean_snr_honors):
        if label > 13.5:
            plt.text(i,j,label,ha='center',va='center', color='white')
        else:
            plt.text(i,j,label,ha='center',va='center', color='black')

    plt.title('Honors')
    plt.xlabel('Location')
    plt.ylabel('UE')
    cbar = plt.colorbar(orientation='horizontal', label='Averge SNR (dB)')
    plt.draw()

    plt.show()

    return



def main():
    #process_round1_collection()
    process_round2_collection()


if __name__ == '__main__':
    main()

