# shellscript to run csi_analysis.py

python3 csi_analysis.py ../../CC/Sounder/logs/ 2 trace-uplink-2022-6-10-17-59-32_1_16x1_0_15_nlos_10.hdf5 trace-uplink-2022-6-10-16-15-22_1_16x1_0_15_nlos_2.hdf5 --signal-offset 154 --alpha 0.01 --ref-subcarr 26 --monitor-dist

