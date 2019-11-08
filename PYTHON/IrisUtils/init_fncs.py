# -*- coding: UTF-8 -*-

"""
 init_fncns.py

Initialization for different functionalities

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

from macros import *


def agc_init(sdr, rssi_target_idx, agc_en):
    """
    AGC setup. Register setup

    ARGS:
        - sdr: SDR object
        - rssi_target_idx: Index of target RSSI. Higher value means lower RSSI threshold 

    RETURNS:
        - None
    """
    # AGC
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_ENABLE_FLAG, 0)           # Enable AGC Flag (set to 0 initially)
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_RESET_FLAG, 1)            # Reset AGC Flag
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_IQ_THRESH, 8000)              # Saturation Threshold: 10300 about -6dBm
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_NUM_SAMPS_SAT, 3)             # Number of samples needed to claim sat.
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_MAX_NUM_SAMPS_AGC, 10)        # Threshold at which AGC stops
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_WAIT_COUNT_THRESH, 20)        # Gain settle takes about 20 samps(value=20)
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_BIG_JUMP, 30)             # Drop gain at initial saturation detection
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_SMALL_JUMP, 3)            # Drop gain at subsequent sat. detections
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_RSSI_TARGET, rssi_target_idx) # RSSI Target for AGC: ideally around. Try 20 for 3.6GHz and 30 for 2.5GHz
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_TEST_GAIN_SETTINGS, 0)    # Enable only for testing gain settings
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_RESET_FLAG, 0)            # Clear AGC reset flag
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_ENABLE_FLAG, agc_en)
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_AGC_GAIN_INIT, 80)            # Initialize gains to this value

    # Pkt Detect Setup
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_THRESH, 500)            # RSSI value at which Pkt is detected
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NUM_SAMPS, 5)         # Number of samples needed to detect frame
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_ENABLE, 1)            # Enable packet detection flag
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NEW_FRAME, 1)         # Finished last frame? (set to 0 initially)
    sdr.writeRegister("IRIS30", FPGA_IRIS030_WR_PKT_DET_NEW_FRAME, 0)
