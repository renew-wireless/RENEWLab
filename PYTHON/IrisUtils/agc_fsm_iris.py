"""
 agc_fsm_iris.py

     Automatic Gain Control - Finite State Machine Implementation
        (specifically designed for the Skylark Iris boards)

     Dependencies: "Transitions" Module

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 ---------------------------------------------------------------------
"""

import sys
sys.path.append('./data_in/')
import SoapySDR
from SoapySDR import *  # SOAPY_SDR_ constants
import random as rand
import numpy as np
import logging
import csv
import time
from transitions import Machine


class AutomaticGainControl(object):
    """ AGC Finite State Machine """

    # States available
    states = ['idle', 'measPwrWait', 'satDet', 'wait']

    # Initialize other params
    targetRSSI = -15
    maxGain = 64             # Maximum possible gain achieved
    minGain = 0
    currentSample = 0
    enableAGCflag = 0        # Enable AGC flag
    numSampsProcess = 100    # Total number of samples to process (full frame)
    numSampThresh = 16       # Number of samples during which we measure RSSI to determine if saturation happened
    numSampAboveThresh = 3   # Number of samples needed to indicate saturation
    rssiSatThresh = -6       # RSSI saturation threshold (for each sample)
    gain = 65                # Start with max gain possible (for debug)
    numSatSamps = 0          # Saturated samples counter
    gainAdjustStage = 0      # Different gain adjustment stages (different gain reduction at each level)
    satDetGoodFlag = False   # If we've reached all the samples to be read for this frame, and no saturation, set flag
    satDetectedFlag = False  # Set if saturation has been detected
    waitCount = 0            # How many times we've entered the "wait" state (how many adjustments)
    maxNumSatAdjust = 3      # How many times are we adjusting gain for saturation
    maxNumFineAdjust = 3     # How many time are we triggering fine tuning
    fineAdjustCount = 0
    cumulativeGainAdjust = 0 # Keep track of cumulative gain adjustments
    P_BB_MAX = rssiSatThresh-4  # Upper BB threshold
    P_BB_MIN = P_BB_MAX-10      # Lower BB threshold
    gainLNA = 30.0           # Arbitrary - start off with some max initial fixed gain
    gainTIA = 12.0           # Arbitrary - start off with some max initial fixed gain
    gainPGA = 19.0           # Arbitrary - start off with some max initial fixed gain

    # Load Gain Table
    # fid = open("./data_in/gainTable_07-20-18.csv", "rb")
    fid = "../IrisUtils/data_in/gainTable_07-20-18.csv"
    gainTable = np.loadtxt(fid, delimiter=",", skiprows=1)

    # Init array
    rssi = np.empty(numSampsProcess).astype(np.float)

    def __init__(self, sdr, ch):

        # State Machine Transitions
        self.machine = Machine(             # Create machine
            model=self,
            states=self.states,
            initial='idle')
        self.machine.add_transition(        # Rule no. 1 (Idle-to-SatDet)
            trigger='newSample',
            source='idle',
            dest='satDet',
            before=['printState'],
            after=['printState'],
            conditions=[self.is_AGCenabled])
        self.machine.add_transition(        # Rule no. 2 (SatDet-to-Wait)
            trigger='newSample',
            source='satDet',
            dest='wait',
            before=['resetSaturation', 'setSatGain'],
            after=['printState', 'stageCounter', 'waitTimeout'],
            conditions=[self.is_saturationDetected])
        self.machine.add_transition(        # Rule no. 3 (SatDet-to-MeasurePwrWait)
            trigger='newSample',
            source='satDet',
            dest='measPwrWait',
            before='resetSaturation',
            after='printState',
            conditions=[self.is_satDetGood])
        self.machine.add_transition(        # Rule no. 4 (Wait-to-SatDet)
            trigger='gainDone',
            source='wait',
            dest='satDet',
            after='printState',
            conditions=[self.is_notFinalStage])
        self.machine.add_transition(        # Rule no. 5 (Wait-to-MeasurePwrWait)
            trigger='gainDone',
            source='wait',
            dest='measPwrWait',
            after='printState',
            conditions=[self.is_finalStage])
        self.machine.add_transition(        # Rule no. 6 (MeasurePwrWait-to-Wait)
            trigger='newSample',
            source='measPwrWait',
            dest='wait',
            before=['setFinalStage', 'fineTuning'],
            after=['printState', 'fineTuningCounter', 'waitTimeout'],
            conditions=[self.is_fineTuningNotDone])
        self.machine.add_transition(        # Rule no. 7 (MeasurePwrWait-to-Wait)
            trigger='newSample',
            source='measPwrWait',
            dest='idle',
            #after=['printState', 'disableAGC'],
            after=['printState', 'full_reset'],               # Keep AGC enabled
            conditions=[self.is_fineTuningDone])

        # Pass other object
        self.sdr = sdr
        self.ch = ch

    # Enable AGC flag
    def enableAGC(self):
        logging.debug("AGC ENABLED")
        self.enableAGCflag = True

    def disableAGC(self):
        logging.debug("AGC DISABLED")
        self.enableAGCflag = False

    def is_AGCenabled(self):
        logging.debug("IS AGC ENABLED? " + str(self.enableAGCflag))
        return self.enableAGCflag

    # Gain Adjustment After Saturation
    def setSatGain(self):
        # Read gains
        self.gainLNA = self.sdr.getGain(SOAPY_SDR_RX, self.ch, 'LNA')
        self.gainTIA = self.sdr.getGain(SOAPY_SDR_RX, self.ch, 'TIA')
        self.gainPGA = self.sdr.getGain(SOAPY_SDR_RX, self.ch, 'PGA')
        if self.gainAdjustStage == 0:
            self.cumulativeGainAdjust = self.cumulativeGainAdjust + 20
            logging.debug("FIRST DROP")

        elif self.gainAdjustStage == 1:
            self.cumulativeGainAdjust = self.cumulativeGainAdjust + 10
            logging.debug("SECOND DROP")

        elif self.gainAdjustStage == 2:
            self.cumulativeGainAdjust = self.cumulativeGainAdjust + 10
            logging.debug("THIRD DROP")

        else:
            print("INVALID!")

        # Find gain settings for this new gain from table
        adjustedGain = self.maxGain - self.cumulativeGainAdjust
        logging.debug("CUMULATIVE GAIN (SAT): {}".format(self.cumulativeGainAdjust))
        if adjustedGain < self.minGain:
            adjustedGain = self.minGain
            logging.debug("CROSSED MIN GAIN THRESH... SET GAIN TO MIN (SAT STAGE)")
        if adjustedGain > self.maxGain:
            adjustedGain = self.maxGain
            logging.debug("CROSSED MAX GAIN THRESH... SET GAIN TO MAX (SAT STAGE)")

        # [Gain column, NF column, LNA column, TIA column, PGA column]
        gain_settings = self.gainTable[self.gainTable[:, 0] == adjustedGain, [2, 3, 4]]
        self.gainLNA = int(gain_settings[0])
        self.gainTIA = int(gain_settings[1])
        self.gainPGA = int(gain_settings[2])

        self.sdr.setGain(SOAPY_SDR_RX, self.ch, 'LNA', self.gainLNA)  # [0:1:30]
        self.sdr.setGain(SOAPY_SDR_RX, self.ch, 'TIA', self.gainTIA)  # [0, 9, 12]
        self.sdr.setGain(SOAPY_SDR_RX, self.ch, 'PGA', self.gainPGA)  # [-12:1:19]
        time.sleep(0.1)
        aggrGain = self.gainLNA + self.gainTIA + np.absolute(self.gainPGA)
        logging.debug("SATURATION-TOTAL: {} \t LNA: {} \t TIA: {} \t PGA: {}".format(aggrGain, self.gainLNA,
                                                                                   self.gainTIA, self.gainPGA))

    # Initiate fine-tuning
    def fineTuning(self):
        # self.gainLNA = self.sdr.getGain(SOAPY_SDR_RX, self.ch, 'LNA')
        # self.gainTIA = self.sdr.getGain(SOAPY_SDR_RX, self.ch, 'TIA')
        # self.gainPGA = self.sdr.getGain(SOAPY_SDR_RX, self.ch, 'PGA')
        currRSSI = self.rssi[self.currentSample]

        # How far are we from the target?
        if currRSSI < 0:
            rssiDiff = np.absolute(self.targetRSSI) - np.absolute(currRSSI)
        else:
            print(" !!!!!!!!! STILL SATURATED, THIS SHOULDN'T HAPPEN !!!!!!!! ")

        # If rssiDiff > 0, gain drops by rssiDiff
        # If rssiDiff < 0, gain increases by rssiDiff
        self.cumulativeGainAdjust += int(rssiDiff)
        logging.debug("**FINE TUNING** RSSI DIFF: " + str(rssiDiff))

        # Find gain settings for this new gain from table
        adjustedGain = self.maxGain - self.cumulativeGainAdjust
        logging.debug("CUMULATIVE GAIN (FINE TUNE): {}".format(self.cumulativeGainAdjust))
        if adjustedGain < self.minGain:
            adjustedGain = self.minGain
            logging.debug("CROSSED MIN GAIN THRESH... SET GAIN TO MIN (FINE TUNE STAGE)")
        if adjustedGain > self.maxGain:
            adjustedGain = self.maxGain
            logging.debug("CROSSED MAX GAIN THRESH... SET GAIN TO MAX (FINE TUNE STAGE)")

        # [Gain column, NF column, LNA column, TIA column, PGA column]
        gain_settings = self.gainTable[self.gainTable[:, 0] == adjustedGain, [2, 3, 4]]
        self.gainLNA = int(gain_settings[0])
        self.gainTIA = int(gain_settings[1])
        self.gainPGA = int(gain_settings[2])

        self.sdr.setGain(SOAPY_SDR_RX, self.ch, 'LNA', self.gainLNA)  # [0:1:30]
        self.sdr.setGain(SOAPY_SDR_RX, self.ch, 'TIA', self.gainTIA)  # [0, 9, 12]
        self.sdr.setGain(SOAPY_SDR_RX, self.ch, 'PGA', self.gainPGA)  # [-12:1:19]
        time.sleep(0.1)
        aggrGain = self.gainLNA + self.gainTIA + np.absolute(self.gainPGA)
        logging.debug("FINETUNE-TOTAL: {} \t LNA: {} \t TIA: {} \t PGA: {}".format(aggrGain, self.gainLNA,
                                                                                   self.gainTIA, self.gainPGA))
        # print("SET GAIN - TOTAL: {} \t LNA: {} \t TIA: {} \t PGA: {}".format(aggrGain, self.gainLNA,
        #                                                                           self.gainTIA, self.gainPGA))

    # Count number of fine tuning adjustments
    def fineTuningCounter(self):
        self.fineAdjustCount += 1
        logging.debug("FineAdjustmentCount: " + str(self.fineAdjustCount))

    # Check if we have reached max number of fine gain adjustments
    def is_fineTuningDone(self):
        if self.fineAdjustCount >= self.maxNumFineAdjust:
            return True

    # Check if we have reached max number of fine gain adjustments
    def is_fineTuningNotDone(self):
        if self.fineAdjustCount < self.maxNumFineAdjust:
            return True

    # Check if saturation detected flag is set
    def is_saturationDetected(self):
        return self.satDetectedFlag

    # Once we reach this stage we should stay within the fine tuning stage only
    def setFinalStage(self):
        logging.debug("SET FINAL STAGE")
        self.gainAdjustStage = self.maxNumSatAdjust

    # Reset Saturation Counter After Each Saturation Detection Stage
    def resetSaturation(self):
        self.satDetectedFlag = False
        logging.debug("RESET SATURATION SAMPLES COUNT")

    # We've reached the max number of samples and no saturation...
    def is_satDetGood(self):
        return self.satDetGoodFlag

    # Count how many times we've been in "wait" state
    def stageCounter(self):
        self.gainAdjustStage += 1
        logging.debug("STAGE COUNTER: " + str(self.gainAdjustStage))

    # Time to wait within "wait" state
    def waitTimeout(self):
        logging.debug("START/END WAIT TIMEOUT")
        # 100ms wait - settle gain
        time.sleep(0.10)
        self.gainDone()

    # Check if final wait state
    def is_finalStage(self):
        if self.gainAdjustStage >= self.maxNumSatAdjust:
            return True

    # Check if not final wait state
    def is_notFinalStage(self):
        if self.gainAdjustStage < self.maxNumSatAdjust:
            return True

    def full_reset(self):
        self.currentSample = 0
        self.enableAGCflag = 0        # Enable AGC flag
        self.numSatSamps = 0          # Saturated samples counter
        self.gainAdjustStage = 0      # Different gain adjustment stages (different gain reduction at each level)
        self.satDetGoodFlag = False   # If we've reached all the samples to be read for this frame, and no saturation, set flag
        self.satDetectedFlag = False  # Set if saturation has been detected
        self.waitCount = 0            # How many times we've entered the "wait" state (how many adjustments)
        self.fineAdjustCount = 0
        self.cumulativeGainAdjust = 0 # Keep track of cumulative gain adjustments
        self.rssi = np.empty(self.numSampsProcess).astype(np.float)
        self.disableAGC()

    # Current state?
    def printState(self):
        logging.debug("State: " + self.state)
