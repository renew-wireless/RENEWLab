#!/usr/bin/python
# -*- coding: UTF-8 -*-
""" 
        Author(s): C. Nicolas Barati nicobarati@rice.edu 
                Rahman Doost-Mohamamdy: doost@rice.edu
                Oscar Bejarano: obejarano@rice.edu

---------------------------------------------------------------------
        Copyright (c) 2018-2019, Rice University
        RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
import SoapySDR
from SoapySDR import *  # SOAPY_SDR_ constants
import numpy as np
import time
import os

#######################################
####### Hub Class:              #######
#######################################


class Hub_py:
        '''
                Iris python class. To act as a middle layer between Matlab classes and SoapySDRs
        '''

        def __init__(self,
                     serial_id=None,
                     ):

                if serial_id is not None:
                        self.sdr = SoapySDR.Device(
                            dict(driver="remote", serial=serial_id))
                        if self.sdr == None:
                            print("Error in initializing the hub!")
                else:
                        self.sdr = None

    # Set trigger:
        def set_trigger(self):
                self.sdr.writeSetting("TRIGGER_GEN", "")

        def sync_delays(self):
                '''Synchronise delays.'''
                self.sdr.writeSetting("SYNC_DELAYS", "")
