########################################################################################
################## Soapy-based Library for collecting calibration CSI ##################
########################################################################################


import SoapySDR
from SoapySDR import * #SOAPY_SDR_ constants
import numpy as np
import time
import os
import math
import sys
import json
from array import array
from type_conv import *

RF_RST_REG = 48
CORR_CONF = 60
CORR_RST = 64
CORR_THRESHOLD = 92
TDD_CONF_REG = 120
SCH_ADDR_REG = 136
SCH_MODE_REG = 140

class CSI:
    def __init__(self, bsdrs, csdrs, trig_dev, txBsStream, rxBsStream, rxClStream, ota_trig, ant, rate, symSamp, pilot, beacon, coeffs, beacon_weights, rf_roundtrip):

        self.pilot = pilot
        self.beacon = beacon
        self.coeffs = coeffs
        self.beacon_weights = beacon_weights
        self.symSamp = symSamp
        self.ota_trig = ota_trig
        self.rate = rate
        self.rf_roundtrip = rf_roundtrip

        self.bsdrs = bsdrs # base station sdrs 
        self.csdrs = csdrs # client sdrs

        self.trig_dev = trig_dev
        self.sdr_ant = ant # 1 or 2

        self.num_bsdrs = len(self.bsdrs)
        self.num_ants = ant * self.num_bsdrs
        self.num_csdrs = len(self.csdrs)
	
        self.debug_enabled = False

        self.bsRxSamps = [[np.array([0]*symSamp).astype(np.complex64) for r in range(self.num_csdrs)] for s in range(self.num_ants)]
        self.clRxSamps = [[np.array([0]*symSamp).astype(np.complex64) for r in range(self.num_csdrs)] for s in range(self.num_ants)]
        self.trigger_counts = np.array([0]*self.num_csdrs, np.int32)

        self.rxClStream = rxClStream 
        self.rxBsStream = rxBsStream
        self.txBsStream = txBsStream

        self.nSyms = 1+self.num_ants+1+self.num_csdrs+1+1 # additional R for noise

    def setup(self):

        # BS
        for i, sdr in enumerate(self.bsdrs):
            sched_main = "PG"+''.join("G"*i*self.sdr_ant)+"T"*self.sdr_ant+''.join("G"*(self.num_ants-(i+1)*self.sdr_ant))+"G"+''.join("R"*(len(self.csdrs)))+"G" 
            #print("BS node %d"%i)
            #print("sched_main   %s"%(sched_main))
            bconf = {"tdd_enabled": True, "frame_mode": "free_running", "symbol_size" : self.symSamp, "frames": [sched_main], "max_frame" : 1}
            sdr.writeSetting("TDD_CONFIG", json.dumps(bconf))
            sdr.writeSetting("TDD_MODE", "true")

        # Client
        for i, sdr in enumerate(self.csdrs):
            sched_main = "GG"+''.join("R"*self.num_bsdrs)+"G"+''.join("G"*i)+"P"+''.join("G"*(self.num_csdrs-(i+1)))+"G"
            #print("Client node %d"%i)
            #print("sched_main   %s"%(sched_main))
            cconf = {"tdd_enabled": True, "frame_mode": "triggered", "symbol_size" : self.symSamp, "frames": [sched_main], "max_frame" : 0}
            sdr.writeSetting("TDD_CONFIG", json.dumps(cconf))
            sdr.writeSetting("TDD_MODE", "true")

        for sdr in self.bsdrs+self.csdrs:
            sdr.writeSetting("TX_SW_DELAY", str(30))

        z = np.empty(self.symSamp).astype(np.complex64)
        if self.ota_trig:
            coe = cfloat2uint32(np.conj(self.coeffs), order='IQ') # FPGA correlator takes coefficients in QI order
            for sdr in self.csdrs:
                sdr.writeRegister("IRIS30", CORR_CONF, int("00004001", 16))  # enable the correlator, with zeros as inputs
                for i in range(128):
                    sdr.writeRegister("ARGCOE", i*4, 0)
                time.sleep(0.1)
                sdr.writeRegister("IRIS30", CORR_RST, 0x1)  # reset corr
                sdr.writeRegister("IRIS30", CORR_RST, 0x0)  # unrst corr
                sdr.writeRegister("IRIS30", CORR_THRESHOLD, 1)
                for i in range(128):
                    sdr.writeRegister("ARGCOE", i*4, int(coe[i]))

                sf_start = self.rf_roundtrip // self.symSamp
                sp_start = self.rf_roundtrip % self.symSamp
                #print("UE starting symbol and sample count (%d, %d)" % (sf_start, sp_start))
                # make sure to set this after TDD mode is enabled "writeSetting("TDD_CONFIG", ..."
                sdr.setHardwareTime(SoapySDR.ticksToTimeNs((sf_start << 16) | sp_start, self.rate), "TRIGGER")
        else:
            for sdr in self.csdrs:
                sdr.setHardwareTime(0, "TRIGGER")

        replay_addr = 0
        for i, sdr in enumerate(self.bsdrs):
            sdr.writeRegisters("TX_RAM_A", replay_addr, cfloat2uint32(self.beacon, order='IQ').tolist())
            if self.beacon_weights is not None:
                sdr.writeRegisters("TX_RAM_B", replay_addr, cfloat2uint32(self.beacon, order='IQ').tolist())
                sdr.writeRegisters("TX_RAM_WGT_A", replay_addr, self.beacon_weights[self.sdr_ant*i].tolist())
                if self.sdr_ant == 2:
                    sdr.writeRegisters("TX_RAM_WGT_B", replay_addr, self.beacon_weights[self.sdr_ant*i+1].tolist())

                sdr.writeRegister("RFCORE", 156, int(self.num_ants))
                sdr.writeRegister("RFCORE", 160, 1) # enable beamsweeping
            else: break # if beamsweep is not active, only send pilot from the first antenna
            sdr.setHardwareTime(0, "TRIGGER")

        for sdr in self.csdrs:
            sdr.writeRegisters("TX_RAM_A", replay_addr, cfloat2uint32(self.pilot, order='IQ').tolist())
            sdr.writeRegisters("TX_RAM_B", replay_addr, cfloat2uint32(z, order='IQ').tolist())

        ret = 0
        dummy = np.empty(self.symSamp).astype(np.complex64)
        dummy2 = np.empty(self.symSamp).astype(np.complex64)
        for r,sdr in enumerate(self.bsdrs):
            #if r != m: 
            while ret >= 0:
                sr = sdr.readStream(self.rxBsStream[r], [dummy, dummy2], self.symSamp, timeoutUs = 0)
                ret = sr.ret
            ret = 0

        for r,sdr in enumerate(self.csdrs):
            #if r != m: 
            while ret >= 0:
                sr = sdr.readStream(self.rxBsStream[r], [dummy, dummy2], self.symSamp, timeoutUs = 0)
                ret = sr.ret
            ret = 0

        # arm correlator in the clients
        if self.ota_trig:    
            for i, sdr in enumerate(self.csdrs):
                sdr.writeRegister("IRIS30", CORR_CONF, int("00004011", 16))  # enable the correlator, with inputs from adc

    def close(self):
        # disarm correlator in the clients
        if self.ota_trig:    
            for i, sdr in enumerate(self.csdrs):
                sdr.writeRegister("IRIS30", CORR_CONF, int("00004001", 16))  # enable the correlator, with inputs 0

        for r,sdr in enumerate(self.bsdrs):
            sdr.deactivateStream(self.txBsStream[r])
            sdr.deactivateStream(self.rxBsStream[r])

        for r,sdr in enumerate(self.csdrs):
            sdr.deactivateStream(self.rxClStream[r])

    def collectCSI(self):
        z = np.empty(self.symSamp).astype(np.complex64)
        dummy = np.empty(self.symSamp).astype(np.complex64)
        dummy2 = np.empty(self.symSamp).astype(np.complex64)
        cur_trigs = np.array([0]*self.num_csdrs, np.int32)

        for r,sdr in enumerate(self.bsdrs):
            sdr.activateStream(self.txBsStream[r])

        flags = 0
        for r, sdr in enumerate(self.bsdrs):
            sdr.activateStream(self.rxBsStream[r], flags, 0)

        flags = 0
        for r, sdr in enumerate(self.csdrs):
            sdr.activateStream(self.rxClStream[r], flags, 0)

        bad_read = False
        # schedule downlink pilot transmission
        curFr = self.bsdrs[0].getHardwareTime() & 0xFFFFFFFF00000000 
        #print("currentFrame %x"%curFr)
        tx_flags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST
        for r,sdr in enumerate(self.bsdrs):
            txTime = curFr + ((2+r*self.sdr_ant) << 16)
            #print("scheduling downlink at time %x"%txTime) 
            sr = sdr.writeStream(self.txBsStream[r], [self.pilot, z], self.symSamp, flags=tx_flags, timeNs=txTime)
            if self.sdr_ant == 2:
                txTime = curFr + ((3+r*self.sdr_ant+1) << 16)
                sr = sdr.writeStream(self.txBsStream[r], [z, self.pilot], self.symSamp, flags=tx_flags, timeNs=txTime)

        self.trig_dev.writeSetting("TRIGGER_GEN", "")

        # collect downlink pilots
        for m in range(self.num_ants):
            #if bad_read: break
            for k in range(self.num_csdrs):
                sr = self.csdrs[k].readStream(self.rxClStream[k], [self.clRxSamps[m][k], dummy], self.symSamp)
                print("PilotDn: m: %d, k %d ret %d"%(m,k,sr.ret))
                if sr.ret < self.symSamp:
                    bad_read = True

        if bad_read: print("Downlink bad read")
        bad_read = False

        # collect uplink pilots
        for k in range(self.num_csdrs):
            #if bad_read: break
            for r in range(self.num_bsdrs):
                sr = self.bsdrs[r].readStream(self.rxBsStream[r], [self.bsRxSamps[self.sdr_ant*r][k], dummy if self.sdr_ant==1 else self.bsRxSamps[self.sdr_ant*r+1][k]], self.symSamp)
                print("PilotUP: k: %d, m %d ret %d"%(k,r,sr.ret))
                if sr.ret < self.symSamp:
                    bad_read = True

        if bad_read: print("Uplink bad read")

        for k, sdr in enumerate(self.csdrs):
            cur_trigs[k] = sdr.readRegister("IRIS30", 92)
            if cur_trigs[k] == self.trigger_counts[k]: print("User %d missed current frame"%k)
            else: print("current trigger count %d"%cur_trigs[k])
            self.trigger_counts[k] = cur_trigs[k]

        return self.bsRxSamps, self.clRxSamps

class CalibCSI:
    def __init__(self, sdrs, trig_dev, txStream, rxStream, ant, numSamps, pilot):

        self.pilot = pilot
        self.numSamps = numSamps
        self.sdrs = sdrs
        self.trig_dev = trig_dev
        self.sdr_ant = ant # 1 or 2
        
        self.debug_enabled = True
        self.num_sdrs = len(self.sdrs)
        self.num_ants = ant * self.num_sdrs

        self.txStream = txStream
        self.rxStream = rxStream
	#read samples into this buffer
        self.sampsRx = [[np.empty(self.numSamps).astype(np.complex64) for r in range(self.num_ants)] for t in range(self.num_ants)]

    def setup(self):
        for sdr in self.sdrs: 
            sdr.writeSetting("TDD_CONFIG", json.dumps({"tdd_enabled":False}))
            sdr.writeSetting("TDD_MODE", "false")
        for r,sdr in enumerate(self.sdrs):
            sdr.activateStream(self.txStream[r])

    '''
	This function sends a pilot from ref to all other antennas at once, and schedules all other nodes to send pilots in turn to the ref. ref receives all, remove the gaurd and create the output uplink vector

    '''
    def collect_calib_pilots(self):
        ref_sdr = None
        z = np.array([0]*self.numSamps, dtype=np.complex64)
        dummy = np.empty(self.numSamps).astype(np.complex64)
        dummy2 = np.empty(self.numSamps).astype(np.complex64)
        txTime = self.trig_dev.getHardwareTime("")
        ret = 0
        for r,sdr in enumerate(self.sdrs):
            #if r != m: 
            while ret >= 0:
                sr = sdr.readStream(self.rxStream[r], [dummy, dummy2], self.numSamps, timeoutUs = 0)
                ret = sr.ret
            ret = 0
            
        for m in range(self.num_sdrs):
            ref_sdr = self.sdrs[m]
            flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST

            # transmit pilot from node m
            sr = ref_sdr.writeStream(self.txStream[m], [self.pilot, z], self.numSamps, flags)
            if sr.ret == -1:
                print("bad write")
            
            for r,sdr in enumerate(self.sdrs):
                if r != m: 
                    sdr.activateStream(self.rxStream[r], flags, 0, self.numSamps)

            self.trig_dev.writeSetting("TRIGGER_GEN", "")
            
            for r,sdr in enumerate(self.sdrs):
                if r != m:
                    sr = sdr.readStream(self.rxStream[r], [self.sampsRx[self.sdr_ant*m][self.sdr_ant*r], dummy if self.sdr_ant == 1 else self.sampsRx[self.sdr_ant*m][self.sdr_ant*r+1]], self.numSamps, timeoutUs=int(1e6))
                    if sr.ret != self.numSamps:
                        print("bad read")
                        #return sr, 0, self.sampsRx 
 
            if self.sdr_ant == 2: 
                
                for r,sdr in enumerate(self.sdrs):
                    if r != m: 
                        while ret >= 0:
                            sr = sdr.readStream(self.rxStream[r], [dummy, dummy2], self.numSamps, timeoutUs = 0)
                            ret = sr.ret
                        ret = 0
                        sdr.activateStream(self.rxStream[r], flags, 0, self.numSamps)

                # transmit pilot from node m
                sr = ref_sdr.writeStream(self.txStream[m], [z, self.pilot], self.numSamps, flags)
                if sr.ret == -1:
                    print("bad write")

                self.trig_dev.writeSetting("TRIGGER_GEN", "")
                
                for r,sdr in enumerate(self.sdrs):
                    if r != m: 
                        sr = sdr.readStream(self.rxStream[r], [self.sampsRx[2*m+1][2*r], self.sampsRx[2*m+1][2*r+1]], self.numSamps, timeoutUs=int(1e6))
                        if sr.ret < 0:
                            print("bad read")
                            return sr, 0, self.sampsRx 
            
        timeDiff = self.trig_dev.getHardwareTime("") - txTime
 
        return None, timeDiff, self.sampsRx


    def sample_cal(self, offset, ref_ant, target_offset=0, forward=True):

        ref_offset = 1 if ref_ant == 0 else 0
        if forward:
            rel_samp_offset = offset[ref_offset] - offset
            synced = True
            for i, sdr in enumerate(self.sdrs):
                if i == ref_ant or i == ref_offset: continue
                ioffset = int(rel_samp_offset[i])       # ignore reference node
                num_iter = abs(ioffset)
                if offset[i] == 0: 
                    print("Bad offset, skip adjusting delay of Node %d"%(i))
                    synced = False
                elif ioffset > 0:
                    print("Adjusting delay of Node %d with %d samples"%(i, ioffset))
                    synced = False
                    for j in range(num_iter):
                        self.sdrs[i].writeSetting("ADJUST_DELAYS","1")
                elif ioffset < 0:
                    print("Adjusting delay of Node %d with %d samples"%(i, ioffset))
                    synced = False
                    for j in range(num_iter):
                        self.sdrs[i].writeSetting("ADJUST_DELAYS","-1")
                else:
                    print("Node %d is synced with ref_ant %d"%(i, ref_offset))
        else:
            rel_samp_offset = target_offset - offset[ref_offset]
            synced = True
            ioffset = int(rel_samp_offset)
            num_iter = abs(ioffset)
            if num_iter < 2: return synced
            if offset[ref_offset] == 0: 
                print("Bad offset, skip adjusting delay of ref node %d"%(ref_ant))
                synced = False
            elif ioffset > 0:
                print("Adjusting delay of ref node %d with %d samples"%(ref_ant, ioffset))
                synced = False
                for j in range(num_iter):
                    self.sdrs[ref_ant].writeSetting("ADJUST_DELAYS","1")
            elif ioffset < 0:
                print("Adjusting delay of ref node %d with %d samples"%(ref_ant, ioffset))
                synced = False
                for j in range(num_iter):
                    self.sdrs[ref_ant].writeSetting("ADJUST_DELAYS","-1")
            else:
                print("ref_ant %d is synced with ant %d"%(ref_ant, ref_offset))

        return synced 

    def close(self):
        [self.sdrs[r].deactivateStream(self.rxStream[r]) for r in range(len(self.sdrs))]
        [self.sdrs[r].deactivateStream(self.txStream[r]) for r in range(len(self.sdrs))]

