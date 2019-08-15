#!/usr/bin/python3
"""
 digital_rssi.py

 Read Digital RSSI values computed by LMS7

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import time
import numpy as np


def setUpDigitalRssiMode(sdr):
    """
        Setup to enable digital RSSI readings
    """
    # Select chain to read from (A or B)
    # Enable SPI read/write chain A
    # write(name, address, value)
    sdr.writeRegister("LMS7IC", 0x0020, sdr.readRegister("LMS7IC", 0x0020) & 0xFFFC)  # clear
    sdr.writeRegister("LMS7IC", 0x0020, sdr.readRegister("LMS7IC", 0x0020) | 0x0001)  # write
    # Don't bypass AGC module - 0x040C[6] = 0
    sdr.writeRegister("LMS7IC", 0x040C, sdr.readRegister("LMS7IC", 0x040C) & 0xFFBF)
    # Set AGC mode to RSSI - 0x040A[13:12] = 1
    sdr.writeRegister("LMS7IC", 0x040A, sdr.readRegister("LMS7IC", 0x040A) & 0xCFFF)  # clear
    sdr.writeRegister("LMS7IC", 0x040A, sdr.readRegister("LMS7IC", 0x040A) | 0x1000)  # write
    # Set RSSI Mode to Normal operation (default) - 0x040A[15:14] = 0
    sdr.writeRegister("LMS7IC", 0x040A, sdr.readRegister("LMS7IC", 0x040A) & 0x3FFF)  # clear
    # Select RSSI value to be captured into 0x040E, 0x040F registers - 0x0400[14:13] = 0
    sdr.writeRegister("LMS7IC", 0x0400, sdr.readRegister("LMS7IC", 0x0400) & 0x9FFF)
    # Select how many samples will be used to calculate RSSI - 0x040A[2:0] = any value from 0 to 7
    # Set to 3: Clear:0xFFF8 Write:0x0003
    # Set to 1: Clear:0xFFF8 Write:0x0001
    # Set to 0: Clear:0xFFF8 Write:0x0000
    # from 2^7 to 2^14 samples
    sdr.writeRegister("LMS7IC", 0x040A, sdr.readRegister("LMS7IC", 0x040A) & 0xFFF8)  # clear
    sdr.writeRegister("LMS7IC", 0x040A, sdr.readRegister("LMS7IC", 0x040A) | 0x0003)  # write


def getDigitalRSSI(sdr, agc_avg):
    """
        Read the digital RSSI registers (18-bits total)
        agc_avg: NO LONGER NEEDED - Should be a value between 0 and 7. The system takes a window of 2^(agc_avg+7) samples
        Return: digital RSSI value and Power in dBm
    """
    # Trigger read (trigger rising edge of CAPTURE) - 0x0400[15] = 0 then 0x0400[15] = 1  1100
    sdr.writeRegister("LMS7IC", 0x0400, sdr.readRegister("LMS7IC", 0x0400) & 0x7FFF)  # clear
    time.sleep(.01)
    sdr.writeRegister("LMS7IC", 0x0400, sdr.readRegister("LMS7IC", 0x0400) | 0x8000)  # write
    # Read Digital RSSI samples (peak voltage: sqrt(I^2 + Q^2))
    rssi = ((sdr.readRegister("LMS7IC", 0x040F) << 2) | (
            sdr.readRegister("LMS7IC", 0x040E) & 0x3)) & 0x3FFFF

    # Vrms = (rssi / 2.0 ** 18) * (1 / np.sqrt(2.0))  # Vrms = Vpeak/sqrt(2) (In Volts)
    # PWRrms = (Vrms ** 2.0) / 50.0                   # 50 Ohms load (PWRrms in Watts) UNKNOWN!!!
    # PWRdBm = 10.0 * np.log10(PWRrms) + 30           # P(dBm)=10*log10(Prms/1mW)   OR   P(dBm)=10*log10(Prms)+30

    maxRSSI = 0x15FF4
    if rssi == 0:
        rssi = 1
    rssi_pwr_dBFs = 20 * np.log10(float(rssi)/maxRSSI)  # NOTE: in dBFs (max RSSI according to LimeSuite)
    return rssi, rssi_pwr_dBFs


def main():
    setUpDigitalRssiMode()
    agc_avg = 3
    rssi, PWRdBFS = getDigitalRSSI(agc_avg)
    print("RSSI: " + str(rssi) + " PWRdBm: " + str(PWRdBFS))


if __name__ == '__main__':
    main()
