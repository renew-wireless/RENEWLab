"""
 print_sensor.py

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import numpy as np
import math


def print_sensor(irises, *args):

    """
    Print sensor values from array of Irises
    Examples of Args:
        ZYNQ_TEMP
        LMS7_TEMP
        FE_TEMP

    Usage Example: printSensor([sdr], 'LMS7_TEMP')
    """

    try:
        iter(irises)
    except TypeError:
        irises = [irises]
    info = irises[0].getSensorInfo(*args)
    name, units = info.name, info.units
    out = name.ljust(25)
    for iris in irises:
        value = iris.readSensor(*args)
        out += ('%g'%float(value)).ljust(10) + " "
    out += units
    print(out)
