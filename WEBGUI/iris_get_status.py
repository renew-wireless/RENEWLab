#!/usr/bin/python3
"""
 iris_get_status.py

 The library handling Iris sensors.

 Author(s):
             Min Zhang: min.zhang@rice.edu

-----------------------------------------------------------------------
 Copyright © 2018-2020. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
-----------------------------------------------------------------------
"""

import os
import SoapySDR


LMS7 = "LMS7 Temperature"
RX = "Rx temperature"
ZYNQ = "Zynq Temperature"
TX = "Tx temperature"
BS_IRIS_ONLY = os.environ.get('CHECK_BS_IRIS_ONLY') or '1'  # 0: BS & client


def get_iris_sns_on_network():
    """
    Scan the Irises on the network and return a list of serial numbers.

    :return irises: A list of Iris serial numbers detected on network.
    """
    
    for i in range(3):
        print("Try {}".format(i))
        irises = os.popen("SoapySDRUtil --find --sparse").read()
        # The above command sometimes returns empty. 
        if len(irises) != 0:
            break
    irises = irises.splitlines()
    # Filter out none Iris devices
    irises = list(filter(lambda iris: (iris.find("IRIS") != -1), irises))
    # Extract Iris serial numbers from the Iris list
    irises = list(map(lambda iris: iris[iris.find("RF"):], irises))

    return irises


"""
Error code: 
    -100.0: sdr.readSensor() error
    -101.0: this temperature does not exist in sensor readings
    -102.0: error when creating a SoapySDR instance
"""
def get_sensors():
    """
    Read the temperatures of all Irises on the network.

    :return irises_sensors: A list of Irises and theirs temperatures.
    """
    
    irises = get_iris_sns_on_network()
    irises_sensors = []
    for iris in irises:
        stat, status = {}, {}
        try:
            SoapySDR.Device.enumerate({'remote:timeout': '1000000'})
            sdr = SoapySDR.Device(dict(driver="iris", serial=iris))
            # Check only base station Irises. 
            if BS_IRIS_ONLY == '1':
                if "ue" not in sdr.getHardwareInfo()["fpga"]:
                    sensors = sdr.listSensors()
                    for sensor in sensors:
                        info = sdr.getSensorInfo(sensor)
                        try:
                            stat[info.name] = sdr.readSensor(sensor)
                        except Exception:
                            stat[info.name] = '-100.0'
                            pass
                        
                    status['sn'] = iris
                    status['LMS7temp'] = stat[LMS7] if LMS7 in stat else '-101.0'
                    status['ZYNQtemp'] = stat[ZYNQ] if ZYNQ in stat else '101.0'
                    status['TXtemp'] = stat[TX] if TX in stat else '-101.0'
                    status['RXtemp'] = stat[RX] if RX in stat else '-101.0'
                    status['LMS7temp'] = "{:.2f}".format(float(status['LMS7temp']))
                    status['ZYNQtemp'] = "{:.2f}".format(float(status['ZYNQtemp']))
                    status['TXtemp'] = "{:.2f}".format(float(status['TXtemp']))
                    status['RXtemp'] = "{:.2f}".format(float(status['RXtemp']))
                    irises_sensors.append(status)
            else:
                sensors = sdr.listSensors()
                for sensor in sensors:
                    info = sdr.getSensorInfo(sensor)
                    try:
                        stat[info.name] = sdr.readSensor(sensor)
                    except Exception:
                        stat[info.name] = '-100.0'
                        pass
                        
                status['sn'] = iris
                status['LMS7temp'] = stat[LMS7] if LMS7 in stat else '-101.0'
                status['ZYNQtemp'] = stat[ZYNQ] if ZYNQ in stat else '101.0'
                status['TXtemp'] = stat[TX] if TX in stat else '-101.0'
                status['RXtemp'] = stat[RX] if RX in stat else '-101.0'
                status['LMS7temp'] = "{:.2f}".format(float(status['LMS7temp']))
                status['ZYNQtemp'] = "{:.2f}".format(float(status['ZYNQtemp']))
                status['TXtemp'] = "{:.2f}".format(float(status['TXtemp']))
                status['RXtemp'] = "{:.2f}".format(float(status['RXtemp']))
                irises_sensors.append(status)
        except:
            status['sn'] = iris
            status['LMS7temp'] = '-102.0'
            status['ZYNQtemp'] = '-102.0'
            status['TXtemp'] = '-102.0'
            status['RXtemp'] = '-102.0'
            irises_sensors.append(status)
            pass

        # irises_sensors.append(status)

    return irises_sensors


def check_overheat(irises_sensors, thresh):
    """
    Check all Irises' temperatures against the overheat threshold. 
    
    :param irises_sensors: A list of Irises and theirs temperatures.
    :param overheat_thresh: The temperature threshold for sensor 
        overheating. 

    :return boolean: True if any Iris's sensor indicates an overheat; 
        False if all Irises' temperatures are within the range. 
    """
    
    thresh = float(thresh)
    for iris_sensors in irises_sensors:
        if (float(iris_sensors['LMS7temp']) > thresh) or \
                (float(iris_sensors['ZYNQtemp']) > thresh) or \
                (float(iris_sensors['TXtemp']) > thresh) or \
                (float(iris_sensors['RXtemp']) > thresh):
            return True

    return False


def check_errors(irises_sensors, thresh):
    """
    Check all Irises' temperatures against the error threshold. 
    
    :param irises_sensors: A list of Irises and theirs temperatures.
    :param error_thresh: The temperature threshold for sensor
        errors. 

    :return boolean: True if any Iris's sensor indicates errors;
        False if all Irises' temperatures are within the range. 
    """
    
    thresh = float(thresh)
    for iris_sensors in irises_sensors:
        if (float(iris_sensors['LMS7temp']) <= thresh) or \
                (float(iris_sensors['ZYNQtemp']) <= thresh) or \
                (float(iris_sensors['TXtemp']) <= thresh) or \
                (float(iris_sensors['RXtemp']) <= thresh):
            return True

    return False


if __name__ == "__main__":
    # get_iris_sns_on_network()
    irises_sensors = get_sensors()
    print(irises_sensors)
    print(len(irises_sensors))
    print("Overheating>50C {}".format(check_overheat(irises_sensors, "50.0")))
    print("Overheating>70C {}".format(check_overheat(irises_sensors, "70.0")))
    print("Error?<=0C {}".format(check_errors(irises_sensors, "0.0")))
    print("Error?<=-100C {}".format(check_errors(irises_sensors, "-100.0")))
    print("Error?<=-200C {}".format(check_errors(irises_sensors, "-200.0")))


