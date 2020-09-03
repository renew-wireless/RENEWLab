import os
import SoapySDR


LMS7 = "LMS7 Temperature"
RX = "Rx temperature"
ZYNQ = "Zynq Temperature"
TX = "Tx temperature"


def get_iris_serial_numbers_on_network():
    irises = os.popen("SoapySDRUtil --find --sparse").read()
    irises = irises.splitlines()
    # filter out none iris devices
    irises= list(filter(lambda iris: (iris.find("IRIS") != -1), irises))
    # extract iris serial numbers from the iris list
    irises = list(map(lambda iris: iris[iris.find("RF"):], irises))
    
    return irises


def get_iris_sensor_readings():
    irises = get_iris_serial_numbers_on_network()
    SoapySDR.Device.enumerate({"remote:timeout": "500000"})
    sdrs = [SoapySDR.Device(dict(driver="iris", serial=s)) for s in irises]
    irises_sensors = []
    status = {}
    for i, sdr in enumerate(sdrs):
        sensors = sdr.listSensors()
        for sensor in sensors:
            info = sdr.getSensorInfo(sensor)
            try:
                status[info.name] = sdr.readSensor(sensor)
            except Exception:
                status[info.name] = '-1.0'
                pass
        irises_sensors.append([irises[i],
                       "{:.2f}".format(float(status[LMS7])),
                       "{:.2f}".format(float(status[ZYNQ])),
                       "{:.2f}".format(float(status[TX])),
                       "{:.2f}".format(float(status[RX]))])

    return irises_sensors


def check_overheating(irises_sensors, overheat_thresh):
    for iris_sensors in irises_sensors:
        for sensor in iris_sensors[1:]:
            if float(sensor) > overheat_thresh:
                return True

    return False


if __name__ == "__main__":
    irises_sensors = get_iris_sensor_readings()
    print(irises_sensors)

    print("Overheating? {}".format(check_overheating(irises_sensors, 50.0)))


