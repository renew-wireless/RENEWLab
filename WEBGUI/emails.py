#!/usr/bin/python3
"""
 emails.py

 The library handling emails.

 Author(s):
             Min Zhang: min.zhang@rice.edu

-----------------------------------------------------------------------
 Copyright © 2018-2020. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
-----------------------------------------------------------------------
"""

import yagmail
import os
import socket


MAIL_SENDER = os.environ.get('MAIL_USERNAME') or 'renew.dashboard@gmail.com'
MAIL_PASSWORD = os.environ.get('MAIL_PASSWORD') or 'r123@d456'
RECEIPIENT = os.environ.get('MAIL_RECEIPIENT') or 'support@renew-wireless.org'
SERVER = os.environ.get('SERVER_NAME') or socket.gethostname()


def send_email_overheat(utc_datetime, irises_sensors, thresh):
    """
    Send an email of overheat warning from MAIL_SENDER to
    MAIL_RECEIPIENT.

    :param utc_datetime: UTC when the warning was generated.
    :param irises_sensors: A list of iris serial numbers and their
        corresponding temperatures on LMS7, Zynq, TX and RX sensors.
    :param thresh: The temperature threshold for sensor overheating.

    :return: None
    """
    
    yag = yagmail.SMTP(user=MAIL_SENDER, password=MAIL_PASSWORD)
    subject = "[RENEW] Overheat alert!"
    msg = 'Some Iris modules on the base station connected to Server ' + \
          SERVER + ' are over ' + str(thresh) + ' C.\n\nSensor Log: ' + \
          utc_datetime
    msg += '\n\nserial_number,LMS7_temp,ZYNQ_temp,TX_temp,RX_temp'

    for iris_sensors in irises_sensors:
        msg += '<br>' + iris_sensors['sn'] + ',' + iris_sensors['LMS7temp'] + \
               ',' + iris_sensors['ZYNQtemp'] + ',' + iris_sensors['TXtemp'] +\
               ',' + iris_sensors['RXtemp']

    msg += '<br>Error Description:\n-100.00: sdr.readSensor() error\n' + \
           '-101.00: does not exist in sensor readings\n-102.00: error ' + \
           'when creating a SoapySDR instance'
    
    yag.send(to=[RECEIPIENT], subject=subject, contents=msg)



def send_email_errors(utc_datetime, irises_sensors, thresh):
    """
    Send an email of Iris errors from MAIL_SENDER to MAIL_RECEIPIENT.

    :param utc_datetime: UTC when the warning was generated.
    :param irises_sensors: A list of iris serial numbers and their
        corresponding temperatures on LMS7, Zynq, TX and RX sensors.
    :param thresh: The threshold for sensor errors.

    :return: None
    """
    
    yag = yagmail.SMTP(user=MAIL_SENDER, password=MAIL_PASSWORD)
    subject = "[RENEW] Sensor error alert!"
    msg = 'Some Iris modules on the base station connected to Server ' + \
          SERVER + 'have errors.\n\nSensor Log: ' + utc_datetime
    msg += '\n\nserial_number,LMS7_temp,ZYNQ_temp,TX_temp,RX_temp'

    for iris_sensors in irises_sensors:
        msg += '<br>' + iris_sensors['sn'] + ',' + iris_sensors['LMS7temp'] + \
               ',' + iris_sensors['ZYNQtemp'] + ',' + iris_sensors['TXtemp'] +\
               ',' + iris_sensors['RXtemp']

    msg += '<br>Error Desciption:\n-100.00: sdr.readSensor() error\n' + \
           '-101.00: does not exist in sensor readings\n-102.00: error ' + \
           'when creating a SoapySDR instance'
    
    yag.send(to=[RECEIPIENT], subject=subject, contents=msg)


