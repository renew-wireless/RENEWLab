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


MAIL_SENDER = os.environ.get('MAIL_USERNAME')
MAIL_PASSWORD = os.environ.get('MAIL_PASSWORD')
MAIL_RECEIPIENTS = ['support@renew-wireless.org']
MAIL_SUBJECT = '[RENEW] Overheat warning!'
SERVER = os.environ.get('SERVER_NAME') or 'RENEW_SERVER'


def send_email(utc_datetime, irises_sensors, thresh):
    """
    Send an email of overheat warning from MAIL_SENDER to
    MAIL_RECEIPIENTS.

    :param utc_datetime: UTC when the overheat warning is generated.
    :param irises_sensors: A list of iris serial numbers and their
        corresponding temperatures on LMS7, Zynq, TX and RX sensors.
    :param thresh: The temperature threshold for sensor overheating.

    :return: None
    """
    yag = yagmail.SMTP(user=MAIL_SENDER, password=MAIL_PASSWORD)
    msg = "Some Iris modules on the base station connected to Server " +\
      SERVER + " is over " + str(thresh) + " C.\n" + '<br>'\
      + utc_datetime

    for iris_sensors in irises_sensors:
        msg += '<br>'
        for sensor in iris_sensors:
            msg += sensor + ','

    yag.send(to=MAIL_RECEIPIENTS, subject=MAIL_SUBJECT, contents=msg)

