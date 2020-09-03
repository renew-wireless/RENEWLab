import yagmail
import os


MAIL_SENDER = os.environ.get('MAIL_USERNAME')
MAIL_PASSWORD = os.environ.get('MAIL_PASSWORD')
MAIL_RECEIPIENTS = ['support@renew-wireless.org']
MAIL_SUBJECT = '[RENEW] Overheat warning!'
SERVER = os.environ.get('SERVER_NAME') or 'RENEW_SERVER'


def send_email(utc_datetime, irises_sensors, thresh):
    yag = yagmail.SMTP(user=MAIL_SENDER, password=MAIL_PASSWORD)
    msg = "Some Iris modules on the base station connected to Server " +\
      SERVER + " is over " + str(thresh) + " C.\n" + '<br>'\
      + utc_datetime

    for iris_sensors in irises_sensors:
        msg += '<br>'
        for sensor in iris_sensors:
            msg += sensor + ','

    yag.send(to=MAIL_RECEIPIENTS, subject=MAIL_SUBJECT, contents=msg)


