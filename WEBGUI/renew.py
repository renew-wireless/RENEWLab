#!/usr/bin/python3
"""
 renew.py

 The web application main file for the RENEW Dashboard.

 Author(s):
             Min Zhang: min.zhang@rice.edu

-----------------------------------------------------------------------
 Copyright © 2018-2020. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
-----------------------------------------------------------------------
"""

import os
from threading import Lock
from flask import Flask, render_template
from config import Config
from flask_sqlalchemy import SQLAlchemy
from flask_migrate import Migrate
from flask_socketio import SocketIO, emit
from datetime import datetime
from iris_get_status import get_sensors, check_overheat, check_errors
from emails import send_email_overheat, send_email_errors, SERVER


# Set this variable to "threading", "eventlet" or "gevent" to test the
# different async modes, or leave it set to None for the application to
# choose the best option based on installed packages.
async_mode = None


app = Flask(__name__)
app.config.from_object(Config)
db = SQLAlchemy(app)
migrate = Migrate(app, db)
socketio = SocketIO(app, async_mode=async_mode)
thread = None
thread_lock = Lock()
OVERHEAT_THRESH = os.environ.get('OVERHEAT_THRESH') or '70.0'  # degree Ci
ERROR_THRESH = os.environ.get('ERROR_THRESH') or '-100.0'  # error codes
SENSOR_REFRESH_RATE = os.environ.get('SENSOR_REFRESH_RATE') or '570'  # sec
# It takes about 30 sec to scan all Iris modules. Plus 570 is 10 minutes.

# DB table definition
class BaseStationIrisTemperature(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    timestamp = db.Column(db.String(30), index=True)
    servername = db.Column(db.String(30))
    iris_sn = db.Column(db.String(30))
    overheat_thresh = db.Column(db.String(30))
    lms7_temp = db.Column(db.String(30))
    zynq_temp = db.Column(db.String(30))
    tx_temp = db.Column(db.String(30))
    rx_temp = db.Column(db.String(30))
    overheat_warning = db.Column(db.Boolean())
    error_warning = db.Column(db.Boolean())

    def __repr__(self):
        return '<Iris {}>'.format(self.iris_sn)


def background_thread():
    """
    Send server generated events to web clients. 
    
    :return: None
    """
    
    socketio.emit('my_response',
                  {'data': get_sensors(),
                   'time': datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S"),
                   'server': SERVER},
                  namespace='/test')
    
    while True:
        socketio.sleep(int(SENSOR_REFRESH_RATE))
        irises_sensors = get_sensors()
        utc_datetime = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
        socketio.emit('my_response',
                      {'data': irises_sensors, 
                       'time': utc_datetime,
                       'server': SERVER},
                      namespace='/test')
        
        # log to dabase
        for iris_sensors in irises_sensors:
            iris = BaseStationIrisTemperature(timestamp=utc_datetime,
              servername=SERVER,
              iris_sn=iris_sensors['sn'],
              overheat_thresh=OVERHEAT_THRESH,
              lms7_temp=iris_sensors['LMS7temp'],
              zynq_temp=iris_sensors['ZYNQtemp'],
              tx_temp=iris_sensors['TXtemp'],
              rx_temp=iris_sensors['RXtemp'],
              overheat_warning=check_overheat(irises_sensors, OVERHEAT_THRESH),
              error_warning=check_errors(irises_sensors, ERROR_THRESH))
            db.session.add(iris)
            db.session.commit()

        # email alerts
        if check_overheat(irises_sensors, OVERHEAT_THRESH):
            send_email_overheat(utc_datetime, irises_sensors, OVERHEAT_THRESH)
        if check_errors(irises_sensors, ERROR_THRESH):
            send_email_errors(utc_datetime, irises_sensors, ERROR_THRESH)


@app.route('/')
def root():
    return render_template('root.html', async_mode=socketio.async_mode)


@app.errorhandler(404)
def page_not_found(e):
    return render_template('404.html'), 404


@app.errorhandler(500)
def internal_server_error(e):
    return render_template('500.html'), 500


@socketio.on('connect', namespace='/test')
def test_connect():
    global thread
    with thread_lock:
        if thread is None:
            thread = socketio.start_background_task(background_thread)
    emit('my_response',
         {'data': 'Connected',
          'time': datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S"),
          'server': SERVER})


if __name__ == '__main__':
    # If you want to broadcast the web app in the network, 
    socketio.run(app, debug=False, host="0.0.0.0", port=3333)
    
    # If you want to use the web app on the server runing this web app,  
    # socketio.run(app, debug=False, host="localhost", port=3333)


