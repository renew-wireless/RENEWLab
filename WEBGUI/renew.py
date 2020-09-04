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
from flask_socketio import SocketIO, emit
from datetime import datetime
from iris_get_status import get_iris_sensor_readings, check_overheating
from emails import send_email


# Set this variable to "threading", "eventlet" or "gevent" to test the
# different async modes, or leave it set to None for the application to
# choose the best option based on installed packages.
async_mode = None


app = Flask(__name__)
app.config['SECRET_KEY'] = os.environ.get('SECRET_KEY') or 'RENEW_secret_key!'
socketio = SocketIO(app, async_mode=async_mode)
thread = None
thread_lock = Lock()
OVERHEAT_THRESH = 70.0  # degree C
SENSOR_REFRESH_RATE = 90  # The default is 2 minutes. It takes around 
                          # 30 seconds to scan all Irises, typically. 


def background_thread():
    """
    Send server generated events to web clients. 
    
    :return: None
    """
    socketio.emit('my_response',
                  {'data': get_iris_sensor_readings(),
                   'time': datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")},
                  namespace='/test')
    while True:
        socketio.sleep(90)
        irises_sensors = get_iris_sensor_readings()
        utc_datetime = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
        socketio.emit('my_response',
                      {'data': irises_sensors, 'time': utc_datetime},
                      namespace='/test')
        if check_overheating(irises_sensors, OVERHEAT_THRESH):
            send_email(utc_datetime, irises_sensors, OVERHEAT_THRESH)


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
          'time': datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")})


if __name__ == '__main__':
    # If you want to broadcast the web app in the network, 
    # socketio.run(app, debug=False, host="0.0.0.0", port=3333)
    
    # If you want to use the web app on the server runing this web app,  
    socketio.run(app, debug=False, host="localhost", port=3333)


