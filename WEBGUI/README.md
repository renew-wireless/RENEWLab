# RENEW Dashboard


# Description
The *RENEW Dashboard* is a python-based Web GUI application. It supports the following functions:  

  1. Online monitoring of the RENEW base station and sending email alerts. 
  2. Running the Sounder framework (TODO)
  3. Running the Argora framework (TODO)

> The design goal for the RENEW Dashboard is for base station health monitoring and for users to run different frameworks with ease. 


# Features

  - Online monitoring of the RENEW base stations
  - Sending emails to support@renew-wireless.org if there is any device overheating or error
  - Logging sensor readings to database


# Resources

The RENEW Dashboard is developed with the following resources:

* [Flask] - Python-based backend
* [Flask extensions] - Flask-SocketIO, Flask-Bootstrap, Flask-SQLAlchemy, Flask-WTF, Flask-Migrate, etc. 
* [node.js] - evented I/O for the backend
* [jQuery] - JavaScript library for HTML


# Installation On A RENEW Server

### 1. ssh into the server. 
### 2. Install dependencies: 

```sh
$ cd WEBGUI
$ pip3 install -r requirements.txt
```

### 3. Set environment variables for emailing:

```sh
$ export MAIL_SENDER=your_email_address
$ export MAIL_PASSWORD=your_email_password
```
Note: Recommend to use a google mail account and enable "allow less secure apps". If no environment variables are set, the appliation shall use the default email "renew.dashboard@gamil.com". 

### 4. Set environment variables for security: 
```sh
$ export SERVER_NAME=your_server_name
$ export SECRET_KEY=your_preferred_server_secret_key
```
Note: For example, if using POWDER, your_server_name can be set to POWDER. If using a Rice University server, it can be set to the connected server name. If no environment variables are set, the application shall extract and use the server name from the computer system, and set a default secret key. 

### 5. Set environment variables for configurations: 
```sh
$ export MAIL_RECEIPIENT=your_email_to_receive_alerts
$ export OVERHEAT_THRESH=your_desired_threshold_in_degree_C
$ export ERROR_THRESH=your_desired threshold
$ export SENSOR_REFRESH_RATE=your_desired_email_alert_rate_in_seconds
```
Note: If no environment variables are set, the application shall use the defaults: support@renew-wireless.org, 70, -100, 570, respectively. 

### 6. Set up database
```sh
$ export FLASK_APP=renew.py
$ flask db migrate -m "your_comments"
$ flask db upgrade
```
Note: Depending on your Python installation, you may need to use "python3 -m flask db ..." for the above commands. 


# How to run it? 

1. Open a terminal, ssh into the server and run the commands below. 
```sh
$ cd WEBGUI
$ python3 renew.py
```

2. Open another terminal, ssh into the server and run the command below to open a web browser. 
```sh
If listening on localhost, 
$ xdg-open 'http://localhost:3333'
```
Note: If listening on the public network, open http://ip-address-of-the-server:3333 on your web browser. 

3. Database file renew.db is in SQLite format. 
Note: Recommend to open with "DB Browser for SQLite". 


# Development

Want to contribute? Great! Email support@renew-wireless.org. 


