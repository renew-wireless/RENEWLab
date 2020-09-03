[![N|Solid](https://renew-wireless.org/figs/cropped-v4-qgkw8d-1024x320.png)](https://nodesource.com/products/nsolid)

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

# RENEW Dashboard

# Description
The *RENEW Dashboard* is a python-based Web GUI application. It supports the following functions:  

  1. Online monitoring of the RENEW base station and sending emails to support@renew-wireless.org if there is any device overheating. 
  2. Running the Sounder framework (TODO)
  3. Running the Argora framework (TODO)

> The design goal for the RENEW Dashboard is for base station health monitoring and 
> for users to rung different frameworks with ease. 

# New Features

  - Online monitoring of the RENEW base station
  - Sending emails to support@renew-wireless.org if there is any device overheating

# Resources

The RENEW Dashboard is developed with the following resources:

* [AngularJS] - HTML enhanced for web apps (TODO)
* [Flask] - Python-based backend
* [Flask extensions] - flask-socketio, flask-bootstrap, flask-sqlalchemy, etc. 
* [node.js] - evented I/O for the backend
* [jQuery] - JavaScript library for HTML

# Installation

### Install dependencies: 

```sh
$ cd web-gui
$ pip3 install -r requirements.txt
```

### Set environment variables for emailing:

```sh
$ export MAIL_SENDER=your_email_address
$ export MAIL_PASSWORD=your_email_password
```
Note: Recommend to use a google mail account and enable "allow less secure apps". 

### Set environment variables for security: 
```sh
$ export SERVER_NAME=your_server_name
$ export SECRET_KEY=your_preferred_server_secret_key
```
Note: For example, if using POWDER, your_test_server_name can be set to POWDER. If using a Rice University server, it can be set it to the connected server name. 

# How to run it? 

1. Open a terminal, ssh into the server and run the commands below. 
```sh
$ cd web-gui
$ python3 renew.py
```

2. Open another terminal, ssh into the server and run the command below to open a web browser. 
```sh
$ xdg-open 'http://localhost:3333'
```

# Development

Want to contribute? Great!



[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)


   [dill]: <https://github.com/joemccann/dillinger>
   [git-repo-url]: <https://github.com/joemccann/dillinger.git>
   [john gruber]: <http://daringfireball.net>
   [df1]: <http://daringfireball.net/projects/markdown/>
   [markdown-it]: <https://github.com/markdown-it/markdown-it>
   [Ace Editor]: <http://ace.ajax.org>
   [node.js]: <http://nodejs.org>
   [Twitter Bootstrap]: <http://twitter.github.com/bootstrap/>
   [jQuery]: <http://jquery.com>
   [@tjholowaychuk]: <http://twitter.com/tjholowaychuk>
   [express]: <http://expressjs.com>
   [AngularJS]: <http://angularjs.org>
   [Gulp]: <http://gulpjs.com>

   [PlDb]: <https://github.com/joemccann/dillinger/tree/master/plugins/dropbox/README.md>
   [PlGh]: <https://github.com/joemccann/dillinger/tree/master/plugins/github/README.md>
   [PlGd]: <https://github.com/joemccann/dillinger/tree/master/plugins/googledrive/README.md>
   [PlOd]: <https://github.com/joemccann/dillinger/tree/master/plugins/onedrive/README.md>
   [PlMe]: <https://github.com/joemccann/dillinger/tree/master/plugins/medium/README.md>
   [PlGa]: <https://github.com/RahulHP/dillinger/blob/master/plugins/googleanalytics/README.md>
