import os
import socket
basedir = os.path.abspath(os.path.dirname(__file__))


class Config(object):
    SECRET_KEY = os.environ.get('SECRET_KEY') or 'renewdashboardsecretkey123@'
    SQLALCHEMY_DATABASE_URI = os.environ.get('DATABASE_URL') or \
        'sqlite:///' + os.path.join(basedir, 'renew.db')
    SQLALCHEMY_TRACK_MODIFICATIONS = False


