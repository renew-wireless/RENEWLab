#!/bin/bash
sudo apt -y install python-scipy python3-scipy python-matplotlib python3-matplotlib python-h5py python3-h5py

mkdir -p deps
cd deps/

git clone https://github.com/pytransitions/transitions.git transitions
cd transitions/
sudo python setup.py install
