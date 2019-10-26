#!/bin/bash

HDF5_LIB=libhdf5-cpp-100
ITPP_LIB=libitpp-dev
if [[ `lsb_release -rs` == "16.04" ]]
then
    HDF5_LIB=libhdf5-cpp-11
fi
sudo apt -y install libhdf5-dev $HDF5_LIB $ITPP_LIB

BASEDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
LIBDIR="$BASEDIR"/CC/Sounder/lib 
mkdir -p $LIBDIR

mkdir -p deps
cd deps/
git clone https://github.com/nlohmann/json.git
cd json/
mkdir -p build/
cd build/
cmake ..
make -j
sudo make install
cd ../..
