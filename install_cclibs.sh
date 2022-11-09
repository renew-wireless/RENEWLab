#!/bin/bash

HDF5_LIB=libhdf5-cpp-100
if [[ `lsb_release -rs` == "16.04" ]]
then
    HDF5_LIB=libhdf5-cpp-11
fi
if [[ $(lsb_release -rs) == "20.04" ]]
then
    HDF5_LIB=libhdf5-cpp-103
fi
if [[ $(lsb_release -rs) == "22.10" ]]
then
    HDF5_LIB=libhdf5-cpp-103
fi

sudo apt -y install make cmake g++ libhdf5-dev $HDF5_LIB libgflags-dev
