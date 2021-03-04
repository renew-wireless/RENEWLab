# RENEWLab

[![Build Status](https://falcon.ecg.rice.edu:443/buildStatus/icon?job=github_public_renewlab%2Fdevelop)](https://falcon.ecg.rice.edu:443/job/github_public_renewlab/job/develop/)

# Description
RENEWLab is an open-source software toolbox for the [RENEW massive MIMO platform](https://renew-wireless.org). It provides a user interface through a set of APIs. Users can generate, manipulate, transmit, and receive RF signals on the RENEW hardware by calling these APIs. 

> The design goal of RENEWLab is to provide the RENEW software library and a quick starting point for users to design, program, and run their experiments on the RENEW massive MIMO platform. 


# Components
The RENEWLab software suite consists of four components. 

  1. [Python Development Suite](https://docs.renew-wireless.org/dev-suite/design-flows/python-design-flow/): 
     It provides a Python-based library which allows users to rapidly program and test the physical layer and the radio layer in real time. It also provides tools for offline data procesing. 

  2. [MATLAB Development Suite](https://docs.renew-wireless.org/dev-suite/design-flows/matlab-design-flow/): 
     It provides a MATLAB-based library which allows users to rapidly develop physical layer algorithms using the MATLAB toolboxes with a highly simplified interface and to perform OTA tests.

  3. [C++ Development Suite](https://docs.renew-wireless.org/dev-suite/design-flows/cpp/): 
     It provides the Sounder framework for running channel sounding experiments.

  4. RENEW Dashboard: 
     It provides a web GUI for the RENEW base station health monitoring and for users to run different software frameworks with ease. Please refer to the README under the WEBGUI/ directory. 


# Installation
### Clone this repository: 
```sh
$ git clone https://github.com/renew-wireless/RENEWLab.git
```

### Install common dependencies: 
```sh
$ cd RENEWLab
$ ./config_ci.sh  Note: You must run this command immediately after the cd command if you are a developer.
$ ./install_soapy.sh
```
Note: This installs the SoapySDR app and its dependencies which include SoapySDR, SoapyRemote, and Sklk-SoapyIris. 
Important for developers: Do not use _ in your branch name. Use - instead. Limit the branch name length to 18 characters or less. 

### Install project-specific dependencies: 
  1. If you are going to use the RENEW Python Development Suite, please install its library as below. 
     ```sh
     $ ./install_pylibs.sh
     ```
  2. If you are going to use the RENEW C++ Development Suite, please install its library and dependencies as below. 
     ```sh
     $ ./install_cclibs.sh
     $ cd CC/Sounder/mufft/
     $ git submodule update --init
     $ cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ./ && make -j
     $ cd ../
     $ mkdir build
     $ cd build
     $ cmake .. -DCMAKE_BUILD_TYPE=Release -DLOG_LEVEL=info && make -j
     ```
       
  3. If you are going to use the RENEW MATLAB Development Suite, no dependency is needed.
  4. If you are going to use the RENEW Dashboard, please follow the README in the WEBGUI/ directory to install dependencies. 

# Contributing and Support

Want to contribute? Great! Please email support@renew-wireless.org. 

# License

RENEWLab shall be used under the [RENEW OPEN SOURCE LICENSE](https://renew-wireless.org/license).

# Acknowledgement

RENEW was funded by the NSF PAWR project.
