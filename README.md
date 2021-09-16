# RENEWLab

[![Build Status](https://falcon.ecg.rice.edu:443/buildStatus/icon?job=github_public_renewlab%2Fbugfix-patchPOWDER)](https://falcon.ecg.rice.edu:443/job/github_public_renewlab/job/bugfix-patchPOWDER/)

# Contents
 * [Description](#description)
 * [Components](#components)
 * [Collect and Process Channel Datasets](#collect-and-process-channel-datasets)
 * [Contributing and Support](#contributing-and-support)
 * [Documentation](#documentation)
 * [License](#license)
 * [Acknowledgement](#acknowledgement)

# Description
RENEWLab is an open-source software toolbox for the [RENEW massive MIMO platform](https://renew-wireless.org). It provides a user interface through a set of APIs. Users can generate, manipulate, transmit, and receive RF signals on the RENEW hardware by calling these APIs. 

> The design goal of RENEWLab is to provide the RENEW software library and a quick starting point for users to design, program, and run their experiments on the RENEW massive MIMO platform. 


# Components
The RENEWLab software suite consists of four components. 

  1. [Python Development Suite](https://docs.renew-wireless.org/dev-suite/design-flows/python-design-flow/): 
     It provides a Python-based library which allows users to rapidly program and test the physical layer and the radio layer in real time. It also provides tools for offline data processing. 

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
     ```   
  3. If you are going to use the RENEW MATLAB Development Suite, no dependency is needed.
  4. If you are going to use the RENEW Dashboard, please follow the README in the WEBGUI/ directory to install dependencies.

# Collect and Process Channel Datasets

To collect channel dataset, use the Sounder software (C++ Development Suite). Currently the software works with Faros massive MIMO base station and Iris030 SDR Clients, both commercially available from [Skylark Wireless](https://www.skylarkwireless.com). The software partially supports USRP radios as well. For the software to work [SoapyUHD](https://github.com/pothosware/SoapyUHD) plugin is required.

 To start, first compile the Sounder C++ project:
```sh
$ cd CC/Sounder/mufft/
$ git submodule update --init
$ cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ./ && make -j
$ cd ../
$ mkdir build
$ cd build
$ cmake .. -DCMAKE_BUILD_TYPE=Release -DLOG_LEVEL=info && make -j
$ cd ../
```
Once compiled successfully, the code can be run as the following.

 1. If your JSON file includes uplink data transmission, first generate the files, including a random bits source file, as below:
     ```sh
     $ ./build/sounder -conf PATH_TO_JSON_CONFIG_FILE -gen_ul_bits
     ```   
 2. Next, to start collecting channel measurement data, run the hardware discovery tool Pyfaros to create a file containing the serials of the boards to be used, and run the Sounder software as follows:
     ```sh
     $ cd ./files/
     $ python3 -m pyfaros.discover --json-out
     $ cd ../
     $ ./build/sounder -conf PATH_TO_JSON_CONFIG_FILE
     ```   
 3. To store the dataset and bits source files in a specific directory use the `-storepath` switch:
     ```sh
     $ ./build/sounder -conf PATH_TO_JSON_CONFIG_FILE -storepath PATH_TO_DIRECTORY -gen_ul_bits
     $ ./build/sounder -conf PATH_TO_JSON_CONFIG_FILE -storepath PATH_TO_DIRECTORY
     ```   
 4. The dataset file generated in the specified directory with a software generated file name including a time stamp, e.g. `trace-uplink-2021-4-16-18-41-21_1x8x2_0_7.hdf5`. You can plot different dimension the dataset, such as the pilot of uplink data received on select base station antennas, or from select users, using the `plot_hdf5.py` tool in the PYTHON directory. For example:
     ```sh
     $ ../../PYTHON/IrisUtils/plot_hdf5.py PATH_TO_DATASET_FILE # add command line options
     ```   
 5. For more info on how to use these tools including all the options available for dataset processing as well as other tools available in the RENEWLab codebase, visit the [RENEW Documentation](https://docs.renew-wireless.org) website.

# Contributing and Support

Want to contribute? Great! Please email support@renew-wireless.org. 

# Documentation

Doxygen documentation generation for RENEWLab can be initiated by running the following command from the repository root directory:
`doxygen RENEWLab_doxygen.conf`.  
The latest hosted output is located at [RENEWLab Doxygen](https://renew-wireless.org/renewlab-doxy/html/index.html).  
Other community resources can be found at the [RENEW Wireless Wiki](https://wiki.renew-wireless.org/)  

# License

RENEWLab shall be used under the [RENEW OPEN SOURCE LICENSE](https://renew-wireless.org/license).

# Acknowledgement

RENEW was funded by the NSF PAWR project.
