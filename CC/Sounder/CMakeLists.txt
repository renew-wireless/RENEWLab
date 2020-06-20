cmake_minimum_required (VERSION 2.8)
project (Sounder)

set(CMAKE_C_FLAGS "-std=c99 -O3 -Wall")
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -O3 -mavx2 -mavx")

find_package(nlohmann_json 3.2.0 REQUIRED)

########################################################################
# SoapySDR dependency
########################################################################
find_package(SoapySDR "0.6" CONFIG)
if (NOT SoapySDR_FOUND)
    message(FATAL_ERROR "SoapySDR development files not found")
    return()
endif ()

find_package(HDF5)
if (NOT HDF5_FOUND)
    message(FATAL_ERROR "HDF5 development files not found")
    return()
endif ()

set(directory "logs")
file(MAKE_DIRECTORY ${directory})

add_definitions(-DJSON)
add_definitions(-DTHREADED_INIT)

message(STATUS "SoapySDR_INCLUDE_DIRS: ${SoapySDR_INCLUDE_DIRS}")
message(STATUS "SoapySDR_LIBRARIES: ${SoapySDR_LIBRARIES}")
message(STATUS "HDF5_INCLUDE_DIRS: ${HDF5_INCLUDE_DIRS}")
message(STATUS "HDF5_LIBRARIES: ${HDF5_LIBRARIES}")
include_directories(${SoapySDR_INCLUDE_DIRS} ${HDF5_INCLUDE_DIRS})

add_executable(sounder main.cc ClientRadioSet.cc config.cc Radio.cc receiver.cc recorder.cc BaseRadioSet.cc BaseRadioSet-calibrate.cc comms-lib.cc comms-lib-avx.cc utils.cc signalHandler.cpp)
target_link_libraries(sounder -lpthread -lhdf5_cpp --enable-threadsafe ${SoapySDR_LIBRARIES} ${HDF5_LIBRARIES} ${CMAKE_SOURCE_DIR}/mufft/libmuFFT.a ${CMAKE_SOURCE_DIR}/mufft/libmuFFT-sse.a ${CMAKE_SOURCE_DIR}/mufft/libmuFFT-sse3.a ${CMAKE_SOURCE_DIR}/mufft/libmuFFT-avx.a)
