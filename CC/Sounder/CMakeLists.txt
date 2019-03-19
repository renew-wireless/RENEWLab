cmake_minimum_required (VERSION 2.8)
project (Sounder)

set(CMAKE_C_FLAGS "-std=c99 -w ")
set(CMAKE_CXX_FLAGS "-std=c++11 -w -O3 -mavx2 -mavx")

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

message(STATUS "SoapySDR_INCLUDE_DIRS: ${SoapySDR_INCLUDE_DIRS}")
message(STATUS "SoapySDR_LIBRARIES: ${SoapySDR_LIBRARIES}")
message(STATUS "HDF5_INCLUDE_DIRS: ${HDF5_INCLUDE_DIRS}")
message(STATUS "HDF5_LIBRARIES: ${HDF5_LIBRARIES}")
include_directories(${SoapySDR_INCLUDE_DIRS} ${HDF5_INCLUDE_DIRS})

add_executable(sounder main.cc config.cc receiver.cc recorder.cc sdr-lib.cc comms-lib.cc utils.cc)
target_link_libraries(sounder -lpthread -lhdf5_cpp --enable-threadsafe ${SoapySDR_LIBRARIES} ${HDF5_LIBRARIES} ${CMAKE_SOURCE_DIR}/lib/libmuFFT.a ${CMAKE_SOURCE_DIR}/lib/libmuFFT-sse.a ${CMAKE_SOURCE_DIR}/lib/libmuFFT-sse3.a ${CMAKE_SOURCE_DIR}/lib/libmuFFT-avx.a)

