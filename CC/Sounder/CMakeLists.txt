cmake_minimum_required (VERSION 2.8)
project (Sounder)

set(CMAKE_C_FLAGS "-std=c99 -O3 -Wall")
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -O3 -mavx2 -mavx")

find_package(nlohmann_json 3.2.0 REQUIRED)

set(USE_UHD False CACHE STRING "USE_UHD defaulting to 'False'")
message(STATUS "USE_UHD: ${USE_UHD}")

if(${USE_UHD})
  message(STATUS "Enabled USE_UHD: ${USE_UHD}")
  add_definitions(-DUSE_UHD)
endif()

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

set(SOUNDER_SOURCES 
    ClientRadioSet.cc
    config.cc
    Radio.cc
    receiver.cc
    recorder.cc
    BaseRadioSet.cc
    BaseRadioSet-calibrate.cc
    comms-lib.cc
    comms-lib-avx.cc
    utils.cc
    signalHandler.cpp)

add_executable(sounder 
    main.cc
    ${SOUNDER_SOURCES})

target_link_libraries(sounder -lpthread -lhdf5_cpp --enable-threadsafe
    ${SoapySDR_LIBRARIES}
    ${HDF5_LIBRARIES}
    ${CMAKE_SOURCE_DIR}/mufft/libmuFFT.a
    ${CMAKE_SOURCE_DIR}/mufft/libmuFFT-sse.a
    ${CMAKE_SOURCE_DIR}/mufft/libmuFFT-sse3.a
    ${CMAKE_SOURCE_DIR}/mufft/libmuFFT-avx.a)

add_library(sounder_module MODULE 
    ${SOUNDER_SOURCES})

target_link_libraries(sounder_module -lpthread -lhdf5_cpp --enable-threadsafe
    -Wl,--whole-archive
    ${CMAKE_SOURCE_DIR}/mufft/libmuFFT.a
    ${CMAKE_SOURCE_DIR}/mufft/libmuFFT-sse.a
    ${CMAKE_SOURCE_DIR}/mufft/libmuFFT-sse3.a
    ${CMAKE_SOURCE_DIR}/mufft/libmuFFT-avx.a
    -Wl,--no-whole-archive
    ${HDF5_LIBRARIES}
    ${SoapySDR_LIBRARIES})
