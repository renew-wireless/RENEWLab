cmake_minimum_required(VERSION 3.10)
project(Sounder)

if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
  set(GCC_COVERAGE_COMPILE_FLAGS "-faligned-new")
endif()

option(FORCE_BUILD_PATH "Hardcode the build directory path to be 'build/'" ON)
if(FORCE_BUILD_PATH)
  message(STATUS "Setting the build directory to build folder")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)
  #set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)
endif()

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}'.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
  message(STATUS "Verbose Makefile")
  set( CMAKE_VERBOSE_MAKEFILE on )
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb3 -fstack-protector-all -fsanitize=undefined -fsanitize=address -fsanitize=null")
  #-fsanitize=thread- fstack-protector-all -fsanitize=address -fstack-check
  #-fsanitize=address -fsanitize=pointer-compare
  #-fsanitize=address -fsanitize=pointer-subtract
  #-fsanitize=leak -fsanitize=undefined -fsanitize=null
endif()

# New versions of GCC enable position-independent executables by default. We
# don't want to compile FlexRAN libraries with -fPIC because it reduces
# performance by around 40%.
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -no-pie -pthread")

if(${CMAKE_C_COMPILER_ID} STREQUAL "GNU")
  message(STATUS "Using GNU compiler, compiler ID ${CMAKE_C_COMPILER_ID}")
  #For Ubuntu 1804 need to keep the c11 std for thread check
  set(CMAKE_C_FLAGS "-std=c11 -Wall")
  set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Wextra -mavx2")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
else()
  message(FATAL_ERROR "Unsupported version of compiler")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiling with CXX Flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "Debug Flags: ${CMAKE_CXX_FLAGS_DEBUG}")
message(STATUS "Release Flags: ${CMAKE_CXX_FLAGS_RELEASE}")

# Console logging level
if(LOG_LEVEL STREQUAL "none")
  message(STATUS "Logging level = none.")
  add_definitions(-DMLPD_LOG_LEVEL=0)
elseif(LOG_LEVEL STREQUAL "error")
  message(STATUS "Logging level = error.")
  add_definitions(-DMLPD_LOG_LEVEL=1)
elseif(LOG_LEVEL STREQUAL "warn")
  message(STATUS "Logging level = warn.")
  add_definitions(-DMLPD_LOG_LEVEL=2)
elseif(LOG_LEVEL STREQUAL "info")
  message(STATUS "Logging level = info.")
  add_definitions(-DMLPD_LOG_LEVEL=3)
elseif(LOG_LEVEL STREQUAL "frame")
  message(STATUS "Logging level = frame. Warning: Performance will be low.")
  add_definitions(-DMLPD_LOG_LEVEL=4)
elseif(LOG_LEVEL STREQUAL "subframe")
  message(STATUS "Logging level = subframe. Warning: Performance will be low.")
  add_definitions(-DMLPD_LOG_LEVEL=5)
elseif(LOG_LEVEL STREQUAL "trace")
  message(STATUS "Logging level = trace. Warning: Performance will be low.")
  add_definitions(-DMLPD_LOG_LEVEL=6)
else()
  message(STATUS "No logging level specified. Using warning level.")
  add_definitions(-DMLPD_LOG_LEVEL=2)
endif()

set(THREADS_PREFER_PTHREAD_FLAG TRUE)
#find_package(Threads REQUIRED)
find_package(nlohmann_json 3.2 REQUIRED)

message(STATUS "Using Pthread Library: ${CMAKE_THREAD_LIBS_INIT}: ${CMAKE_USE_PTHREADS_INIT}")

set(USE_UHD False CACHE STRING "USE_UHD defaulting to 'False'")
message(STATUS "USE_UHD: ${USE_UHD}")

if(${USE_UHD})
  message(STATUS "Enabled USE_UHD: ${USE_UHD}")
  add_definitions(-DUSE_UHD)
endif()

########################################################################
# SoapySDR dependency
########################################################################
find_package(SoapySDR 0.7 CONFIG)
if (NOT SoapySDR_FOUND)
    message(FATAL_ERROR "SoapySDR development files not found")
    return()
endif ()

find_package(HDF5 1.10 REQUIRED)
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
    recorder_worker.cc
    recorder_thread.cc
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
