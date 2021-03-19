cmake_minimum_required(VERSION 3.15)
project (Sounder)

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}'.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
  message(STATUS "Verbose Makefile")
  set( CMAKE_VERBOSE_MAKEFILE on )
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb3 -fsanitize=address -fstack-protector-all")
  #-fsanitize=thread- -fstack-protector-all -fsanitize=address -fstack-check
endif()

# New versions of GCC enable position-independent executables by default. We
# don't want to compile FlexRAN libraries with -fPIC because it reduces
# performance by around 40%.
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -no-pie -pthread")

if(${CMAKE_C_COMPILER_ID} STREQUAL "GNU")
  message(STATUS "Using GNU compiler, compiler ID ${CMAKE_C_COMPILER_ID}")
  #For Ubuntu 1804 need to keep the c11 std for thread check
  set(CMAKE_C_FLAGS "-std=c11 -Wall")
  set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Wextra -mavx2 -mavx")
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

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_definitions(-DTEST_BENCH)

INCLUDE_DIRECTORIES( "../../include" )
add_executable(comm-testbench test-main.cc
	${SOURCE_DIR}/comms-lib.cc
	${SOURCE_DIR}/comms-lib-avx.cc
	${SOURCE_DIR}/utils.cc)
target_link_libraries(comm-testbench 
	-lpthread --enable-threadsafe
	${SOURCE_DIR}/mufft/libmuFFT.a
       	${SOURCE_DIR}/mufft/libmuFFT-sse.a
	${SOURCE_DIR}/mufft/libmuFFT-sse3.a
	${SOURCE_DIR}/mufft/libmuFFT-avx.a)
