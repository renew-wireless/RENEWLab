cmake_minimum_required (VERSION 2.8)
project (Sounder)

set(CMAKE_C_FLAGS "-std=c99 -Wall")
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -ggdb3 -mavx2 -mavx")

#find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(PythonLibs REQUIRED)
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
