# the purpose of this file is to generate a make file which we can then use to compile our C++ files
# the reason compilation is important is b/c we are also using Python and Julia. we need the C++ files to be called as a library in these codes. 
#Keerthana Srinivasan
#October 23, 2026

cmake_minimum_required(VERSION 3.14)
project(bitflipper CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# builds a shared library from our C++ files so Python and Julia can call our code
add_library(bitflip SHARED bitflip.cpp capi.cpp)     
set_target_properties(bitflip PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(bitflip_monitor main.cpp bitflip.cpp)
