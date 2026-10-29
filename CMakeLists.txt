cmake_minimum_required(VERSION 3.14)
project(bitflipper CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(bitflip SHARED bitflip.cpp capi.cpp)      # libbitflip.so for Python / Julia
set_target_properties(bitflip PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(bitflip_monitor main.cpp bitflip.cpp)
