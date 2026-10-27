cmake_minimum_required(VERSION 3.13...4.2.3 FATAL_ERROR)
project(ivpico CXX)

if(((NOT DEFINED CMAKE_CXX_STANDARD) OR ("${CMAKE_CXX_STANDARD}" STREQUAL "03") OR ("${CMAKE_CXX_STANDARD}" STREQUAL "11") OR ("${CMAKE_CXX_STANDARD}" STREQUAL "14")))
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  message(VERBOSE "Setting CMAKE_CXX_STANDARD to 17 as required by IvPico. If you wish to use a newer standard, set CMAKE_CXX_STANDARD directly")
endif()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# MOOS-IvP's helm libraries (IVP_BUILD_HELM_LIBS_ONLY)
find_package(IVP CONFIG REQUIRED)

include(CTest)

add_subdirectory(src)
