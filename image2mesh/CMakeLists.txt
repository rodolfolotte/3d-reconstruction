project(image2mesh)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})
cmake_minimum_required(VERSION 2.6.2)
add_definitions(-std=c++11)

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" VERSION_GREATER 2.6)
  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" VERSION_GREATER 2.8.3)
    cmake_policy(VERSION 2.8.4)
  else()
    cmake_policy(VERSION 2.6)
  endif()
endif()

set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true )
set(CMAKE_CXX_FLAGS "-O2")
 
if ( COMMAND cmake_policy )
  cmake_policy( SET CMP0003 NEW )  
endif()

# Set a default build type if none was specified
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                     "MinSizeRel" "RelWithDebInfo")
ENDIF()

# Eigen 
find_package(Eigen3 REQUIRED)
if (EIGEN3_FOUND)
  include(${EIGEN3_USE_FILE})
endif()

# PUGIXML - 
# First, install it by: sudo apt-get install libpugixml-dev
set(PUGIXML_LIBRARY "/usr/lib/x86_64-linux-gnu/libpugixml.so")
set(PUGIXML_INCLUDE_DIR "/usr/include/")

# TBB
# First, install it by: sudo apt-get install libtbb-dev
set(TBB_LIBRARY "/usr/lib/x86_64-linux-gnu/libtbb.so")
set(TBB_INCLUDE_DIR "/usr/include/")

# EMBREE
# First, install it by: git clone https://github.com/embree/embree.git
# cd embree
# mkdir bin
# cd bin
# .. cmake .
# make
# make install
# set(EMBREE_LIBRARY "/usr/local/lib/libembree.so")
set(EMBREE_LIBRARY "/usr/local/lib/libembree.so")
set(EMBREE_INCLUDE_DIR "/usr/local/include/")

# VTK
# reference: https://www.vtk.org/Wiki/VTK/Building/Linux
# git clone git://vtk.org/VTK.git
#mkdir bin
#cd bin
#cmake ..
#cd ..
#mkdir release
#cd release
#cmake -DCMAKE_BUILD_TYPE:STRING=Release ..
#cd ..
#make -j<# of cores> - example: make -j4
#make install

find_package(VTK)
IF(VTK_FOUND)
  include(${VTK_USE_FILE})
  message("found VTK. Version:" ${VTK_VERSION}. VTK_DIR: ${VTK_DIR})
ELSE(VTK_FOUND)
  MESSAGE(FATAL_ERROR
    "Cannot build the executable without VTK. Please set the VTK_DIR")
ENDIF(VTK_FOUND)

include_directories(include/)
add_subdirectory(src/)
