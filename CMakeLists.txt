cmake_minimum_required(VERSION 3.8.0)
project(argos3_examples)

# Set the compilation type
if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel ..." FORCE)
endif(CMAKE_BUILD_TYPE STREQUAL "")

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(ARGoS REQUIRED)
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})

add_subdirectory(sim_test_footbot)
add_subdirectory(sim_test_pipuck)
add_subdirectory(wms)
