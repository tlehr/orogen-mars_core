PROJECT(simulation)
#include_directories("tasks/")
cmake_minimum_required(VERSION 2.6)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(simulationBase)

add_subdirectory(scripts)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

