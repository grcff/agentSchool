cmake_minimum_required(VERSION 2.8.3)

project(agentSchool)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})

FIND_PACKAGE(PythonInterp)
FIND_PACKAGE(PythonLibs)
FIND_PACKAGE(Boost COMPONENTS python)

ENABLE_TESTING()
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
LINK_LIBRARIES(${Boost_LIBRARIES}) # Deprecated but so convenient!

file(
GLOB_RECURSE
main_files
main.cpp
source_files
src/*
)

#add_executable(
#        agentSchool-bin
#        experience.cpp
#        experience.h
#        ${main_files}
#        ${source_files}
#)

PYTHON_ADD_MODULE(experience experience.cpp)
FILE(COPY agentSchool.py DESTINATION .)
ADD_TEST(. agentSchool.py)

