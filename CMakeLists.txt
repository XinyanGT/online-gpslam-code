cmake_minimum_required(VERSION 2.6)
enable_testing()
project(gpslam CXX C)

#####################################
# Find dependent libraries
#####################################

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBS gtsam)

# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")
include(GtsamTesting)

# Boost - same requirement as gtsam
find_package(Boost 1.43 REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

include_directories(${Boost_INCLUDE_DIR})

# CppUnitLite for tests
#find_package(CppUnitLite REQUIRED)

#####################################
# Process subdirs
#####################################

# Ensure that local folder is searched before library folders
include_directories(BEFORE "${PROJECT_SOURCE_DIR}/cpp")

add_subdirectory(cpp)

set_target_properties(gpslam PROPERTIES LINKER_LANGUAGE CXX)
