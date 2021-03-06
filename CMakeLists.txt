﻿cmake_minimum_required(VERSION 2.8.3)
project(LaserOdomCalibration)

# Detection of CSM library using CMake routines
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(CSM REQUIRED csm)

##IF(${CSM_FOUND})
#        MESSAGE("CSM_LIBRARY_DIRS: ${CSM_LIBRARY_DIRS}")
#        MESSAGE("CSM_LIBRARIES: ${CSM_LIBRARIES}")
#        MESSAGE("CSM_INCLUDE_DIRS: ${CSM_INCLUDE_DIRS}")

#        INCLUDE_DIRECTORIES(${CSM_INCLUDE_DIRS})
#        LINK_DIRECTORIES(${CSM_LIBRARY_DIRS})

##ELSE(${CSM_FOUND})
#        MESSAGE(SEND_ERROR "CSM not found. Make sure PKG_CONFIG_PATH contains the directory where the file 'csm.pc' can be found.")
##ENDIF(${CSM_FOUND})

SET(CSM_SRC 
  src/csm/laser_data.cpp
  src/csm/math_utils.cpp
  src/csm/math_utils_gsl.cpp
  src/csm/utils.cpp
  src/csm/logging.cpp
  src/csm/sm_options.cpp
  src/csm/orientation.cpp
  src/csm/clustering.cpp)

SET(EGSL_SRC
	src/egsl/egsl.cpp
	src/egsl/egsl_conversions.cpp
	src/egsl/egsl_misc.cpp
	src/egsl/egsl_ops.cpp)

SET(GPC_SRC
	src/gpc/gpc.cpp
	src/gpc/gpc_utils.cpp)

SET(ICP_SRC
  src/icp/icp.cpp
  src/icp/icp_corr_dumb.cpp
  src/icp/icp_corr_tricks.cpp
  src/icp/icp_covariance.cpp
  src/icp/icp_debug.cpp
  src/icp/icp_loop.cpp
  src/icp/icp_outliers.cpp)

SET(OPTIONS_SRC  
   src/options/options.cpp
   src/options/options_interface.cpp)

ADD_LIBRARY(csm_eigen
	${CSM_SRC}
	${EGSL_SRC} 
	${GPC_SRC}
	${ICP_SRC}
	${OPTIONS_SRC})

# Various flags

set(CMAKE_CXX_FLAGS "-std=c++11")
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ggdb")
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -O2 ")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    rosbag
    message_generation
    )

 
add_message_files(
  FILES
  velocity.msg
)

generate_messages(DEPENDENCIES
  std_msgs 
)    

 
catkin_package(CATKIN_DEPENDS  message_runtime)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# A test
# ADD_EXECUTABLE(test_calib src/main.cpp)
# TARGET_LINK_LIBRARIES(test_calib ${CSM_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIR})

ADD_EXECUTABLE(io_test src/io_test.cpp src/io.cpp src/utils.cpp src/scan_match.cpp src/synchronizer.cpp src/solver.cpp)
TARGET_LINK_LIBRARIES(io_test ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIR} csm_eigen)
