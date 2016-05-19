cmake_minimum_required(VERSION 2.8.3)
project(amr_perception)
# Load catkin and all dependencies required for this package

find_package(Boost REQUIRED 
  COMPONENTS 
    thread
    filesystem
)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    dynamic_reconfigure
    amr_msgs
    amr_srvs
    sensor_msgs
)

include_directories(include 
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  cfg/cpp
)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

# add dynamic reconfigure API
generate_dynamic_reconfigure_options(
  cfg/LaserRepublisher.cfg
  cfg/ObstacleDetector.cfg
)

# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#...: compiler options :......................................................

#...: gnu++0x
if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        add_definitions(-std=gnu++0x)
    else(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        message(SEND_ERROR "You need GCC version 4.6 or greater to compile this package.")
    endif(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
endif(CMAKE_COMPILER_IS_GNUCXX)

#...: treat warnings as errors and disable centain warnings
add_definitions(-Werror)
add_definitions(-Wno-error=unused-variable)
add_definitions(-Wno-error=unknown-pragmas)
add_definitions(-Wno-unknown-pragmas)
add_definitions(-Wno-deprecated)

#...: target executables :....................................................

add_executable(obstacle_detector
  nodes/obstacle_detector.cpp
)

add_dependencies(obstacle_detector 
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(obstacle_detector 
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

#add_executable(obstacle_detector_youbot_simulation
#  nodes/obstacle_detector_youbot_simulation.cpp
#)

add_executable(laser_republisher
  nodes/laser_republisher.cpp
)

target_link_libraries(laser_republisher 
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(laser_republisher 
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#    DEPENDENCIES geometry_msgs std_msgs
#)

catkin_package(
    DEPENDS roscpp geometry_msgs dynamic_reconfigure amr_msgs amr_srvs sensor_msgs
    CATKIN_DEPENDS roscpp geometry_msgs dynamic_reconfigure amr_msgs amr_srvs sensor_msgs
    INCLUDE_DIRS
    LIBRARIES
)