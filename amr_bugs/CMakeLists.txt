cmake_minimum_required(VERSION 2.8.3)
project(amr_bugs)

# Load catkin and all dependencies required for this package

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    smach
    smach_ros
    smach_viewer
    dynamic_reconfigure
    actionlib
    tf
    std_srvs
    geometry_msgs
    amr_msgs
    amr_srvs
    amr_stage
    amr_perception
    amr_navigation
)

include_directories(include
  include
  cfg/cpp
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

catkin_python_setup()

# add dynamic reconfigure API
generate_dynamic_reconfigure_options(
  cfg/Wallfollower.cfg
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


catkin_package(
    DEPENDS
    CATKIN_DEPENDS
      roscpp
      rospy
      smach
      smach_ros
      smach_viewer
      dynamic_reconfigure
      actionlib
      tf
      std_srvs
      geometry_msgs
      amr_msgs
      amr_srvs
      amr_stage
      amr_perception
      amr_navigation
    INCLUDE_DIRS
    LIBRARIES
)

