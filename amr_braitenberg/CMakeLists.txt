cmake_minimum_required(VERSION 2.8.3)
project(amr_braitenberg)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    dynamic_reconfigure
    amr_msgs
    amr_srvs
    amr_stage
)

include_directories(include
#  include
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
  cfg/BraitenbergVehicle.cfg
)

# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#...: compiler options :......................................................

add_definitions(-std=gnu++0x)
add_definitions(-Wall)
add_definitions(-Wno-error=unused-variable)
add_definitions(-Wno-error=unknown-pragmas)
add_definitions(-Wno-error=address)
add_definitions(-Wno-deprecated)


catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp dynamic_reconfigure amr_msgs amr_srvs amr_stage
    INCLUDE_DIRS
    LIBRARIES
)