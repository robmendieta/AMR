cmake_minimum_required(VERSION 2.8.3)
project(amr_mapping)

set(CMAKE_VERBOSE_MAKEFILE on)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    nav_msgs
    tf
    amr_msgs
    amr_srvs
    amr_stage
)

find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)

include_directories(include
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

catkin_python_setup()

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

#...: determine OS type
if((CMAKE_SIZEOF_VOID_P MATCHES 4) OR (CMAKE_CL_64 MATCHES 0))
  set(SUFFIX _x32)
elseif((CMAKE_SIZEOF_VOID_P MATCHES 8) OR (CMAKE_CL_64 MATCHES 1))
  set(SUFFIX _x64)
else()
  message(SEND_ERROR "Unable to determine whether the OS is 32 or 64 bit.")
endif()


catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp nav_msgs tf amr_msgs amr_srvs amr_stage
    INCLUDE_DIRS include
    LIBRARIES map_store_py
)


#...: target libraries :......................................................

add_library(mapstore${SUFFIX}
  src/map_store.cpp
  src/map_store_beam.cpp
  src/map_store_cone.cpp
  src/map_store_circle.cpp
)

add_library(map_store_py 
  src/map_store_bindings.cpp
)

#...: target executables :....................................................

#...: occupancy_query_server
add_executable(occupancy_query_server
  nodes/occupancy_query_server.cpp
)
target_link_libraries(occupancy_query_server
  mapstore${SUFFIX}
)

add_dependencies(occupancy_query_server 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(occupancy_query_server
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

#...: map store bindings

target_link_libraries(map_store_py
  mapstore${SUFFIX}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

add_dependencies(map_store_py
  ${catkin_EXPORTED_TARGETS}
)

set_target_properties(map_store_py PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )
