cmake_minimum_required(VERSION 2.8.3)
project(amr_binaries)
# Load catkin and all dependencies required for this package

find_package(catkin REQUIRED )

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
#generate_messages(
#    DEPENDENCIES geometry_msgs std_msgs
#)

catkin_package(
    DEPENDS geometry_msgs std_msgs
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)