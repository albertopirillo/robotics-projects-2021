cmake_minimum_required(VERSION 3.0.2)
project(project)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    tf
    tf2_ros
)

## Generate messages in the 'msg' folder
#add_message_files(
#        FILES
#
#)

## Generate services in the 'srv' folder
#add_service_files(
#        FILES
#
#
#)

# Generate added messages and services with any dependencies listed here
#generate_messages(
#        DEPENDENCIES
#        std_msgs
#)

## Generate dynamic reconfigure parameters in the 'cfg' folder
#generate_dynamic_reconfigure_options(
#
#)

## Declare things to be passed to dependent projects
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS

)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(conv src/conversion.cpp)


## Custom messages dependencies
#add_dependencies(pub_odom project_generate_messages_cpp)


## Add cmake target dependencies of the executable
#add_dependencies(pub_odom ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(conv ${catkin_LIBRARIES})
