cmake_minimum_required(VERSION 3.0.2)
project(project)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    robotics_hw1
    message_filters
    roscpp
    rospy
    std_msgs
    message_generation
    geometry_msgs
    nav_msgs
    dynamic_reconfigure
    tf
    tf2
    tf2_ros
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    Odom_and_method.msg

)

## Generate services in the 'srv' folder
add_service_files(
  FILES
    Reset_odom.srv
    Set_odom.srv

)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
    robotics_hw1
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/parameters.cfg

)

## Declare things to be passed to dependent projects
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    robotics_hw1
    message_runtime
    std_msgs
    nav_msgs

)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(calib src/calibration.cpp)
add_executable(pub_twist src/pub_twist)
add_executable(params src/params.cpp)
add_executable(pub_odom src/pub_odom.cpp)
add_executable(tf_node src/tf.cpp)
add_executable(reset src/reset_client.cpp)
add_executable(set src/set_client.cpp)

## Custom messages dependencies
add_dependencies(calib project_generate_messages_cpp)
add_dependencies(pub_odom project_generate_messages_cpp)
add_dependencies(pub_twist project_generate_messages_cpp)
add_dependencies(tf_node project_generate_messages_cpp)

## Add cmake target dependencies of the executable
add_dependencies(params ${PROJECT_NAME}_gencfg)
add_dependencies(pub_odom ${catkin_EXPORTED_TARGETS})
add_dependencies(reset ${catkin_EXPORTED_TARGETS})
add_dependencies(set ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(calib ${catkin_LIBRARIES})
target_link_libraries(pub_twist ${catkin_LIBRARIES})
target_link_libraries(params ${catkin_LIBRARIES})
target_link_libraries(pub_odom ${catkin_LIBRARIES})
target_link_libraries(tf_node ${catkin_LIBRARIES})
target_link_libraries(reset ${catkin_LIBRARIES})
target_link_libraries(set ${catkin_LIBRARIES})
