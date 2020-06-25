cmake_minimum_required(VERSION 2.8.3)
project(myserial)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  serial
  std_msgs
  tf
)
catkin_package(
        CATKIN_DEPENDS
        serial
        std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
set(myserial_srclib
        src/crc.cpp
        src/command.cpp
        )

add_library(${PROJECT_NAME} ${myserial_srclib})


add_executable( myserialnode main.cpp )
target_link_libraries( myserialnode
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
        )
add_executable( myserial_receivenode receive.cpp )
target_link_libraries( myserial_receivenode
${catkin_LIBRARIES}
${PROJECT_NAME}
)
