cmake_minimum_required(VERSION 3.0.2)
project(dump_extract)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(PCL REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp roslib std_msgs tf2 tf2_geometry_msgs tf2_ros
 DEPENDS PCL 
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(extract_node src/extract_node.cpp)

target_link_libraries(extract_node
${catkin_LIBRARIES} 
${PCL_LIBRARIES}
)
