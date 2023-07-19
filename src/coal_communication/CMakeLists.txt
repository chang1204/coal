cmake_minimum_required(VERSION 3.0.2)
project(coal_communication)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  coal_msgs
)

find_package(yaml-cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES coal_communication
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(udp_pub src/coal_udp_pub.cpp)
target_link_libraries(udp_pub 
${catkin_LIBRARIES} 
yaml-cpp
)

add_executable(udp_sub src/coal_udp_sub.cpp)
target_link_libraries(udp_sub 
${catkin_LIBRARIES} 
yaml-cpp
)