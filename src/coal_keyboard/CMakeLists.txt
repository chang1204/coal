cmake_minimum_required(VERSION 3.0.2)
project(coal_keyboard)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  coal_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES coal_keyboard
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(key_test src/key_test.cpp)
target_link_libraries(key_test 
${catkin_LIBRARIES} 
)

add_executable(coal_keyboard src/coal_keyboard.cpp)
target_link_libraries(coal_keyboard 
${catkin_LIBRARIES} 
)
