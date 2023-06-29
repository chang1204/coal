cmake_minimum_required(VERSION 3.0.2)
project(coal_uart)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  serial
  std_msgs
  coal_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES coal_uart
#  CATKIN_DEPENDS ros_cpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#可执行文件 串口节点
add_executable(uart_node src/uart_node.cpp )
target_link_libraries( uart_node  ${catkin_LIBRARIES})
#链接库文件 协议编码/解码
add_library(coal_uart SHARED src/coal_uart.cpp src/crc.cpp)
target_link_libraries( coal_uart  ${catkin_LIBRARIES})
target_link_libraries( uart_node  coal_uart)

# add_library(easylog SHARED src/easylog/easylogging++.cc)
# target_link_libraries( uart_node  easylog)