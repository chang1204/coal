cmake_minimum_required(VERSION 3.0.2)
project(coal_msgs)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  message_generation 
  std_msgs 
  geometry_msgs
)


add_message_files(
  FILES
  test.msg
  sensorDataReception.msg
  chassisControl.msg
  udpControl.msg
  # Message2.msg
)

#generate_messages必须在catkin_package前面
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
 
  CATKIN_DEPENDS message_runtime
 
)

