cmake_minimum_required(VERSION 3.0.2)
project(ethernet_udp_com)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(DIRECTORY msg FILES trajectory.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
CATKIN_DEPENDS message_runtime std_msgs rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/autera_udp_recv.py scripts/autera_udp_send.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
