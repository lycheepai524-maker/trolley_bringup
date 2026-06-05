cmake_minimum_required(VERSION 3.0.2)
project(trolley_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/encoder_to_odom.py
  scripts/calibrate_ticks.py
  scripts/drive_test.py
  scripts/wifi_resolver.py
  scripts/wifi_locate.py
  scripts/trolley_command_node.py
  scripts/record_location.py
  scripts/goto_location.py
  scripts/cmd_vel_boost.py
  scripts/lidar_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
