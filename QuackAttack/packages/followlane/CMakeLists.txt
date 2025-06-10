cmake_minimum_required(VERSION 2.8.3)
project(followlane)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  src/Challenge1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
