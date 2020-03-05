cmake_minimum_required(VERSION 2.8.3)
project(nbv_underwater)

find_package(catkin REQUIRED
  rospy
  ORB_SLAM2
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/rough_nbv_planner.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
