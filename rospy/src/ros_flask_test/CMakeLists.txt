cmake_minimum_required(VERSION 3.0.2)
project(ros_flask_test)

find_package(catkin REQUIRED)

catkin_package(
)

include_directories(
)

catkin_install_python(
  PROGRAMS
  scripts/flask_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
