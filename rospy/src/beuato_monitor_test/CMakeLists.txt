cmake_minimum_required(VERSION 3.0.2)
project(beuato_monitor_test)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    actionlib
)

add_action_files(
    FILES
    BeuatoBalancer.action
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)



catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)



catkin_install_python(
  PROGRAMS
  scripts/beuato_server.py
  scripts/beuato_client.py
  scripts/beuato_client_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
