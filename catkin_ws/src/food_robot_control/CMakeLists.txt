cmake_minimum_required(VERSION 3.0.2)
project(food_robot_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  actionlib
  food_robot_msgs
  geometry_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs std_srvs actionlib food_robot_msgs geometry_msgs
)

catkin_install_python(PROGRAMS
  scripts/compartment_control.py
  scripts/status_publisher.py
  scripts/delivery_goal_service.py
  scripts/navigate_action_server.py
  scripts/navigate_action_server_v2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
