cmake_minimum_required(VERSION 3.0.2)
project(food_robot_bringup)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/send_goal_example.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
