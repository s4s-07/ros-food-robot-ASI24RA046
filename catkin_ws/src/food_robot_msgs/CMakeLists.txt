cmake_minimum_required(VERSION 3.0.2)
project(food_robot_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
)

add_message_files(
  FILES
  RobotStatus.msg
)

add_service_files(
  FILES
  SetDeliveryGoal.srv
)

add_action_files(
  DIRECTORY action
  FILES
  NavigateToGoal.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs
)
