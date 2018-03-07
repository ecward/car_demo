cmake_minimum_required(VERSION 2.8.3)
project(rcv_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  control_command.msg
  value_monitor.msg
  Control.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)


add_custom_target(rcv_msgs_dummy_target)
add_dependencies(rcv_msgs_dummy_target ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
