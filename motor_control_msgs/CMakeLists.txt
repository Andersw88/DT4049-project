cmake_minimum_required(VERSION 2.8.3)
project(motor_control_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  std_srvs
  message_generation
)

add_message_files(
  FILES
  MotorState.msg
  ControlMsg.msg
  Position.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motor_control_msgs
  CATKIN_DEPENDS std_msgs std_srvs message_runtime
)

