cmake_minimum_required(VERSION 3.0.2)
project(calqui)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/Calqui.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES calqui
#  CATKIN_DEPENDS calqui_msgs rospy std_msgs
#  DEPENDS system_lib
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/goto_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
