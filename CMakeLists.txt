cmake_minimum_required(VERSION 3.0.2)
project(burg_scene_visualizer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  ros_numpy
  rospy
  sensor_msgs
  tf2_ros
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES burg_scene_visualizer
#  CATKIN_DEPENDS geometry_msgs message_filters ros_numpy rospy sensor_msgs tf2_ros
#  DEPENDS system_lib
)

#############
## Install ##
#############
# python nodes
catkin_install_python(PROGRAMS
  nodes/visualize_scene.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
