cmake_minimum_required(VERSION 2.8.3)
project(ex2)
 
find_package(catkin REQUIRED COMPONENTS
    ex1
)

#generate_messages(
#   DEPENDENCIES
#   geometry_msgs   sensor_msgs   std_msgs
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs   sensor_msgs   std_msgs
# )


 
###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ex2
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
 
