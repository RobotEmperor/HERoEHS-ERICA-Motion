################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(erica_manager)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED
  roscpp
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_controller_msgs
  robotis_math
  cmake_modules
  erica_base_module
  heroehs_math
  #heroehs_online_walking_pattern_generator
  #heroehs_pd_balance_controller
)

find_package(Eigen3 REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  LIBRARIES erica_manager
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(erica_manager_node src/erica_manager_node.cpp)

add_dependencies(erica_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(erica_manager_node
  ${catkin_LIBRARIES}
)
