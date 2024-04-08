cmake_minimum_required(VERSION 3.0.2)
project(jaco_inv_dynamics_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
  kinova_msgs
  message_generation
  controller_manager_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

find_package(pinocchio REQUIRED)
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  JointTrajectoryPoint_w_bool.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  MoveTo_jaco.srv
  Torque_call.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs trajectory_msgs kinova_msgs message_runtime controller_manager_msgs
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS} #include pinoccio
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/jaco_inv_dynamics_controller_node.cpp)
add_executable(joint_dynamic_controller src/joint_inv_dynamics_controller.cpp)
add_executable(joint_dynamic_planner src/joint_inv_dynamics_planner.cpp)
add_executable(joint_dynamic_controller_arm src/joint_inv_dynamics_controller_arm.cpp)
add_executable(joint_dynamic_planner_arm src/joint_inv_dynamics_planner_arm.cpp)
add_executable(joint_PID_planner_arm src/PID_planner_arm.cpp)
add_executable(gazebo_jaco_initializer src/Gazebo_controllers_initialize.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(joint_dynamic_controller
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
)

target_link_libraries(joint_dynamic_planner
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
)

target_link_libraries(joint_dynamic_controller_arm
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
)

target_link_libraries(joint_dynamic_planner_arm
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
)

target_link_libraries(joint_PID_planner_arm
  ${catkin_LIBRARIES}
)

target_link_libraries(gazebo_jaco_initializer
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS src/joint_states_unique_extract.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS
  nodes/joint_trajectory_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  nodes/mpc.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  nodes/dynamic_loader.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_jaco_inv_dynamics_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
