# M_Eng_Project

Kinova arm model used: j2n6s300 - jaco2, non-spherical, 6dof, service, 3fingers

**Using inverse dynamics controller for Gazebo simulation**:

config file: `inv_dynamics_controller.yaml`
src files: `joint_inv_dynamics_controller.cpp` + `joint_inv_dynamics_planner.cpp`
launch file: `inv_dynamics_controller.launch`

Joint feedback positions/velocities come from topic `/j2n6s300/joint_states` (published by joint_state_controller)
Torque commands published onto topic `/j2n6s300/joint_group_effort_controller/command` (subscribed by joint_group_effort_controller)

Order of launch files to run:
1. kinova-ros/kinova_gazebo/launch/robot_simple_launch.launch with paused:= true
2. jaco_inv_dynamics_controller/launch/inv_dynamics_controller.launch

Sending target joint positions: done through service call `/j2n6s300/joint_planner/move_to_jaco`

**Using inverse dynamics controller on Kinova arm**:

config file: `inv_dynamics_controller_arm.yaml`
src files: `joint_inv_dynamics_controller_arm.cpp` + `joint_inv_dynamics_planner_arm.cpp`
launch file: `inv_dynamics_controller_arm.launch`

Joint feedback positions/velocities come from topic `$(arg kinova_robotName)_driver/out/joint_state` (published by kinova_arm.h/.cpp)
Torque commands published onto topic `$(arg kinova_robotName)_driver/in/joint_torque"` (subscribed by kinova_arm.h/.cpp)

Order of launch files to run:
1. kinova-ros/kinova_bringup/launch/kinova_robot.launch
2. jaco_inv_dynamics_controller/launch/inv_dynamics_controller_arm.launch

Sending target joint positions: done through service call `/j2n6s300/joint_planner/move_to_jaco`

**Using PID planner on Kinova arm**:

config file: `joint_trajectory_controller.yaml`
src files: `PID_planner_arm.cpp`
launch file: `PID_planner_arm.launch`

Joint feedback positions/velocities come from topic `$(arg kinova_robotName)_driver/out/joint_state` (published by kinova_arm.h/.cpp)
Trajectory commands published onto topic `$(arg kinova_robotName)_driver/trajectory_controller/command"` 
(subscribed by kinova_joint_trajectory_controller.h/.cpp)

Order of launch files to run:
1. kinova-ros/kinova_bringup/launch/kinova_robot.launch
2. jaco_inv_dynamics_controller/launch/PID_planner_arm.launch

Sending target joint positions: done through service call `/j2n6s300/joint_planner/move_to_jaco`
