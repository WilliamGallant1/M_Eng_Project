
#include "jaco_inv_dynamics_controller/PID_planner_arm.h"

int main(int argc, char **argv)
{
//initialize the ROS system and become a node
ros::init(argc,argv, "joint_planner");

ros::NodeHandle nh("~"); //nodehandle to subscribe and publish

PID_controller joint_ctrl(nh);

ros::Rate loopRate(100); //100hz, same frequency as the controller publishing to the robot
while (ros::ok())
{
joint_ctrl.update();
ros::spinOnce();
loopRate.sleep();
}
return 0;
}


