
#include "jaco_inv_dynamics_controller/joint_inv_dynamics_controller_arm.h"

int main(int argc, char **argv)
{
//initialize the ROS system and become a node
ros::init(argc,argv, "joint_controller");

ros::NodeHandle nh("~"); //nodehandle to subscribe and publish

joint_pos_controller joint_ctrl(nh);

ros::Rate loopRate(joint_ctrl.param_publish_rate_);
while (ros::ok())
{
joint_ctrl.update();
ros::spinOnce();
loopRate.sleep();
}
return 0;
}

