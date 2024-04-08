#include <ros/ros.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>


int main(int argc, char** argv) {
	ros::init(argc, argv, "initialize");
	ros::NodeHandle node_handle("~");

	ROS_INFO("initialize jaco");

	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
		ros::console::notifyLoggerLevelsChanged();
	}

	if ( !ros::service::waitForService("/j2n6s300/controller_manager/load_controller", ros::Duration(30.0)) ) {
		ROS_ERROR_STREAM("Gazebo is not ready. Shutdown");
		return -1;
	}

	ROS_DEBUG_STREAM("Wait for controller_manager_msgs::LoadController") ;

	ros::ServiceClient load_client = node_handle.serviceClient<controller_manager_msgs::LoadController>("/j2n6s300/controller_manager/load_controller");
	controller_manager_msgs::LoadController load_msg ;

	load_msg.request.name = "joint_state_controller" ;
	if ( load_client.call(load_msg) ) {
		ROS_DEBUG_STREAM("controller_manager_msgs::LoadController");
	}
	else {
		ROS_ERROR_STREAM("cannot load joint_state_controller");
	}

	load_msg.request.name = "joint_group_effort_controller" ;
	// load_client.call(load_msg) ;
	if ( load_client.call(load_msg) ) {
		ROS_DEBUG_STREAM("controller_manager_msgs::LoadController");
	}
	else {
		ROS_ERROR_STREAM("cannot load joint_state_controller");
	}

	ROS_DEBUG_STREAM("Wait for controller_manager_msgs::SwitchController") ;
	if ( !ros::service::waitForService("/j2n6s300/controller_manager/switch_controller", ros::Duration(20.0)) ) {
		ROS_ERROR_STREAM("Gazebo is not ready. Shutdown");
		return -1;
	}
	ros::ServiceClient switch_client = node_handle.serviceClient<controller_manager_msgs::SwitchController>("/j2n6s300/controller_manager/switch_controller");
	controller_manager_msgs::SwitchController switch_msg ;
	switch_msg.request.start_controllers = {"joint_state_controller", "joint_group_effort_controller"} ;
	switch_msg.request.strictness = 2 ;
	//switch_msg.request.timeout = 10 ;
	switch_client.call(switch_msg) ;
	ROS_DEBUG_STREAM("controller_manager_msgs::SwitchController");

	return 0;
}
