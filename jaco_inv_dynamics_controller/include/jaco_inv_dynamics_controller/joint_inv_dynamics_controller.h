
//Pinocchio files for RNEA
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

//utility
#include <ros/ros.h>
#include <cmath>
#include <Eigen/Dense>

//messaging
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include "jaco_inv_dynamics_controller/MoveTo_jaco.h"

class joint_pos_controller
{
public:
    joint_pos_controller(ros::NodeHandle& nh1);
    bool reading_parameters(); //load gains, target position, etc. from parameter server / .yaml file
    void sensor_callback(const sensor_msgs::JointState& msg); //for reading joint positions/velocities feedback
    void trajector_callback(const trajectory_msgs::JointTrajectoryPoint& msg); //for reading from planner
    void update_inverse_dynamics(); //RNEA to compute torques
    void update(); //main update loop called by node

    ros::NodeHandle& node_handle_;

    //messaging topic/service
    ros::Subscriber sub_; //subscribe to joint feedback
    ros::Subscriber sub2_; //subscribe to sampled trajectory point from planner
    ros::Publisher pub_; //publish joint torques
    ros::ServiceClient client_move_; //client for first trajectory

    std_msgs::Float64MultiArray torque_msg_; //torque message
    std::vector<double>::const_iterator itr_; //iterator for torque message
    jaco_inv_dynamics_controller::MoveTo_jaco srv_move_; //first trajectory service

    //from parameters
    std::string robot_urdf_name_; //for Pinocchio to build model
    std::vector<double> P_gains_; //holds the P gains of my joints
    std::vector<double> D_gains_; //hold the D gains of my joints
    std::vector<double> param_des_joint_pos_; //holds initial desired joint positions
    double param_publish_rate_; //controller publishing rate

    //feedback reading
    Eigen::VectorXd eig_current_joints_position_; //feedback joint position
    Eigen::VectorXd eig_current_joints_velocity_; //feedback joint velocity

    //update inverse dynamics
    Eigen::MatrixXd gain_P_matrix_; //stiffness matrix
    Eigen::MatrixXd gain_D_matrix_; //damping matrix
    Eigen::VectorXd eig_desired_joint_pos_; //reference joint position
    Eigen::VectorXd eig_desired_joint_vel_; //reference joint velocity
    Eigen::VectorXd eig_desired_joint_acc_; //reference joint acceleration
    Eigen::VectorXd eig_command_joints_acceleration_; //command acceleration with PD
    Eigen::VectorXd eig_tau_commands_; //torque commands from RNEA
    std::vector<double> tau_commands_; //torque commands to publish

    //logic
    bool topic_flag_;
    bool service_flag_;

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
};

joint_pos_controller::joint_pos_controller(ros::NodeHandle& nh1): node_handle_(nh1)
{
    //messaging topic/service
    sub_ = node_handle_.subscribe("/j2n6s300/joint_states",1,&joint_pos_controller::sensor_callback,this);
    sub2_ = node_handle_.subscribe("/j2n6s300/joint_planner/planner_traj_points",1,&joint_pos_controller::trajector_callback,this);
    pub_ = node_handle_.advertise<std_msgs::Float64MultiArray>("/j2n6s300/joint_group_effort_controller/command",1);
    client_move_ = node_handle_.serviceClient<jaco_inv_dynamics_controller::MoveTo_jaco>("/j2n6s300/joint_planner/move_to_jaco");

    torque_msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
    torque_msg_.layout.dim[0].size = 6;
    torque_msg_.layout.dim[0].stride = 6;

    //feedback reading
    eig_current_joints_position_ = Eigen::VectorXd::Zero(6);
    eig_current_joints_velocity_ = Eigen::VectorXd::Zero(6);

    //update inverse dynamics
    gain_P_matrix_ = Eigen::MatrixXd::Zero(6,6);
    gain_D_matrix_ = Eigen::MatrixXd::Zero(6,6);
    eig_desired_joint_pos_ = Eigen::VectorXd::Zero(6);
    eig_desired_joint_vel_ = Eigen::VectorXd::Zero(6);
    eig_desired_joint_acc_ = Eigen::VectorXd::Zero(6);
    eig_command_joints_acceleration_ = Eigen::VectorXd::Zero(6);
    eig_tau_commands_ = Eigen::VectorXd::Zero(6);
    tau_commands_.resize(6);

    //logic
    topic_flag_ = false;
    service_flag_ = true; //sending home position immediately

    reading_parameters();

    //Pinocchio
    pinocchio::urdf::buildModel(robot_urdf_name_, model_,false);
    pinocchio::Data data(model_);
    data_ = data;

    ROS_INFO("model.nq = %d",model_.nq);
    ROS_INFO("model.nv = %d",model_.nv);
    ROS_INFO("model.njoints = %d",model_.njoints);
}

bool joint_pos_controller::reading_parameters()
{
    if(!node_handle_.getParam("/j2n6s300/urdf_file_name",robot_urdf_name_))
    {
        ROS_INFO("Unable to load the parameter /j2n6s300/urdf_file_name, returning false");
        return false;
    }    
    if(!node_handle_.getParam("/j2n6s300/P_gains",P_gains_))
    {
        ROS_INFO("Unable to read the parameter /j2n6s300/P_gains, returning false.");
        return false;
    }
    if(!node_handle_.getParam("/j2n6s300/D_gains",D_gains_))
    {
        ROS_INFO("Unable to read the parameter /j2n6s300/D_gains, returning false.");
        return false;
    }
    if(!node_handle_.getParam("/j2n6s300/target_positions",param_des_joint_pos_))
    {
        ROS_INFO("Unable to read the parameter /j2n6s300/target_positions, returning false.");
        return false;
    }
    if(!node_handle_.getParam("/j2n6s300/param_publish_rate",param_publish_rate_))
    {
        ROS_INFO("Unable to read the parameter /j2n6s300/param_publish_rate, returning false.");
        return false;
    }

    for(int i = 0; i < 6; i++) //filling diagonal elements
    {
        gain_P_matrix_(i,i) = P_gains_[i];
        gain_D_matrix_(i,i) = D_gains_[i];
    }

    srv_move_.request.p1 = param_des_joint_pos_[0];
    srv_move_.request.p2 = param_des_joint_pos_[1];
    srv_move_.request.p3 = param_des_joint_pos_[2];
    srv_move_.request.p4 = param_des_joint_pos_[3];
    srv_move_.request.p5 = param_des_joint_pos_[4];
    srv_move_.request.p6 = param_des_joint_pos_[5];
    srv_move_.request.T = 10.0; 

    return true;
}

void joint_pos_controller::sensor_callback(const sensor_msgs::JointState& msg)
{
    for (int i = 0; i < 6; i++)
    {
        eig_current_joints_position_(i) = msg.position[i];
        eig_current_joints_velocity_(i) = msg.velocity[i];
    }
    topic_flag_ = true; //have at least one reading
}

void joint_pos_controller::trajector_callback(const trajectory_msgs::JointTrajectoryPoint& msg)
{
    for(int i = 0; i < 6; i++)
    {
        eig_desired_joint_pos_(i) = msg.positions[i];
        eig_desired_joint_vel_(i) = msg.velocities[i];
        eig_desired_joint_acc_(i) = msg.accelerations[i];
    }
}

void joint_pos_controller::update_inverse_dynamics()
{
    //x_dotdot_cmd = x_ddot_ref + D * (x_dot_ref - x_dot_fbk) + K * (x_ref - x_fbk)
    eig_command_joints_acceleration_ = eig_desired_joint_acc_ + gain_D_matrix_ * (eig_desired_joint_vel_ - eig_current_joints_velocity_) +
    gain_P_matrix_ * (eig_desired_joint_pos_ - eig_current_joints_position_);
    
    //tau_cmd = M*q_ddot_command + nle
    pinocchio::computeAllTerms(model_, data_, eig_current_joints_position_, eig_current_joints_velocity_);
    eig_tau_commands_ = data_.M * eig_command_joints_acceleration_ + data_.nle; //nle includes coriolis/centrifugal + gravity
}

void joint_pos_controller::update()
{
    if(topic_flag_)
    {
        if(service_flag_) //as soon as unpausing gazebo/receive feedback, send single service call
        {
            if (!ros::service::waitForService("/j2n6s300/joint_planner/move_to_jaco", ros::Duration(3.0))) 
            {
                ROS_ERROR("Move to jaco service is not available after waiting 3 seconds");
            } 
            else 
            {
                if(client_move_.call(srv_move_)) {ROS_INFO("Sending home position");}
                service_flag_ = false;
            }
        }

        update_inverse_dynamics();

        for(int i = 0; i < 6; i++)
        {
            tau_commands_[i] = eig_tau_commands_(i); //conversion Eigen::VectorXd --> std::vector
        }
        for(itr_ = tau_commands_.begin(); itr_!= tau_commands_.end(); ++itr_) 
        {
            torque_msg_.data.push_back(*itr_); 
        }
        pub_.publish(torque_msg_);
        torque_msg_.data.clear();
    }
}





