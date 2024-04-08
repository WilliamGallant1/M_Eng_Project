
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
#include <sensor_msgs/JointState.h>
#include "jaco_inv_dynamics_controller/JointTrajectoryPoint_w_bool.h"
#include "jaco_inv_dynamics_controller/MoveTo_jaco.h"

//kinova messaging
#include <kinova_msgs/JointTorque.h>
#include <kinova_msgs/SetTorqueControlMode.h>

class joint_pos_controller
{
public:
    joint_pos_controller(ros::NodeHandle& nh1);
    bool reading_parameters(); //load gains, target position, etc. from parameter server / .yaml file
    void sensor_callback(const sensor_msgs::JointState& msg); //for reading joint positions/velocities feedback
    void trajector_callback(const jaco_inv_dynamics_controller::JointTrajectoryPoint_w_bool& msg); //for reading from planner
    void update_inverse_dynamics(); //RNEA to compute torques
    void update(); //main update loop called by node

    ros::NodeHandle& node_handle_;

    //messaging topic/service
    ros::Subscriber sub_; //subscribe to joint feedback
    ros::Subscriber sub2_; //subscribe to sampled trajectory point from planner
    ros::Publisher pub_; //publish joint torques
    ros::ServiceClient client_move_; //client for first trajectory
    ros::ServiceClient client_torque_; //client for mode switch

    kinova_msgs::JointTorque torque_msg_; //torque message
    jaco_inv_dynamics_controller::MoveTo_jaco srv_move_; //first trajectory service
    kinova_msgs::SetTorqueControlMode srv_torque_; //mode switch service

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

    //logic
    bool topic_flag_;
    bool subscribe_flag_;
    bool service_flag_;

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
};

joint_pos_controller::joint_pos_controller(ros::NodeHandle& nh1): node_handle_(nh1)
{
    //messaging topic/service
    sub_ = node_handle_.subscribe("/j2n6s300_driver/out/joint_state",1,&joint_pos_controller::sensor_callback,this);
    sub2_ = node_handle_.subscribe("/j2n6s300/joint_planner/planner_traj_points",1,&joint_pos_controller::trajector_callback,this);
    pub_ = node_handle_.advertise<kinova_msgs::JointTorque>("/j2n6s300_driver/in/joint_torque",1);
    client_move_ = node_handle_.serviceClient<jaco_inv_dynamics_controller::MoveTo_jaco>("/j2n6s300/joint_planner/move_to_jaco");
    client_torque_ = node_handle_.serviceClient<kinova_msgs::SetTorqueControlMode>("/j2n6s300_driver/in/set_torque_control_mode");

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

    //logic
    topic_flag_ = false;
    subscribe_flag_ = false;
    service_flag_ = true;

    reading_parameters();

    //Pinocchio
    pinocchio::urdf::buildModel(robot_urdf_name_, model_,false);
    pinocchio::Data data(model_);
    data_ = data;

    ROS_INFO("model.nq = %d",model_.nq); //6
    ROS_INFO("model.nv = %d",model_.nv); //6
    ROS_INFO("model.njoints = %d",model_.njoints); //7

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
        ROS_INFO("Unable to read the parameter /j2n6s300_target_positions, returning false.");
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
    srv_move_.request.T = 5.0;
    srv_torque_.request.state = 1; //to enable torque control
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

void joint_pos_controller::trajector_callback(const jaco_inv_dynamics_controller::JointTrajectoryPoint_w_bool& msg)
{
    for(int i = 0; i < 6; i++)
    {
        eig_desired_joint_pos_(i) = msg.traj_point.positions[i];
        eig_desired_joint_vel_(i) = msg.traj_point.velocities[i];
        eig_desired_joint_acc_(i) = msg.traj_point.accelerations[i];
    }
    subscribe_flag_ = msg.service_called;
}

void joint_pos_controller::update_inverse_dynamics()
{
    //x_dotdot_cmd = x_ddot_ref + D * (x_dot_ref - x_dot_fbk) + K * (x_ref - x_fbk)
    eig_command_joints_acceleration_ = eig_desired_joint_acc_ + gain_D_matrix_ * (eig_desired_joint_vel_ - eig_current_joints_velocity_) +
    gain_P_matrix_ * (eig_desired_joint_pos_ - eig_current_joints_position_);
    
    //tau_cmd = M*q_ddot_command + nle - gravity
    pinocchio::computeAllTerms(model_, data_, eig_current_joints_position_, eig_current_joints_velocity_);
    eig_tau_commands_ = data_.M * eig_command_joints_acceleration_ + data_.nle - data_.g; //nle includes coriolis/centrifugal + gravity
}

void joint_pos_controller::update()
{
    if(topic_flag_)
    {
        if(service_flag_) //as soon as robot starts/receive feedback, switch to torque mode and send service call
        {
            if (!ros::service::waitForService("/j2n6s300/joint_planner/move_to_jaco", ros::Duration(3.0))) 
            {
                ROS_ERROR("Move to jaco service is not available after waiting 3 seconds");
            } 
            else 
            {
                if(client_torque_.call(srv_torque_)) {ROS_INFO("Switching to torque mode");}
                if(client_move_.call(srv_move_)) {ROS_INFO("Sending home position");}
                service_flag_ = false;
            }
        }

        update_inverse_dynamics();

        if(subscribe_flag_) //once we receive service call
        {
            torque_msg_.joint1 = (float)eig_tau_commands_(0);
            torque_msg_.joint2 = (float)eig_tau_commands_(1);
            torque_msg_.joint3 = (float)eig_tau_commands_(2);
            torque_msg_.joint4 = (float)eig_tau_commands_(3);
            torque_msg_.joint5 = (float)eig_tau_commands_(4);
            torque_msg_.joint6 = (float)eig_tau_commands_(5);
            torque_msg_.joint7 = 0.0; //only 6DOF
            pub_.publish(torque_msg_);
        } 
    }
}


