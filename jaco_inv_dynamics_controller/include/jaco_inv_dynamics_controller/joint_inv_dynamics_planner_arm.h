
//utility
#include <ros/ros.h>
#include <cmath>
#include <Eigen/Dense>

//messaging
#include <sensor_msgs/JointState.h>
#include "jaco_inv_dynamics_controller/JointTrajectoryPoint_w_bool.h"
#include "jaco_inv_dynamics_controller/MoveTo_jaco.h"

//trajectory
#include "quintic_poly.h"

class joint_pos_planner
{
public:
    joint_pos_planner(ros::NodeHandle& nh1);
    bool reading_parameters(); //load publish rate from parameter server
    void sensor_callback(const sensor_msgs::JointState& msg); //reading feedback
    bool trajectory_callback(jaco_inv_dynamics_controller::MoveTo_jaco::Request &req,
    jaco_inv_dynamics_controller::MoveTo_jaco::Response &resp); //new trajectory
    void startup_planner(const std::array<double,6>& desired_joint_pos); //5th order poly
    void update_planner(); //sample trajectory over time and publish reference
    void update(); //main update loop called by node

    ros::NodeHandle& node_handle_;

    //messaging topic/service
    ros::Subscriber sub_; //subscribe to joint feedback
    ros::Publisher pub_; //publish sampled trajectory point
    ros::ServiceServer service_; //server for new trajectory
    jaco_inv_dynamics_controller::JointTrajectoryPoint_w_bool traj_msg_; //trajectory point message + boolean

    //from parameters
    double param_publish_rate_; //planner publish rate

    //quintic trajectory
    quintic_poly trajectory_poly_; //instance of class from "quintic_poly.h"
    std::array<double,6> array_final_desired_joints_position_; //6 angles
    std::array<double,6> array_current_joints_position_; // 6 angles
    std::vector<double> sampled_states_pos_; //sampled position from quintic
    std::vector<double> sampled_states_vel_; //sampled velocity from quintic
    std::vector<double> sampled_states_acc_; //sampled acceleration from quintic
        
    //feedback reading
    Eigen::VectorXd eig_current_joints_position_; //feedback joint position
    Eigen::VectorXd eig_current_joints_velocity_; //feedback joint velocity

    //update planner
    Eigen::VectorXd eig_desired_joint_pos_; //reference joint position
    Eigen::VectorXd eig_desired_joint_vel_; //reference joint velocity
    Eigen::VectorXd eig_desired_joint_acc_; //reference joint acceleration

    //logic
    bool topic_flag_;
    bool service_flag_;
    bool publish_flag_;

    //times
    double duration_;
    double starting_time_;
    double update_time_;
};

joint_pos_planner::joint_pos_planner(ros::NodeHandle& nh1): node_handle_(nh1)
{
    //messaging topic/service
    sub_ = node_handle_.subscribe("/j2n6s300_driver/out/joint_state",1,&joint_pos_planner::sensor_callback,this);
    pub_ = node_handle_.advertise<jaco_inv_dynamics_controller::JointTrajectoryPoint_w_bool>("/j2n6s300/joint_planner/planner_traj_points",1);
    service_ = node_handle_.advertiseService("move_to_jaco",&joint_pos_planner::trajectory_callback,this);

    traj_msg_.traj_point.positions.resize(6);
    traj_msg_.traj_point.velocities.resize(6);
    traj_msg_.traj_point.accelerations.resize(6);
    traj_msg_.service_called = false;

    //quintic trajectory
    sampled_states_pos_.resize(6);
    sampled_states_vel_.resize(6);
    sampled_states_acc_.resize(6);
    eig_desired_joint_pos_ = Eigen::VectorXd::Zero(6);
    eig_desired_joint_vel_ = Eigen::VectorXd::Zero(6);
    eig_desired_joint_acc_ = Eigen::VectorXd::Zero(6);

    //topic reading
    eig_current_joints_position_ = Eigen::VectorXd::Zero(6);
    eig_current_joints_velocity_ = Eigen::VectorXd::Zero(6);

    //logic
    topic_flag_ = false;
    service_flag_ = false;
    publish_flag_ = false;

    //times
    duration_ = 0.0;
    starting_time_ = 0.0;

    reading_parameters();
}

bool joint_pos_planner::reading_parameters()
{
    if(!node_handle_.getParam("/j2n6s300/param_publish_rate",param_publish_rate_))
    {
        ROS_INFO("Unable to read the parameter /j2n6s300/param_publish_rate, returning false.");
        return false;
    }
    return true;
}

void joint_pos_planner::sensor_callback(const sensor_msgs::JointState& msg)
{
    for (int i = 0; i < 6; i++)
    {
        eig_current_joints_position_(i) = msg.position[i]; //rad
        eig_current_joints_velocity_(i) = msg.velocity[i]; //rad/s
    }
    topic_flag_ = true; //have at least one reading
}

bool joint_pos_planner::trajectory_callback(jaco_inv_dynamics_controller::MoveTo_jaco::Request &req,
jaco_inv_dynamics_controller::MoveTo_jaco::Response &resp)
{
    ROS_INFO("Planner receiving a new trajectory!");
    //can read p1 to p6, the joint positions (rad) that you want to achieve
    array_final_desired_joints_position_[0] = req.p1;
    array_final_desired_joints_position_[1] = req.p2;
    array_final_desired_joints_position_[2] = req.p3;
    array_final_desired_joints_position_[3] = req.p4;
    array_final_desired_joints_position_[4] = req.p5;
    array_final_desired_joints_position_[5] = req.p6;    
    duration_ = req.T;

    resp.exec = true;
    service_flag_ = true;
    return true;
}

void joint_pos_planner::startup_planner(const std::array<double,6>& desired_joint_pos)
{
    //receive feedback to calculate quintic polynomial
    for (int i = 0; i < 6; i++)
    {
        array_current_joints_position_[i] = eig_current_joints_position_(i); //conversion
    }
    trajectory_poly_.init(0.0,array_current_joints_position_,duration_,desired_joint_pos); //calculate the quintic coefficients
    
    starting_time_ = ros::Time::now().toSec();
    service_flag_ = false;
    publish_flag_ = true;
}

void joint_pos_planner::update_planner()
{
    update_time_ = ros::Time::now().toSec(); //get current time in seconds

    if(update_time_ - starting_time_ <= duration_) //make it within duration
    {
        for (int i = 0; i < 6; i++) //updates sampled states, following a quintic polynomial trajectory.
        {
            trajectory_poly_.sampling(update_time_ - starting_time_,trajectory_poly_.coefs_[i],sampled_states_pos_[i],
            sampled_states_vel_[i],sampled_states_acc_[i]);
        }
    }
    else // for t > T
    {
        for (int i = 0; i < 6; i++)
        {
            sampled_states_acc_[i] = 0.0;
            sampled_states_vel_[i] = 0.0;
        }
    }
    for (int i = 0; i < 6; i++)
    {
        eig_desired_joint_pos_(i) = sampled_states_pos_[i]; //conversion
        eig_desired_joint_vel_(i) = sampled_states_vel_[i];
        eig_desired_joint_acc_(i) = sampled_states_acc_[i];
    }

    for (int i = 0; i < 6; i++) //fills up the message
    {
        traj_msg_.traj_point.positions[i] = eig_desired_joint_pos_(i);
        traj_msg_.traj_point.velocities[i] = eig_desired_joint_vel_(i);
        traj_msg_.traj_point.accelerations[i] = eig_desired_joint_acc_(i);
    }
    traj_msg_.service_called = publish_flag_;
    pub_.publish(traj_msg_); //publish trajectory point
}

void joint_pos_planner::update()
{
    if(topic_flag_) //wait for first feedback reading
    {
        if(service_flag_) //wait for service call
        {
            startup_planner(array_final_desired_joints_position_);
        }
        update_planner();
    }
}





