
//utility
#include <ros/ros.h>
#include <cmath>
#include <Eigen/Dense>

//messaging
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "jaco_inv_dynamics_controller/MoveTo_jaco.h"

//trajectory
#include "quintic_poly.h"

class PID_controller
{
public:
    PID_controller(ros::NodeHandle& nh1);
    bool reading_parameters(); //load initial target postion from parameter server / .yaml file
    void sensor_callback(const sensor_msgs::JointState& msg); //for reading joint positions
    bool trajectory_callback(jaco_inv_dynamics_controller::MoveTo_jaco::Request &req,
    jaco_inv_dynamics_controller::MoveTo_jaco::Response &resp); //new trajectory
    void startup_planner(const std::array<double,6>& desired_joint_pos); //finds quintic trajectory coefficients
    void update_planner(); //sample entire trajectory and publish
    void update(); //main update loop called by node

    ros::NodeHandle& node_handle_;

    //messaging topic/service
    ros::Subscriber sub_; //subscribe to joint feedback
    ros::Publisher pub_; //publish entire trajectory
    ros::ServiceServer service_; //server for new trajectory
    trajectory_msgs::JointTrajectoryPoint traj_point_; //single trajectory point
    trajectory_msgs::JointTrajectory traj_msg_; //entire trajectory
    int traj_point_count_; //count for total number of points within trajectory

    //from parameters
    std::vector<double> param_des_joint_pos_; //holds initial desired joint positions
    double duration_; //updated trajectory duration

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

    //logic
    bool topic_flag_;
    bool service_flag_;
    bool initial_flag_;

    //times
    double sampling_time_;
};

PID_controller::PID_controller(ros::NodeHandle& nh1): node_handle_(nh1)
{
    //messaging topic/service
    sub_ = node_handle_.subscribe("/j2n6s300_driver/out/joint_state",1,&PID_controller::sensor_callback,this);
    pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>("/j2n6s300_driver/trajectory_controller/command",1);
    service_ = node_handle_.advertiseService("move_to_jaco",&PID_controller::trajectory_callback,this);

    traj_msg_.joint_names.resize(6);
    traj_msg_.joint_names = {"j2n6s300_joint_1","j2n6s300_joint_2","j2n6s300_joint_3",
    "j2n6s300_joint_4","j2n6s300_joint_5","j2n6s300_joint_6"};
    traj_point_.positions.resize(6);
    traj_point_.velocities.resize(6);
    traj_point_.accelerations.resize(6);
    sampling_time_ = 0.0;

    //quintic trajectory
    sampled_states_pos_.resize(6);
    sampled_states_vel_.resize(6);
    sampled_states_acc_.resize(6);

    //feedback reading
    eig_current_joints_position_ = Eigen::VectorXd::Zero(6);
    eig_current_joints_velocity_ = Eigen::VectorXd::Zero(6);

    //logic
    topic_flag_ = false;
    service_flag_ = false;
    initial_flag_ = true;

    reading_parameters();
    traj_point_count_ = (int)(duration_/0.01); ///fix to 100 Hz
    traj_msg_.points.resize(traj_point_count_);
}

bool PID_controller::reading_parameters()
{
    if(!node_handle_.getParam("/j2n6s300/target_home_positions",param_des_joint_pos_))
    {
        ROS_INFO("Unable to read the parameter /j2n6s300/target_home_positions, returning false.");
        return false;
    }
    if(!node_handle_.getParam("/j2n6s300/target_time",duration_))
    {
        ROS_INFO("Unable to read the parameter /j2n6s300/target_time, returning false.");
        return false;
    }
    for(int i = 0; i < 6; i++)
    {
        array_final_desired_joints_position_[i] = param_des_joint_pos_[i];
    }
    return true;
}

void PID_controller::sensor_callback(const sensor_msgs::JointState& msg)
{
    for (int i = 0; i < 6; i++)
    {
        eig_current_joints_position_(i) = msg.position[i];
        eig_current_joints_velocity_(i) = msg.velocity[i];
    }
    topic_flag_ = true; //have at least one reading
}

bool PID_controller::trajectory_callback(jaco_inv_dynamics_controller::MoveTo_jaco::Request &req,
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

    traj_point_count_ = (int)(duration_/0.01); //update count
    traj_msg_.points.clear();
    traj_msg_.points.resize(traj_point_count_);

    resp.exec = true;
    service_flag_ = true;
    return true;
}

void PID_controller::startup_planner(const std::array<double,6>& desired_joint_pos)
{
    for (int i = 0; i < 6; i++)
    {
        array_current_joints_position_[i] = eig_current_joints_position_(i); //conversion Eigen::VectorXd --> std::array
    }
    trajectory_poly_.init(0.0,array_current_joints_position_,duration_,desired_joint_pos); //calculate the quintic coefficients
}

void PID_controller::update_planner()
{
    for (int count = 0; count < traj_point_count_; count++)
    {
        traj_point_.time_from_start = ros::Duration(sampling_time_);
        for (int i = 0; i < 6; i++) //updates sampled states, following a quintic polynomial trajectory
        {
            trajectory_poly_.sampling(sampling_time_,trajectory_poly_.coefs_[i],sampled_states_pos_[i],
            sampled_states_vel_[i],sampled_states_acc_[i]);
            traj_point_.positions[i] = sampled_states_pos_[i];
            traj_point_.velocities[i] = sampled_states_vel_[i];
            traj_point_.accelerations[i] = sampled_states_acc_[i];
        }
        traj_msg_.points[count] = traj_point_; //fill it up one at a time
        sampling_time_ += 0.01; //sample every 0.01seconds
    }

    traj_msg_.header.stamp = ros::Time::now();
    pub_.publish(traj_msg_); //one time publishing of full trajectory
    sampling_time_ = 0.0; //restarts it for next service call
}

void PID_controller::update()
{
    if(topic_flag_) //wait for first feedback reading
    {
        if(initial_flag_) //sends initial trajectory from parameters
        {
            startup_planner(array_final_desired_joints_position_);
            initial_flag_ = false;
        }
        else if (service_flag_) //wait for service call
        {
            startup_planner(array_final_desired_joints_position_);
            service_flag_ = false;
        }
    update_planner();
    }
}








