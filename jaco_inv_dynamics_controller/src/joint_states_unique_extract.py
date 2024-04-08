#!/usr/bin/env python
import sys
import os
import csv
import math
import rosbag
import rospy
import copy

# USAGE EXAMPLE IN COMMAND LINE:
#first start the rosbag:
#rosbag record -O /directory-name/jointmsr.bag 
# /j2n6s300/joint_states /j2n6s300/joint_group_effort_controller/command [for simulation]
# /j2n6s300_driver/out/joint_state /j2n6s300_driver/in/joint_torque [for Jaco]

#then rosrun to convert .bag to .csv :
#rosrun jaco_inv_dynamics_controller joint_states_unique_extract.py 
# /directory-name jointmsr sim [for simulation]
#                          jaco [for Jaco]
                                                                                                            
directory = sys.argv[1] # directory-name
filename = sys.argv[2] # jointmsr
type = sys.argv[3] # sim/jaco

# Read the rosbag file
print("Reading the rosbag file")
if not directory.endswith("/"):
  directory += "/"
extension = ""
if not filename.endswith(".bag"):
  extension = ".bag"
bag = rosbag.Bag(directory + filename + extension) #searching for the bagfile

print("Writing joint states data to CSV")
with open(directory + filename + '_joint_states.csv', mode='w') as data_file:
  data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
  data_writer.writerow(['time','act pos 1','act vel 1','act pos 2','act vel 2','act pos 3','act vel 3','act pos 4','act vel 4','act pos 5','act vel 5','act pos 6','act vel 6'])
  if(type == 'sim'): # Record trajectory for gazebo simulation
      count = 1  
      for topic, msg, t in bag.read_messages(topics=['/j2n6s300/joint_states']):
        if((count % 25 == 0)): #save it every 0.025 seconds for 1000hz
          act_pos_1 = msg.position[0]
          act_pos_2 = msg.position[1]
          act_pos_3 = msg.position[2]
          act_pos_4 = msg.position[3]
          act_pos_5 = msg.position[4]
          act_pos_6 = msg.position[5]
          act_vel_1 = msg.velocity[0]
          act_vel_2 = msg.velocity[1]
          act_vel_3 = msg.velocity[2]
          act_vel_4 = msg.velocity[3]
          act_vel_5 = msg.velocity[4]
          act_vel_6 = msg.velocity[5]
          data_writer.writerow([t.to_sec()-bag.get_start_time(),act_pos_1,act_vel_1,act_pos_2,act_vel_2,
                                act_pos_3,act_vel_3,act_pos_4,act_vel_4,act_pos_5,act_vel_5,act_pos_6,act_vel_6])
        count += 1

  elif(type == 'jaco'): # Record trajectory for real-life arm
    for topic, msg, t in bag.read_messages(topics=['/j2n6s300_driver/out/joint_state']):
      act_pos_1 = msg.position[0]
      act_pos_2 = msg.position[1]
      act_pos_3 = msg.position[2]
      act_pos_4 = msg.position[3]
      act_pos_5 = msg.position[4]
      act_pos_6 = msg.position[5]
      act_vel_1 = msg.velocity[0]
      act_vel_2 = msg.velocity[1]
      act_vel_3 = msg.velocity[2]
      act_vel_4 = msg.velocity[3]
      act_vel_5 = msg.velocity[4]
      act_vel_6 = msg.velocity[5]
      data_writer.writerow([t.to_sec()-bag.get_start_time(),act_pos_1,act_vel_1,act_pos_2,act_vel_2,act_pos_3,act_vel_3,act_pos_4,act_vel_4,act_pos_5,act_vel_5,act_pos_6,act_vel_6])

  data_writer.writerow([]) # skips a line
  data_writer.writerow(['time','tau 1','tau 2','tau 3','tau 4','tau 5','tau 6'])

  if (type == 'sim'):
    for topic, msg, t in bag.read_messages(topics=['/j2n6s300/joint_group_effort_controller/command']):
      tau1 = msg.data[0]
      tau2 = msg.data[1]
      tau3 = msg.data[2]
      tau4 = msg.data[3]
      tau5 = msg.data[4]
      tau6 = msg.data[5]
      data_writer.writerow([t.to_sec()-bag.get_start_time(),tau1,tau2,tau3,tau4,tau5,tau6])
  
  elif (type == 'jaco'):
    for topic, msg, t in bag.read_messages(topics=['/j2n6s300_driver/in/joint_torque']):
      tau1 = msg.joint1
      tau2 = msg.joint2
      tau3 = msg.joint3
      tau4 = msg.joint4
      tau5 = msg.joint5
      tau6 = msg.joint6
      data_writer.writerow([t.to_sec()-bag.get_start_time(),tau1,tau2,tau3,tau4,tau5,tau6])
  
print("Finished creating csv file!")
bag.close()