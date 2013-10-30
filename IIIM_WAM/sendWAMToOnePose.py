#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

def talker():
    #Publish on ROS topic "joint_traj"
    pub = rospy.Publisher('joint_traj', JointTrajectory)
    #this node is called trajectory_giver
    rospy.init_node('trajectory_giver')
    #Seven joints on a Barrettt WAM Arm.
    number_of_joints = 7   
    all_sets_of_joint_coordinates = JointTrajectory()
    one_set_of_joint_coordinates = JointTrajectoryPoint()
    all_sets_of_joint_coordinates.header.stamp = rospy.Time.now()
    #Create J1 through J7
    for joint_number in range (1, number_of_joints + 1):  
       all_sets_of_joint_coordinates.joint_names.append("J%i"%joint_number)
    #This program is run by the user giving joint positions for all 7 joints that they want the robot to hold.
    for i in sys.argv[1:]:
      one_set_of_joint_coordinates.positions.append(float(i))
    time_resolution = 0.01 # In seconds. The smaller the number, the less choppy the animation will be. Gravity takes hold in between the time points.
    start_time = 0.0# In seconds
    current_time = start_time
    end_time = 10.0 # In seconds
    number_of_time_steps = int(end_time / time_resolution)
    #Put the same joint coordinate into the trajectory every time_resolution seconds from the start_time to the end_time; tell the robot to hold a pose.
    for timestep in range(0, number_of_time_steps , 1):
      current_time = current_time + time_resolution
      # You must create a new JointTrajectoryPoint every iteration because the JointTrajectory stores pointers to each Point, and each point can only hold one time value.
      tempJointCoordinates = None 
      tempJointCoordinates = JointTrajectoryPoint()
      tempJointCoordinates.positions = one_set_of_joint_coordinates.positions
      tempJointCoordinates.time_from_start = rospy.Duration.from_sec(current_time)
      all_sets_of_joint_coordinates.points.append(tempJointCoordinates)
    while not rospy.is_shutdown():
        str = "Published a joint trajectory."
        rospy.loginfo(str)
        pub.publish(all_sets_of_joint_coordinates)
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
