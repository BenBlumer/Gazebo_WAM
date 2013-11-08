#!/usr/bin/env python

#    Example ROS node. Demonstrates how to write a ROS node to control the simulated WAM.
#    Copyright (C) 2013  Benjamin Blumer
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""
This script is used to publish a trajectory on the ROS topic joint_traj.

The included WAM plugin subscribes to this topic, and carries out the motion.
The trajectory starts every joint off at a value of .1 and increases the
value by .01 rads every .1 seconds. You can easily replace this with a 
meaningful trajectory by filling one_set_of_joint_coordinates with the
coordinates you want before appending it to all_sets_of_joint_coordinates.

I've heavily commented the code to make it accessible for people who are new
to ROS. In doing so, I've violated the good practice of "Assume the
reader knows Python better than you". I've done so in the hope that it will
help people get their own projects up and running quickly.  If you have any
recomendations on making this more accessible, please let me know
via my github account or make the change yourself and request a "pull".
"""

import rospy
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#import _Num.py


def talker():
    # Publish on ROS topic "joint_traj"
    pub = rospy.Publisher('joint_traj', JointTrajectory)
    # This node is called trajectory_giver
    rospy.init_node('trajectory_giver')
    # Seven joints on a Barrettt WAM Arm.
    number_of_joints = 7   
    all_sets_of_joint_coordinates = JointTrajectory()
    # Each joint coordinate/joint point consists of desired positions for each 
    # joint and a time for when the joints should assume their positions.
    one_set_of_joint_coordinates = JointTrajectoryPoint()
    all_sets_of_joint_coordinates.header.stamp = rospy.Time.now()
    # Create  J1 through J7
    for joint_number in range (1, number_of_joints + 1):  
       all_sets_of_joint_coordinates.joint_names.append("J%i"%joint_number)
    # Usually you'd be feeding the joint trajectory in from somewhere intelligent. Here we create a hundred points.
    # Each point is going to move all 7 joints an additional .01 degrees.
    # Each point takes place .01 seconds after the other.
    number_of_boring_points = 100
    # initial starting position for all joints
    joint_coord = .1
    # time at which to take initial positions
    time_for_move = .01
    for i in range (0, number_of_boring_points): 
       for joint_number in range (1, number_of_joints + 1):
	  # for one point, set all desired joint positions equal to joint_coord
          one_set_of_joint_coordinates.positions.append(joint_coord)
       # And set the time for the desired joint positions
       one_set_of_joint_coordinates.time_from_start = rospy.Duration.from_sec(time_for_move)
       # append that joint coordinate to the list of joint coordinates
       all_sets_of_joint_coordinates.points.append(one_set_of_joint_coordinates)
       # Reset the holding variable for one joint coordinate
       one_set_of_joint_coordinates = None
       one_set_of_joint_coordinates = JointTrajectoryPoint()
       # Increment the desired position.
       joint_coord = joint_coord + .01
       # Increment the time.
       time_for_move += .01

    while not rospy.is_shutdown():
        str = "Published a joint trajectory as time %s" % rospy.get_time()
	#Send a comment to the terminal stating the time the trajectory was published
        rospy.loginfo(str)
        #Publish the set of joint coordinates
        pub.publish(all_sets_of_joint_coordinates)
        #Wait.
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
