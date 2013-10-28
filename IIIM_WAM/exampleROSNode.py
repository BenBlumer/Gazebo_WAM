#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#import _Num.py


def talker():
    #Publish on ROS topic "joint_traj"
    pub = rospy.Publisher('joint_traj', JointTrajectory)
    #this node is called trajectory_giver
    rospy.init_node('trajectory_giver')
    #Seven joints on a Barrettt WAM Arm.
    number_of_joints = 7   
   
    all_sets_of_joint_coordinates = JointTrajectory()
    # Each joint coordinate/joint point consists of desired positions for each joint and a time for when the joints should assume their positions.
    one_set_of_joint_coordinates = JointTrajectoryPoint()
    
    all_sets_of_joint_coordinates.header.stamp = rospy.Time.now()
    #Create  J1 through J7
    for joint_number in range (1, number_of_joints + 1):  
       all_sets_of_joint_coordinates.joint_names.append("J%i"%joint_number)
    
    #Usually you'd be feeding the joint trajectory in from somewhere intelligent. Here we create a hundred points.
    # Each point is going to move all 7 joints an additional .01 degrees.
    # Each point takes place .01 seconds after the other.
    number_of_boring_points = 100
    #initial starting position for all joints
    joint_coord = .1
    # time at which to take initial positions
    time_for_move = .01
    #
    for i in range (0, number_of_boring_points): 
       for joint_number in range (1, number_of_joints + 1):
	  # for one point, set all desired joint positions equal to joint_coord
          one_set_of_joint_coordinates.positions.append(joint_coord)
       #And set the time for the desired joint positions
       one_set_of_joint_coordinates.time_from_start = rospy.Duration.from_sec(time_for_move)
       #append that joint coordinate to the list of joint coordinates
       all_sets_of_joint_coordinates.points.append(one_set_of_joint_coordinates)
       # Reset the holding variable for one joint coordinate
       one_set_of_joint_coordinates = None
       one_set_of_joint_coordinates = JointTrajectoryPoint()
       #Increment the desired position
       joint_coord = joint_coord + .01
       #Increment the time.
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
