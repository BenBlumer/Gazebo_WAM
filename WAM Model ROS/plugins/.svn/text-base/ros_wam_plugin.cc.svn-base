/*
* Modified from the Gazebo example at http://gazebosim.org/wiki/Tutorials/1.5/ros_enabled_model_plugin . 
* The information on use of the ROS functions can be found at http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29.
* 
* Benjamin Blumer 2013
*
*
*/

#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

namespace gazebo
{   
  class ROSModelPlugin : public ModelPlugin
  {
    
    public: ROSModelPlugin()
    {
      
      // Start up ROS
      std::string name = "ros_box_model_listener";
      int argc = 0;
      // argc so it can any commands from commandline, NULL is argv -- the length of the string.  Setting it to zero is probably telling ROS to no use any of the commandline arguments
      // Name is the name of the node. ros::init is required to start talking to ROS.
      ros::init(argc, NULL, name); 
    }
    
    public: ~ROSModelPlugin()
    {
      delete this->node;
    }
    
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      //We don't have a trajectory from ROS yet.
      this->have_a_trajectory = false;
      this->current_joint_trajectory_point = 0;
      //       // Find these joint names in the SDF, and store pointers to these joints.
      this->j1 = this->model->GetJoint("j1_joint"); 
      this->j2 = this->model->GetJoint("j2_joint"); 
      this->j3 = this->model->GetJoint("j3_joint"); 
      this->j4 = this->model->GetJoint("j4_joint"); 
      this->j5 = this->model->GetJoint("j5_joint"); 
      this->j6 = this->model->GetJoint("j6_joint"); 
      this->j7 = this->model->GetJoint("j7_joint"); 
      
      // ROS Nodehandle
      this->node = new ros::NodeHandle(""); 
      // Set a joint controller to associate itself with our WAM model.
      this->j2_controller = new physics::JointController(model);
      //Subscribe to the ROS topic "joint_traj".
      this->sub = this->node->subscribe("joint_traj", 1000, &ROSModelPlugin::ROSCallback, this);
 
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStart(
	boost::bind(&ROSModelPlugin::OnUpdate, this));
      //Record the time when our plugin
      this->current_gazebo_time = this->model->GetWorld()->GetSimTime();
    }
    
    // This is called everytime Gazebo asks for news from our plugin.
    // It's also called when the plugin is first loaded. 
    public: void OnUpdate()
    {
      if (this->have_a_trajectory == false){
	//See if ROS has a trajectory for us.
	ros::spinOnce();
      }
      else{

	this->current_gazebo_time = this->model->GetWorld()->GetSimTime(); //This returns the value that Gazebo shows on it's "Sim Time" clock.
	//Check if we still have joint coordinates to visit. If we do, has enough time elapsed to make the move?
	if ((this->current_joint_trajectory_point < number_of_joint_trajectory_points) &&
	  ((this->current_gazebo_time.Double() - time_when_trajectory_received.Double()) > (*joint_trajectory).points[this->current_joint_trajectory_point].time_from_start.toSec()))
	{
	  //Commands Gazebo to set the joint positions of our model according to the values in our trajectory.
	  j2_controller->SetJointPosition(j1, (*joint_trajectory).points[this->current_joint_trajectory_point].positions[0]);
	  j2_controller->SetJointPosition(j2, (*joint_trajectory).points[this->current_joint_trajectory_point].positions[1]);
	  j2_controller->SetJointPosition(j3, (*joint_trajectory).points[this->current_joint_trajectory_point].positions[2]);
	  j2_controller->SetJointPosition(j4, (*joint_trajectory).points[this->current_joint_trajectory_point].positions[3]);
	  j2_controller->SetJointPosition(j5, (*joint_trajectory).points[this->current_joint_trajectory_point].positions[4]);
	  j2_controller->SetJointPosition(j6, (*joint_trajectory).points[this->current_joint_trajectory_point].positions[5]);
	  j2_controller->SetJointPosition(j7, (*joint_trajectory).points[this->current_joint_trajectory_point].positions[6]);
	  this->current_joint_trajectory_point++;
	}  
	//If we're at the end of the trajectory, reset it and set the flag to look for another trajectory.
	if (this->current_joint_trajectory_point == number_of_joint_trajectory_points){
	 this->current_joint_trajectory_point = 0;	  
	  this->have_a_trajectory = false;
	}
      }
      
    }
    
    //This function is called everytime we get a message from ROS.
      void ROSCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
      {
	//This tells us when we started the trajectory.  All time decisions will be relative to this starting point.
	this->time_when_trajectory_received = this->model->GetWorld()->GetSimTime(); 
        //If this function's been called, it's because we've received a joint trajectory from ROS.
	this->have_a_trajectory = true;
	//Check how many coordinates there are in the received trajectory.
	this->number_of_joint_trajectory_points = (*msg).points.size();
	//Send output to the terminal:
	ROS_INFO("Subscriber got a trajectory");
	//Store the trajectory information for use in the onUpdate function.
	this->joint_trajectory = msg;
      }
      
      // Pointer to the model
      private: physics::ModelPtr model;
      private: physics::JointController * j2_controller;   
      // Pointer to the update event connection
      private: event::ConnectionPtr updateConnection;
      private: gazebo::physics::JointPtr j1, j2, j3, j4, j5, j6, j7;
      private: common::Time current_gazebo_time;
      private: bool have_a_trajectory;
      private: common::Time time_when_trajectory_received;
      private: int number_of_joint_trajectory_points;
      private: 	int current_joint_trajectory_point;
  private: trajectory_msgs::JointTrajectory::ConstPtr joint_trajectory;
      // ROS Nodehandle
      private: ros::NodeHandle* node;
      // ROS Subscriber
      ros::Subscriber sub;
  };
  
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSModelPlugin)
}