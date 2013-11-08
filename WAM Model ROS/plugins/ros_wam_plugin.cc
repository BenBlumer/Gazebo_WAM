/*    WAM Plugin: Gazebo plugin for controlling a WAM model from a ROS trajectory.
 *    Copyright (C) 2013  Benjamin Blumer
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************************
 *
 * This plugin listens for trajectories on the ROS topic joint_traj.
 * 
 * I've heavily commented the code to make it accessible for people who are new
 * to ROS (ros.org) and Gazebo (gazebosim.org) plugins. In doing so, 
 * I've violated the good practice of 
 * "Assume the reader knows the language better than you". 
 * I've done so in the hope that it will help people get their own projects up 
 * and running quickly.  If you have any recomendations on making this more 
 * accessible, please let me know via my github account or make the change yourself
 * and request a "pull".
 */


#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

namespace gazebo {   
  class ROSModelPlugin : public ModelPlugin {    
  public: 
    ROSModelPlugin() {    
      std::string node_name = "ROS_WAM_listener";
      int argc = 0;
      // ros::init sets up our ROS node. It requires the command line arguments,
      // but passing NULL as the second argument tells it not to use them. 
      ros::init(argc, NULL, node_name); 
      this->node = new ros::NodeHandle; 
    }
    ~ROSModelPlugin() { delete this->node; }  
    
    void Load(physics::ModelPtr _parent, sdf::ElementPtr) {
      // This pointer to the model is passed by Gazebo. It's our way to alter 
      // and get information about the model.
      this->model = _parent;
      // We don't have a trajectory from ROS on initialization. 
      this->have_a_trajectory = false;
      // So we're on the zeroth coordinate.
      this->current_joint_trajectory_point = 0;
      // Find these joint names in the SDF, and store pointers to these joints.
      this->j1 = this->model->GetJoint("j1_joint"); 
      this->j2 = this->model->GetJoint("j2_joint"); 
      this->j3 = this->model->GetJoint("j3_joint"); 
      this->j4 = this->model->GetJoint("j4_joint"); 
      this->j5 = this->model->GetJoint("j5_joint"); 
      this->j6 = this->model->GetJoint("j6_joint"); 
      this->j7 = this->model->GetJoint("j7_joint");       
      // Set a joint controller to associate itself with our WAM model.
      this->WAM_joint_controller = new physics::JointController(model);
      // Subscribe to the ROS topic "joint_traj" This is published by the
      // included ROS exampleROSNODE.py. The parameter 10 specifies how
      // many trajectories to queue if they come in faster than they are
      // executed. Because joint trajectories can be long, and we don't 
      // the model to play for long after the ROS publisher is killed, 
      // we limit it to 10. When we find a trajectory on joint_traj,
      // call ROSCallback to do something with it. The final parameter
      // indicates that ROSCallback method is accessible through an instance
      // of this class. It's not needed if Callback is a function external to
      // a class rather than a method.
      this->sub = this->node->subscribe("joint_traj", 10, &ROSModelPlugin::ROSCallback, this);
      // This allows the OnUpdate() method to be called
      // every time Gazebo computes another iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ROSModelPlugin::OnUpdate, this));
      // Record the time when our plugin is activated. All trajectory time
      // stamps are counted from this time. 
      this->current_gazebo_time = this->model->GetWorld()->GetSimTime();
    }
    
    // This is called everytime Gazebo asks for news from our plugin.
    // It's also called when the plugin is first loaded. 
    void OnUpdate() {
      // See if ROS has a trajectory for us. If not, wait.
      if (this->have_a_trajectory == false){
        ros::spinOnce();
      // If there is a trajectory, see if it's time to move to the next trajectory point.
      } else {
        // This returns the value that Gazebo shows on it's "Sim Time" clock.
        this->current_gazebo_time = this->model->GetWorld()->GetSimTime(); 
        // Check if we still have joint coordinates to visit. If we do, has enough time elapsed to make the move?
        if ((this->current_joint_trajectory_point < number_of_joint_trajectory_points) &&
          ((this->current_gazebo_time.Double() - time_when_trajectory_received.Double()) > (*joint_trajectory).points[this->current_joint_trajectory_point].time_from_start.toSec())) {
          //Commands Gazebo to set the joint positions of our model according to the values in our trajectory.
          WAM_joint_controller->SetJointPosition(j1, (*joint_trajectory).points[this->current_joint_trajectory_point].positions[0]);
          WAM_joint_controller->SetJointPosition(j2, (*joint_trajectory).points[this->current_joint_trajectory_point].positions[1]);
          WAM_joint_controller->SetJointPosition(j3, (*joint_trajectory).points[this->current_joint_trajectory_point].positions[2]);
          WAM_joint_controller->SetJointPosition(j4, (*joint_trajectory).points[this->current_joint_trajectory_point].positions[3]);
          WAM_joint_controller->SetJointPosition(j5, (*joint_trajectory).points[this->current_joint_trajectory_point].positions[4]);
          WAM_joint_controller->SetJointPosition(j6, (*joint_trajectory).points[this->current_joint_trajectory_point].positions[5]);
          WAM_joint_controller->SetJointPosition(j7, (*joint_trajectory).points[this->current_joint_trajectory_point].positions[6]);
          this->current_joint_trajectory_point++;
          }  
          // If we're at the end of the trajectory, reset the point counter
          // and set the flag to look for another trajectory.
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
    private: physics::JointController * WAM_joint_controller;   
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    private: gazebo::physics::JointPtr j1, j2, j3, j4, j5, j6, j7;
    private: common::Time current_gazebo_time;
    private: bool have_a_trajectory;
    private: common::Time time_when_trajectory_received;
    private: int number_of_joint_trajectory_points;
    private: int current_joint_trajectory_point;
    private: trajectory_msgs::JointTrajectory::ConstPtr joint_trajectory;
    private: ros::NodeHandle* node;
    ros::Subscriber sub;
  };
  
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSModelPlugin)
}
