//    Example ROS node. Demonstrates how to write a ROS node to control the simulated WAM.
//    Copyright (C) 2013  Benjamin Blumer
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// This script is used to have a WAM (Barrett.com) model in Gazebo (gazebosim.org) follow a joint trajectory.
// 
// I've been liberal with comments and explaining Gazebo objects/methods/functions.
// In doing so, I've violated the good practice of "Assume the
// reader knows C++ better than you". I've done so in the hope that it will
// help people get their own projects up and running quickly. The Gazebo
// documentation, as of this writing, is lacking. If you have any
// recomendations on making this more accessible, please let me know
// via my github account or make the change yourself and request a "pull".
// Also welcome: changes that make the code better adhere to the Google
// C++ style guide.

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <vector>
#include <fstream>  
#include <iostream>
#include <sstream>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh> 
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo {   
  class SetJointsTimed : public ModelPlugin {
    // Load is called when the model is first loaded. Here we can set up the
    // the plugin's access to the model and load the jointtrajectory. 
    // It gets passed a pointer to the model, as well as to the sdf
    // that contains the model. We don't use the latter.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
      // Store the pointer to the model
      this->model_ = _parent;
      // Find these joint names in the SDF, and store pointers to these joints.
      this->j1_ = this->model_->GetJoint("j1_joint"); 
      this->j2_ = this->model_->GetJoint("j2_joint"); 
      this->j3_ = this->model_->GetJoint("j3_joint"); 
      this->j4_ = this->model_->GetJoint("j4_joint"); 
      this->j5_ = this->model_->GetJoint("j5_joint"); 
      this->j6_ = this->model_->GetJoint("j6_joint"); 
      this->j7_ = this->model_->GetJoint("j7_joint"); 
      // This is a Joint Controller pointer.  This connects to the model and 
      // has control over each joint.
      this->joint_controller_ = new physics::JointController(model_);      
      // Listen to the update event. This event is broadcast every calculation
      // iteration in Gazebo. When a message is received, it will call the
      // onUpdate method.
      this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&SetJointsTimed::OnUpdate, this, _1));
      std::ifstream joint_coordinate_commands;
      // This checks Gazebos paths to try to find the joint coordinate list.
      std::string path_to_joint_coord_file = common::SystemPaths::Instance()->FindFileURI("model://WAM Model Timed/forGazebo");
      joint_coordinate_commands.open(path_to_joint_coord_file.c_str());
      if (joint_coordinate_commands.good()){
	std::cout << "\n Gazebo found the joint coordinate list file at " << path_to_joint_coord_file << std::endl;
      } else {
	std::cout << "\n Gazebo failed to find the joint coordinate list file. There should be a file named forGazebo consisting of 7 CSVs per line in the \"WAM Model Timed\" folder."
	"The first CSV is for joint 0, the second for j1, and so on.	Since you tried to insert the WAM Arm which calls this plugin, and the plugin "
	"couldn't find the joint file, Gazebo is about to crash.  Bye." << std::endl;
      }     
      std::vector< double > one_row_of_joint_commands;
      std::string time_and_all_7_joints, each_joint;
      // This gets one line from forGazebo and stores it as time_and_all_7_joints      
      while (getline(joint_coordinate_commands, time_and_all_7_joints)) {
	std::stringstream time_and_all_7_joints_stream(time_and_all_7_joints);
	// This reads from all_7_joints until it hits a comma. It stores the value in each_joint
	while (getline(time_and_all_7_joints_stream, each_joint, ',')) {
	  one_row_of_joint_commands.push_back(atof(each_joint.c_str())); 
	}    
	all_joint_commands_.push_back(one_row_of_joint_commands);
	one_row_of_joint_commands.clear();
      }
      joint_coordinate_commands.close();
      all_joint_commands_it_ = all_joint_commands_.begin();
      std::cout << "There are " << all_joint_commands_.size() << " different position commands " << std::endl;
      // Need to give this an initial value so it doesn't crash on the first
      // Gazebo calculation iteration.
      this->gazebo_time_at_init_ = this->model_->GetWorld()->GetSimTime();
      this->current_gazebo_time_ = this->model_->GetWorld()->GetSimTime();   
    }
 
    // Called every time updateConnection_ gets a new message.
    public: void OnUpdate(const common::UpdateInfo &){    
      // This returns the value that Gazebo shows on it's "Sim Time" clock.
      this->current_gazebo_time_ = this->model_->GetWorld()->GetSimTime(); 
      // Causes the model to repeat the joint trajectory until it's hard stopped.
      if(all_joint_commands_it_ == all_joint_commands_.end()) {
	all_joint_commands_it_ = all_joint_commands_.begin();
        // Record the time at which we start the trajectory. This is our new "zero time".
	this->gazebo_time_at_init_ = this->model_->GetWorld()->GetSimTime();
      }
      // Check how many seconds have elapsed since we (re)started the trajectory.
      // See if this is greater than the timestamp in the trajectory file.
      // If so, move to the next trajectory point.
      if ((this->current_gazebo_time_.Double() - this->gazebo_time_at_init_.Double()) >= (*all_joint_commands_it_)[0]) {
	joint_controller_->SetJointPosition(j1_, (*all_joint_commands_it_)[1]);
	joint_controller_->SetJointPosition(j2_, (*all_joint_commands_it_)[2]);
	joint_controller_->SetJointPosition(j3_, (*all_joint_commands_it_)[3]);
	joint_controller_->SetJointPosition(j4_, (*all_joint_commands_it_)[4]);
	joint_controller_->SetJointPosition(j5_, (*all_joint_commands_it_)[5]);
	joint_controller_->SetJointPosition(j6_, (*all_joint_commands_it_)[6]);
	joint_controller_->SetJointPosition(j7_, (*all_joint_commands_it_)[7]);
	all_joint_commands_it_++;	
      }        
    }

    private: physics::ModelPtr model_;
    private: event::ConnectionPtr updateConnection_;
    private: physics::JointController * joint_controller_;   
    private: std::vector< std::vector<double> > all_joint_commands_;
    private: std::vector< std::vector<double> >::iterator all_joint_commands_it_;   
    private: gazebo::physics::JointPtr j1_, j2_, j3_, j4_, j5_, j6_, j7_;
    private: common::Time gazebo_time_at_init_;
    private: common::Time current_gazebo_time_;
  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SetJointsTimed)
}
