/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <boost/bind.hpp>
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"
#include <iostream>

/// \example examples/plugins/model_push.cc
/// This example creates a ModelPlugin, and applies a force to a box to move
/// it alone the ground plane.
namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->box_link_pt = this->model->GetLink("box_link");
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelPush::OnUpdate, this));
     

      std::ifstream pose_coordinate_commands;
      std::string path_to_pose_coord_file =  common::SystemPaths::Instance()->FindFileURI("model://WAM Model Timed With Camera/blockPose");
      
      pose_coordinate_commands.open(path_to_pose_coord_file.c_str());
      if (pose_coordinate_commands.good())
      {
	std::cout << "\n Gazebo found the block pose coordinate list file at " << path_to_pose_coord_file << std::endl;
      }
      else
      {
	std::cout << "\n Gazebo failed to find the block pose coordinate list file. There should be a file named blockPose consisting of 7 CSVs per line in the \"WAM Model Timed With Camera\" folder."
	"The first CSV is for the time, the second, third and fourth, for X, Y, and Z. The fifth, sixth, and seventh for roll, pitch, and yaw. Since you tried to insert the block which calls this plugin, and the plugin "
	"couldn't find the pose file, Gazebo is about to crash.  Bye." << std::endl; 
      }
      
      
      std::vector< double > one_row_of_pose_commands;
      std::string time_and_all_6_pose, each_pose;
      // This gets one line from forGazebo and stores it as time_and_all_6_pose
      
      while (getline(pose_coordinate_commands, time_and_all_6_pose)) 
      {
	std::stringstream time_and_all_6_pose_stream(time_and_all_6_pose);
	//This reads from all_7_joints until it hits a comma. It stores the value in each_pose
	while (getline(time_and_all_6_pose_stream, each_pose, ',')) 
	{
	  one_row_of_pose_commands.push_back(atof(each_pose.c_str())); 
	}    
	all_pose_commands.push_back(one_row_of_pose_commands);
	//std::cout << "Pushing back: " << one_row_of_pose_commands[0] << std::endl;
	one_row_of_pose_commands.clear();
      }
      pose_coordinate_commands.close();
      all_pose_commands_it = all_pose_commands.begin();
      std::cout << "There are " << all_pose_commands.size() << " different position commands " << std::endl;
      
      this->gazebo_time_at_init = this->model->GetWorld()->GetSimTime();
      this->current_gazebo_time = this->model->GetWorld()->GetSimTime();
      
      
      
    }
    
    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      this->current_gazebo_time = this->model->GetWorld()->GetSimTime(); //This returns the value that Gazebo shows on it's "Sim Time" clock.
      
      //Have we gone through another iteration of the full motion?
      if(all_pose_commands_it == all_pose_commands.end())
      {
	all_pose_commands_it = all_pose_commands.begin(); // Then we'll start from the beginning.
	       this->gazebo_time_at_init = this->model->GetWorld()->GetSimTime();

      }
      
      //The first time through, the difference between the current clock time and the time when the model started has to be greater than the time stamp in the file.
      // The second time, it has to be twice as great.  This is what iteration_number does.
      if ((this->current_gazebo_time.Double() - this->gazebo_time_at_init.Double()) > (*all_pose_commands_it)[0])
      {
	math::Pose box_pose ( (*all_pose_commands_it)[1], (*all_pose_commands_it)[2], (*all_pose_commands_it)[3], (*all_pose_commands_it)[4], (*all_pose_commands_it)[5], (*all_pose_commands_it)[6]);
	
	this->model->SetLinkWorldPose(box_pose, box_link_pt);
	
	all_pose_commands_it++;	
      } 
    }
    
    // Pointer to the model
    private: physics::ModelPtr model;
    
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    private: common::Time gazebo_time_at_init;
    private: common::Time current_gazebo_time;
    private: std::vector< std::vector<double> > all_pose_commands;
    private: std::vector< std::vector<double> >::iterator all_pose_commands_it;  
    private: gazebo::physics::LinkPtr box_link_pt;

    
  };
  
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
