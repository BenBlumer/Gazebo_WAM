/* 
 *  		 Written by Benjamin Blumer for controlling a Gazebo (gazebosim.org) model of the Barrett WAM Arm by IRI.
 * 		 No guarantees on functionality or performance. 
 * 		 Feel free to distribute the code, but leave the credits intact.
 * 		 BenjaminAaronBlumer@gmail.com
 * 
 */

#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <vector>
#include <fstream>  
#include <iostream>
#include <sstream>
#include <stdlib.h>

namespace gazebo
{   
  class SetJointsTimedCam : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
    {
      // Store the pointer to the model
      this->model = _parent;
      //       // Find these joint names in the SDF, and store pointers to these joints.
      this->j1 = this->model->GetJoint("j1_joint"); 
      this->j2 = this->model->GetJoint("j2_joint"); 
      this->j3 = this->model->GetJoint("j3_joint"); 
      this->j4 = this->model->GetJoint("j4_joint"); 
      this->j5 = this->model->GetJoint("j5_joint"); 
      this->j6 = this->model->GetJoint("j6_joint"); 
      this->j7 = this->model->GetJoint("j7_joint"); 
      
      
      //       //This is a Joint Controller pointer.  This connects to the model and has control over each joint.
      this->j2_controller = new physics::JointController(model);
      //       
      //       // Listen to the update event. This event is broadcast every
      //       // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SetJointsTimedCam::OnUpdate, this, _1));
      std::ifstream joint_coordinate_commands;
      std::string path_to_joint_coord_file =  common::SystemPaths::Instance()->FindFileURI("model://WAM Model Timed With Camera/forGazebo");
      
      joint_coordinate_commands.open(path_to_joint_coord_file.c_str());
      if (joint_coordinate_commands.good())
      {
	std::cout << "\n Gazebo found the joint coordinate list file at " << path_to_joint_coord_file << std::endl;
      }
      else
      {
	std::cout << "\n Gazebo failed to find the joint coordinate list file. There should be a file named forGazebo consisting of 7 CSVs per line in the \"WAM Model Timed With Camera\" folder."
	"The first CSV is for joint 0, the second for j1, and so on.	Since you tried to insert the WAM Arm which calls this plugin, and the plugin "
	"couldn't find the joint file, Gazebo is about to crash.  Bye." << std::endl;
      }
      
      std::vector< double > one_row_of_joint_commands;
      std::string time_and_all_7_joints, each_joint;
      // This gets one line from forGazebo and stores it as time_and_all_7_joints
      
      while (getline(joint_coordinate_commands, time_and_all_7_joints)) 
      {
	std::stringstream time_and_all_7_joints_stream(time_and_all_7_joints);
	//This reads from all_7_joints until it hits a comma. It stores the value in each_joint
	while (getline(time_and_all_7_joints_stream, each_joint, ',')) 
	{
	  one_row_of_joint_commands.push_back(atof(each_joint.c_str())); 
	}    
	all_joint_commands.push_back(one_row_of_joint_commands);
	//std::cout << "Pushing back: " << one_row_of_joint_commands[0] << std::endl;
	one_row_of_joint_commands.clear();
      }
      joint_coordinate_commands.close();
      all_joint_commands_it = all_joint_commands.begin();
      std::cout << "There are " << all_joint_commands.size() << " different position commands " << std::endl;
      
      this->gazebo_time_at_init = this->model->GetWorld()->GetSimTime();
      this->current_gazebo_time = this->model->GetWorld()->GetSimTime();
      
    }
    
    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {    
      
      this->current_gazebo_time = this->model->GetWorld()->GetSimTime(); //This returns the value that Gazebo shows on it's "Sim Time" clock.
      
      
      if(all_joint_commands_it == all_joint_commands.end())
      {
	all_joint_commands_it = all_joint_commands.begin();
       this->gazebo_time_at_init = this->model->GetWorld()->GetSimTime();


      }
      
      if ((this->current_gazebo_time.Double() - this->gazebo_time_at_init.Double()) >=  (*all_joint_commands_it)[0])
      {
	j2_controller->SetJointPosition(j1, (*all_joint_commands_it)[1]);
	j2_controller->SetJointPosition(j2, (*all_joint_commands_it)[2]);
	j2_controller->SetJointPosition(j3, (*all_joint_commands_it)[3]);
	j2_controller->SetJointPosition(j4, (*all_joint_commands_it)[4]);
	j2_controller->SetJointPosition(j5, (*all_joint_commands_it)[5]);
	j2_controller->SetJointPosition(j6, (*all_joint_commands_it)[6]);
	j2_controller->SetJointPosition(j7, (*all_joint_commands_it)[7]);
	all_joint_commands_it++;	

      }    
      
    }
    
    //Pointer to the model
    private: physics::ModelPtr model;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    private: physics::JointController * j2_controller;   
    private: int i, number_of_joint_commands_in_file;
    private: std::vector< std::vector<double> > all_joint_commands;
    private: std::vector< std::vector<double> >::iterator all_joint_commands_it;   
    private: gazebo::physics::JointPtr j1, j2, j3, j4, j5, j6, j7;
    private: common::Time gazebo_time_at_init;
    private: common::Time current_gazebo_time;
    
  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SetJointsTimedCam)
}
