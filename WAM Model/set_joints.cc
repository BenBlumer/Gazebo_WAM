/* 
 *  		 Written by Benjamin Blumer for controlling a Gazebo (gazebosim.org) model of the Barrett WAM Arm.
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
#define NUMBER_OF_JOINT_COORDINATES 435 // This is the number of rows in the file of joint coordinates.
namespace gazebo
{   
  class SetJoints : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
    {
      // Store the pointer to the model
      this->model = _parent;
      // Find these joint names in the SDF, and store pointers to these joints.
      this->j1 = this->model->GetJoint("j1_joint"); 
      this->j2 = this->model->GetJoint("j2_joint"); 
      this->j3 = this->model->GetJoint("j3_joint"); 
      this->j4 = this->model->GetJoint("j4_joint"); 
      this->j5 = this->model->GetJoint("j5_joint"); 
      this->j6 = this->model->GetJoint("j6_joint"); 
      this->j7 = this->model->GetJoint("j7_joint"); 
      
      //This is a Joint Controller pointer.  This connects to the model and has control over each joint.
      this->j2_controller = new physics::JointController(model);
      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
	boost::bind(&SetJoints::OnUpdate, this, _1));
      
      i = 0;
      iteration_number = 0;
      
      double vtime,v1,v2,v3,v4,v5,v6,v7;
      JointAngles_file = fopen("forGazebo", "r");
      
      for (int ii=0; ii<NUMBER_OF_JOINT_COORDINATES; ++ii){
	  fscanf(JointAngles_file," %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", &v1, &v2, &v3, &v4, &v5, &v6, &v7);
	  pos1[ii][0]=v1; 		
	  pos1[ii][1]=v2;
	  pos1[ii][2]=v3;
	  pos1[ii][3]=v4;
	  pos1[ii][4]=v5;
	  pos1[ii][5]=v6;
	  pos1[ii][6]=v7;
      }
      
    }
    
    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(math::Vector3(.03, 0, 0));
      
       iteration_number++;
     
      if( i > NUMBER_OF_JOINT_COORDINATES)
      {
	i = 0 ;
      }
      
      if(iteration_number > 15){
	  
	iteration_number = 0;
      j2_controller->SetJointPosition(j1, pos1[i][0]);
      j2_controller->SetJointPosition(j2, pos1[i][1]);
      j2_controller->SetJointPosition(j3, pos1[i][2]);
      j2_controller->SetJointPosition(j4, pos1[i][3]);
      j2_controller->SetJointPosition(j5, pos1[i][4]);
      j2_controller->SetJointPosition(j6, pos1[i][5]);
      j2_controller->SetJointPosition(j7, pos1[i][6]);
       i++; 
      }
      
    }
    
    // Pointer to the model
    private: physics::ModelPtr model;
    
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    private: physics::JointController * j2_controller;
    
    private: int i, iteration_number;
    private: double angle;
    private: FILE* JointAngles_file;
    private: double pos1[NUMBER_OF_JOINT_COORDINATES][7];
    private: gazebo::physics::JointPtr j1, j2, j3, j4, j5, j6, j7;
  };
  
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SetJoints)
}