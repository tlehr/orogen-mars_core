/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AuvMotion.hpp"
#include <mars/interfaces/sim/NodeManagerInterface.h>

using namespace simulation;

AuvMotion::AuvMotion(std::string const& name)
    : AuvMotionBase(name)
{
  
  //Initialize the center of buoyancy and the damping-coefficients
  _cob.set( base::Vector3d::Zero()) ;
  _linear_damp.set( base::Vector6d::Zero());
  _square_damp.set( base::Vector6d::Zero());
  
}

AuvMotion::AuvMotion(std::string const& name, RTT::ExecutionEngine* engine)
    : AuvMotionBase(name, engine)
{

  //Initialize the center of buoyancy and the damping-coefficients
  _cob.set( base::Vector3d::Zero()) ;
  _linear_damp.set( base::Vector6d::Zero());
  _square_damp.set( base::Vector6d::Zero());
  
}

AuvMotion::~AuvMotion()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See AuvMotion.hpp for more detailed
// documentation about them.

bool AuvMotion::configureHook()
{
    if (! AuvMotionBase::configureHook())
        return false;
    
    return true;
}
bool AuvMotion::startHook()
{
    if (! AuvMotionBase::startHook())
        return false;
    
    Actuators::startHook(); //Initialize superclass

    //Initialize Thruster coefficient matrix
    TCM = base::MatrixXd(6,amount_of_actuators);
    
    for (unsigned int i=0; i<amount_of_actuators; i++) {
      for(unsigned int j=0; j<3; j++){
	TCM(j, i) = thruster_direction[i][j];
      }
      
      //Calculate the angular thruster force, according to the position and direction
      TCM(3, i) = thruster_position[i].y() * thruster_direction[i].z() + 
		    (-thruster_position[i].z() * thruster_direction[i].y());
      
      TCM(4, i) = thruster_position[i].z() * thruster_direction[i].x() + 
		    (-thruster_position[i].x() * thruster_direction[i].z());
		    
      TCM(5, i) = (-thruster_position[i].y() * thruster_direction[i].x()) + 
		    thruster_position[i].x() * thruster_direction[i].y();
    }
    
    linearDamp = _linear_damp.get();
    squareDamp = _square_damp.get();
    
    if(_thruster_coefficients.get().size() == amount_of_actuators){
      thruster_coefficients = _thruster_coefficients.get();
    }else{
      std::cout << "Thruster coefficients not set." << std::endl;
      return false;
    }
      
    centerOfBuoyancy = _cob;
    voltage = _voltage;
    
    return true;
}
void AuvMotion::updateHook()
{
    AuvMotionBase::updateHook();
    
    Actuators::updateHook();
}
void AuvMotion::errorHook()
{
    AuvMotionBase::errorHook();
}
void AuvMotion::stopHook()
{
    AuvMotionBase::stopHook();
}
void AuvMotion::cleanupHook()
{
    AuvMotionBase::cleanupHook();
}


void AuvMotion::update(double time) {
        if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning
	
	base::Vector6d force = base::Vector6d::Zero();
	base::Vector6d vel;
	mars::utils::Quaternion ori = control->nodes->getRotation(vehicle_id);
	base::Vector3d pos = control->nodes->getPosition(vehicle_id);
	
	
	base::VectorXd thrust(amount_of_actuators);
	
	//Calculate thruster force
	for(int i = 0; i < amount_of_actuators; i++){
	  thrust[i] = std::fabs(thruster_force[i] * _voltage) * (thruster_force[i] * _voltage) * thruster_coefficients[i];
	}
	
	force += (TCM * thrust) ;
	
	//Get vehicle velocity, in the body frame
	vel.block<3,1>(0,0) = ori.conjugate() * control->nodes->getLinearVelocity(vehicle_id);
	vel.block<3,1>(3,0) = ori.conjugate() * control->nodes->getAngularVelocity(vehicle_id);
	
	base::Vector6d damp;
	
	//calculate damping force
	for(int i = 0; i < 6; i++){
	  damp[i] = (vel[i] * linearDamp[i]) + (vel[i] * std::fabs(vel[i]) * squareDamp[i]);
	}
	  
	force -= damp;
	
	//Convert force to the world frame
	force.block<3,1>(0,0) = ori * force.block<3,1>(0,0);
	force.block<3,1>(3,0) = ori * force.block<3,1>(3,0);
	
	//Add buoyancy forces
	force += gravity_buoyancy(ori);
	
	//Apply the forces
	control->nodes->applyForce(vehicle_id, force.block<3,1>(0,0), pos);
	control->nodes->applyTorque(vehicle_id, force.block<3,1>(3,0));	

}

base::Vector6d AuvMotion::gravity_buoyancy(const Eigen::Quaternion<double> q)
{
    base::Vector6d gravitybuoyancy;
  
    double mass;
    control->nodes->getNodeMass(vehicle_id ,&mass);
    double buoyancy = _buoyancy_force;
    base::Vector3d pos = control->nodes->getPosition(vehicle_id);
  
    // In Quaternion form
    float e1 = q.x();
    float e2 = q.y();
    float e3 = q.z();
    float e4 = q.w();
    float xg = 0.0;
    float yg = 0.0;
    float zg = 0.0;
    float xb = centerOfBuoyancy[0];
    float yb = centerOfBuoyancy[1];
    float zb = centerOfBuoyancy[2];;

    gravitybuoyancy(0) = 0.0;
    gravitybuoyancy(1) = 0.0;
    
    if(pos[2] + centerOfBuoyancy[2] < 0.0){ //The vehicle is completly under water
      gravitybuoyancy(2) = buoyancy - mass;
    }else if(pos[2] < 0.0 && centerOfBuoyancy[2] != 0){ //The vehicle is partwise underwater -> apply buoyancy partwise
      gravitybuoyancy(2) = (buoyancy * (-pos[2]/ centerOfBuoyancy[2] ) ) - mass;
    }else{ //The vehicle is over the surface -> no buoyancy
      gravitybuoyancy(2) = -mass;
    }
    
    //Angular buoancy forces
    gravitybuoyancy(3) = ((-(e4 * e4)+(e1 * e1)+(e2 * e2)-(e3 * e3))*((yg*mass)-(yb*buoyancy)))+(2*((e4*e1)+(e2*e3))*((zg*mass)-(zb*buoyancy)));
    gravitybuoyancy(4) =-((-(e4 * e4)+(e1 * e1)+(e2 * e2)-(e3 * e3))*((xg*mass)-(xb*buoyancy)))+(2*((e4*e2)-(e1*e3))*((zg*mass)-(zb*buoyancy)));
    gravitybuoyancy(5) =-(2*((e4*e1)+(e2*e3))*((xg*mass)-(xb*buoyancy)))-(2*((e4*e2)-(e1*e3))*((yg*mass)-(yb*buoyancy)));
    
    //Convert angular forces to world frame
    gravitybuoyancy.block<3,1>(3,0) = q * gravitybuoyancy.block<3,1>(3,0);  
    
    return gravitybuoyancy;
}



