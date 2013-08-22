/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AuvController.hpp"
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>

using namespace simulation;

AuvController::AuvController(std::string const& name)
    : AuvControllerBase(name)
{
    auv_id = 0;
}

AuvController::AuvController(std::string const& name, RTT::ExecutionEngine* engine)
    : AuvControllerBase(name, engine)
{
    auv_id = 0;
}

AuvController::~AuvController()
{
}

bool AuvController::setOrientation(::base::Quaterniond const & value)
{
    orientation_updated=true;	
    return(simulation::AuvControllerBase::setOrientation(value));
}

bool AuvController::setPosition(::base::Vector3d const & value)
{
    position_updated=true;	
    return(simulation::AuvControllerBase::setPosition(value));
}

bool AuvController::configureHook()
{
    if (! AuvControllerBase::configureHook())
        return false;
    auv_id = control->nodes->getID(_node_name.get());
    if(auv_id<1){
        fprintf(stderr,"Could not get AUV in the scene, the node \"%s\" could not be found\n",_node_name.get().c_str());
        return false;
    }
    return true;
}
bool AuvController::startHook()
{
    if (! AuvControllerBase::startHook())
        return false;
    return true;
}


void AuvController::updateHook()
{
    AuvControllerBase::updateHook();
}
void AuvController::errorHook()
{
    AuvControllerBase::errorHook();
}
void AuvController::stopHook()
{
    AuvControllerBase::stopHook();
}
void AuvController::cleanupHook()
{
    AuvControllerBase::cleanupHook();
}

void AuvController::update(double time_ms){
        if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning
        
        //set new position if requested
        if(position_updated)
        {
            control->nodes->setPosition(auv_id,_position.get());
            position_updated = false;
        }
        if(orientation_updated)
        {
            control->nodes->setRotation(auv_id,_orientation.get());
            orientation_updated = false;
        }

        //Buoyancy for AUV
        mars::utils::Vector auv_pos = control->nodes->getPosition(auv_id);

        //TODO will be as smaller as more the vehicle comes out of the water factor
        double weight;
        double mass;
        control->nodes->getNodeMass(auv_id,&mass);
        
        if(auv_pos[2] > 0){ //AUV is above the water
            //The AUV has to be dragged down below the surface, so apply the whole vehicle force down...
            weight = -mass;
        }else if(auv_pos[2] + _cob.get()[2] > 0){ //AUV is partwise below the water surface so apply only a pice
            weight = (mass * auv_pos[2] + _cob.get()[2])/_cob.get()[2];
        }else{ //AUV is below the surface
            weight = _buoyancy_force.get();
        }
        mars::utils::Vector buoyancy_vector = weight * Eigen::Vector3d(0,0,1);
        control->nodes->applyForce(auv_id, buoyancy_vector);
       
        mars::utils::Vector weight_vector = mass * Eigen::Vector3d(0,0,-1); 

        Eigen::Vector3d cob = _cob.get();
        mars::utils::Vector torque = buoyancy_vector * cob.squaredNorm();
        control->nodes->applyTorque(auv_id, torque);
       

        
        //TODO Apply torque to the AUV to let the vehicle stand up
        /*
        mars::utils::Quaternion auv_rot = control->nodes->getRotation(auv_id);
        sRotation rotation = quaternionTosRotation(auv_rot);
        tmp[0] = -sin(rotation.alpha/180*M_PI)*max_thruster_force;
        tmp[1] = -sin(rotation.beta/180*M_PI)*max_thruster_force;
        tmp[2] = 0;
        control->nodes->applyTorque(auv_id, auv_rot *tmp);
        */
}

