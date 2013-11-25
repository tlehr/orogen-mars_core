/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ForceApplier.hpp"
#include <mars/interfaces/sim/NodeManagerInterface.h>

using namespace base::actuators;
using namespace simulation;
using namespace mars::utils;


ForceApplier::ForceApplier(std::string const& name)
    : ForceApplierBase(name){

    pthread_mutex_init(&node_update_mutex, NULL);
}


ForceApplier::ForceApplier(std::string const& name, RTT::ExecutionEngine* engine)
    : ForceApplierBase(name, engine){
    pthread_mutex_init(&node_update_mutex, NULL);
}

ForceApplier::~ForceApplier(){
	pthread_mutex_destroy(&node_update_mutex);
}


/// The following lines are template definitions for the various state machine
bool ForceApplier::startHook()
{
    if (! ForceApplierBase::startHook())
        return false;

    if(!Mars::getSimulatorInterface())
        throw std::runtime_error("Cannot start Actuators. The simulator is not running in the same process.");
	std::string node_name;

	node_name = _node_name.get();
	amount_of_actuators = _amount_of_actuators.get();
	maximum_thruster_force.resize(amount_of_actuators);
	thruster_position.resize(amount_of_actuators);
	thruster_direction.resize(amount_of_actuators);
	for (unsigned int i=0; i<amount_of_actuators; i++) {
		maximum_thruster_force[i] = _maximum_thruster_force.get()[i];

		base::Vector3d thruster_pos = _thruster_position.get()[i];
		thruster_position[i].x() = thruster_pos(0,0);
		thruster_position[i].y() = thruster_pos(1,0);
		thruster_position[i].z() = thruster_pos(2,0);

		base::Vector3d thruster_dir = _thruster_direction.get()[i];
		thruster_direction[i].x() = thruster_dir(0,0);
		thruster_direction[i].y() = thruster_dir(1,0);
		thruster_direction[i].z() = thruster_dir(2,0);
	}

	vehicle_id = control->nodes->getID(node_name);
	if( !vehicle_id )
		throw std::runtime_error("There is no node by the name of " + node_name + " in the scene");

	std::string groupName, dataName;
	control->nodes->getDataBrokerNames(vehicle_id, &groupName, &dataName);
	control->dataBroker->registerTimedReceiver(this, groupName, dataName, "mars_sim/simTimer", RATE,vehicle_id);
	RATE = 10;
	thruster_force.resize(amount_of_actuators);
	for (unsigned int i=0; i<amount_of_actuators; i++) {
		thruster_force[i] = 0.0;
	}

    return true;
}

void ForceApplier::updateHook()
{
    ForceApplierBase::updateHook();

    base::commands::Joints command;
    if (_command.readNewest(command) == RTT::NewData)
    {
        //we have to check it here otherwise the port
        //might be no longer triggeres
        if (!Mars::getSimulatorInterface()->isSimRunning())
            return;

        //check we have n actuator commands
        if(command.size() != amount_of_actuators) {
			char buffer[50];
			sprintf(buffer, "Simulation need a command (base::comands::Joints) with size of %d", amount_of_actuators);
            throw std::runtime_error(buffer);
		}

        //check that the drive mode is pwm
        for(unsigned i = 0; i < command.size() ; i++)
            if(command[i].isRaw())
                throw std::runtime_error("Simulation does only support Joints with RAW commands");

        pthread_mutex_lock(&node_update_mutex);
        for(unsigned int i=0;i<amount_of_actuators;i++)
            thruster_force[i] = command[i].raw;
        pthread_mutex_unlock(&node_update_mutex);

        /*// write actuator status
        base::actuators::Status status;
        status.time = base::Time::now();
        status.index = 0; //TODO: ?
        status.resize(amount_of_actuators);
        for (unsigned int i=0; i<amount_of_actuators; i++) {
        	status.states[i].pwm = -command.target[i];
        }
        */
        _status.write(command);
        
    }
}

void ForceApplier::errorHook()
{
    ForceApplierBase::errorHook();
}



void ForceApplier::update(double time) {
        if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning

	mars::utils::Quaternion vehicle_rot = control->nodes->getRotation(vehicle_id);
	mars::utils::Vector tmp1, tmp2;
	for(unsigned int i=0; i<amount_of_actuators;++i) {
		tmp1 = vehicle_rot * thruster_position[i];
		tmp1 += control->nodes->getPosition(vehicle_id);
		tmp2 = vehicle_rot *thruster_direction[i];
		tmp2 *= std::max(-maximum_thruster_force[i],std::min(maximum_thruster_force[i],thruster_force[i]*maximum_thruster_force[i]));
		control->nodes->applyForce(vehicle_id, tmp2, tmp1);
	}
}
