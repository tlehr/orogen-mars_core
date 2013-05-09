/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Actuators.hpp"
#include <mars/interfaces/sim/NodeManagerInterface.h>

using namespace base::actuators;
using namespace simulation;
using namespace mars::utils;



namespace simulation {
struct ThrusterPlugin : public MarsPlugin {

	unsigned long vehicle_id;
	const unsigned int amountOfActuators;
	std::vector <double> max_thruster_force;
	std::vector <mars::utils::Vector> thruster_pos;
	std::vector <mars::utils::Vector> thruster_dir;
	std::vector <mars::interfaces::sReal> thruster_force;
	pthread_mutex_t* node_update_mutex;

	unsigned int RATE;

	ThrusterPlugin(std::string node_name, unsigned int amountOfActuators,
					std::vector <double> max_thruster_force,
					std::vector <mars::utils::Vector> thruster_pos,
					std::vector <mars::utils::Vector> thruster_dir)
		: amountOfActuators(amountOfActuators), max_thruster_force(max_thruster_force), thruster_pos(thruster_pos), thruster_dir(thruster_dir), node_update_mutex(new pthread_mutex_t)
	{
		vehicle_id = control->nodes->getID(node_name);
		if( !vehicle_id )
			throw std::runtime_error("There is no node by the name of " + node_name + " in the scene");

		std::string groupName, dataName;
		control->nodes->getDataBrokerNames(vehicle_id, &groupName, &dataName);
		control->dataBroker->registerTimedReceiver(this, groupName, dataName, "mars_sim/simTimer", RATE,vehicle_id);
		RATE = 10;
		thruster_force.resize(amountOfActuators);
		for (unsigned int i=0; i<amountOfActuators; i++) {
			thruster_force[i] = 0.0;
		}
	}

	bool getPose(Eigen::Vector3d &pos, Eigen::Quaterniond &orientation){
		pthread_mutex_lock(node_update_mutex);
		mars::utils::Vector vehicle_pos = control->nodes->getPosition(vehicle_id);
		mars::utils::Quaternion vehicle_rot = control->nodes->getRotation(vehicle_id);
		pos = Eigen::Vector3d(vehicle_pos.x(),vehicle_pos.y(),vehicle_pos.z());
		orientation = Eigen::Quaterniond(vehicle_rot.w(),vehicle_rot.x(),vehicle_rot.y(),vehicle_rot.z());
		pthread_mutex_unlock(node_update_mutex);
		return true;
	}

	bool getVelocities(Eigen::Vector3d &lin_vel, Eigen::Vector3d &ang_vel){
		pthread_mutex_lock(node_update_mutex);
		mars::utils::Vector vehicle_lin_vel = control->nodes->getLinearVelocity(vehicle_id);
		mars::utils::Vector vehicle_ang_vel = control->nodes->getAngularVelocity(vehicle_id);
		lin_vel = Eigen::Vector3d(vehicle_lin_vel.x(),vehicle_lin_vel.y(),vehicle_lin_vel.z());
		ang_vel = Eigen::Vector3d(vehicle_ang_vel.x(),vehicle_ang_vel.y(),vehicle_ang_vel.z());
		pthread_mutex_unlock(node_update_mutex);
		return true;
	}

	void setTarget(const std::vector<double> &target){
		if(target.size() != amountOfActuators) {
			char buffer[50];
			sprintf(buffer, "Object has %d motors!!!", amountOfActuators);
			throw std::runtime_error(buffer);
		}

		pthread_mutex_lock(node_update_mutex);
		for(unsigned int i=0;i<amountOfActuators;i++)
			thruster_force[i] = target[i];
		pthread_mutex_unlock(node_update_mutex);
	}

	void update(double time) {
		mars::utils::Quaternion vehicle_rot = control->nodes->getRotation(vehicle_id);
		mars::utils::Vector tmp1, tmp2;
		for(unsigned int i=0; i<amountOfActuators;++i) {
			tmp1 = vehicle_rot * thruster_pos[i];
			tmp1 += control->nodes->getPosition(vehicle_id);
			tmp2 = vehicle_rot *thruster_dir[i];
			tmp2 *= std::max(-max_thruster_force[i],std::min(max_thruster_force[i],thruster_force[i]*max_thruster_force[i]));
			control->nodes->applyForce(vehicle_id, tmp2, tmp1);
		}
	}
};
}



Actuators::Actuators(std::string const& name)
    : ActuatorsBase(name){}


Actuators::Actuators(std::string const& name, RTT::ExecutionEngine* engine)
    : ActuatorsBase(name, engine){}

Actuators::~Actuators(){}


/// The following lines are template definitions for the various state machine
bool Actuators::startHook()
{
    if (! ActuatorsBase::startHook())
        return false;

    if(!Mars::getSimulatorInterface())
        throw std::runtime_error("Cannot start Actuators. The simulator is not running in the same process.");

	std::string node_name;
	std::vector <double> maximum_thruster_force;
	std::vector <mars::utils::Vector> thruster_position;
	std::vector <mars::utils::Vector> thruster_direction;

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

	thruster_plugin = new simulation::ThrusterPlugin(node_name, amount_of_actuators, maximum_thruster_force, thruster_position, thruster_direction);

    return true;
}

void Actuators::updateHook()
{
    ActuatorsBase::updateHook();

    base::actuators::Command command;
    if (_command.readNewest(command) == RTT::NewData)
    {
        //we have to check it here otherwise the port
        //might be no longer triggeres
        if (!Mars::getSimulatorInterface()->isSimRunning())
            return;

        //check we have n actuator commands
        if(command.mode.size() != amount_of_actuators ||command.target.size() != amount_of_actuators) {
			char buffer[50];
			sprintf(buffer, "Simulation need a target and mode size of %d", amount_of_actuators);
            throw std::runtime_error(buffer);
		}

        //check that the drive mode is pwm
        std::vector<DRIVE_MODE>::iterator iter = command.mode.begin();
        for(;iter != command.mode.end();++iter)
            if(*iter != DM_PWM)
                throw std::runtime_error("Simulation does only support DM_PWM");

        //TODO
        //for now we are copping it because we do not want to change
        //the mapping for the simulation model
        //
        std::vector<double> pwm;
        for (unsigned int i=0; i<command.target.size(); i++) {
			pwm.push_back(command.target[i]);
        }

        thruster_plugin->setTarget(pwm);


        // write actuator status
        base::actuators::Status status;
        status.time = base::Time::now();
        status.index = 0; //TODO: ?
        status.resize(amount_of_actuators);
        for (unsigned int i=0; i<amount_of_actuators; i++) {
        	status.states[i].pwm = -command.target[i];
        }

        _status.write(status);
    }
}

void Actuators::errorHook()
{
    ActuatorsBase::errorHook();
}
