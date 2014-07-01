/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ForceTorque6DOF.hpp"

#include <mars/interfaces/sim/SensorManagerInterface.h>
#include <mars/sim/Joint6DOFSensor.h>

using namespace simulation;

ForceTorque6DOF::ForceTorque6DOF(std::string const& name)
    : ForceTorque6DOFBase(name)
{
}

ForceTorque6DOF::ForceTorque6DOF(std::string const& name, RTT::ExecutionEngine* engine)
    : ForceTorque6DOFBase(name, engine)
{
}

ForceTorque6DOF::~ForceTorque6DOF()
{
}


void ForceTorque6DOF::init()
{
// for each of the names, get the mars motor id
    for( size_t i=0; i<mars_ids.size(); ++i )
    {
	std::string &name( mars_names[i] );
        int marsId = control->sensors->getSensorID( name );

        if( marsId )
	    mars_ids[i] = marsId;
	else
	    throw std::runtime_error("there is no sensor by the name of " + name);
    }
}

void ForceTorque6DOF::update(double delta_t)
{
    if(!isRunning()) return;

	std::vector< base::Vector3d > force;
	std::vector< base::Vector3d > torque;
	force.resize(mars_ids.size());
	torque.resize(mars_ids.size());

    for( size_t i=0; i<mars_ids.size(); ++i ){
    	mars::interfaces::BaseSensor* base = control->sensors->getSimSensor(mars_ids[i]);
    	if (base){
    		mars::sim::Joint6DOFSensor* sensor = dynamic_cast<mars::sim::Joint6DOFSensor*>(base);
			if (sensor){
				//printf("sensor %s found\n",mars_names[i].c_str());
				mars::utils::Vector marsforce,marstorque;
				sensor->getForceData(&marsforce);
				sensor->getTorqueData(&marstorque);

				//mars::utils::Vector is the same typedef as base::Vector3d
				//so we can assign directly
				force[i] = marsforce;
				torque[i] = marstorque;
			}
    	}
    }

    //TODO: convert to new wrench base type, when available
    _force.write(force);
    _torque.write(torque);

}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ForceTorque6DOF.hpp for more detailed
// documentation about them.

bool ForceTorque6DOF::configureHook()
{
    if (! ForceTorque6DOFBase::configureHook())
        return false;

    size_t num_sensors = _names.value().size();

    mars_ids.clear();
    mars_ids.resize(num_sensors);
    mars_names.clear();
    mars_names = _names.value();

    return true;
}
bool ForceTorque6DOF::startHook()
{
    if (! ForceTorque6DOFBase::startHook())
        return false;
    return true;
}
void ForceTorque6DOF::updateHook()
{
    ForceTorque6DOFBase::updateHook();
}
void ForceTorque6DOF::errorHook()
{
    ForceTorque6DOFBase::errorHook();
}
void ForceTorque6DOF::stopHook()
{
    ForceTorque6DOFBase::stopHook();
}
void ForceTorque6DOF::cleanupHook()
{
    ForceTorque6DOFBase::cleanupHook();
}
