/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Joints.hpp"
#include <boost/foreach.hpp>
#include <mars/sim/SimMotor.h>
#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <base/Logging.hpp>

using namespace simulation;

Joints::Joints(std::string const& name)
    : JointsBase(name)
{
}

Joints::Joints(std::string const& name, RTT::ExecutionEngine* engine)
    : JointsBase(name, engine)
{
}

Joints::~Joints()
{
}

void Joints::init()
{
    // for each of the names, get the mars motor id
    BOOST_FOREACH( std::string name, mars_ids.names )
    {
        int marsMotorId = control->motors->getID( name );
        if( marsMotorId )
	    mars_ids.elements.push_back( marsMotorId );
	else
	    throw std::runtime_error("there is no motor by the name of " + name);
    }

    assert( mars_ids.names.size() == mars_ids.elements.size() );
}

void Joints::update(double delta_t)
{
    // if there was a command, write it to the simulation
    if( _cmd.read( cmd ) == RTT::NewData )
    {
	for( size_t i=0; i<cmd.size(); ++i )
	{
	    // for each command input look up the name in the mars_ids structure
	    mars::sim::SimMotor *motor = 
		control->motors->getSimMotor( mars_ids[cmd.names[i]] );
	    if( cmd[i].hasPosition() )
		motor->setValue( cmd[i].position );
	    if( cmd[i].hasSpeed() )
		motor->setVelocity( cmd[i].speed );
	    if( cmd[i].hasEffort() )
	    {
		LOG_WARN_S << "Effort command ignored";
	    }
	    if( cmd[i].hasRaw() )
	    {
		LOG_WARN_S << "Raw command ignored";
	    }
	}
    }

    // in any case read out the status
    for( size_t i=0; i<mars_ids.size(); ++i )
    {
	mars::sim::SimMotor *motor = control->motors->getSimMotor( mars_ids[i] );

	base::JointState state;
	state.position = motor->getValue();
	state.speed = motor->getVelocity();
	state.effort = motor->getTorque();
	status[i] = state;
    }

    // and write it to the output port
    status.time = getTime();
    _status.write( status );
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Joints.hpp for more detailed
// documentation about them.

bool Joints::configureHook()
{
    if (! JointsBase::configureHook())
        return false;

    // copy the joint names 
    mars_ids.names = _names.value();

    // and resize the input/output structures
    cmd.resize( mars_ids.names.size() );
    status.resize( mars_ids.names.size() );
    status.names = _names.value();

    return true;
}
bool Joints::startHook()
{
    if (! JointsBase::startHook())
        return false;
    return true;
}
void Joints::updateHook()
{
    JointsBase::updateHook();
}
void Joints::errorHook()
{
    JointsBase::errorHook();
}
void Joints::stopHook()
{
    JointsBase::stopHook();
}
void Joints::cleanupHook()
{
    JointsBase::cleanupHook();
}
