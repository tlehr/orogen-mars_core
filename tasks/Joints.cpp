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
    for( size_t i=0; i<mars_ids.names.size(); ++i )
    {
	std::string &name( mars_ids.names[i] );
        int marsMotorId = control->motors->getID( name );
        if( marsMotorId )
	    mars_ids.elements[i].mars_id = marsMotorId;
	else
	    throw std::runtime_error("there is no motor by the name of " + name);
    }
}

void Joints::update(double delta_t)
{
    // if there was a command, write it to the simulation
    if( _cmd.read( cmd ) == RTT::NewData )
    {
	for( size_t i=0; i<cmd.size(); ++i )
	{
	    // for each command input look up the name in the mars_ids structure
	    JointConversion conv = mars_ids[cmd.names[i]];
	    mars::sim::SimMotor *motor = 
		control->motors->getSimMotor( conv.mars_id );
	    if( cmd[i].hasPosition() )
		motor->setValue( conv.toMars( cmd[i].position ) );
	    if( cmd[i].hasSpeed() )
		motor->setVelocity( conv.toMars( cmd[i].speed ) );
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
    for( size_t i=0; i<status.size(); ++i )
    {
	JointConversion conv = mars_ids[status.names[i]];
	mars::sim::SimMotor *motor = control->motors->getSimMotor( conv.mars_id );

	base::JointState state;
	state.position = conv.fromMars( motor->getValue() );
	state.speed = conv.fromMars( motor->getVelocity() );
	state.effort = conv.fromMars( motor->getTorque() );
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

    size_t num_joints = _names.value().size();

    // test if scaling is valid
    if( !_scaling.value().empty() && _scaling.value().size() != num_joints )
    {
	LOG_ERROR_S << "The scaling property needs to be empty or of the same size as names.";
	return false;
    }
    if( !_offset.value().empty() && _offset.value().size() != num_joints )
    {
	LOG_ERROR_S << "The offset property needs to be empty or of the same size as names.";
	return false;
    }

    // fill the joint structure 
    mars_ids.resize( num_joints );
    mars_ids.names = _names.value();
    for( size_t i=0; i<num_joints; i++ )
    {
	if( !_scaling.value().empty() )
	    mars_ids.elements[i].scaling = _scaling.value()[i];
	if( !_offset.value().empty() )
	    mars_ids.elements[i].offset = _offset.value()[i];
    }

    // and resize the input/output structures
    cmd.resize( num_joints);
    status.resize( num_joints );
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
