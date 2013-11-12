/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Joints.hpp"
#include <boost/foreach.hpp>
#include <mars/sim/SimMotor.h>
#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <base/Logging.hpp>
#include <base/samples/RigidBodyState.hpp>

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
    for( size_t i=0; i<mars_ids.size(); ++i )
    {
	std::string &name( mars_ids[i].marsName );
        int marsMotorId = control->motors->getID( name );
        if( marsMotorId )
	    mars_ids[i].mars_id = marsMotorId;
	else
	    throw std::runtime_error("there is no motor by the name of " + name);
    }
}

void Joints::update(double delta_t)
{
    // if there was a command, write it to the simulation
    if( _command.read( cmd ) == RTT::NewData )
    {
	for( size_t i=0; i<mars_ids.size(); ++i )
	{
            
	    // for each command input look up the name in the mars_ids structure
	    JointConversion conv = mars_ids[i];
            
            //ignore the case that the input data stream has not commands for our other joints
            std::vector<std::string>::const_iterator it = std::find(cmd.names.begin(), cmd.names.end(), conv.externalName);
            if (it == cmd.names.end()){
                continue;
            }
            
            base::JointState &curCmd(cmd[*it]);

	    mars::sim::SimMotor *motor = 
		control->motors->getSimMotor( conv.mars_id );
                
	    if( curCmd.hasPosition() )
            {
                //set maximum speed that is allowed for turning
                if(curCmd.hasSpeed())
                    motor->setMaximumVelocity(conv.toMars(curCmd.speed) );

                motor->setValue( conv.toMars( curCmd.position ) );
            }
            else
            {
                if( curCmd.hasSpeed() )
                    motor->setVelocity( conv.toMars( curCmd.speed ) );
            }
	    if( curCmd.hasEffort() )
	    {
		LOG_WARN_S << "Effort command ignored";
	    }
	    if( curCmd.hasRaw() )
	    {
		LOG_WARN_S << "Raw command ignored";
	    }
	}
    }

    // in any case read out the status
    for( size_t i=0; i<status.size(); ++i )
    {
	JointConversion conv = mars_ids[i];
	mars::sim::SimMotor *motor = control->motors->getSimMotor( conv.mars_id );

	base::JointState state;
	state.position = conv.fromMars( motor->getActualPosition() );
	state.speed = conv.fromMars( motor->getJoint()->getVelocity() );
	state.effort = conv.fromMars( motor->getTorque() );
	status[i] = state;
    }

    // and write it to the output port
    status.time = getTime();
    _status_samples.write( status );
    
    // see if we have configuration for the joint_transforms 
    // and the output port for it is connected
    std::vector<base::samples::RigidBodyState> rbs;
    if( !_joint_transform.value().empty() && _transforms.connected() )
    {
        _joint_transform.value().setRigidBodyStates( status, rbs );
        for( size_t i=0; i < rbs.size(); ++i )
            _transforms.write( rbs[i] );
    }
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
    cmd.resize( num_joints);
    status.resize( num_joints );
    std::vector<std::string> marsNames = _names.value();
    status.names = _names.value();
    std::vector<std::string> rename = _name_remap.get(); 
    for( size_t i=0; i<num_joints; i++ )
    {
	if( !_scaling.value().empty() )
	    mars_ids[i].scaling = _scaling.value()[i];
	if( !_offset.value().empty() )
	    mars_ids[i].offset = _offset.value()[i];
        
        mars_ids[i].marsName = marsNames[i];
        
        if(rename.empty() || rename[i].empty())
        {
            mars_ids[i].externalName = marsNames[i];
            
        }
        else
        {
            status.names[i] = rename[i];
            mars_ids[i].externalName = rename[i];            
        }
    }

    
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
