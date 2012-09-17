/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MarsIMU.hpp"
#include "MarsPlugin.hpp"
#include <interfaces/MotorManagerInterface.h>
#include <interfaces/NodeManagerInterface.h>

using namespace simulation;

namespace simulation {
struct IMUPlugin : public MarsPlugin
{
    MarsIMU &task;
    long node_id;
    base::samples::RigidBodyState rbs;

    IMUPlugin( MarsIMU& task, const std::string& name )
	: task( task )
    {
	node_id = control->nodes->getID( name );
	if( !node_id )
	    throw std::runtime_error("There is no node by the name of " + name + " in the scene");

	rbs.initSane();
	rbs.position.setZero();
    }

    void update( double time )
    {
	rbs.time = getTime();
	rbs.sourceFrame = task._imu_frame.value();
	rbs.targetFrame = task._world_frame.value();
	rbs.orientation = control->nodes->getRotation( node_id ).normalized();
	rbs.cov_orientation = base::Matrix3d::Ones() * 1e-6;

	task._orientation_samples.write( rbs );
    }
};
}

MarsIMU::MarsIMU(std::string const& name)
    : MarsIMUBase(name), plugin(0)
{
}

MarsIMU::MarsIMU(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsIMUBase(name, engine), plugin(0)
{
}

MarsIMU::~MarsIMU()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MarsIMU.hpp for more detailed
// documentation about them.

// bool MarsIMU::configureHook()
// {
//     if (! MarsIMUBase::configureHook())
//         return false;
//     return true;
// }
bool MarsIMU::startHook()
{
    if (! MarsIMUBase::startHook())
        return false;

    plugin = new IMUPlugin( *this, _name.value() );

    return true;
}
void MarsIMU::updateHook()
{
    MarsIMUBase::updateHook();
}
// void MarsIMU::errorHook()
// {
//     MarsIMUBase::errorHook();
// }
void MarsIMU::stopHook()
{
    delete plugin;

    MarsIMUBase::stopHook();
}
// void MarsIMU::cleanupHook()
// {
//     MarsIMUBase::cleanupHook();
// }

