/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MarsServo.hpp"
#include <mars/interfaces/sim/SimulatorInterface.h>
//#include <mars_sim/ControlCenter.h>
#include "MarsPlugin.hpp"
#include <mars/sim/SimMotor.h>
#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>

using namespace simulation;

namespace simulation {
struct ServoPlugin : public MarsPlugin
{
    int motor_id;
    double motor_pos;
    double target_pos;
    double desired_velocity;

    ServoPlugin()
        : motor_id(0),
        motor_pos(0.0),
        target_pos(0.0),
        desired_velocity(0.1)
    {}

    void update( double time )
    {
        mars::sim::SimMotor *theMotor = control->motors->getSimMotor(motor_id);

//        theMotor->setMaximumVelocity( desired_velocity );
        theMotor->setValue( target_pos );
        // todo: is it desired to get the target_pos back from the motor?
        motor_pos = theMotor->getActualPosition( );
    }

    void setMotorName( const std::string& name )
    {
        motor_id = control->motors->getID( name );
        if( !motor_id )
            throw std::runtime_error("There is no motor by the name of " + name + " in the scene");
        
//        mars::sim::SimMotor *theMotor = control->motors->getSimMotor(motor_id);
//        desired_velocity = theMotor->getMaximumVelocity();
    }
};
}

MarsServo::MarsServo(std::string const& name)
    : MarsServoBase(name),
      plugin(0)
{
}

MarsServo::MarsServo(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsServoBase(name, engine),
    plugin(0)
{
}

MarsServo::~MarsServo()
{
}

base::Time MarsServo::getTime()
{
    return plugin->getTime();
}

bool MarsServo::set_angle( double angle )
{
    plugin->target_pos = angle;
    return true;
}

double MarsServo::get_angle()
{
    return plugin->motor_pos;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MarsServo.hpp for more detailed
// documentation about them.

// bool MarsServo::configureHook()
// {
//     if (! MarsServoBase::configureHook())
//         return false;
//     return true;
// }
bool MarsServo::startHook()
{
    plugin = new ServoPlugin();
    plugin->setMotorName( _name.value() );

    if (! MarsServoBase::startHook())
        return false;
    return true;
}
void MarsServo::updateHook()
{
    plugin->desired_velocity = _moving_speed.value();
    MarsServoBase::updateHook();
}
// void MarsServo::errorHook()
// {
//     MarsServoBase::errorHook();
// }
void MarsServo::stopHook()
{
    delete plugin;
    MarsServoBase::stopHook();
}
// void MarsServo::cleanupHook()
// {
//     MarsServoBase::cleanupHook();
// }
