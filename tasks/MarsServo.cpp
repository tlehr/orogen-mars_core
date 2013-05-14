/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MarsServo.hpp"
#include <mars/interfaces/sim/SimulatorInterface.h>
//#include <mars_sim/ControlCenter.h>
#include "MarsPlugin.hpp"
#include <mars/sim/SimMotor.h>
#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>

using namespace simulation;


MarsServo::MarsServo(std::string const& name)
    : MarsServoBase(name)
{
}

MarsServo::MarsServo(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsServoBase(name, engine)
{
}

MarsServo::~MarsServo()
{
}

bool MarsServo::set_angle( double angle )
{
    target_pos = angle;
    return true;
}

double MarsServo::get_angle()
{
    return motor_pos;
}

bool MarsServo::startHook()
{
    motor_pos = 0;
    target_pos = 0;
    desired_velocity = 0;
    motor_id = control->motors->getID( _name.value() );
    if( !motor_id ){
        std::vector<mars::interfaces::core_objects_exchange> motorList;
        control->motors->getListMotors(&motorList);
        std::cerr << "Availible Motors:" << std::endl;
        for(int i=0;i<motorList.size();i++){
            std::cerr << motorList[i].name << std::endl;
        }
        std::cerr <<  "There is no motor by the name of " + _name.value() + " in the scene" << std::endl;
        return false;
    }

    if (! MarsServoBase::startHook())
        return false;
    
    _rotation_axis.value() = base::Vector3d::UnitX();
    upper2lower.initSane();
    upper2lower.position.setZero();
    upper2lower.sourceFrame = _upper_frame.value();
    upper2lower.targetFrame = _lower_frame.value();
    
    return true;
}
void MarsServo::updateHook()
{
    //TODO CLEANUP
    double present_angle = get_angle();
    _angle.write( present_angle );
    
    //also report as transformation
    upper2lower.time = getTime();
    upper2lower.orientation = Eigen::AngleAxisd( -present_angle, _rotation_axis.value() );
    _upper2lower.write(upper2lower);

    switch(_mode.value())
    {
	case servo::POSITION:
	    //return if no angle was set and in mode 0
	    if(_cmd_angle.readNewest( target_angle ) == RTT::NoData)
	    {
		return;
	    }
	    set_angle(target_angle);

	    break;
	case servo::SWEEP:
	    if(fabs(last_angle - present_angle) < .2/180*M_PI)
	    {
		if(fabs(present_angle - _upper_sweep_angle.value()) < 4.0/180*M_PI 
		   && target_angle == _upper_sweep_angle.value())
		{
		    target_angle = _lower_sweep_angle.value();
		    set_angle(target_angle);
		}
   
		if(fabs(present_angle - _lower_sweep_angle.value()) < 4.0/180*M_PI 
		   && target_angle == _lower_sweep_angle.value())
		{
		    target_angle = _upper_sweep_angle.value();
		    set_angle(target_angle);
		}
	    }
	    
	    if(target_angle != _lower_sweep_angle.value() &&
	       target_angle != _upper_sweep_angle.value())
	    {
		target_angle = _upper_sweep_angle.value();
		set_angle(target_angle);
	    }
	    //note sending target angle over and over results in 
	    //non fluent movement of the servo in speed mode
	    break;
    }
    last_angle = present_angle;
    //ENDE CLEANUP

    desired_velocity = _moving_speed.value();
    MarsServoBase::updateHook();
}
// void MarsServo::errorHook()
// {
//     MarsServoBase::errorHook();
// }
void MarsServo::stopHook()
{
    MarsServoBase::stopHook();
}
// void MarsServo::cleanupHook()
// {
//     MarsServoBase::cleanupHook();
// }
    
void MarsServo::update( double time )
{
    mars::sim::SimMotor *theMotor = control->motors->getSimMotor(motor_id);

//        theMotor->setMaximumVelocity( desired_velocity );
    theMotor->setValue( target_pos );
    // todo: is it desired to get the target_pos back from the motor?
    motor_pos = theMotor->getActualPosition( );
}

