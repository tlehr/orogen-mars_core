/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MarsServo.hpp"
#include <mars/interfaces/sim/SimulatorInterface.h>
//#include <mars_sim/ControlCenter.h>
#include "MarsPlugin.hpp"
#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>

using namespace simulation;


MarsServo::MarsServo(std::string const& name)
    : MarsServoBase(name), motor_id(0), motor(NULL), upper2lower(), 
            last_angle(0.0), target_angle(0.0)
{
}

MarsServo::MarsServo(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsServoBase(name, engine), motor_id(0), motor(NULL), 
            upper2lower(), last_angle(0.0), target_angle(0.0)
{
}

MarsServo::~MarsServo()
{
}

bool MarsServo::set_angle( double angle )
{
    target_angle = angle;
    return true;
}

double MarsServo::get_angle()
{
    if(motor != NULL)
        return motor->getActualPosition();
    else
        return 0.0;
}

bool MarsServo::startHook()
{
    motor_id = control->motors->getID( _name.value() );
    if( !motor_id ){
        std::vector<mars::interfaces::core_objects_exchange> motorList;
        control->motors->getListMotors(&motorList);
        std::cerr << "Availible Motors:" << std::endl;
        for(unsigned int i=0;i<motorList.size();i++){
            std::cerr << motorList[i].name << std::endl;
        }
        std::cerr <<  "There is no motor by the name of " + _name.value() + " in the scene" << std::endl;
        return false;
    }
    
    motor = control->motors->getSimMotor(motor_id);

    if (! MarsServoBase::startHook())
        return false;
    
    // Why did you add this Matthias!?
    //_rotation_axis.value() = base::Vector3d::UnitX();
    upper2lower.initSane();
    upper2lower.position.setZero();
    upper2lower.sourceFrame = _upper_frame.value();
    upper2lower.targetFrame = _lower_frame.value();
    
    last_angle = 0;
    target_angle = 0;
    motor->setMaximumVelocity( _moving_speed.value() );
    
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
	    
	    // Sets a target if not already set.
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
    updateHook();
    
    motor->setValue( target_angle );
}

