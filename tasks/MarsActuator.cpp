/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MarsActuator.hpp"
#include <mars/interfaces/sim/SimulatorInterface.h>
//#include <mars_sim/ControlCenter.h>
#include "MarsPlugin.hpp"
#include <mars/sim/SimMotor.h>
#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>

using namespace simulation;

MarsActuator::MarsActuator(std::string const& name)
    : MarsActuatorBase(name)
{
}

MarsActuator::MarsActuator(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsActuatorBase(name, engine)
{
}

MarsActuator::~MarsActuator()
{
}

namespace simulation {
struct ActuatorPlugin 
{
    MarsActuator *task;
    //actuator id in the task.
    int actuatorId;
    int marsMotorId;
    base::actuators::DRIVE_MODE mode;
    double value;
    bool gotCommand;
    std::string name;
   

    ActuatorPlugin(const std::string& name, MarsActuator *task, int actuatorId)
        : task(task), actuatorId(actuatorId), marsMotorId(0), gotCommand(false),name(name)
    {
    }

    bool init(){
        marsMotorId = task->control->motors->getID( name );
        if( !marsMotorId ){
            std::cerr << "There is no motor by the name of " + name + " in the scene";	
            return false;
        }
        return true;
    }

    void setCommand(base::actuators::DRIVE_MODE mode, double value)
    {
	this->mode = mode;
	this->value = value;
	gotCommand = true;
    }
    
    void update( base::Time time )
    {
        mars::sim::SimMotor *motor = task->control->motors->getSimMotor(marsMotorId);

	if(gotCommand)
	{
	    switch(mode)
	    {
		case base::actuators::DM_PWM:
		    throw std::runtime_error("No clue how to implementent PWM in mars");
		    break;
		case base::actuators::DM_SPEED:
			motor->setType(mars::interfaces::MOTOR_TYPE_DC);
		    motor->setVelocity(value);
		    break;
		case base::actuators::DM_POSITION:
			motor->setType(mars::interfaces::MOTOR_TYPE_PID);
		    motor->setValue( value );
		    break;
		default:
		    throw std::runtime_error("Given controller mode is not mapped to firmware controller");
		    break;
	    }
	}
	
	base::actuators::MotorState ms;
	ms.current =  motor->getTorque();
	ms.position = motor->getValue();
	ms.positionExtern = motor->getActualPosition();
	ms.pwm = motor->getVelocity();
	task->setNewActuatorStatus(actuatorId, time, ms);
    }
};
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MarsActuator.hpp for more detailed
// documentation about them.

void MarsActuator::setCommand(int32_t actuatorId, base::actuators::DRIVE_MODE mode, double value)
{
    ActuatorPlugin *plugin = plugins[actuatorId];
    plugin->setCommand(mode, value);
}


void MarsActuator::statusDispatchAdded(int dispatchId, std::vector< int > actuatorIds)
{

}


bool MarsActuator::configureHook()
{
    
    if (!::simulation::MarsPlugin::configureHook())
        return false;
    return true;
    
}



bool MarsActuator::startHook()
{
    std::cout << "ACTUATOR START HOOK" << std::endl;
    if (! ::simulation::MarsPlugin::startHook())
        return false;
    
    std::vector<std::string> motorNames = _names.get();    
    
    for(std::vector<OrocosCommandDispatcher *>::const_iterator it = cmdDispatches.begin();
	it != cmdDispatches.end();it++)
    {
	const std::vector<int32_t> ids = (*it)->getActuatorIds();
	for(std::vector<int32_t>::const_iterator id = ids.begin(); id != ids.end(); id++)
	{
	    if(motorNames.size() < *id)
	    {
		std::cout << "Can not dispatch nonexisting actuator " << *id << std::endl;
		return false;
	    }

	    ActuatorPlugin *plugin;
	    plugin = new ActuatorPlugin(motorNames[*id], this, *id);
	    if(!plugin->init()){	
		//TODO CLEANUP MEMORY
		return false;
	    }
	    
	    plugins.push_back(plugin);
	}
    }
    std::cout << "ACTUATOR START HOOK OK" << std::endl;
    
    return true;
    
}




//    processDispatched();

    

void MarsActuator::update(double delta_t){
        if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning
	for(int i=0;i<plugins.size();i++){
            plugins[i]->update(getTime());
        }
}

void MarsActuator::updateHook()
{
    simulation::MarsPlugin::updateHook();
    processDispatched();
}

void MarsActuator::errorHook()
{
    simulation::MarsPlugin::errorHook();
}

void MarsActuator::stopHook()
{
    simulation::MarsPlugin::stopHook();
}

void MarsActuator::cleanupHook()
{
    simulation::MarsPlugin::cleanupHook();
}

bool MarsActuator::dispatch(::std::string const & name, ::std::vector< boost::int32_t > const & actuatorMap){
    return Actuator::dispatch(name,actuatorMap,this);    
}
