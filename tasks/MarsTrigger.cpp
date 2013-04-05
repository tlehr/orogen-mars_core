/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Mars.hpp"
#include <mars/interfaces/sim/SimulatorInterface.h>
#include "MarsTrigger.hpp"
#include <mars/interfaces/sim/ControlCenter.h>

using namespace simulation;

MarsTrigger::MarsTrigger(std::string const& name, TaskCore::TaskState initial_state)
    : MarsTriggerBase(name, initial_state)
{
    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&mutex_cv, NULL);
}

MarsTrigger::MarsTrigger(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : MarsTriggerBase(name, engine, initial_state)
{
    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&mutex_cv, NULL);
}

MarsTrigger::~MarsTrigger()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MarsTrigger.hpp for more detailed
// documentation about them.




bool MarsTrigger::configureHook()
{
    
    if (! RTT::TaskContext::configureHook())
        return false;
    return true;
    
}



bool MarsTrigger::startHook()
{
    
    if (! RTT::TaskContext::startHook())
        return false;
    

    //Check if mars is really deployed in the same context and started already
    assert(Mars::getSimulatorInterface());
    assert(Mars::getSimulatorInterface()->getControlCenter()->dataBroker->registerTriggeredReceiver(this,"mars_sim", "simTime","mars_sim/postPhysicsUpdate",1));
    return true;
    
}



void MarsTrigger::updateHook()
{
    if(!_do_step.get()) return; 
    RTT::TaskContext::updateHook();
    pthread_mutex_lock(&mutex);
    Mars::getSimulatorInterface()->singleStep();
    pthread_cond_wait(&mutex_cv, &mutex);
    pthread_mutex_unlock(&mutex);
}



void MarsTrigger::errorHook()
{
    
    RTT::TaskContext::errorHook();
}



void MarsTrigger::stopHook()
{
    
    RTT::TaskContext::stopHook();
}



void MarsTrigger::cleanupHook()
{
    
    RTT::TaskContext::cleanupHook();
}

void MarsTrigger::receiveData(
        const mars::data_broker::DataInfo& info,
        const mars::data_broker::DataPackage& package,
        int id) 
{
    pthread_mutex_lock(&mutex);
    pthread_cond_signal(&mutex_cv);
    pthread_mutex_unlock(&mutex);
}
