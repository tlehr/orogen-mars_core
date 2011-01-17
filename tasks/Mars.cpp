#include "Mars.hpp"
#include <mars/ControlCenter.h>

#include <mars/multisim-plugin/MultiSimPlugin.h>

using namespace simulation;


Mars::Mars(std::string const& name)
    : MarsBase(name), initDone(false)
{
    int ret = pthread_create(&thread_info, NULL, startMarsFunc, this); 
    if(ret)
	throw std::runtime_error("Failed to create MARS thread");

}


void* Mars::startMarsFunc(void* a)
{
    int argc = 0;
    char **argv = 0; 
    
    Mars *mars = static_cast<simulation::Mars *>(a);
    mars->simulatorInterface = SimulatorInterface::getInstance();
   
    // should be don after runSimulation
    mars->simulatorInterface->runSimulation(argc, argv);
    return 0;
}




/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Mars.hpp for more detailed
// documentation about them.

bool Mars::configureHook()
{
     ControlCenter* controlCenter = simulatorInterface->getControlCenter();
     if(!controlCenter->arg_no_gui)
     {
	 // wait until graphics has been initialized
    	 while(!controlCenter->graphics)
  	       usleep(10000);
     }

    while(!controlCenter->nodes)
  	      usleep(10000);

    bool isDistributedSimulation = _distributed_simulation.get();

    if(isDistributedSimulation)
    {
     ControlCenter* controlCenter = simulatorInterface->getControlCenter();
     PluginInterface* plugin = new MultiSimPlugin(controlCenter);

     pluginStruct pstruct;
     pstruct.name = "MultiSimPlugin";
     pstruct.p_interface = plugin;
     pstruct.p_destroy = NULL;
     
     simulatorInterface->addPlugin(pstruct);

    }
/*
     ControlCenter* controlCenter = simulatorInterface->getControlCenter();
     PluginInterface* plugin = new RimresEnv(controlCenter);

     pluginStruct pstruct;
     pstruct.name = "RimresPlugin";
     pstruct.p_interface = plugin;
     pstruct.p_destroy = NULL;

     
     simulatorInterface->addPlugin(pstruct);
*/
     return true;

}


bool Mars::startHook()
{
    return true;
}

void Mars::updateHook()
{
}

void Mars::errorHook()
{
}

void Mars::stopHook()
{
}

void Mars::cleanupHook()
{
}

