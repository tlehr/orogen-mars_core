#include "Mars.hpp"
#include <mars/ControlCenter.h>

#include <mars/multisim-plugin/MultiSimPlugin.h>
#include <mars/utils/pathes.h>

using namespace simulation;

std::string Mars::configDir;
bool Mars::marsRunning;

Mars::Mars(std::string const& name)
    : MarsBase(name), simulatorInterface(0), enableGui(false) 
{
	Mars::marsRunning=false;
}

void* Mars::startMarsFunc(void* argument)
{
	int argc = 0;
	char **argv = 0; 

	MarsArguments* marsArguments = static_cast<MarsArguments*>(argument);
	char c_[Mars::configDir.length()+2];
	strcpy(c_,Mars::configDir.c_str());
	// Using the 'command-line' interface to pass
	// arguments use --nogui, i.e. -n from Mars interface
	if(!marsArguments->enable_gui)
	{
		argc = 4;
		argv = (char**) calloc(argc,sizeof(char**));
		argv[0] = "mars_core";
		argv[1] = "-n";
		argv[2] = "-C";
		argv[3] = c_;

	}


	Mars *mars = marsArguments->mars; 
	mars->simulatorInterface = SimulatorInterface::getInstance();
	Mars::marsRunning=true;
	// should be don after runSimulation
	mars->simulatorInterface->runSimulation(argc, argv);
	Mars::marsRunning=false;
	return 0;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Mars.hpp for more detailed
// documentation about them.

bool Mars::configureHook()
{

	/*
	 Configure the resource directories required by mars
	 The resource dir servers as root and fallback to the
	 default locations
	 When a property is given, that path is used without
	 any additional checks

	 Prepare paths before starting Mars, otherwise they
	 will not be set properly
	*/

	std::string resource_dir = _resource_dir.get();
	std::string stuff_path = _stuff_path.get();
	std::string gui_path = _gui_path.get();
	std::string tmp_path = _tmp_path.get();
	std::string save_path = _save_path.get();
	std::string plugin_path = _plugin_path.get();
	std::string debug_path = _debug_path.get();

	if(resource_dir == "")
		throw new  std::runtime_error("Resource directory is not set");	 
	else if ( resource_dir.find_last_of("/") == resource_dir.size() -1 )
	{
		// do nothing / is last character
	} else
	{
		// append folder separator ( not portable, yes)
		resource_dir += "/";
	}


	if(stuff_path == "")
		stuff_path = resource_dir;

	if(gui_path == "")
		gui_path = resource_dir;

	if(tmp_path == "")
		tmp_path = resource_dir + "tmp/";

	if(save_path == "")
		save_path = resource_dir + "save/";

	if(plugin_path == "")
		plugin_path = resource_dir + "plugins/";

	if(debug_path == "")
		debug_path = resource_dir + "debug/";

	Pathes::setStuffPath(stuff_path);
	Pathes::setGuiPath(gui_path);
	Pathes::setTmpPath(tmp_path);
	Pathes::setSavePath(save_path);
	Pathes::setPluginPath(plugin_path);
	Pathes::setDebugPath(debug_path);

	
	Mars::configDir = _config_dir.value();

	enableGui = _enable_gui.get();

	// Startup of simulation after pathes have been read and configured
	MarsArguments argument;
	argument.mars = this;
	argument.enable_gui = enableGui;

	int ret = pthread_create(&thread_info, NULL, startMarsFunc, &argument);
	if(ret)
		throw std::runtime_error("Failed to create MARS thread");

	sleep(1);
	while(!marsRunning){
		printf("Mars is not completly started, we have to wait\n");
		sleep(1);
	}
	ControlCenter* controlCenter = 0;
	// Using pointer initialization to make sure
	// the simulation has been started before trying to 
	// access it
	while(!controlCenter)
	{
		if(simulatorInterface)
			controlCenter = simulatorInterface->getControlCenter();
	}

	// Dealing with couple of more time issues 
	if(enableGui)
	{
		// wait until graphics has been initialized
		while(!controlCenter->graphics)
			usleep(10000);
	}

	while(!controlCenter->nodes)
	      usleep(10000);

	// Simulation is now up and running and plugins can be added


	// Configure basic functionality of simulation
	// Check if distributed simulation should be activated
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

	return true;
}


bool Mars::startHook()
{
	// start simulation automatically, when gui is not available
//	if(!simulatorInterfaceenableGui)
//               simulatorInterface->startStopTrigger();

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
	exit(0);
}

