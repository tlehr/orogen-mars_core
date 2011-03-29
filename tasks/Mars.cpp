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
	MarsArguments* marsArguments = static_cast<MarsArguments*>(argument);
	char c_[Mars::configDir.length()+2];
	strcpy(c_,Mars::configDir.c_str());

	// Using the 'command-line' interface to pass
	// arguments use --nogui, i.e. -n from Mars interface
        // set the option to "" if it does not require further args
        std::vector<Option> rawOptions = marsArguments->raw_options;

        // Default arguments
        if(!Mars::configDir.empty())
        {
            Option option("-C", std::string(c_));
            rawOptions.push_back(option);
        }

        // Optional arguments
	if(!marsArguments->enable_gui)
	{
            Option noGuiOption("-n","");
            rawOptions.push_back(noGuiOption);
	}
       
        if(marsArguments->controller_port > 0)
        {
            char buffer[10];
            sprintf(buffer, "%d", marsArguments->controller_port);
            Option controllerPortOption("-c", std::string(buffer));
            rawOptions.push_back(controllerPortOption);
        }

	Mars *mars = marsArguments->mars; 
	mars->simulatorInterface = SimulatorInterface::getInstance();
	Mars::marsRunning=true;

        char** argv = mars->setOptions(rawOptions);
        int count = mars->getOptionCount(rawOptions);
        fprintf(stderr, "Simulator: arguments: \n");
        // Plus one for executable name
        for(int i = 0; i < count + 1; i++)
        {
            fprintf(stderr, "%s ", argv[i]);
        }
        fprintf(stderr, "\n");

	mars->simulatorInterface->runSimulation(count + 1, argv);

	// should be done after runSimulation
	Mars::marsRunning=false;
	return 0;
}

int Mars::getOptionCount(const std::vector<Option>& options)
{
        std::vector<Option>::const_iterator it;

        // First just counting the number of arguments
        int count = 0;
        for(it = options.begin(); it != options.end(); it++)
        {
            Option option = *it;
            // Differentiate between option with args and without
            if(option.parameter != "")
                count += 2;
            else
                count += 1;
        }

        return count;
}

char** Mars::setOptions(const std::vector<Option>& options)
{
        int count = getOptionCount(options)+ 1;
	char** argv = (char **) calloc(count, sizeof(char**));

        // Set executable name to mars_core
        count = 0;
	argv[count++] = "mars_core";

        std::vector<Option>::const_iterator it;
        for(it = options.begin(); it != options.end(); it++)
        {
            Option opt = *it;

            if(opt.name == "")
                continue;

            argv[count++] = strdup(opt.name.c_str());
            if(opt.parameter != "")
            {
                argv[count++] = strdup(opt.parameter.c_str());
            }
        }

        return argv;
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
        argument.controller_port = _controller_port.get();
        argument.raw_options = _raw_options.get();

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

	// Dealing with couple of more timing issues 
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
        // Simulation should be either started manually, 
        // or by using the control_action input_port
	return true;
}

void Mars::updateHook()
{
    simulation::Control controlAction;
    if(_control_action.read(controlAction) == RTT::NewData)
    {
        switch(controlAction)
        {
            case START:
                if(!simulatorInterface->isRunning())
                    simulatorInterface->startStopTrigger();
                break;
            case PAUSE:
                if(simulatorInterface->isRunning())
                    simulatorInterface->startStopTrigger();
                break;
            case RESET:
                simulatorInterface->spotReload();
                break;
            case STEP:
                simulatorInterface->singleStep();
                break;
            default:
                fprintf(stderr, "Simulation: Unknown control action %d received\n", controlAction);

        }

    }
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

