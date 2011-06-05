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

Mars::Mars(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsBase(name, engine), simulatorInterface(0), enableGui(false) 
{
	Mars::marsRunning=false;
}

Mars::~Mars()
{
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

bool Mars::configureHook()
{
	if(_config_dir.get().empty())
		throw std::runtime_error("Config directory is not set! Can not start mars");	 

        //check if the environemnt was sourced more than once and the path has more than one entry
        int pos = _config_dir.get().rfind(":/");
        if(pos != _config_dir.get().size()-1)
            _config_dir.set(_config_dir.get().substr(pos+1));

        //mars is not setting the config path propper
        //therfore we have to go into to the config folder
            
        
        if(0 != chdir(_config_dir.get().c_str()))
		throw std::runtime_error(string("Config directory ") +_config_dir.get() +" does not exist. Can not start mars.");	 

	// Startup of simulation
	MarsArguments argument;
	argument.mars = this;
	argument.enable_gui = _enable_gui.get();
        argument.controller_port = _controller_port.get();
        argument.raw_options = _raw_options.get();

	int ret = pthread_create(&thread_info, NULL, startMarsFunc, &argument);
	if(ret)
		throw std::runtime_error("Failed to create MARS thread");

	ControlCenter* controlCenter = 0;
	// Using pointer initialization to make sure
	// the simulation has been started before trying to 
	// access it
        
        //wait until simulator is started
        //because some one smart has overwritten isRunning we have to cast it to the base class 
	for(int i=0;!simulatorInterface || !(dynamic_cast<QThread*>(simulatorInterface)->isRunning());++i)
        {
                //give up after 10 sec
                if(i > 1000)
                      throw std::runtime_error("Can not start mars thread!");
		usleep(10000);
        }

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

