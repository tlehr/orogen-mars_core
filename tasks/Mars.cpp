#include "Mars.hpp"
#include <mars_sim/Simulator.h>
#include <mars_core/Thread.h>
#include <mars_sim/SimulatorInterface.h>

#include <simulation/tasks/MarsControl.hpp>
#include <mars_gui/MarsGui.h>
#include <gui_core/MainGUI.h>
#include <gui_core/GuiInterface.h>
#include <mars_graphics/GraphicsManager.h>
#include <mars_app/GraphicsTimer.h>

#include <mars/multisim-plugin/MultiSimPlugin.h>

#include <lib_manager/LibManager.h>
#include <QApplication>
#include <QPlastiqueStyle>

using namespace simulation;

SimulatorInterface *Mars::simulatorInterface = 0;
GraphicsTimer *Mars::graphicsTimer = 0;
lib_manager::LibManager* Mars::libManager = 0; 

Mars::Mars(std::string const& name)
    : MarsBase(name)
    , multisimPlugin(0)
{
}

Mars::Mars(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsBase(name, engine)
    , multisimPlugin(0)
{
}

Mars::~Mars()
{
}

SimulatorInterface* Mars::getSimulatorInterface()
{
    return simulatorInterface;
}

void* Mars::startMarsFunc(void* argument)
{
    MarsArguments* marsArguments = static_cast<MarsArguments*>(argument);

    Mars* mars = marsArguments->mars;

    // Using the 'command-line' interface to pass
    // arguments to mars interface
    // set the option to "" if it does not require further args
    std::vector<Option> rawOptions = marsArguments->raw_options;
       
    if(marsArguments->controller_port > 0)
    {
        char buffer[10];
        sprintf(buffer, "%d", marsArguments->controller_port);
        Option controllerPortOption("-c", std::string(buffer));
        rawOptions.push_back(controllerPortOption);
    }

    char** argv = mars->setOptions(rawOptions);
    int count = mars->getOptionCount(rawOptions);
    // Plus one for executable name
    for(int i = 0; i < count + 1; i++)
    {
        RTT::log(RTT::Info) << "Simulator: argument #" << i << " " << argv[i] << RTT::endlog();
    }
 
    // Prepare Qt Application Thread which is required  
    // for core simulation and gui
    int argc = count + 1;
    QApplication* app = new QApplication(argc, argv);
    app->setStyle(new QPlastiqueStyle);

    setlocale(LC_ALL,"C");
    struct lconv* locale = localeconv();
    RTT::log(RTT::Info) << "Active locale (LC_ALL): " << RTT::endlog();
      
    if( *(locale->decimal_point) != '.')
    {
        RTT::log(RTT::Error) << "Current locale conflicts with mars" << RTT::endlog();
        exit(1);
    }

    // Prepare the LibManager and required configuration files
    libManager = new lib_manager::LibManager();

    std::string corelibsConfigPath;
    // If the graphical interface should be disabled, the configuration 
    // needs to exclude the shared library with graphics options
    // Thus, the nogui configuration file needs to be used for the startup 
    if(marsArguments->enable_gui)
    {
        corelibsConfigPath = marsArguments->config_dir + "/core_libs.txt";
    } else {
        corelibsConfigPath = marsArguments->config_dir + "/core_libs-nogui.txt";
    }

    libManager->loadConfigFile(corelibsConfigPath);

    // Setting the configuration directory and loading the preferences
    lib_manager::LibInterface* lib = libManager->getLibrary(std::string("cfg_dfki"));
    if(lib)
    {
 	cfg_dfki::CFGInterface* cfg = 0;
        if(cfg = dynamic_cast<cfg_dfki::CFGInterface*>(lib))
        {

            cfg_dfki::cfgPropertyStruct configPath;
            configPath = cfg->getOrCreateProperty("Config", "config_path", marsArguments->config_dir);

	    // overriding any defaults 
            configPath.sValue = marsArguments->config_dir;
            cfg->setProperty(configPath);
 	    /*
            if(!cfg->setProperty(configPath))
            {
                RTT::log(RTT::Error) << "Configuration path property could not be set" << RTT::endlog();
                exit(1);
            } 
            */
            configPath = cfg->getOrCreateProperty("Config", "config_path", marsArguments->config_dir);

            if(configPath.sValue != marsArguments->config_dir)
            {
		RTT::log(RTT::Error) << "Property was not set correctly: " << configPath.sValue << RTT::endlog();
                exit(1);
            } else {
                RTT::log(RTT::Error) << "Configuration path property set to " << configPath.sValue << RTT::endlog();
            }

            string loadFile = configPath.sValue;
            loadFile.append("/mars_Preferences.yaml");
            cfg->loadConfig(loadFile.c_str());
            loadFile = configPath.sValue;
            loadFile.append("/mars_Simulator.yaml");
            cfg->loadConfig(loadFile.c_str());

            loadFile = configPath.sValue;
            loadFile.append("/mars_Physics.yaml");
            cfg->loadConfig(loadFile.c_str());
            
            bool loadLastSave = false;
            cfg->getPropertyValue("Config", "loadLastSave", "value",
                                           &loadLastSave);
            if (loadLastSave) 
            {
                loadFile = configPath.sValue;
                loadFile.append("/mars_saveOnClose.yaml");
                cfg->loadConfig(loadFile.c_str());
            }      
        } else {
            RTT::log(RTT::Error) << "Error casting to cfg_dfki" << RTT::endlog();
        }
    } else {
        RTT::log(RTT::Error) << "Could not load library cfg_dfki" << RTT::endlog();
    }

    lib = libManager->getLibrary("mars_sim");
    if(!lib)
    {
        RTT::log(RTT::Error) << "Simulation library failed to load" << RTT::endlog();
        RTT::log(RTT::Error) << "Configuration loaded from " << corelibsConfigPath << RTT::endlog();
        exit(2);
    }


    // Prepare the simulation instance, load the argument and run
    mars->simulatorInterface = dynamic_cast<Simulator*>(lib); 
    if(!mars->simulatorInterface)
    {
        RTT::log(RTT::Error) << "Simulation could not be retrieved via lib_manager" << RTT::endlog();
        exit(3);
    }

    mars->simulatorInterface->readArguments(count + 1, argv);
    std::string cmd;
    for(int i = 0; i < count+1;++i)
    {
        cmd += std::string(argv[i]);
        cmd += " ";
    }

    RTT::log(RTT::Info) << "Starting mars with: " << cmd << RTT::endlog();

    // GraphicsTimer will be later called with the marsGraphics reference
    // which can be also NULL for a disabled gui
    GraphicsManager* marsGraphics = NULL;

    // if we have a main gui, show it 
    if(marsArguments->enable_gui)
    {

        MarsGui *marsGui = NULL;
        lib = libManager->getLibrary("mars_gui");
        if(lib)
        {
            if( (marsGui = dynamic_cast<MarsGui*>(lib)) )
            {
                marsGui->setupGui();
            }
        } else {
            RTT::log(RTT::Error) << "Simulator: mars_gui not found, cannot show GUI" << RTT::endlog();
            exit(4);
        }

        gui_core::MainGUI* mainGui;
        lib = libManager->getLibrary("gui_core");
        if(lib && (mainGui = dynamic_cast<gui_core::MainGUI*>(lib)) )
        {
            // all good
        } else {
            RTT::log(RTT::Error) << "Simulator: gui_core not found, cannot show GUI" << RTT::endlog();
            exit(5);
        }


        lib = libManager->getLibrary("mars_graphics");
        if(lib) 
        {
            if( (marsGraphics = dynamic_cast<GraphicsManager*>(lib)) )
            {
                // init osg
                //initialize graphicsFactory
                marsGraphics->initializeOSG(NULL);
                void* widget = marsGraphics->getQTWidget(1);	
                if (widget && mainGui) 
                {
                    //control->gui->addDockWidget((void*)newWidget,1);
                    mainGui->mainWindow_p()->setCentralWidget((QWidget*)widget);
                    ((QWidget*)widget)->show();
                }     
            }
        }
        
        mars->simulatorInterface->runSimulation();

        mainGui->show();
        
    } else {
        mars->simulatorInterface->runSimulation();
    }

    // Loading libraries that are specified in other_libs.txt
    std::string addonsConfigPath = marsArguments->config_dir + "/other_libs.txt";
    libManager->loadConfigFile(addonsConfigPath);

    // GraphicsTimer allows to update the graphics interface 
    // every 10 ms
    Mars::graphicsTimer = new GraphicsTimer(marsGraphics, mars->simulatorInterface);
    Mars::graphicsTimer->run();

    // Synchronize with configureHook
    marsArguments->initialized = true;
    app->exec();

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
    char** argv = (char**) calloc(count, sizeof(char**));

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
    {
        RTT::log(RTT::Error) << "Config directory is not set! Cannot start mars." << RTT::endlog();
        throw std::runtime_error("Config directory is not set! Can not start mars");     
    }

    //check if the environemnt was sourced more than once and the path has more than one entry
    int pos = _config_dir.get().rfind(":/");
    if(pos != _config_dir.get().size()-1)
        _config_dir.set(_config_dir.get().substr(pos+1));

    RTT::log(RTT::Info) << "Calling configure: with " << _config_dir.get() << RTT::endlog();

    //mars is not setting the config path properly
    //therefore we have to go into to the config folder
    if(0 != chdir(_config_dir.get().c_str()))
    {
        RTT::log(RTT::Error) << "Config directory " << _config_dir.get() << " does not exist. Cannot start mars." << RTT::endlog();
        throw std::runtime_error(string("Config directory ") +_config_dir.get() +" does not exist. Can not start mars.");    
    }

    // Startup of simulation
    MarsArguments argument;
    argument.mars = this;
    argument.enable_gui = _enable_gui.get();
    argument.controller_port = _controller_port.get();
    argument.raw_options = _raw_options.get();
    argument.config_dir = _config_dir.get();
    argument.initialized = false;

    int ret = pthread_create(&thread_info, NULL, startMarsFunc, &argument);
    if(ret)
    {
        RTT::log(RTT::Error) << "Failed to create MARS thread: pthread error " << ret << RTT::endlog();
        throw std::runtime_error("Failed to create MARS thread");
    }

    for(int i=0; !argument.initialized;++i)
    {
        //give up after 10 sec
        if(i > 1000)
        {
            RTT::log(RTT::Error) << "Cannot start mars thread" << RTT::endlog();
            throw std::runtime_error("Cannot start mars thread!");
        }
        usleep(10000);
    }

    RTT::log(RTT::Info) << "Mars running" << RTT::endlog();

    // Simulation is now up and running and plugins can be added
    // Configure basic functionality of simulation
    // Check if distributed simulation should be activated
    if(_distributed_simulation.get())
    {
        RTT::log(RTT::Info) << "Loading MultiSimPlugin" << RTT::endlog();
        multisimPlugin = new MultiSimPlugin(libManager);
        RTT::log(RTT::Info) << "MultiSimPlugin loaded" << RTT::endlog();
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
                RTT::log(RTT::Info) << "ControlAction: Start received" << RTT::endlog();
                if(!simulatorInterface->isSimRunning())
                    simulatorInterface->startStopTrigger();
                break;
            case PAUSE:
                RTT::log(RTT::Info) << "ControlAction: Pause received" << RTT::endlog();
                if(simulatorInterface->isSimRunning())
                    simulatorInterface->startStopTrigger();
                break;
            case RESET:
                RTT::log(RTT::Info) << "ControlAction: Reset received" << RTT::endlog();
                simulatorInterface->resetSim();
                break;
            case STEP:
                RTT::log(RTT::Info) << "ControlAction: Step received" << RTT::endlog();
                simulatorInterface->singleStep();
                break;
            default:
                RTT::log(RTT::Warning) << "Simulation: Unknown control action " << controlAction << " received" << RTT::endlog();

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
    simulatorInterface->exitMars();
    while( simulatorInterface->isSimRunning()) ;

    libManager->unloadLibrary("mars_sim");
    libManager->unloadLibrary("mars_gui");
    libManager->unloadLibrary("mars_graphics");
    libManager->unloadLibrary("gui_core");

    delete libManager;
    delete multisimPlugin;
}

