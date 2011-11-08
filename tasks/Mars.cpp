#include "Mars.hpp"
#include <mars/ControlCenter.h>

#include <mars/multisim-plugin/MultiSimPlugin.h>
#include <mars/utils/pathes.h>

#include <lib_manager/LibManager.h>
#include <QApplication>
#include <QPlastiqueStyle>

using namespace simulation;

lib_manager::LibManager* Mars::libManager = 0; 

Mars::Mars(std::string const& name)
    : MarsBase(name)
    , simulatorInterface(0) 
{
}

Mars::Mars(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsBase(name, engine)
    , simulatorInterface(0)
{
}

Mars::~Mars()
{
}

void* Mars::startMarsFunc(void* argument)
{
    MarsArguments* marsArguments = static_cast<MarsArguments*>(argument);

    Mars* mars = marsArguments->mars;

    // Using the 'command-line' interface to pass
    // arguments use --nogui, i.e. -n from Mars interface
    // set the option to "" if it does not require further args
    std::vector<Option> rawOptions = marsArguments->raw_options;

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
        exit(0);
    }

    Pathes::loadPathes(marsArguments->config_dir);
    app->setWindowIcon(QIcon(QString::fromStdString(Pathes::getGuiPath()) + "images/mars_icon.ico"));

    // Prepare the LibManager and required configuration files
    libManager = new lib_manager::LibManager();
    std::string corelibsConfigPath = marsArguments->config_dir + "/core_libs.txt";
    std::string addonsConfigPath = marsArguments->config_dir + "/other_libs.txt";
    libManager->loadConfigFile(corelibsConfigPath);
    libManager->loadConfigFile(addonsConfigPath);

    // Prepare the simulation instance, load the argument and run
    mars->simulatorInterface = SimulatorInterface::getInstance(libManager); 
    mars->simulatorInterface->readArguments(count + 1, argv);
    mars->simulatorInterface->runSimulation();

    // if we have a main gui, show it 
    if(marsArguments->enable_gui)
    {
        lib_manager::LibInterface* lib = libManager->getLibrary("gui_core");
        if(lib)
        {
            gui_core::GuiInterface* mainGui;
            if( (mainGui = dynamic_cast<gui_core::GuiInterface*>(lib)) )
            {
                mainGui->show();
            }
        } else {
          RTT::log(RTT::Error) << "Simulator: gui_core not found, cannot show GUI" << RTT::endlog();
        }
    }

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

    int ret = pthread_create(&thread_info, NULL, startMarsFunc, &argument);
    if(ret)
    {
        RTT::log(RTT::Error) << "Failed to create MARS thread" << RTT::endlog();
        throw std::runtime_error("Failed to create MARS thread");
    }

    //wait until simulator is started
    for(int i=0;!simulatorInterface || !dynamic_cast<QThread*>(simulatorInterface)->isRunning();++i)
    {
        //give up after 10 sec
        if(i > 1000)
        {
            RTT::log(RTT::Error) << "Cannot start mars thread" << RTT::endlog();
            throw std::runtime_error("Cannot start mars thread!");
        }
        usleep(10000);
    }

    // Simulation is now up and running and plugins can be added
    // Configure basic functionality of simulation
    // Check if distributed simulation should be activated
    if(_distributed_simulation.get())
    {
        PluginInterface* plugin = new MultiSimPlugin(libManager);
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
    simulatorInterface->exitMars();
    while( simulatorInterface->isRunning()) ;

    delete libManager;
}

