#include "Mars.hpp"
#include <mars/sim/Simulator.h>
#include <mars/utils/Thread.h>
#include <mars/utils/mathUtils.h>
#include <mars/interfaces/sim/SimulatorInterface.h>

#include <simulation/tasks/MarsControl.hpp>
#include <mars/gui/MarsGui.h>
#include <mars/main_gui/MainGUI.h>
#include <mars/main_gui/GuiInterface.h>
#include <mars/cfg_manager/CFGManagerInterface.h>
#include <mars/graphics/GraphicsManager.h>
#include <mars/app/GraphicsTimer.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>

//#include <mars/multisim-plugin/MultiSimPlugin.h>

#include <mars/lib_manager/LibManager.h>
#include <QApplication>
#include <QPlastiqueStyle>

using namespace simulation;
using namespace mars;

mars::interfaces::SimulatorInterface *Mars::simulatorInterface = 0;
simulation::Mars *Mars::taskInterface = 0;
mars::app::GraphicsTimer *Mars::graphicsTimer = 0;
mars::lib_manager::LibManager* Mars::libManager = 0; 
SimulationTime Mars::simTime;

Mars::Mars(std::string const& name)
    : MarsBase(name)
    , multisimPlugin(0)
{
    Mars::taskInterface = this;
    setlocale(LC_ALL,"C"); //Make sure english Encodings are used
    setenv("LANG","C",true);
    app = 0;
    _gravity.set(Eigen::Vector3d(0,0,-9.81));
}

Mars::Mars(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsBase(name, engine)
    , multisimPlugin(0)
{
    Mars::taskInterface = this;
    app = 0;
    setlocale(LC_ALL,"C"); //Make sure english Encodings are used
    setenv("LANG","C",true);
}

Mars::~Mars()
{
    delete app;
}


void Mars::loadScene(::std::string const & path)
{
    if(simulatorInterface){
        simulatorInterface->loadScene(path, path,true,true);
    }else{
        RTT::log(RTT::Error) << "Simulator not yet started cout not load scenefile" << RTT::endlog();        
    }
}

mars::interfaces::SimulatorInterface* Mars::getSimulatorInterface()
{
    return simulatorInterface;
}

simulation::Mars* Mars::getTaskInterface(){
    return taskInterface;
}

void* Mars::startMarsFunc(void* argument)
{
    setlocale(LC_ALL,"C"); //Make sure english Encodings are used
    setenv("LANG","C",true);
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
    if(!Mars::getTaskInterface()->app){
        //Initialize Qapplication only once! and keep the instance
        Mars::getTaskInterface()->app = new QApplication(argc, argv);
        Mars::getTaskInterface()->app->setStyle(new QPlastiqueStyle);
    }

    setlocale(LC_ALL,"C");
    setenv("LANG","C",true);
    struct lconv* locale = localeconv();
    RTT::log(RTT::Info) << "Active locale (LC_ALL): " << RTT::endlog();
      
    if( *(locale->decimal_point) != '.')
    {
        RTT::log(RTT::Error) << "Current locale conflicts with mars" << RTT::endlog();
        marsArguments->failed_to_init = true;
        return 0;
    }

    // Prepare the LibManager and required configuration files
    libManager = new mars::lib_manager::LibManager();

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
    mars::lib_manager::LibInterface* lib = libManager->getLibrary(std::string("cfg_manager"));
    if(lib)
    {
 	cfg_manager::CFGManagerInterface* cfg = dynamic_cast<cfg_manager::CFGManagerInterface*>(lib);
        if(cfg)
        {

            cfg_manager::cfgPropertyStruct configPath;
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
		RTT::log(RTT::Error) << "CRITICAL (cause abort): Property was not set correctly: " << configPath.sValue << RTT::endlog();
                marsArguments->failed_to_init = true;
                return 0;
            } else {
                RTT::log(RTT::Error) << "Configuration path property set to " << configPath.sValue << RTT::endlog();
            }

            std::string loadFile = configPath.sValue;
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
        RTT::log(RTT::Error) << "CRITICAL (cause abort) Simulation library failed to load" << RTT::endlog();
        RTT::log(RTT::Error) << "Configuration loaded from " << corelibsConfigPath << RTT::endlog();
        marsArguments->failed_to_init = true;
        return 0;
    }


    // Prepare the simulation instance, load the argument and run
    mars->simulatorInterface = dynamic_cast<sim::Simulator*>(lib); 
    if(!mars->simulatorInterface)
    {
        RTT::log(RTT::Error) << "CRITICAL (cause abort) Simulation could not be retrieved via lib_manager" << RTT::endlog();
        marsArguments->failed_to_init = true;
        return 0;
    }

    mars->simulatorInterface->readArguments(count + 1, argv);
    std::string cmd;
    for(int i = 0; i < count+1;++i)
    {
        cmd += std::string(argv[i]);
        cmd += " ";
    }

    RTT::log(RTT::Info) << "Starting mars with: " << cmd << RTT::endlog();


    // if we have a main gui, show it 
    if(marsArguments->enable_gui)
    {

        gui::MarsGui *marsGui = NULL;
        lib = libManager->getLibrary("mars_gui");
        if(lib)
        {
            if( (marsGui = dynamic_cast<gui::MarsGui*>(lib)) )
            {
                marsGui->setupGui();
            }
        } else {
            RTT::log(RTT::Error) << "CRITICAL (cause abort) Simulator: mars_gui not found, cannot show GUI" << RTT::endlog();
            marsArguments->failed_to_init = true;
            return 0;
        }

        main_gui::MainGUI* mainGui;
        lib = libManager->getLibrary("main_gui");
        if(lib && (mainGui = dynamic_cast<main_gui::MainGUI*>(lib)) )
        {
            // all good
        } else {
            RTT::log(RTT::Error) << "CRITICAL (cause abort) Simulator: gui_core not found, cannot show GUI" << RTT::endlog();
            marsArguments->failed_to_init = true;
            return 0;
        }


        lib = libManager->getLibrary("mars_graphics");
        if(lib) 
        {
            if( (Mars::getTaskInterface()->marsGraphics = dynamic_cast<graphics::GraphicsManager*>(lib)) )
            {
                // init osg
                //initialize graphicsFactory
                Mars::getTaskInterface()->marsGraphics->initializeOSG(NULL);
                void* widget = Mars::getTaskInterface()->marsGraphics->getQTWidget(1);	
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
    Mars::graphicsTimer = new app::GraphicsTimer(Mars::getTaskInterface()->marsGraphics, mars->simulatorInterface);
    Mars::graphicsTimer->run();

    if(marsArguments->add_floor){
        mars->simulatorInterface->getControlCenter()->nodes->createPrimitiveNode("Boden",mars::interfaces::NODE_TYPE_PLANE,false,mars::utils::Vector(0,0,0.0),mars::utils::Vector(600,600,0));
    }
    //assert(mars->simulatorInterface->getControlCenter()->dataBroker->registerTriggeredReceiver(mars,"mars_sim", "simTime","mars_sim/postPhysicsUpdate",1));
    int result = mars->simulatorInterface->getControlCenter()->dataBroker->registerTriggeredReceiver(mars,"mars_sim", "simTime","mars_sim/postPhysicsUpdate",1);
    assert(result);
    
    // is realtime calc requested?
    if(marsArguments->realtime_calc){
    	mars->simulatorInterface->getControlCenter()->cfg->setPropertyValue("Simulator", "realtime calc", "value", marsArguments->realtime_calc);
    }
    
    // Synchronize with configureHook
    marsArguments->initialized = true;
    Mars::getTaskInterface()->app->exec();
   
   
    libManager->releaseLibrary("mars_graphics");
    libManager->releaseLibrary("main_gui");
    libManager->releaseLibrary("mars_gui");
    libManager->releaseLibrary("mars_sim");
    libManager->releaseLibrary("cfg_manager");
   
    //Workaround release waht whereever is acquired
    libManager->releaseLibrary("data_broker");
    libManager->releaseLibrary("avalonplugin");
    libManager->releaseLibrary("mars_sim");
    libManager->releaseLibrary("main_gui");

    delete Mars::graphicsTimer;
    delete libManager;
    //Do not delete the QApplication it does not like it to be restarted
    std::cout << "Qapplication exec ended" << std::endl;

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

bool Mars::setShow_coordinate_system(bool value)
{
        printf("Hook called with: %s\n",value?"true":"false");

	//TODO Add your code here 
        if(!marsGraphics){
            fprintf(stderr,"Could not change view of coordinate systems without an Graphics interface\n");
            return false;
        }

  	//Call the base function, DO-NOT Remove
        if(value)
            marsGraphics->hideCoords();
        else
            marsGraphics->showCoords();

	return(simulation::MarsBase::setShow_coordinate_system(value));
}

bool Mars::setReaction_to_physics_error(::std::string const & value)
{
	//TODO Add your code here
        if(isConfigured()){
            if(!simulatorInterface){
                fprintf(stderr,"Mars is not running could not set reaction to physics error");
                return false;
            }
            if(value == "abort" || value == "reset" || value == "warn" || value == "shutdown"){
                simulatorInterface->getControlCenter()->cfg->setPropertyValue("Simulator", "onPhysicsError","value", value);
            }else{
                fprintf(stderr,"Could not ser rection to physics: Possible Values: abort (killing sim), reset (ressing scene and simulation), warn (keep simulation running an print warnings), shutdown (stop physics but keep mars-running and set this tas to the error state)");
                return false;
            }
        }
        
        //Call the base function, DO-NOT Remove
	return(simulation::MarsBase::setReaction_to_physics_error(value));
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
    //if(0 != chdir(_config_dir.get().c_str()))
    //{
    //    RTT::log(RTT::Error) << "Config directory " << _config_dir.get() << " does not exist. Cannot start mars." << RTT::endlog();
    //    throw std::runtime_error(std::string("Config directory ") +_config_dir.get() +" does not exist. Can not start mars.");    
    //}

    // Startup of simulation
    MarsArguments argument;
    argument.mars = this;
    argument.enable_gui = _enable_gui.get();
    argument.controller_port = _controller_port.get();
    argument.raw_options = _raw_options.get();
    argument.config_dir = _config_dir.get();
    argument.initialized = false;
    argument.add_floor = _add_floor.get();
    argument.failed_to_init=false;
    argument.realtime_calc = _realtime_calc.get();

    int ret = pthread_create(&thread_info, NULL, startMarsFunc, &argument);
    if(ret)
    {
        RTT::log(RTT::Error) << "Failed to create MARS thread: pthread error " << ret << RTT::endlog();
        throw std::runtime_error("Failed to create MARS thread");
    }

    for(int i=0; !argument.initialized && !argument.failed_to_init;++i)
    {
        //give up after 10 sec
        if(i > 1000)
        {
            RTT::log(RTT::Error) << "Cannot start mars thread" << RTT::endlog();
            throw std::runtime_error("Cannot start mars thread!");
        }
        usleep(10000);
    }
    if(argument.failed_to_init){
            RTT::log(RTT::Error) << "Mars failed to start, see Error above" << RTT::endlog();
            return false;
    }

    RTT::log(RTT::Info) << "Mars running" << RTT::endlog();

    // Simulation is now up and running and plugins can be added
    // Configure basic functionality of simulation
    // Check if distributed simulation should be activated

    // todo: should be loaded via lib_manager
    /*
    if(_distributed_simulation.get())
    {
        RTT::log(RTT::Info) << "Loading MultiSimPlugin" << RTT::endlog();
        multisimPlugin = new MultiSimPlugin(libManager);
        RTT::log(RTT::Info) << "MultiSimPlugin loaded" << RTT::endlog();
    }
    */
    if(!_initial_scene.get().empty()){
        printf("name: %s",_initial_scene.get().c_str());
        simulatorInterface->loadScene(_initial_scene.get(), std::string("initial"),true,true);
    }
    std::vector<std::string> sceneNames = _initial_scenes.get();
    if(!sceneNames.empty()){
		for (std::vector< std::string >::iterator scene = sceneNames.begin(); scene != sceneNames.end();scene++){
			printf("name: %s",scene->c_str());
			simulatorInterface->loadScene(*scene, *scene,true,true);
		}
    }
    

	std::vector<Positions> positions = _positions.get();
    if(!positions.empty()){
    	for (std::vector< Positions >::iterator offset = positions.begin(); offset != positions.end();offset++){
    		printf("moving name: %s\n",offset->nodename.c_str());
    		//simulatorInterface->loadScene(*scene, *scene,true,true);
    		mars::interfaces::NodeManagerInterface* nodes = simulatorInterface->getControlCenter()->nodes;
    		mars::interfaces::NodeId id = nodes->getID(offset->nodename);
    		//mars::utils::Vector pos
    		if (id){
    			printf("found id : %li\n",id);
				mars::interfaces::NodeData nodedata = nodes->getFullNode(id);

				utils::Vector pos = nodes->getPosition(id);

				printf("actual position %.2f %.2f %.2f\n", pos.x(),pos.y(),pos.z());
				printf("offset position %.2f %.2f %.2f\n", offset->posx,offset->posy,offset->posz);

				pos.x() = offset->posx;
				pos.y() = offset->posy;
				pos.z() = offset->posz;

				mars::utils::Quaternion rot = nodes->getRotation(id);

				mars::utils::Vector rotoff;

				rotoff.x() =  offset->rotx;// * (M_PI/180.0);
				rotoff.y() =  offset->roty;// * (M_PI/180.0);
				rotoff.z() =  offset->rotz;// * (M_PI/180.0);

				mars::utils::Quaternion newrot = mars::utils::eulerToQuaternion(rotoff);





				printf("new position %.2f %.2f %.2f\n", pos.x(),pos.y(),pos.z());
        nodedata.pos = pos;
        nodedata.rot = newrot;

				nodes->editNode(&nodedata, mars::interfaces::EDIT_NODE_POS | mars::interfaces::EDIT_NODE_MOVE_ALL);
				nodes->editNode(&nodedata, mars::interfaces::EDIT_NODE_ROT | mars::interfaces::EDIT_NODE_MOVE_ALL);
				// nodes->setPosition(id, pos);
				// nodes->setRotation(id,newrot);

    		}
    	}

    }

    {//Setting the Step-with for the simulation
    cfg_manager::cfgPropertyStruct c = simulatorInterface->getControlCenter()->cfg->getOrCreateProperty("Simulator", "calc_ms", _sim_step_size.get()*1000.0);
    c.dValue = _sim_step_size.get()*1000.0;
    simulatorInterface->getControlCenter()->cfg->setProperty(c);
    }


    {
    std::string value = _reaction_to_physics_error.get();
    if(value == "abort" || value == "reset" || value == "warn" || value == "shutdown"){
        simulatorInterface->getControlCenter()->cfg->setPropertyValue("Simulator", "onPhysicsError","value", value);
    }else{
        fprintf(stderr,"Wront selection for physic error setting\n");
        return false;
    }
    }

    setGravity_internal(_gravity.get());

    return updateDynamicProperties();
}


bool Mars::startHook()
{
    // Simulation should be either started manually, 
    // or by using the control_action input_port
    //
    if (_start_sim.get()){
    	simulatorInterface->StartSimulation();
    }
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

    if(simulatorInterface->hasSimFault()){
        std::cerr << "Simulation detected a Physics error, stopping all plugins and go to Exception state" << std::endl;
        for(unsigned int i=0;i<plugins.size();i++){
            plugins[i]->handleMarsShudown();
        }
        exception(PHYSICS_ERROR);
 //       QCoreApplication::quit(); //Quitting QApplication too
    }
}

void Mars::errorHook()
{
    std::cout << "ERROR HOOK" << std::endl;
}

void Mars::stopHook()
{
    /*
    std::cout << "STOP HOOK" << std::endl;
    for(unsigned int i=0;i<plugins.size();i++){
        plugins[i]->handleMarsShudown();
    }
    simulatorInterface->exitMars();

    std::cout << "STOP HOOK quitting qapp" << std::endl;
    QCoreApplication::quit(); //Quitting QApplication too
    std::cout << "STOP HOOK quitting qapp finish" << std::endl;
    */
}
        
void Mars::registerPlugin(MarsPlugin* plugin){
    plugins.push_back(plugin);
}

void Mars::unregisterPlugin(MarsPlugin* plugin){
    plugins.push_back(plugin);
}

void Mars::cleanupHook()
{
    std::cout << "CLEANUP HOOK" << std::endl;
   

    for(unsigned int i=0;i<plugins.size();i++){
        plugins[i]->handleMarsShudown();
    }
    plugins.clear();

    simulatorInterface->exitMars();
    while( simulatorInterface->isSimRunning()) ;
    
    QCoreApplication::quit(); //Quitting QApplication too

    std::cout << "CLEANUP HOOK quitting qapp finish" << std::endl;

   // delete libManager;
    
//    libManager->releaseLibrary("mars_sim");
//    libManager->releaseLibrary("mars_gui");
//    libManager->releaseLibrary("mars_graphics");
//    libManager->releaseLibrary("gui_core");


 //   if(multisimPlugin) delete multisimPlugin;
}
/*
bool Mars::recover(){
    std::cout << "RECOVER HOOK" << std::endl;
    return MarsBase::recover();
}
void Mars::fatal(){
    std::cout << "FATAL HOOK" << std::endl;
    MarsBase::fatal();
}
void Mars::exception(){
    std::cout << "EXCEPTION HOOK" << std::endl;
    MarsBase::exception();
}
*/

void Mars::receiveData(
        const data_broker::DataInfo& info,
        const data_broker::DataPackage& package,
        int id) 
{
    double ms;
    package.get("simTime", &ms);
    // update the simulation time
    simTime.setElapsedMs( ms );

    //update the time output ports
    _time.write( simTime.getElapsedMs() );
    _simulated_time.write(simTime.get());
}

bool Mars::setGravity_internal(::base::Vector3d const & value){
 simulatorInterface->setGravity(value);
}

bool Mars::setGravity(::base::Vector3d const & value)
{
 if(!isConfigured()){
     //The configuration will be done within the configure hook later
     return(simulation::MarsBase::setGravity(value));
 }
 
 setGravity_internal(value);
 return(simulation::MarsBase::setGravity(value));
}


bool Mars::setSim_step_size(double value)
{
    //convert to ms
    value *= 1000.0;
    if(!isConfigured()){
        //The configuration will be done within the configure hook later
        return(simulation::MarsBase::setSim_step_size(value));
    }
    cfg_manager::cfgPropertyStruct c = simulatorInterface->getControlCenter()->cfg->getOrCreateProperty("Simulator", "calc_ms", value);
    c.dValue = value;
    simulatorInterface->getControlCenter()->cfg->setProperty(c);
    return(simulation::MarsBase::setSim_step_size(value));
}
