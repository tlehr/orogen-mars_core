/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MarsPlugin.hpp"

using namespace simulation;

MarsPlugin::MarsPlugin(std::string const& name)
    : MarsPluginBase(name) ,PluginInterface(0), sim(0), simTime(0.0)
{
    setlocale(LC_ALL,"C"); //Make sure english Encodings are used
}

MarsPlugin::MarsPlugin(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsPluginBase(name, engine) ,PluginInterface(0), sim(0), simTime(0.0)
{
    setlocale(LC_ALL,"C"); //Make sure english Encodings are used
}

MarsPlugin::~MarsPlugin()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MarsPlugin.hpp for more detailed
// documentation about them.
bool MarsPlugin::configureHook()
{
    if (! RTT::TaskContext::configureHook())
    {
        RTT::log(RTT::Warning) << "MarsPlugin: configure failed." << RTT::endlog();
        return false;
    }
    
    if(!connect())
    {
        RTT::log(RTT::Warning) << "MarsPlugin: establishing connection with Mars failed. Configure hook returning false." << RTT::endlog();
        return false;
    }

    control->dataBroker->registerSyncReceiver(this, "mars_sim", "simTime", 1);
    return true;
}

bool MarsPlugin::startHook()
{
    if (! RTT::TaskContext::startHook())
        return false;
    return true;
}

void MarsPlugin::updateHook()
{
    RTT::TaskContext::updateHook();
}

void MarsPlugin::errorHook()
{
    RTT::TaskContext::errorHook();
}


void MarsPlugin::stopHook()
{
    RTT::TaskContext::stopHook();
}

void MarsPlugin::cleanupHook()
{
    RTT::TaskContext::cleanupHook();
}
        
void MarsPlugin::update(double delta_t){
}
    
void MarsPlugin::init()
{
    // register for sim time
    control->dataBroker->registerSyncReceiver(this, "mars_sim", "simTime", 1);
}


/** get the simulation time
 */
base::Time MarsPlugin::getTime()
{
    //return base::Time::fromSeconds( simTime );
    return base::Time::now();
}

double MarsPlugin::getSimTime(){
    return simTime; 
}

bool MarsPlugin::connect()
{
    // get simulator interface from singleton
    if( sim )
        disconnect();
    else
    {
        sim = Mars::getSimulatorInterface();
        if( !sim ){
            std::cerr << "MarsPlugin: could not get singleton instance of simulator interface." << std::endl;
            RTT::log(RTT::Error) << "MarsPlugin: could not get singleton instance of simulator interface." << std::endl;
            return false;
        }
    }

    // register as plugin
    mars::interfaces::pluginStruct newplugin;
    newplugin.name = "RockPlugin";
    newplugin.p_interface = dynamic_cast<mars::interfaces::PluginInterface*>(this);
    newplugin.p_destroy = 0;
    sim->addPlugin(newplugin);

    // get controlcenter
    control = sim->getControlCenter();
    Mars::getTaskInterface()->registerPlugin(this);

    return true;
}

void MarsPlugin::disconnect()
{
    if( sim ){
        sim->removePlugin( this );
        Mars::getTaskInterface()->unregisterPlugin(this);
    }
}

void MarsPlugin::reset() {};

void MarsPlugin::receiveData(
        const mars::data_broker::DataInfo& info,
        const mars::data_broker::DataPackage& package,
        int id) 
{
    package.get("simTime", &simTime);
}

void MarsPlugin::handleMarsShudown(){
    fprintf(stderr,"Shutting down %s, because mars instance is shutting down\n",getName().c_str());
    exception(LOST_MARS_CONNECTION);
}
