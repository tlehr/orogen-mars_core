#ifndef MARS_CORE_MARS_PLUGIN__
#define MARS_CORE_MARS_PLUGIN__

#include "Mars.hpp"
#include <mars/lib_manager/LibManager.h>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/data_broker/ReceiverInterface.h>


using namespace mars;

namespace simulation
{

/** helper class, which implements a mars plugin interface.
 */
class MarsPlugin : public mars::interfaces::PluginInterface, public mars::data_broker::ReceiverInterface
{
    mars::interfaces::SimulatorInterface* sim;
    double simTime;

public:
    MarsPlugin()
	: PluginInterface(0), sim(0), simTime(0.0)
    {
	connect();
    }

    ~MarsPlugin()
    {
	disconnect();
    }

    void init()
    {
	// register for sim time
	control->dataBroker->registerSyncReceiver(this, "mars_sim", "simTime", 1);
    }

    /** get the simulation time
     */
    base::Time getTime()
    {
	//return base::Time::fromSeconds( simTime );
	return base::Time::now();
    }

    double getSimTime(){
        return simTime; 
    }

    void connect()
    {
	// get simulator interface from singleton
	if( sim )
	    disconnect();
	else
	{
	    sim = Mars::getSimulatorInterface();
	    if( !sim )
		throw std::runtime_error("MarsPlugin: could not get singleton instance of simulator interface.");
	}

	// register as plugin
        mars::interfaces::pluginStruct newplugin;
	newplugin.name = "RockPlugin";
	newplugin.p_interface = dynamic_cast<mars::interfaces::PluginInterface*>(this);
	newplugin.p_destroy = 0;
	sim->addPlugin(newplugin);

	// get controlcenter
	control = sim->getControlCenter();
    }

    void disconnect()
    {
	if( sim )
	    sim->removePlugin( this );
    }

    void reset() {};

    void receiveData(
	    const data_broker::DataInfo& info,
	    const data_broker::DataPackage& package,
	    int id) 
    {
	package.get("simTime", &simTime);
    }
};

}

#endif
