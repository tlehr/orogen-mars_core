#ifndef MARS_CORE_MARS_PLUGIN__
#define MARS_CORE_MARS_PLUGIN__

#include "tasks/Mars.hpp"
#include <lib_manager/LibManager.h>
#include <mars_sim/SimulatorInterface.h>
#include <mars_sim/ControlCenter.h>
#include <data_broker/ReceiverInterface.h>

namespace simulation
{

/** helper class, which implements a mars plugin interface.
 */
class MarsPlugin : public PluginInterface, public data_broker::ReceiverInterface
{
    SimulatorInterface* sim;
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
	pluginStruct newplugin;
	newplugin.name = "RockPlugin";
	newplugin.p_interface = dynamic_cast<PluginInterface*>(this);
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
