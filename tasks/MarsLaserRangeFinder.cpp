/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MarsLaserRangeFinder.hpp"
#include "MarsPlugin.hpp"
#include <sim/RaySensor.h>
#include <base/time.h>
#include <interfaces/SensorManagerInterface.h>


using namespace simulation;

namespace simulation {
struct LaserRangeFinderPlugin : public MarsPlugin
{
    mars::sim::RaySensor* sensor;
    base::samples::LaserScan scan;
    RTT::OutputPort< base::samples::LaserScan >& port;

    LaserRangeFinderPlugin( RTT::OutputPort< base::samples::LaserScan >& port )
	: port( port )
    {
    }

    void update( double time )
    {
	scan.time = getTime();
	std::vector<double> ranges = sensor->getSensorData();
	scan.ranges.resize( ranges.size() );
	scan.minRange = 10;
	scan.maxRange = sensor->getConfig().maxDistance * 1000;
	for( size_t i=0; i<ranges.size(); i++ )
	{
	    const long range = ranges[i] * 1000;
	    scan.ranges[i] = ( range < scan.maxRange ) ? range : base::samples::TOO_FAR;
	}
	// assume scan to be centered
	if( !scan.ranges.empty() )
	{
	    const double opening_width = sensor->getConfig().opening_width; 
	    scan.start_angle = -opening_width / 2.0;
	    scan.angular_resolution = opening_width / scan.ranges.size();
	    port.write( scan );
	}
    }

    void setName( const std::string& name )
    {
	int sensor_id = control->sensors->getSensorID( name );
	if( !sensor_id )
	    throw std::runtime_error("There is no sensor by the name of " + name + " in the scene");

	sensor = dynamic_cast<mars::sim::RaySensor*>( control->sensors->getSimSensor( sensor_id ) );
	if( !sensor )
	    throw std::runtime_error("The sensor with " + name + " is not of the correct type (RaySensor)");

    }
};
}

MarsLaserRangeFinder::MarsLaserRangeFinder(std::string const& name)
    : MarsLaserRangeFinderBase(name)
{
}

MarsLaserRangeFinder::MarsLaserRangeFinder(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsLaserRangeFinderBase(name, engine)
{
}

MarsLaserRangeFinder::~MarsLaserRangeFinder()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MarsLaserRangeFinder.hpp for more detailed
// documentation about them.

// bool MarsLaserRangeFinder::configureHook()
// {
//     if (! MarsLaserRangeFinderBase::configureHook())
//         return false;
//     return true;
// }
bool MarsLaserRangeFinder::startHook()
{
    if (! MarsLaserRangeFinderBase::startHook())
        return false;

    plugin = new LaserRangeFinderPlugin( _scans );
    plugin->setName( _name.value() );

    return true;
}
// void MarsLaserRangeFinder::updateHook()
// {
//     MarsLaserRangeFinderBase::updateHook();
// }
// void MarsLaserRangeFinder::errorHook()
// {
//     MarsLaserRangeFinderBase::errorHook();
// }
void MarsLaserRangeFinder::stopHook()
{
    MarsLaserRangeFinderBase::stopHook();

    delete plugin;
}
// void MarsLaserRangeFinder::cleanupHook()
// {
//     MarsLaserRangeFinderBase::cleanupHook();
// }

