/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MarsLaserRangeFinder.hpp"
#include "MarsPlugin.hpp"
#include <mars/sim/RaySensor.h>
#include <base/time.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>


using namespace simulation;

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

bool MarsLaserRangeFinder::startHook()
{
    if (! MarsLaserRangeFinderBase::startHook())
        return false;

    int sensor_id = control->sensors->getSensorID( _name.value() );
    if( !sensor_id ){
        std::cout <<  "There is no sensor by the name of " << _name.value() << " in the scene" << std::endl;
        return false;
    }

    sensor = dynamic_cast<mars::sim::RaySensor*>( control->sensors->getSimSensor( sensor_id ) );
    if( !sensor ){
        std::cerr  << "The sensor with " <<  _name.value() <<  " is not of the correct type (RaySensor)" << std::endl;
        return false;
    }

    return true;
}

void MarsLaserRangeFinder::stopHook()
{
    MarsLaserRangeFinderBase::stopHook();
}

void  MarsLaserRangeFinder::update( double time )
{
    if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning
    scan.time = getTime();
    std::vector<double> ranges = sensor->getSensorData();
    scan.ranges.resize( ranges.size() );
    scan.minRange = _min_range.get() * 1000;
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
        _scans.write( scan );
    }
}
