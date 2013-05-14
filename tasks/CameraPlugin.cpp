/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CameraPlugin.hpp"
#include <mars/sim/CameraSensor.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>

using namespace simulation;

CameraPlugin::CameraPlugin(std::string const& name)
    : CameraPluginBase(name)
{
}

CameraPlugin::CameraPlugin(std::string const& name, RTT::ExecutionEngine* engine)
    : CameraPluginBase(name, engine)
{
}

CameraPlugin::~CameraPlugin()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See CameraPlugin.hpp for more detailed
// documentation about them.
bool CameraPlugin::configureHook()
{
    if (! simulation::MarsPlugin::configureHook())
        return false;
    return true;
}

bool CameraPlugin::startHook()
{
    if (! simulation::MarsPlugin::startHook())
        return false;
    
    sensor_id = control->sensors->getSensorID( _name.value() );
    if( !sensor_id ){
	std::cerr << "CameraPlugin: There is no camera by the name of " + _name.value() + " in the scene";
        return false;
    }
    
    camera = dynamic_cast<mars::sim::CameraSensor *>(control->sensors->getSimSensor(sensor_id));
    if( !camera){
	std::cerr << "CameraPlugin: Given sensor name is not a camera";
        return false;
    }

    width = camera->getConfig().width;
    height = camera->getConfig().height;
    
    control->graphics->addGraphicsUpdateInterface(this);
    
    fps = 1.0 / fps * 1000;
    return true;
}

void CameraPlugin::updateHook()
{
    simulation::MarsPlugin::updateHook();
}

void CameraPlugin::errorHook()
{
    simulation::MarsPlugin::errorHook();
}



void CameraPlugin::stopHook()
{
    simulation::MarsPlugin::stopHook();
}



void CameraPlugin::cleanupHook()
{
    simulation::MarsPlugin::cleanupHook();
}

void CameraPlugin::update(mars::interfaces::sReal time_ms)
{
    lastUpdateTime += time_ms;
}

void CameraPlugin::postGraphicsUpdate(void )
{
    if(lastUpdateTime - lastGrabTime < fps)
	return;
    
    lastGrabTime = lastUpdateTime;
    
    getData();
}

void CameraPlugin::getData(){
    //This method has to be implemented by the sublasses and is not allowed to be called
    assert(false);
}
