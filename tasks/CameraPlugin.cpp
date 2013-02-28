#include "CameraPlugin.hpp"
#include <mars/sim/CameraSensor.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>


CameraPlugin::CameraPlugin(const std::string& name, double fps)
    : lastUpdateTime(0), lastGrabTime(0), fps(fps)
{
    sensor_id = control->sensors->getSensorID( name );
    if( !sensor_id )
	throw std::runtime_error("CameraPlugin: There is no camera by the name of " + name + " in the scene");
    
    camera = dynamic_cast<mars::sim::CameraSensor *>(control->sensors->getSimSensor(sensor_id));
    if( !camera)
	throw std::runtime_error("CameraPlugin: Given sensor name is not a camera");

    width = camera->getConfig().width;
    height = camera->getConfig().height;
    
    control->graphics->addGraphicsUpdateInterface(this);
    
    fps = 1.0 / fps * 1000;
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

