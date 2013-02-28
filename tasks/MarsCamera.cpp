/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MarsCamera.hpp"
#include "MarsPlugin.hpp"
#include <mars/sim/CameraSensor.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>
#include "CameraPlugin.hpp"

using namespace simulation;

class MarsCameraPlugin : public ::CameraPlugin
{
    MarsCamera &task;
    base::samples::frame::Frame *image;
    std::vector<mars::sim::Pixel> marsImage;
    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> ro_ptr;
public:
    MarsCameraPlugin(MarsCamera& task, const std::string& name, double fps)
	: CameraPlugin(name, fps), task( task )
    {
	image = new base::samples::frame::Frame(width,height,8,base::samples::frame::MODE_RGB);
	ro_ptr.reset(image);
	marsImage.resize(width * height);
    }

    
    virtual void getData()
    {	
	camera->getImage(marsImage);

	image = ro_ptr.write_access();
	
	//copy image data
	//data format is ARGB therefore we have to skip every 4th byte
	//to convert it into RGB
	//image is flipped
	const mars::sim::Pixel *image_src = marsImage.data();
	uint8_t *image_dst = image->getImagePtr();
	for(int i=height-1;i>=0;--i)
	{
	    image_src = marsImage.data()+width*i;
	    for(int i2=0;i2<width;++i2)
	    {
		*(image_dst++) = image_src->b;
		*(image_dst++) = image_src->g;
		*(image_dst++) = image_src->r;
		++image_src;
	    }
	}
	//set attributes
	image->time = base::Time::fromSeconds(lastUpdateTime);
	image->received_time = image->time;
	image->frame_status = base::samples::frame::STATUS_VALID;
	
	ro_ptr.reset(image);
	task._frame.write(ro_ptr);
    }
};


MarsCamera::MarsCamera(std::string const& name)
    : MarsCameraBase(name)
{
}

MarsCamera::MarsCamera(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsCameraBase(name, engine)
{
}

MarsCamera::~MarsCamera()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MarsCamera.hpp for more detailed
// documentation about them.




bool MarsCamera::configureHook()
{
    
    if (! RTT::TaskContext::configureHook())
        return false;
    
    return true;
    
}



bool MarsCamera::startHook()
{
    
    if (! RTT::TaskContext::startHook())
        return false;
    
    cameraPlugin = new MarsCameraPlugin(*this, _name.get(), _fps.get());

    return true;
    
}



void MarsCamera::updateHook()
{
    
    RTT::TaskContext::updateHook();
    

    

    
}



void MarsCamera::errorHook()
{
    
    RTT::TaskContext::errorHook();
    

    

    
}



void MarsCamera::stopHook()
{
    
    RTT::TaskContext::stopHook();
    

    

    
}



void MarsCamera::cleanupHook()
{
    
    RTT::TaskContext::cleanupHook();
    

    

    
}

