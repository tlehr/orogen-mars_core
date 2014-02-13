/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MarsCamera.hpp"

using namespace simulation;

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
    
    if (! simulation::CameraPlugin::configureHook())
        return false;
    

    

    
    return true;
    
}



bool MarsCamera::startHook()
{
    
    if (! simulation::CameraPlugin::startHook())
        return false;
    
    image = new base::samples::frame::Frame(width,height,8,base::samples::frame::MODE_RGB);
    ro_ptr.reset(image);
    marsImage.resize(width * height);
    
    return true;
    
}



void MarsCamera::updateHook()
{
    
    simulation::CameraPlugin::updateHook();
    

    

    
}



void MarsCamera::errorHook()
{
    
    simulation::CameraPlugin::errorHook();
    

    

    
}



void MarsCamera::stopHook()
{
    
    simulation::CameraPlugin::stopHook();
    

    

    
}



void MarsCamera::cleanupHook()
{
    
    simulation::CameraPlugin::cleanupHook();
    

    

    
}

void MarsCamera::getData()
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
            *(image_dst++) = image_src->r;
            *(image_dst++) = image_src->g;
            *(image_dst++) = image_src->b;
            ++image_src;
        }
    }
    //set attributes
    image->time = getTime();
    image->received_time = image->time;
    image->frame_status = base::samples::frame::STATUS_VALID;
    
    ro_ptr.reset(image);
    _frame.write(ro_ptr);
}
