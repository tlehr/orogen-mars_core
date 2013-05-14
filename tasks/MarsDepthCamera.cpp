/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MarsDepthCamera.hpp"

using namespace simulation;

MarsDepthCamera::MarsDepthCamera(std::string const& name)
    : MarsDepthCameraBase(name)
{
}

MarsDepthCamera::MarsDepthCamera(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsDepthCameraBase(name, engine)
{
}

MarsDepthCamera::~MarsDepthCamera()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MarsDepthCamera.hpp for more detailed
// documentation about them.
bool MarsDepthCamera::configureHook()
{
    
    if (! RTT::TaskContext::configureHook())
        return false;
    return true;
}

bool MarsDepthCamera::startHook()
{
    
    if (! RTT::TaskContext::startHook())
        return false;
    image = new base::samples::DistanceImage(width, height);
    image->setSize(width, height);
    ro_ptr.reset(image);
    return true;
}

void MarsDepthCamera::updateHook()
{
    RTT::TaskContext::updateHook();
}

void MarsDepthCamera::errorHook()
{
    RTT::TaskContext::errorHook();
}

void MarsDepthCamera::stopHook()
{
    RTT::TaskContext::stopHook();
}

void MarsDepthCamera::cleanupHook()
{
    RTT::TaskContext::cleanupHook();
}

void MarsDepthCamera::getData()
{	
    image = ro_ptr.write_access();
    camera->getDepthImage(image->data);
    
    // get the camera info for the intrinsic parameters of the virtual camera
    mars::interfaces::cameraStruct camInfo;
    camera->getCameraInfo(&camInfo);
    image->setIntrinsic(camInfo.scale_x, camInfo.scale_y, 
                        camInfo.center_x, camInfo.center_y );

    //TODO camera might be rotated

    //set attributes
    image->time = base::Time::fromSeconds(lastUpdateTime);
    
    ro_ptr.reset(image);
    _distance_image.write(ro_ptr);
}

