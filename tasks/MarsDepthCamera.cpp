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
    
    if (! MarsDepthCameraBase::configureHook())
        return false;
    return true;
}

bool MarsDepthCamera::startHook()
{
        std::cout << "MarsDepthCamera startHook" <<  std::endl;
    if (! MarsDepthCameraBase::startHook())
        return false;
    image = new base::samples::DistanceImage(width, height);
    image->setSize(width, height);
    ro_ptr.reset(image);
    return true;
}

void MarsDepthCamera::updateHook()
{
    MarsDepthCameraBase::updateHook();
}

void MarsDepthCamera::errorHook()
{
    MarsDepthCameraBase::errorHook();
}

void MarsDepthCamera::stopHook()
{
    MarsDepthCameraBase::stopHook();
}

void MarsDepthCamera::cleanupHook()
{
    MarsDepthCameraBase::cleanupHook();
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
    //image->time = base::Time::fromSeconds(lastUpdateTime);
    image->time = getTime();
    
    ro_ptr.reset(image);
    
    _distance_image.write(ro_ptr);
}
