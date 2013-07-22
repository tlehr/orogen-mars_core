/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MarsHighResRangeFinder.hpp"

using namespace simulation;

MarsHighResRangeFinder::MarsHighResRangeFinder(std::string const& name)
    : MarsHighResRangeFinderBase(name)
{
}

MarsHighResRangeFinder::MarsHighResRangeFinder(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsHighResRangeFinderBase(name, engine)
{
}

MarsHighResRangeFinder::~MarsHighResRangeFinder()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MarsHighResRangeFinder.hpp for more detailed
// documentation about them.

bool MarsHighResRangeFinder::configureHook()
{
    if (! MarsHighResRangeFinderBase::configureHook())
        return false;
        
    return true;
}
bool MarsHighResRangeFinder::startHook()
{
    if (! MarsHighResRangeFinderBase::startHook())
        return false;
        
    // Calculate starting pixel (bottom left within the image plane)
    struct mars::sim::CameraConfigStruct config = camera->getConfig();
    pixel_per_rad_horizontal = config.width / ((config.opening_width / 180.0) * M_PI);
    pixel_per_rad_vertical = config.height / ((config.opening_height / 180.0) * M_PI);
    
    // Sets the borders.
    mars::interfaces::cameraStruct camInfo;
    camera->getCameraInfo(&camInfo);
    
    lower_pixel = _lower_limit.get() * pixel_per_rad_vertical + config.height/2.0;
    upper_pixel = _upper_limit.get() * pixel_per_rad_vertical + config.height/2.0;
    left_pixel = _left_limit.get() * pixel_per_rad_horizontal + config.width/2.0;
    right_pixel = _right_limit.get() * pixel_per_rad_horizontal + config.width/2.0;
    
    if(lower_pixel < 0) {
        std::cout << "Lower limit exceeds the image plane, will be scaled down by " << 
                std::fabs(lower_pixel) << " pixel" << std::endl;
        lower_pixel = 0;
    }
    if(upper_pixel > config.height) {
        std::cout << "Upper limit exceeds the image plane, will be scaled down by " << 
                upper_pixel - config.height << " pixel" << std::endl;
        upper_pixel = config.height;
    }
    if(left_pixel < 0) {
        std::cout << "Left limit exceeds the image plane, will be scaled down by " << 
                std::fabs(left_pixel) << " pixel" << std::endl;
        left_pixel = 0;
    }
    if(right_pixel > config.width) {
        std::cout << "Right limit exceeds the image plane, will be scaled down by " << 
                right_pixel - config.width << " pixel" << std::endl;
        right_pixel = config.width;
    }
   
    v_steps = _resolution_vertical.get() * pixel_per_rad_vertical;
    h_steps = _resolution_horizontal.get() * pixel_per_rad_horizontal;
    
    return true;
}

void MarsHighResRangeFinder::updateHook()
{
    MarsHighResRangeFinderBase::updateHook();
}
void MarsHighResRangeFinder::errorHook()
{
    MarsHighResRangeFinderBase::errorHook();
}
void MarsHighResRangeFinder::stopHook()
{
    MarsHighResRangeFinderBase::stopHook();
}
void MarsHighResRangeFinder::cleanupHook()
{
    MarsHighResRangeFinderBase::cleanupHook();
}

void MarsHighResRangeFinder::getData()
{	
    MarsDepthCamera::getData(); // Requests the distance image.
    
    Eigen::Matrix<double, 3, 1> scene_p;
    base::samples::Pointcloud pointcloud;
    size_t x_t = 0, y_t = 0; 
    for(double y = lower_pixel; y < upper_pixel; y += v_steps) {
        for(double x = left_pixel; x < right_pixel; x += h_steps) {
            // Pixel contains a distance value between min and max
            // and lies within the image plane.
            x_t = (size_t) x;
            y_t = (size_t) y;
            if(image->data[x_t+y_t*image->width] >= _minimum_distance.get() &&
                    image->data[x_t+y_t*image->width] <= _maximum_distance.get() &&
                    image->getScenePoint<double>( (size_t) x, (size_t) y, scene_p )) {
                // Transforms to robot frame (x: front, z: up) and adds to the pointcloud.
                scene_p = base::Vector3d(scene_p[2], -scene_p[0], -scene_p[1]);
                pointcloud.points.push_back(scene_p);
            }
        }
    }
    _pointcloud.write(pointcloud);
}
