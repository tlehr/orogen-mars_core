/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */
#include <math.h>
#include "MarsHighResRangeFinder.hpp"
#include <mars/interfaces/sim/SensorManagerInterface.h>

using namespace simulation;

MarsHighResRangeFinder::MarsHighResRangeFinder(std::string const& name)
    : MarsHighResRangeFinderBase(name), mCameras(), mFirstCameraAdded(false)
{
}

MarsHighResRangeFinder::MarsHighResRangeFinder(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsHighResRangeFinderBase(name, engine), mCameras(), mFirstCameraAdded(false)
{
}

MarsHighResRangeFinder::~MarsHighResRangeFinder()
{
    std::vector<Camera*>::iterator it = mCameras.begin();
    for(; it != mCameras.end(); ++it) {
        delete *it;
    }
    mCameras.clear();
}

bool MarsHighResRangeFinder::addCamera(::std::string const & name, double orientation)
{
    long sensor_id = control->sensors->getSensorID( name );
    if( !sensor_id ){
	    RTT::log(RTT::Error) << "There is no camera by the name of " << name << " in the scene" << RTT::endlog();
        return false;
    }
    
    mars::sim::CameraSensor* cam_sensor = dynamic_cast<mars::sim::CameraSensor *>(control->sensors->getSimSensor(sensor_id));
    if( !cam_sensor){
        RTT::log(RTT::Error) << "CameraPlugin: Given sensor name is not a camera" << RTT::endlog();
    }
    
    Camera* camera = new Camera(sensor_id, cam_sensor, orientation, name);
    calcCamParameters(camera);
    mCameras.push_back(camera);
    return true;
}
    
    
void MarsHighResRangeFinder::calcCamParameters(Camera* camera) {
    
    int width = camera->cam_sensor_info.width;
    int height = camera->cam_sensor_info.height;
    double opening_width = camera->cam_sensor_info.opening_width;
    double opening_height = camera->cam_sensor_info.opening_height;
    
    // Calculate starting pixel (bottom left within the image plane)
    camera->pixel_per_rad_horizontal = width / ((opening_width / 180.0) * M_PI);
    camera->pixel_per_rad_vertical = height / ((opening_height / 180.0) * M_PI);
    
    // Sets the borders.    
    camera->lower_pixel = _lower_limit.get() * camera->pixel_per_rad_vertical + height/2.0;
    camera->upper_pixel = _upper_limit.get() * camera->pixel_per_rad_vertical + height/2.0;
    camera->left_pixel = _left_limit.get() * camera->pixel_per_rad_horizontal + width/2.0;
    camera->right_pixel = _right_limit.get() * camera->pixel_per_rad_horizontal + width/2.0;
    
    // Regard image borders and updates the limits.
    if(camera->lower_pixel < 0) {
        RTT::log(RTT::Warning) << "Lower limit exceeds the image plane, will be scaled down by " << 
                std::fabs(camera->lower_pixel) << " pixel" << RTT::endlog();
        camera->lower_pixel = 0;
    }
    if(camera->upper_pixel > height) {
        RTT::log(RTT::Warning) << "Upper limit exceeds the image plane, will be scaled down by " << 
                camera->upper_pixel - height << " pixel" << RTT::endlog();
        camera->upper_pixel = height;
    }
    if(camera->left_pixel < 0) {
        RTT::log(RTT::Warning) << "Left limit exceeds the image plane, will be scaled down by " << 
                std::fabs(camera->left_pixel) << " pixel" << RTT::endlog();
        camera->left_pixel = 0;
    }
    if(camera->right_pixel > width) {
        RTT::log(RTT::Warning) << "Right limit exceeds the image plane, will be scaled down by " << 
                camera->right_pixel - width << " pixel" << RTT::endlog();
        camera->right_pixel = width;
    }
   
    camera->v_steps_pixel = _resolution_vertical.get() * camera->pixel_per_rad_vertical;
    camera->h_steps_pixel = _resolution_horizontal.get() * camera->pixel_per_rad_horizontal;
    
    RTT::log(RTT::Info) << "Camera " << camera->sensor_id << " (" << camera->name << ") added" << RTT::endlog();
    RTT::log(RTT::Info) << "opening_width " << opening_width << ", opening_height " << opening_height << RTT::endlog();
    RTT::log(RTT::Info) << "Horizontal: Every " << camera->h_steps_pixel << " pixel " << " will be used from " <<
            camera->left_pixel << " to " << camera->right_pixel << RTT::endlog();
    RTT::log(RTT::Info) << "Vertical: Every " << camera->v_steps_pixel << " pixel " << " will be used from " <<
            camera->lower_pixel << " to " << camera->upper_pixel << RTT::endlog();
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
        
    // Adds this camera to the list of cameras once.
    if(!mFirstCameraAdded) {
        Camera* camera = new Camera(sensor_id, this->camera, 0.0, "velodyne0");
        calcCamParameters(camera);
        mCameras.push_back(camera);
        mFirstCameraAdded = true;
    }
    
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
    Eigen::Matrix<double, 3, 1> scene_p;
    size_t x_t = 0, y_t = 0;
    int counter = 0;
    
    // Fill pointcloud.
    base::samples::Pointcloud pointcloud;
    pointcloud.time = getTime();
    std::vector<Camera*>::iterator it = mCameras.begin();
    for(; it < mCameras.end(); ++it) {
        counter = 0;
        // Request image and store it within the base distance image.
        (*it)->camera_sensor->getDepthImage((*it)->image->data);
        
        for(double y = (*it)->lower_pixel; y < (*it)->upper_pixel; y += (*it)->v_steps_pixel) {
            for(double x = (*it)->left_pixel; x < (*it)->right_pixel; x += (*it)->h_steps_pixel) {
                // Pixel contains a distance value between min and max
                // and lies within the image plane.
                x_t = (size_t) x;
                y_t = (size_t) y;
                if((*it)->image->data[x_t+y_t*(*it)->image->width] >= _minimum_distance.get() &&
                        (*it)->image->data[x_t+y_t*(*it)->image->width] <= _maximum_distance.get() &&
                        (*it)->image->getScenePoint<double>( (size_t) x, (size_t) y, scene_p )) {
                        
                    // Rotate camera around the y axis.
                    scene_p = (*it)->orientation * scene_p;
                    // Transforms to robot frame (x: front, z: up) and adds to the pointcloud.
                    scene_p = base::Vector3d(scene_p[2], -scene_p[0], -scene_p[1]);
                    pointcloud.points.push_back(scene_p);
                    counter++;
                }
            }
        }
        RTT::log(RTT::Info) << counter << " points have been added by camera " << (*it)->name <<
                " ID "<< (*it)->sensor_id << RTT::endlog();
    }
    _pointcloud.write(pointcloud);
    
    // Fill multi level scan.
    velodyne_lidar::MultilevelLaserScan multi_level_scan;
    multi_level_scan.min_range = _minimum_distance.get() * 1000;
    multi_level_scan.max_range = _maximum_distance.get() * 1000;
    
    it = mCameras.begin();
    double verticalOpeningAngle = 30.0 / 180.0 * M_PI;
    double verticalStartAngle = -15.0 / 180.0 * M_PI;
    double scansVertical = 30;
    double scansHorizontal = 1000;

    double stepHorizontal = M_PI *2 / scansHorizontal;
    double stepVertical = verticalOpeningAngle / scansVertical;
    
    
    double curAngle = 0;
    
    float distance = 0;
        
    int height = (*it)->height;
    int width = (*it)->width;

    double b_x = (width / 2.0) / tan(M_PI/4.0); 
    double b_y = (height / 2.0) / tan(M_PI/4.0); 
    
    // Run through the image horizontally.  
    for(double horAngle = 0; horAngle < M_PI * 2; horAngle += stepHorizontal) {
        velodyne_lidar::MultilevelLaserScan::VerticalMultilevelScan vertical_multi_level_scan;
        curAngle += stepHorizontal;
        
        if(curAngle > M_PI/2.0)
        {
            counter++;
            std::cout << "Camera " << counter << std::endl;
            curAngle -= M_PI/2.0;
            it++;
            if(it == mCameras.end())
                break;
            
            (*it)->camera_sensor->getDepthImage((*it)->image->data);
        }

//         std::cout << "tan(curAngle "<<curAngle<< "  - M_PI / 4.0) " << tan(curAngle - M_PI / 4.0) << " * b_x " << b_x << " + (width / 2.0) " <<  (width / 2.0)  << std::endl;
        int x = tan(curAngle - M_PI / 4.0) * b_x + (width / 2.0);
        
        for(double verAngle = verticalStartAngle; verAngle < verticalOpeningAngle + verticalStartAngle; verAngle += stepVertical) {

            int y = tan(verAngle) * b_y + (height / 2.0);
//             std::cout << " X " << x << " Y " << y << std::endl; 

//            distance = (*it)->image->data[y * width + x];
            std::cout << "distance foo " << distance << std::endl;
            if((*it)->image->getScenePoint( x, y, scene_p ))
            {                
// //                std::cout << "in Image " << scene_p.transpose()<<  std::endl;
                distance = scene_p.norm();
            }
            else
            {
                //invalid
                distance = 0;
            }

            velodyne_lidar::MultilevelLaserScan::SingleScan single_scan;
            single_scan.range = distance * 1000;
            vertical_multi_level_scan.vertical_scans.push_back(single_scan);
        }            
        vertical_multi_level_scan.vertical_start_angle = base::Angle::fromRad(verticalStartAngle);
        vertical_multi_level_scan.vertical_angular_resolution = stepVertical;
        vertical_multi_level_scan.horizontal_angle = base::Angle::fromRad(-horAngle + M_PI / 4.0);

        multi_level_scan.horizontal_scans.push_back(vertical_multi_level_scan);
    }
    std::cout << "End " << std::endl;
    _laser_scans.write(multi_level_scan);
}
