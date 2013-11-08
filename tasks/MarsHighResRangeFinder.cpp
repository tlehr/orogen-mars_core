/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

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
    
    float distance = 0;
    for(it = mCameras.begin(); it < mCameras.end(); ++it) {
        counter = 0;
        // Request image and store it within the base distance image.
        (*it)->camera_sensor->getDepthImage((*it)->image->data);
          
        // Run through the image horizontally.  
        for(double x = (*it)->left_pixel; x < (*it)->right_pixel; x += (*it)->h_steps_pixel) {
        
            velodyne_lidar::MultilevelLaserScan::VerticalMultilevelScan vertical_multi_level_scan;
            // Calculate the current horizontal angle, camera with orientation 0 
            // would be (-45,45], orientation 90 (45,135] ...
            //vertical_multi_level_scan.horizontal_angle.rad = 
            //        (x - (*it)->width/2.0) / (*it)->pixel_per_rad_horizontal - (*it)->rotation_y;
            // TODO: Do the same angle calculation like you did for the horizontal angle.

        
            // Run through the image vertically.
            bool horizontal_angle_calculated = false;
            Eigen::Matrix<double, 3, 1> scene_p_cpy;
            double angle_rad = 0.0;
            // The real vertical resolution has to be calculated using the first and the last
            // vertical vector and the number of vectors - 1
            Eigen::Matrix<double, 3, 1> first_vertical_scene_p, last_vertical_scene_p;
            
            for(double y = (*it)->lower_pixel; y < (*it)->upper_pixel; y += (*it)->v_steps_pixel) {
                x_t = (size_t) x;
                y_t = (size_t) y;
                
                distance = 0;
                
                if((*it)->image->getScenePoint<double>( x_t, y_t, scene_p )) {
                    distance = scene_p.norm();
                    
                    // TODO: What happens if the first and/or the last point got NaN?
                    // Answer: Not scaled values are used, enough for calculations.
                    // Store first point.
                    
                    
                    // The first point of each vertical line has to be used to calculate
                    // the real horizontal angle of the point.
                    if(!horizontal_angle_calculated) {
                        
                        // Calculate angle in radians between the z-axis and the scene point.
                        scene_p_cpy = scene_p;
                        scene_p_cpy.y() = 0;
                        angle_rad = acos(scene_p_cpy.dot(Eigen::Vector3d::UnitZ()) / scene_p_cpy.norm());
                        // Assign prefix.
                        if(scene_p_cpy.x() < 0) {
                            angle_rad *= -1;
                        }
                        // Store in the MultilevelLaserScan structure.
                        vertical_multi_level_scan.horizontal_angle.rad = angle_rad - (*it)->rotation_y;
                        horizontal_angle_calculated = true;
                    }
                } else {
                    RTT::log(RTT::Debug) << "NaN-distance have been returned by camera " << 
                        (*it)->name  << " for pixel (" << x_t << ", " << y_t << ")" << RTT::endlog();
                }
                
                // Store first and last vertical vector.                
                if(y == (*it)->lower_pixel) {
                    first_vertical_scene_p = scene_p;
                }
                last_vertical_scene_p = scene_p;
                
                
                if(distance < _minimum_distance.get() || distance > _maximum_distance.get()) {
                    RTT::log(RTT::Debug) << "Distance " << distance << " have exceeded the limit for pixel (" << 
                        x_t << ", " << y_t << ")" << RTT::endlog();
                    distance = 0;
                }
                
  
                velodyne_lidar::MultilevelLaserScan::SingleScan single_scan;
                single_scan.range = distance * 1000;
                vertical_multi_level_scan.vertical_scans.push_back(single_scan);
                counter++;
            }
                        
            // Calculate real vertical start angle.
            #if 1
            first_vertical_scene_p.x() = 0;
            last_vertical_scene_p.x() = 0;
            vertical_multi_level_scan.vertical_start_angle.rad = 
                    acos(first_vertical_scene_p.dot(Eigen::Vector3d::UnitZ()) / first_vertical_scene_p.norm());
            // Assign prefix.
            if(first_vertical_scene_p.y() > 0) {
                vertical_multi_level_scan.vertical_start_angle.rad *= -1;
            }
            
            // Calculate real vertical resolution.
            double angle_between_vertical_vectors = 
                    acos(first_vertical_scene_p.dot(last_vertical_scene_p) / first_vertical_scene_p.norm());
            vertical_multi_level_scan.vertical_angular_resolution = angle_between_vertical_vectors / (vertical_multi_level_scan.vertical_scans.size() - 1);
            
            #else
            
            vertical_multi_level_scan.vertical_start_angle.rad = 
                    ((*it)->lower_pixel - (*it)->height/2.0) / (*it)->pixel_per_rad_vertical;
            vertical_multi_level_scan.vertical_angular_resolution = _resolution_vertical.get();
            
            #endif
            
            RTT::log(RTT::Info) << "Vertical start angle " << vertical_multi_level_scan.vertical_start_angle.rad << 
                    ", vertical resolution " << vertical_multi_level_scan.vertical_angular_resolution <<
                    ", angle between upper and lower vectors " << angle_between_vertical_vectors <<
                    "number of vertical vectors " << vertical_multi_level_scan.vertical_scans.size() << RTT::endlog();
              
            multi_level_scan.horizontal_scans.push_back(vertical_multi_level_scan);
        }
        RTT::log(RTT::Info) << counter << " points have been added by camera " << (*it)->name <<
                " ID "<< (*it)->sensor_id << RTT::endlog();
    }
    _laser_scans.write(multi_level_scan);
}
