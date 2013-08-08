/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SIMULATION_MARSHIGHRESRANGEFINDER_TASK_HPP
#define SIMULATION_MARSHIGHRESRANGEFINDER_TASK_HPP

#include "simulation/MarsHighResRangeFinderBase.hpp"

namespace simulation {

    /*! \class MarsHighResRangeFinder 
     * Allows to simulate 360deg laserscanners using distance images to increase performance.
     * At the moment each camera sensor in MARS provides an opening angle of 90deg, so 
     * four cameras are required to cover the complete 360deg. Initially a single camera
     * is used to create the pointcloud and with 'addCamera()' further cameras can be
     * added. The following example shows how to start this sensor 
     * using two cameras in ruby:
     *   \code
     *   velodyne = TaskContext.get 'mars_velodyne'
     *   velodyne.name = 'velodyne'
     *   velodyne.configure
     *   velodyne.start   
     *   velodyne.addCamera('velodyne90',90);
     *   \endcode
     * See the documentation of addCamera() for a scene file example.\n
     * The z-axis of the image scene coordinate system points towards the image plane,
     * the x-axis points right and the y-axis down.
     */
    class MarsHighResRangeFinder : public MarsHighResRangeFinderBase
    {
	friend class MarsHighResRangeFinderBase;
    protected:
        struct Camera {
            /**
             * Stores the informations of all cameras which are used to simulate this sensor.
             * \param id Id of the sensor in MARS.
             * \param cam New CameraSensor which has been added.
             * \param rot_y Used to rotate all scene points which have been extracted from the image
             * to match the orientation of the camera. Pass the same rotation which has been used 
             * in the scene file for the orientation_offset-yaw-value of the camera. In the 
             * 'velodyne90' example below that would be 90.
             */
            Camera(long id, mars::sim::CameraSensor* cam, double rot_y) : 
                    sensor_id(id), camera_sensor(cam), name()
            {
                width = camera_sensor->getConfig().width;
                height = camera_sensor->getConfig().height;
                image = new base::samples::DistanceImage(width, height);
                image->setSize(width, height);
                camera_sensor->getCameraInfo(&cam_info);
                image->setIntrinsic(cam_info.scale_x, cam_info.scale_y, 
                        cam_info.center_x, cam_info.center_y );
                cam_sensor_info = camera_sensor->getConfig();
                // Degree to rad.
                // INVERTS ROTATION, has to be done to match the viewing direction of the MARS camera.
                rot_y = (-rot_y / 180.0) * M_PI;
                Eigen::AngleAxis<double> rot(rot_y, Eigen::Vector3d(0.0, 1.0, 0.0));
                orientation = rot;
            }
            
            ~Camera() {
                delete image; image = NULL;
            }
            
            long sensor_id;
            mars::sim::CameraSensor* camera_sensor;
            int width;
            int height;
            mars::interfaces::cameraStruct cam_info;
            mars::sim::CameraConfigStruct cam_sensor_info;
            base::samples::DistanceImage* image;
            // Rotation of the camera around the y-axis within the camera frame.
            Eigen::Quaternion<double, Eigen::DontAlign> orientation;
            
            double pixel_per_rad_horizontal;
            double pixel_per_rad_vertical;
            double lower_pixel;
            double upper_pixel;
            double left_pixel;
            double right_pixel;
            double v_steps;
            double h_steps;
            std::string name;
        };
    
        std::vector<Camera*> cameras;
        
        /**
         * Loads another camera from the scene file which will be used for pointcloud creation.
         * Within the scene file the yaw angle has to be used to create a full 360deg sensor.\n
         * E.g. the following scene file shows the front and the left camera.\n
         *   \code
         *   <sensor name="velodyne" type="CameraSensor">
         *     <index>1</index>
         *     <rate>10</rate>
         *     <attached_node>1</attached_node>
         *     <depth_image>1.0</depth_image>
         *     <show_cam hud_idx="1">1.0</show_cam>
         *     <position_offset x="-0.03838" y="0.00122" z="0.527"/>
         *     <orientation_offset yaw="0" pitch="0" roll="0"/>
         *   </sensor>
         *   <sensor name="velodyne90" type="CameraSensor">
         *     <index>2</index>
         *     <rate>10</rate>
         *     <attached_node>1</attached_node>
         *     <depth_image>1.0</depth_image>
         *     <show_cam hud_idx="2">1.0</show_cam>
         *     <position_offset x="-0.03838" y="0.00122" z="0.527"/>
         *     <orientation_offset yaw="90" pitch="0" roll="0"/>
         *   </sensor>
         *   \endcode
         * \param name Name of the camera within the scene file. In the example above it would be 'velodyne90'.
         * \param orientation Orientation of the camera around the y-axis within the camera frame.
         */
        virtual bool addCamera(::std::string const & name, double orientation);
        
        /**
         * Calculates which pixel will be converted into scene points in regard to the 
         * opening angle and the scan resolution.
         */
        void calcCamParameters(Camera* camera);

    public:
        /** TaskContext constructor for MarsHighResRangeFinder
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        MarsHighResRangeFinder(std::string const& name = "simulation::MarsHighResRangeFinder");

        /** TaskContext constructor for MarsHighResRangeFinder 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        MarsHighResRangeFinder(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of MarsHighResRangeFinder
         */
         ~MarsHighResRangeFinder();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
        
        /**
         * Requests the distance image by calling MarsDepthCamera::getData() 
         * and generates the pointcloud using the image data.
         */
        virtual void getData();
    };
}

#endif

