/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SIMULATION_SONAR_TASK_HPP
#define SIMULATION_SONAR_TASK_HPP

#include "simulation/SonarBase.hpp"

namespace simulation {

namespace simulation
{
    //sonar configuration should be implemented in mars
    struct SonarConfig
    {
        SonarConfig()
        {
            motor_speed = 1;
            speed_of_sound = 1500;
            distance_resolution = 0.1;
            min_distance = 1.0;
            max_distance = 50;
            beamwidth_vertical = 35.0/180*M_PI;
            beamwidth_horizontal = 6.0/180*M_PI;
            start_angle = M_PI;
            end_angle = -M_PI;

            default_response_value = 255;
            default_value = 0;

            ping_pong_mode = false;
        };

        float motor_speed;
        float speed_of_sound;
        float distance_resolution;
        float min_distance;
        float max_distance;
        float beamwidth_vertical;
        float beamwidth_horizontal;

        float start_angle;
        float end_angle;

        bool ping_pong_mode;

        unsigned char default_response_value;
        unsigned char default_value;
    };
}

    class Sonar : public SonarBase
    {
	friend class SonarBase;
    protected:
		base::samples::SonarBeam sonar_beam;
		unsigned long node_id;
        simulation::SonarConfig *sonar_config;
        int sonar_motor_direction;
        pthread_mutex_t* sonar_update_mutex;

        void update( double time );


    public:
        Sonar(std::string const& name = "simulation::Sonar");
        Sonar(std::string const& name, RTT::ExecutionEngine* engine);

        double getSonarBeamBearing(unsigned long sonar_mounting_id);
        void updateTopSonarPosition();

        bool getSonarData(base::samples::SonarBeam &sonar_beam);


        /* Dynamic Property setter of left_limit
         */
        virtual bool setLeft_limit(double value);

        /* Dynamic Property setter of maximum_distance
         */
        virtual bool setMaximum_distance(double value);

        /* Dynamic Property setter of ping_pong_mode
         */
        virtual bool setPing_pong_mode(bool value);

        /* Dynamic Property setter of resolution
         */
        virtual bool setResolution(double value);

        /* Dynamic Property setter of right_limit
         */
        virtual bool setRight_limit(double value);
	~Sonar();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         *
         *   task_context "TaskName" do
         *     needs_configuration
         *     ...
         *   end
         */
        // bool configureHook();

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
        // void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        // void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        // void cleanupHook();
    };
}

#endif

