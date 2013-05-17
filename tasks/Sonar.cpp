/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

/*

task_context "SonarTop" do
        output_port("sonar_beam","base::samples::SonarBeam").
            doc('top sonar beam')

        property("left_limit","double",Math::PI)
        property("right_limit","double",-Math::PI)
        property("resolution","double",0.1)
        property("maximum_distance","double",100.0)
        property("ping_pong_mode","bool",false).
            doc('if true ping pong mode is activated')
end

task_context "SonarRear" do
        output_port("sonar_beam","base::samples::SonarBeam").
            doc('rear sonar beam')

        property("left_limit","double",0.7*Math::PI)
        property("right_limit","double",0.3*Math::PI)
        property("resolution","double",0.1)
        property("maximum_distance","double",50.0)
        property("ping_pong_mode","bool",true).
            doc('if true ping pong mode is activated')
end
*/
#include "Sonar.hpp"
#include <mars/sim/ScanningSonar.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>

using namespace simulation;


Sonar::Sonar(std::string const& name)
    : SonarBase(name), sonar_update_mutex(new pthread_mutex_t)
{
}

Sonar::Sonar(std::string const& name, RTT::ExecutionEngine* engine)
    : SonarBase(name, engine), sonar_update_mutex(new pthread_mutex_t)
{
}

Sonar::~Sonar()
{
	pthread_mutex_destroy(sonar_update_mutex);
}

bool Sonar::startHook()
{
    if (!SonarBase::startHook())
        return false;
	pthread_mutex_init(sonar_update_mutex, NULL);
	node_id = control->sensors->getSensorID(_node_name.get());
    return true;
}

void Sonar::updateHook()
{
    SonarBase::updateHook();

    simulation::SonarConfig *config = new simulation::SonarConfig();

    config->start_angle = _left_limit.get();
    config->end_angle = _right_limit.get();
    config->max_distance = _maximum_distance.get();
    config->distance_resolution = _resolution.get();
    config->ping_pong_mode = _ping_pong_mode.get();

    pthread_mutex_lock(sonar_update_mutex);
    sonar_config = config;
    mars::sim::ScanningSonar* sonar = dynamic_cast<mars::sim::ScanningSonar*>(control->sensors->getSimSensor(node_id));
    sonar->setPingPongConfig(config->ping_pong_mode, config->start_angle, config->end_angle);
    pthread_mutex_unlock(sonar_update_mutex);

    if(getSonarData(sonar_beam)){
       if (_sonar_beam.connected())
           _sonar_beam.write(sonar_beam);
    }
    else{
        sonar_beam.beam.clear();
    }

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Sonar.hpp for more detailed
// documentation about them.

// bool Sonar::configureHook()
// {
//     if (! SonarBase::configureHook())
//         return false;
//     return true;
// }

// void Sonar::updateHook()
// {
//     SonarBase::updateHook();
// }
// void Sonar::errorHook()
// {
//     SonarBase::errorHook();
// }
// void Sonar::stopHook()
// {
//     SonarBase::stopHook();
// }
// void Sonar::cleanupHook()
// {
//     SonarBase::cleanupHook();
// }

bool Sonar::getSonarData(base::samples::SonarBeam &sonar_beam){
    //just return false if the simulation is not running
    if(!control->sim->isSimRunning())
      return false;

    mars::interfaces::sReal *sensor_data=0;

    //get data
    pthread_mutex_lock(sonar_update_mutex);
    int n = control->sensors->getSensorData(node_id, &sensor_data);
    pthread_mutex_unlock(sonar_update_mutex);

    if(sensor_data == NULL){
        printf("No Sensor data!\n");
        return false;
    }

    sonar_beam.time = base::Time::now();
    sonar_beam.speed_of_sound = sonar_config->speed_of_sound;
    sonar_beam.beamwidth_horizontal= sonar_config->beamwidth_horizontal;
    sonar_beam.beamwidth_vertical= sonar_config->beamwidth_vertical;
    sonar_beam.sampling_interval = sonar_config->distance_resolution * 2.0 /sonar_config->speed_of_sound;


    if(n == 2){ //Asume Ray Sensor
        sonar_beam.bearing = base::Angle::fromRad(sensor_data[0]);
        int number_of_values = sonar_config->max_distance/sonar_config->distance_resolution+1;
        int index = sensor_data[1]/sonar_config->distance_resolution+0.49999;
        sonar_beam.beam.resize(number_of_values);
        memset(&sonar_beam.beam[0], sonar_config->default_value, number_of_values);

        if(index < number_of_values)
        {
            for(int i = std::max(index-2,0);i<std::min(index+3,number_of_values);i++)
                sonar_beam.beam.at(i) = sonar_config->default_response_value*5.0/(rand()%5+1);
        }
    }else if(n > 2){

        int number_of_values = n-1;
        sonar_beam.beam.resize(number_of_values);
        sonar_beam.bearing = base::Angle::fromRad(sensor_data[0]);
        for(int i=0;i<number_of_values;i++){
            sonar_beam.beam[i] = sensor_data[i+1];
        }

    }else {
        sonar_beam.beam.clear();
        return false;
    }

    if(sensor_data)
    delete[] sensor_data;
    return true;
}

void Sonar::update( double time ) {
}
