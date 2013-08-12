/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MarsAltimeter.hpp"
#include <mars/sim/RaySensor.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>

using namespace simulation;

MarsAltimeter::MarsAltimeter(std::string const& name)
    : MarsAltimeterBase(name)
{
}

MarsAltimeter::MarsAltimeter(std::string const& name, RTT::ExecutionEngine* engine)
    : MarsAltimeterBase(name, engine)
{
}

MarsAltimeter::~MarsAltimeter()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MarsAltimeter.hpp for more detailed
// documentation about them.

bool MarsAltimeter::configureHook()
{
    if (! MarsAltimeterBase::configureHook())
        return false;
    
    node_id = control->sensors->getSensorID(_node_name.get());
    sonar = dynamic_cast<mars::sim::RaySensor*>(control->sensors->getSimSensor(node_id));
    if(sonar==0){
        fprintf(stderr,"[FATAL] Given node name is not a RaySensor (Name: %s, internal id: %i).",_node_name.get().c_str(), node_id);
        return false;
    }

    return true;
}
bool MarsAltimeter::startHook()
{
    if (! MarsAltimeterBase::startHook())
        return false;
    return true;
}
void MarsAltimeter::updateHook()
{
    MarsAltimeterBase::updateHook();

    std::vector<double> sensor_data = sonar->getSensorData();
    if(sensor_data.size() != 1){
        fprintf(stderr,"Warning Sensor data size should be one for GroundDistance but is: %i\n",sensor_data.size());
    }

    gdist.position = Eigen::Vector3d(0.0,0.0,sensor_data[0]);
    _ground_distance.write(gdist);
}

void MarsAltimeter::errorHook()
{
    MarsAltimeterBase::errorHook();
}
void MarsAltimeter::stopHook()
{
    MarsAltimeterBase::stopHook();
}
void MarsAltimeter::cleanupHook()
{
    MarsAltimeterBase::cleanupHook();
}
