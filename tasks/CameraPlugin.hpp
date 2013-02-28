#ifndef CAMERAPLUGIN_HPP
#define CAMERAPLUGIN_HPP

#include "MarsPlugin.hpp"
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/sim/CameraSensor.h>

class CameraPlugin : simulation::MarsPlugin, public mars::interfaces::GraphicsUpdateInterface
{
protected:
    long sensor_id;
    int width;
    int height;
    double lastUpdateTime;
    double lastGrabTime;
    mars::sim::CameraSensor *camera;
    double fps;
private:
    virtual void postGraphicsUpdate(void );
    virtual void update(mars::interfaces::sReal time_ms);
public:
    CameraPlugin(const std::string& name, double fps);

    /**
     * This method gets called every time when the sensor has new data
     * AND an amount of time, matching fps has passed.
     * Note that the simualtor runs with 100 Herz and that you
     * should choose the fps accordingly.
     * */
    virtual void getData() = 0;
};

#endif // CAMERAPLUGIN_HPP
