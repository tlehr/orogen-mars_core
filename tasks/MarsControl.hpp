#ifndef SIMULATION_MARS_CONTROL_HPP
#define SIMULATION_MARS_CONTROL_HPP

#ifndef __orogen
#include <string>
#endif

namespace simulation {

   enum Control { NONE = 0, START, PAUSE, RESET, STEP };

   struct Option
   {
       std::string name;
       std::string parameter;

       #ifndef __orogen
       Option() : name(), parameter() {}
       Option(const std::string& _name, const std::string& _parameter) : name(_name), parameter(_parameter){ }
       #endif

   };

}
#endif // SIMULATION_MARS_CONTROL_HPP
