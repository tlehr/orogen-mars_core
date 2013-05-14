#ifndef SIMULATION_MARS_TASK_HPP
#define SIMULATION_MARS_TASK_HPP

#include "simulation/MarsBase.hpp"
#include <vector>
#include <mars/data_broker/ReceiverInterface.h>
#include "MarsPlugin.hpp"  

/** From MARS */
//
namespace mars{
    namespace interfaces{
        class SimulatorInterface;
        class PluginInterface;
    };
    namespace app{
        class GraphicsTimer;
    };
    namespace lib_manager {
        class LibManager;
    }
};

namespace simulation {

    class Mars;

    // Argument to pass to startMarsFunc 
    struct MarsArguments
    {
	    Mars* mars;
	    bool enable_gui;
            int controller_port;
	    std::string config_dir;
            bool initialized;
            bool add_floor;
            bool failed_to_init;
            // Raw command line option can be passed to mars
            // using this option vector
            std::vector<Option> raw_options;
    };


    /**
    * Core module that brings up the mars simulation and
    * makes it accessible as a orogen module
    *
    * use subclassing to derive robot specific modules, e.g.
    * 
    * task_context 'RobotSimulation' do
    *         subclasses 'simulation::Mars'
    * ..
    * end
    *
    */
    class Mars : public MarsBase, public mars::data_broker::ReceiverInterface

    {
	friend class MarsBase;
    protected:
    	static mars::app::GraphicsTimer *graphicsTimer;
	static mars::interfaces::SimulatorInterface* simulatorInterface;
	static simulation::Mars* taskInterface;
	static void* startMarsFunc(void *);
        static std::string configDir;
	static bool marsRunning;
    //    unsigned int dbSimTimeId; //Id for the simulation time
        double simTime;
	pthread_t thread_info; 
	static mars::lib_manager::LibManager* libManager;

        mars::interfaces::PluginInterface* multisimPlugin;

        int getOptionCount(const std::vector<Option>& options);

        char** setOptions(const std::vector<Option>& options);

        /* Handler for the loadScene operation
         */
        virtual void loadScene(::std::string const & path);

        std::vector<MarsPlugin*> plugins;

    public:
	/** get the singleton instance of the simulator interface
	 */
	static mars::interfaces::SimulatorInterface* getSimulatorInterface();
	static simulation::Mars* getTaskInterface();

        Mars(std::string const& name = "simulation::Mars");
        Mars(std::string const& name, RTT::ExecutionEngine* engine);

	~Mars();

        void registerPlugin(MarsPlugin* plugin);
        void unregisterPlugin(MarsPlugin* plugin);

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
         bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
         bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called. See README.txt for different
         * triggering options.
         *
         * The warning(), error() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeWarning, RunTimeError and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recovered()
         * allows you to go back into the Running state.  In the second case,
         * the errorHook() will be called instead of updateHook() and in the
         * third case the component is stopped and resetError() needs to be
         * called before starting it again.
         *
         */
         void updateHook();
        

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recovered() to go back in the Runtime state.
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
    
         void receiveData(const mars::data_broker::DataInfo& info,const mars::data_broker::DataPackage& package,int id);
        
    };
}

#endif

