/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROCK_GAZEBO_SENSORTASK_TASK_HPP
#define ROCK_GAZEBO_SENSORTASK_TASK_HPP

#include "rock_gazebo/SensorTaskBase.hpp"
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/Node.hh>

namespace rock_gazebo{

    /*! \class SensorTask
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     *
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','rock_gazebo::SensorTask')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class SensorTask : public SensorTaskBase
    {
	friend class SensorTaskBase;
    protected:
        gazebo::sensors::SensorPtr mSensor;

    public:
        /** TaskContext constructor for SensorTask
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        SensorTask(std::string const& name = "rock_gazebo::SensorTask");

        /** TaskContext constructor for SensorTask
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         *
         */
        SensorTask(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of SensorTask
         */
	~SensorTask();

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

        typedef gazebo::physics::ModelPtr ModelPtr;
        typedef gazebo::physics::LinkPtr LinkPtr;
        virtual void setGazeboModel(ModelPtr model, sdf::ElementPtr sdfSensor);

    protected:
        template<typename M, typename T>
        void topicSubscribe(void(T::*_fp)(const boost::shared_ptr< M const > &), std::string topicName)
        {
            subscriber = node->Subscribe(topicName, _fp, static_cast<T*>(this));
            gzmsg << getName() << ": subscribed to gazebo topic ~/" + topicName << std::endl;
        }

        ModelPtr gazeboModel;
        LinkPtr gazeboLink;
        sdf::ElementPtr sdfSensor;
        gazebo::transport::SubscriberPtr subscriber;
        gazebo::transport::NodePtr node;
        std::mutex readMutex;
        std::string sensorFullName;
        std::string baseTopicName;
    };
}

#endif

