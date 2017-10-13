/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */
//======================================================================================
// Brazilian Institute of Robotics 
// Authors: Thomio Watanabe
// Date: December 2014
//====================================================================================== 
#ifndef ROCK_GAZEBO_MODELTASK_TASK_HPP
#define ROCK_GAZEBO_MODELTASK_TASK_HPP

#include "rock_gazebo/ModelTaskBase.hpp"	
#include <base/commands/Joints.hpp>
#include <gazebo/physics/physics.hh>

namespace rock_gazebo {
    class ModelTask : public ModelTaskBase
    {
        public:
            typedef gazebo::physics::Joint_V Joint_V;
            typedef gazebo::physics::Link_V Link_V;
            typedef gazebo::physics::ModelPtr ModelPtr;
            typedef gazebo::physics::JointPtr JointPtr;
            typedef gazebo::physics::LinkPtr LinkPtr;
	        
        friend class ModelTaskBase;
        private:
            ModelPtr model;
            sdf::ElementPtr sdf;

            Joint_V gazebo_joints;

            base::samples::Joints joints_in;
            void setupJoints();

            typedef base::samples::Wrench Wrench;
            typedef base::samples::RigidBodyState RigidBodyState;
            typedef base::samples::RigidBodyAcceleration RigidBodyAcceleration;
            typedef RTT::InputPort<Wrench> WrenchInPort;
            typedef RTT::OutputPort<RigidBodyState> RBSOutPort;
            typedef RTT::OutputPort<RigidBodyAcceleration> RBAOutPort;

            struct InternalLinkExport : public LinkExport
            {
                LinkPtr source_link_ptr;
                LinkPtr target_link_ptr;
                std::string rba_port_name;
                std::string wrench_port_name;
                WrenchInPort* wrench_port;
                RBSOutPort* port;
                RBAOutPort* rba_port;
                base::Time last_update;
                base::samples::Wrench wrench_in;
                base::samples::Wrench lastWrenchCommand;
                base::Time lastWrenchCommandTime;

                InternalLinkExport()
                    : wrench_port(nullptr)
                    , port(nullptr)
                    , rba_port(nullptr) { }

                InternalLinkExport(InternalLinkExport const& link) = default;

                InternalLinkExport(LinkExport const& src)
                    : LinkExport(src)
                    , wrench_port(nullptr)
                    , port(nullptr)
                    , rba_port(nullptr) { }
            };

            typedef std::vector<InternalLinkExport> LinkExportSetup;
            LinkExportSetup link_export_setup;

            typedef RTT::InputPort<base::samples::Joints>  JointsInputPort;
            typedef RTT::OutputPort<base::samples::Joints> JointsOutputPort;
            struct InternalJointExport
            {
                bool permanent;
                base::Time port_period;
                std::vector<JointPtr> gazebo_joints;
                std::vector<std::string> expected_names;

                base::samples::Joints joints_in;
                JointsInputPort* in_port;
                base::samples::Joints joints_out;
                JointsOutputPort* out_port;
                base::Time last_command;

                InternalJointExport()
                    : permanent(false)
                    , in_port(nullptr)
                    , out_port(nullptr)
                {
                }

                void addJoint(JointPtr joint, std::string name);
            };

            typedef std::vector<InternalJointExport> JointExportSetup;
            JointExportSetup joint_export_setup;

            void setupLinks();
            void warpModel(base::samples::RigidBodyState const& modelPose);
            void updateLinks(base::Time const& time);
            void writeExportedJointSamples(base::Time const& time, InternalJointExport& exported_joint);
            void readExportedJointCmd(base::Time const& time, InternalJointExport& exported_joint);
            void updateModelPose(base::Time const& time);

            std::string checkExportedLinkElements(std::string, std::string, std::string);

            void releaseLinks();
            void releaseJoints();

        protected:

        public:
            void setGazeboModel(WorldPtr, ModelPtr);

            bool startHook();
            void updateHook();
            bool configureHook();
            void cleanupHook();

		    /** TaskContext constructor for ModelTask
		     * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
		     * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
		     */
            ModelTask(std::string const& name = "gazebo::ModelTask");

		    /** TaskContext constructor for ModelTask 
		     * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
		     * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
		     * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
		     */
            ModelTask(std::string const& name, RTT::ExecutionEngine* engine);

		    /** Default deconstructor of ModelTask
		     */
            ~ModelTask();
    };
}

#endif

