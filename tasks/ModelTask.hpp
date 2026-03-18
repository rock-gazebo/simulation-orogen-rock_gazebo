/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */
//======================================================================================
// Brazilian Institute of Robotics
// Authors: Thomio Watanabe
// Date: December 2014
//======================================================================================
#ifndef ROCK_GAZEBO_MODELTASK_TASK_HPP
#define ROCK_GAZEBO_MODELTASK_TASK_HPP

#include "rock_gazebo/ModelTaskBase.hpp"
#include "rock_gazebo/PluginTaskI.hpp"
#include <base/commands/Joints.hpp>
#include <gz/sim/Entity.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

namespace rock_gazebo {
    class ModelTask : public ModelTaskBase {
    public:
        typedef std::vector<gz::sim::Entity> Joint_V;
        typedef std::vector<gz::sim::Entity> Link_V;

        friend class ModelTaskBase;

    private:
        gz::sim::Entity m_model = gz::sim::kNullEntity;
        gz::sim::EntityComponentManager* m_ecm = nullptr;
        sdf::ElementPtr m_sdf;

        base::samples::Joints m_joints_in;

        typedef base::samples::Wrench Wrench;
        typedef base::samples::RigidBodyState RigidBodyState;
        typedef base::samples::RigidBodyAcceleration RigidBodyAcceleration;
        typedef RTT::InputPort<Wrench> WrenchInPort;
        typedef RTT::OutputPort<RigidBodyState> RBSOutPort;
        typedef RTT::OutputPort<RigidBodyAcceleration> RBAOutPort;

        struct InternalLinkExport : public LinkExport {
            gz::sim::Entity source_link_ptr;
            gz::sim::Entity target_link_ptr;
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
                , rba_port(nullptr)
            {
            }

            InternalLinkExport(InternalLinkExport const& link) = default;

            InternalLinkExport(LinkExport const& src)
                : LinkExport(src)
                , wrench_port(nullptr)
                , port(nullptr)
                , rba_port(nullptr)
            {
            }
        };

        typedef std::vector<InternalLinkExport> LinkExportSetup;
        LinkExportSetup link_export_setup;

        typedef RTT::InputPort<base::samples::Joints> JointsInputPort;
        typedef RTT::OutputPort<base::samples::Joints> JointsOutputPort;
        struct InternalJointExport {
            bool permanent;
            base::Time port_period;
            bool ignore_joint_names;
            std::vector<gz::sim::Entity> gazebo_joints;
            std::vector<std::string> expected_names;

            base::samples::Joints joints_in;
            JointsInputPort* in_port;
            base::samples::Joints joints_out;
            JointsOutputPort* out_port;
            base::Time last_command;

            std::vector<double> position_offsets;

            InternalJointExport()
                : permanent(false)
                , in_port(nullptr)
                , out_port(nullptr)
            {
            }

            void addJoint(gz::sim::Entity joint, std::string name);
        };

        /** Return the JointExport structure that exports all joints of the model
         */
        InternalJointExport createAllJointsExport();

        void setupJoints();

        typedef std::vector<InternalJointExport> JointExportSetup;
        JointExportSetup joint_export_setup;

        std::pair<gz::sim::Entity, std::string> resolveSelectedLink(
            std::string const& key,
            std::string const& user_value);
        void validateExportRequestPortName(std::set<std::string> const& used_names,
            std::string const& port_name);
        void setupLinks();
        void warpModel(base::samples::RigidBodyState const& modelPose);
        void updateLinks(base::Time const& time);
        void writeExportedJointSamples(base::Time const& time,
            InternalJointExport& exported_joint);
        void readExportedJointCmd(base::Time const& time,
            InternalJointExport& exported_joint);
        bool validateExportedJointCmd(InternalJointExport const& exported_joint) const;
        void updateModelPose(base::Time const& time);

        std::string optionOrDefault(std::string const& key,
            std::string const& value,
            std::string const& default_value);

        void releaseLinks();
        void releaseJoints();

    protected:
    public:
        void setGazebo(gz::sim::Entity const& entity,
            sdf::ElementConstPtr const& model_sdf,
            gz::sim::EntityComponentManager& ecm,
            gz::sim::EventManager& event_manager) override;

        bool startHook();
        void updateHook();
        bool configureHook();
        void cleanupHook();

        /** TaskContext constructor for ModelTask
         * \param name Name of the task. This name needs to be unique to make it
         * identifiable via nameservices. \param initial_state The initial TaskState of
         * the TaskContext. Default is Stopped state.
         */
        ModelTask(std::string const& name = "gazebo::ModelTask");

        /** TaskContext constructor for ModelTask
         * \param name Name of the task. This name needs to be unique to make it
         * identifiable for nameservices. \param engine The RTT Execution engine to be
         * used for this task, which serialises the execution of all commands, programs,
         * state machines and incoming events for a task. \param initial_state The initial
         * TaskState of the TaskContext. Default is Stopped state.
         */
        ModelTask(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of ModelTask
         */
        ~ModelTask();
    };
}

#endif
