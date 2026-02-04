/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */
//======================================================================================
// Brazilian Institute of Robotics
// Authors: Thomio Watanabe
// Date: December 2014
//======================================================================================

#include "ModelTask.hpp"
#include <Eigen/src/Geometry/Transform.h>
#include <base-logging/Logging.hpp>
#include <gz/sim/Entity.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointType.hh>
#include <sdf/Joint.hh>
#include <stdexcept>

#include "Helpers.hpp"

using namespace std;
using namespace gz;
using namespace gz::sim;
using namespace gz_rock;
using gz::math::Pose3d;
using gz::math::Quaterniond;
using gz::math::Vector3d;

ModelTask::ModelTask(string const& name)
    : ModelTaskBase(name)
{
    _joint_command_timeout.set(base::Time::fromSeconds(1.0));
    _wrench_command_timeout.set(base::Time::fromSeconds(1.0));
    _cov_position.set(base::Matrix3d::Ones() * base::unset<double>());
    _cov_orientation.set(base::Matrix3d::Ones() * base::unset<double>());
    _cov_velocity.set(base::Matrix3d::Ones() * base::unset<double>());
}
ModelTask::ModelTask(string const& name, RTT::ExecutionEngine* engine)
    : ModelTaskBase(name, engine)
{
}

ModelTask::~ModelTask()
{
    releaseLinks();
}

void ModelTask::setGazebo(
    std::string const& plugin_name,
    gz::sim::Entity const& model_entity,
    sdf::ElementConstPtr const& model_sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_manager
) {
    m_model = model_entity;
    m_ecm = &ecm;

    ModelTaskBase::setGazebo(plugin_name, model_entity, model_sdf, ecm, event_manager);

    string name = "gazebo::" + scopedName(model_entity, ecm, "::");
    provides()->setName(name);
    _name.set(name);

    if (_model_frame.get().empty()) {
        _model_frame.set(scopedName(m_model, ecm, "::"));
    }

    if (_world_frame.get().empty()) {
        auto world = gz::sim::worldEntity(model_entity, ecm);
        _world_frame.set(gz::sim::World(world).Name(ecm).value_or("world"));
    }

    auto link_entity = Model(m_model).CanonicalLink(*m_ecm);
    Link(link_entity).EnableAccelerationChecks(*m_ecm, true);
    Link(link_entity).EnableVelocityChecks(*m_ecm, true);
}

void ModelTask::InternalJointExport::addJoint(Entity joint, std::string name)
{
    gazebo_joints.push_back(joint);
    expected_names.push_back(name);
    joints_in.names.push_back(name);
    joints_in.elements.push_back(base::JointState::Effort(0.0));
    joints_out.names.push_back(name);
    joints_out.elements.push_back(base::JointState::Effort(0.0));
}

void ModelTask::setupJoints()
{
    JointExportSetup exported_joints;
    exported_joints.push_back(createAllJointsExport());

    auto model = Model(m_model);

    std::vector<JointExport> requested_exports = _exported_joints.get();
    for (auto const& export_request : requested_exports) {
        string prefix = export_request.prefix;
        size_t export_size = export_request.joints.size();

        if (!export_request.position_offsets.empty()) {
            if (export_request.position_offsets.size() != export_size) {
                throw std::invalid_argument(
                    "ModelTask: joint export position_offsets field must either be "
                    "empty, or of the same size of the joints");
            }
        }

        InternalJointExport export_setup;
        for (auto const& gz_joint_name : export_request.joints) {
            if (gz_joint_name.substr(0, prefix.size()) != prefix) {
                throw std::invalid_argument(
                    "ModelTask: the name of the exported joint " + gz_joint_name +
                    " does not start with the expected prefix '" + prefix + "'");
            }
            string joint_name = gz_joint_name.substr(prefix.size(), std::string::npos);

            auto gz_joint = model.JointByName(*m_ecm, gz_joint_name);
            if (gz_joint == kNullEntity) {
                throw std::invalid_argument("ModelTask: cannot find joint " +
                                            gz_joint_name + " requested in export");
            }

            if (Joint(gz_joint).Type(*m_ecm).value() == sdf::JointType::FIXED) {
                std::invalid_argument("ModelTask: requesting to export joint " +
                                      gz_joint_name + " which is a fixed joint");
            }

            export_setup.addJoint(gz_joint, joint_name);
        }

        export_setup.port_period = export_request.port_period;
        export_setup.in_port = new JointsInputPort(export_request.port_name + "_cmd");
        export_setup.out_port =
            new JointsOutputPort(export_request.port_name + "_samples");
        export_setup.ignore_joint_names = export_request.ignore_joint_names;
        ports()->addPort(*export_setup.in_port);
        ports()->addPort(*export_setup.out_port);
        if (export_request.position_offsets.empty()) {
            export_setup.position_offsets.resize(export_setup.gazebo_joints.size(), 0);
        }
        else {
            export_setup.position_offsets = export_request.position_offsets;
        }
        exported_joints.push_back(export_setup);
    }

    for (auto& export_setup: exported_joints) {
        for (auto entity: export_setup.gazebo_joints) {
            Joint j{entity};
            j.EnablePositionCheck(*m_ecm);
            j.EnableVelocityCheck(*m_ecm);
        }
    }

    this->joint_export_setup = exported_joints;
}

ModelTask::InternalJointExport ModelTask::createAllJointsExport() {
    InternalJointExport all_joints;
    all_joints.permanent = true;
    all_joints.ignore_joint_names = _ignore_joint_names.get();
    all_joints.in_port = &_joints_cmd;
    all_joints.out_port = &_joints_samples;

    m_ecm->Each<components::JointType, components::Name>(
        [&](Entity const& entity, components::JointType const* joint_type, components::Name const* name) -> bool {
            if (joint_type->Data() != sdf::JointType::FIXED) {
                all_joints.addJoint(entity, name->Data());
            }
            return true;
        }
    );

    all_joints.position_offsets.resize(
        all_joints.gazebo_joints.size(), 0
    );

    return all_joints;
}

static void throwInvalidLinkNames(EntityComponentManager& ecm,
    Entity model,
    std::string const& msg)
{
    auto links = Model(model).Links(ecm);
    string link_names =
        std::accumulate(links.begin(), links.end(), string(), [&](string s, Entity l) {
            return s + ", " + Link(l).Name(ecm).value();
        });
    throw std::invalid_argument(
        "ModelTask: " + msg + " in model, known links: " + link_names);
}

void ModelTask::setupLinks()
{
    // The robot configuration YAML file must define the exported links.
    vector<LinkExport> export_conf = _exported_links.get();

    std::set<string> used_names;
    for (auto const& export_request : export_conf) {
        validateExportRequestPortName(used_names, export_request.port_name);

        InternalLinkExport exported_link;
        tie(exported_link.source_link_ptr, exported_link.source_link) =
            resolveSelectedLink("source_link", export_request.source_link);
        tie(exported_link.target_link_ptr, exported_link.source_link) =
            resolveSelectedLink("target_link", export_request.target_link);

        exported_link.port_name = export_request.port_name;
        exported_link.rba_port_name = export_request.port_name + "_acceleration";
        exported_link.wrench_port_name = export_request.port_name + "_wrench";
        exported_link.port_period = export_request.port_period;

        used_names.insert(exported_link.port_name);
        link_export_setup.push_back(exported_link);
    }

    gzmsg << "ModelTask: link exports from model "
          << scopedName(m_model, *m_ecm, "::") << ":\n";
    for (auto& export_setup : link_export_setup) {
        gzmsg << "  source=" << scopedName(export_setup.source_link_ptr, *m_ecm, "::") << endl;
        gzmsg << "  target=" << scopedName(export_setup.target_link_ptr, *m_ecm, "::") << endl;
        gzmsg << "    rbs port=" << export_setup.port_name << endl;
        gzmsg << "    rba port=" << export_setup.rba_port_name << endl;
        gzmsg << "    wrench port=" << export_setup.wrench_port_name << endl;
        gzmsg << "    update period=" << export_setup.port_period.toSeconds() << endl;

        export_setup.wrench_port = new WrenchInPort(export_setup.wrench_port_name);
        export_setup.port = new RBSOutPort(export_setup.port_name);
        export_setup.rba_port = new RBAOutPort(export_setup.rba_port_name);
        ports()->addPort(*export_setup.wrench_port);
        ports()->addPort(*export_setup.port);
        ports()->addPort(*export_setup.rba_port);
    }
}

bool ModelTask::startHook()
{
    if (!ModelTaskBase::startHook())
        return false;

    for (auto& exported_joint : joint_export_setup) {
        exported_joint.last_command = base::Time();
        exported_joint.joints_out.time = base::Time();
    }

    for (auto& exported_link : link_export_setup) {
        exported_link.lastWrenchCommandTime = base::Time();
        exported_link.last_update = base::Time();
    }

    return true;
}
void ModelTask::updateHook()
{
    base::Time time = getCurrentTime();

    base::samples::RigidBodyState modelPose;
    if (_model_pose.read(modelPose) == RTT::NewData)
        warpModel(modelPose);

    updateModelPose(time);
    for (auto& exported_joint : joint_export_setup) {
        writeExportedJointSamples(time, exported_joint);
        readExportedJointCmd(time, exported_joint);
    }
    updateLinks(time);
}

void ModelTask::warpModel(base::samples::RigidBodyState const& modelPose)
{
    Eigen::Vector3d v(modelPose.position);
    Vector3d model2world_v(v.x(), v.y(), v.z());
    Eigen::Quaterniond q(modelPose.orientation);
    Quaterniond model2world_q(q.w(), q.x(), q.y(), q.z());
    Pose3d model2world;
    model2world.Set(model2world_v, model2world_q);
    Model(m_model).SetWorldPoseCmd(*m_ecm, model2world);
}

void ModelTask::updateModelPose(base::Time const& time)
{
    auto link_entity = Model(m_model).CanonicalLink(*m_ecm);
    auto link = Link(link_entity);
    auto model2world = link.WorldPose(*m_ecm);
    auto model2world_angular_vel = link.WorldAngularVelocity(*m_ecm);
    auto model2world_vel = link.WorldLinearVelocity(*m_ecm);

    auto model2world_pos = model2world->Pos();
    auto model2world_rot = model2world->Rot();

    RigidBodyState rbs;
    rbs.invalidate();
    rbs.time = time;
    rbs.sourceFrame = _model_frame.get();
    rbs.targetFrame = _world_frame.get();
    rbs.position = base::Vector3d(model2world_pos.X(),
        model2world_pos.Y(),
        model2world_pos.Z());
    rbs.cov_position = _cov_position.get();
    rbs.orientation = base::Quaterniond(model2world_rot.W(),
        model2world_rot.X(),
        model2world_rot.Y(),
        model2world_rot.Z());
    rbs.cov_orientation = _cov_orientation.get();
    rbs.velocity =
        base::Vector3d(model2world_vel->X(), model2world_vel->Y(), model2world_vel->Z());
    rbs.cov_velocity = _cov_velocity.get();

    rbs.angular_velocity = base::Vector3d(model2world_angular_vel->X(),
        model2world_angular_vel->Y(),
        model2world_angular_vel->Z());
    rbs.cov_angular_velocity = _cov_angular_velocity.get();
    _pose_samples.write(rbs);
}

void ModelTask::writeExportedJointSamples(base::Time const& time,
    InternalJointExport& exported_joint)
{
    if (time - exported_joint.joints_out.time < exported_joint.port_period) {
        return;
    }

    size_t size = exported_joint.gazebo_joints.size();
    for (unsigned int i = 0; i < size; ++i) {
        base::JointState& state = exported_joint.joints_out.elements[i];
        auto joint = Joint(exported_joint.gazebo_joints[i]);

        state.speed = joint.Velocity(*m_ecm).value().at(0);
        state.position = joint.Position(*m_ecm).value().at(0);
        state.position += exported_joint.position_offsets[i];
    }
    exported_joint.joints_out.time = time;
    exported_joint.out_port->write(exported_joint.joints_out);
}

void ModelTask::readExportedJointCmd(base::Time const& time,
    InternalJointExport& exported_joint)
{
    RTT::FlowStatus flow = exported_joint.in_port->read(exported_joint.joints_in);

    if (flow == RTT::NewData) {
        exported_joint.last_command = time;
        if (!validateExportedJointCmd(exported_joint)) {
            return exception(INVALID_JOINT_COMMAND);
        }
    }
    else if (exported_joint.last_command.isNull()) {
        return;
    }
    else if (time - exported_joint.last_command >= _joint_command_timeout.get()) {
        return;
    }

    size_t size = exported_joint.gazebo_joints.size();
    for (unsigned int i = 0; i < size; ++i) {
        base::JointState const& cmd = exported_joint.joints_in.elements[i];
        auto joint = Joint(exported_joint.gazebo_joints[i]);
        double position_offset = exported_joint.position_offsets[i];

        // Apply effort to joint
        if (cmd.isEffort()) {
            joint.SetForce(*m_ecm, {cmd.effort});
        }
        else if (cmd.isPosition()) {
            joint.ResetPosition(*m_ecm, {cmd.position - position_offset});
        }
        else if (cmd.isSpeed()) {
            joint.SetVelocity(*m_ecm, {cmd.speed});
        }
        else {
            LOG_ERROR_S << "Received command that is neither a pure effort, "
                        << "position or speed" << std::endl;
            LOG_ERROR_S << "p=" << cmd.position << " s=" << cmd.speed
                        << " e=" << cmd.effort << " r=" << cmd.raw
                        << " a=" << cmd.acceleration << std::endl;
            return exception(INVALID_JOINT_COMMAND);
        }
    }
}

bool ModelTask::validateExportedJointCmd(InternalJointExport const& exported_joint) const
{
    size_t size = exported_joint.gazebo_joints.size();
    if (exported_joint.joints_in.elements.size() != size) {
        LOG_ERROR_S << "Received command with size "
                    << exported_joint.joints_in.elements.size() << " expected " << size
                    << std::endl;
        return false;
    }

    if (exported_joint.ignore_joint_names) {
        return true;
    }

    if (exported_joint.joints_in.names.size() != size) {
        string joint_names = "";
        for (auto const& s : exported_joint.expected_names) {
            joint_names += " " + s;
        }
        LOG_ERROR_S << "Received command with " << exported_joint.joints_in.names.size()
                    << " names, expected " << size << ":" << joint_names << std::endl;
        return false;
    }

    for (unsigned int i = 0; i < size; ++i) {
        std::string const& name = exported_joint.joints_in.names[i];
        std::string const& expected_name = exported_joint.expected_names[i];
        if (name != expected_name) {
            LOG_ERROR_S << "Expected " << i << "th joint to be " << expected_name
                        << " but it is " << name << std::endl;
            return false;
        }
    }

    return true;
}

void ModelTask::updateLinks(base::Time const& time)
{
    auto simTime = getSimTime();
    for (auto& exported_link : link_export_setup) {
        // do not update the link if the last port writing happened
        // in less then link_period.
        if (!(exported_link.last_update.isNull())) {
            if ((simTime - exported_link.last_update) < exported_link.port_period)
                return;
        }

        Eigen::Isometry3d source2world = Eigen::Isometry3d::Identity();
        Eigen::Vector3d source2world_linv_in_world = Eigen::Vector3d::Zero();
        Eigen::Vector3d source2world_angv_in_world = Eigen::Vector3d::Zero();
        Eigen::Vector3d source2world_linacc_in_world = Eigen::Vector3d::Zero();
        Eigen::Vector3d source2world_angacc_in_world = Eigen::Vector3d::Zero();
        if (exported_link.source_link_ptr) {
            auto link = Link(exported_link.source_link_ptr);
            source2world = gz2Eigen(link.WorldPose(*m_ecm));
            source2world_linv_in_world = gz2Eigen(link.WorldLinearVelocity(*m_ecm));
            source2world_angv_in_world = gz2Eigen(link.WorldAngularVelocity(*m_ecm));
            source2world_linacc_in_world = gz2Eigen(link.WorldLinearAcceleration(*m_ecm));
            source2world_angacc_in_world =
                gz2Eigen(link.WorldAngularAcceleration(*m_ecm));
        }

        Eigen::Isometry3d target2world = Eigen::Isometry3d::Identity();
        if (exported_link.target_link_ptr) {
            target2world =
                gz2Eigen(Link(exported_link.target_link_ptr).WorldPose(*m_ecm));
        }

        Eigen::Isometry3d world2source = source2world.inverse();
        Eigen::Isometry3d world2target = target2world.inverse();
        Eigen::Isometry3d source2target = world2target * source2world;

        Eigen::Vector3d source2world_linv_in_target =
            world2target.rotation() * source2world_linv_in_world;
        Eigen::Vector3d source2world_angv_in_source =
            world2source.rotation() * source2world_angv_in_world;
        Eigen::Vector3d source2world_linacc_in_target =
            world2target.rotation() * source2world_linacc_in_world;
        Eigen::Vector3d source2world_angacc_in_target =
            world2target.rotation() * source2world_angacc_in_world;

        RigidBodyState rbs;
        rbs.sourceFrame = exported_link.source_frame;
        rbs.targetFrame = exported_link.target_frame;
        rbs.position = source2target.translation();
        rbs.cov_position = exported_link.cov_position;
        rbs.orientation = source2target.rotation();
        rbs.cov_orientation = exported_link.cov_orientation;
        rbs.velocity = source2world_linv_in_target;
        rbs.cov_velocity = exported_link.cov_velocity;
        rbs.angular_velocity = source2world_angv_in_source;
        rbs.cov_angular_velocity = exported_link.cov_angular_velocity;
        rbs.time = time;
        exported_link.port->write(rbs);

        base::samples::RigidBodyAcceleration rba;
        rba.cov_acceleration = exported_link.cov_acceleration;
        rba.acceleration = source2world_linacc_in_target;
        rba.angular_acceleration = source2world_angacc_in_target;
        rba.cov_angular_acceleration = exported_link.cov_angular_acceleration;
        rba.time = time;
        exported_link.rba_port->write(rba);

        exported_link.last_update = simTime;
    }

    for (auto& exported_link : link_export_setup) {
        RTT::FlowStatus flow =
            exported_link.wrench_port->readNewest(exported_link.wrench_in);
        if (flow == RTT::NewData) {
            exported_link.lastWrenchCommandTime = time;
            exported_link.lastWrenchCommand = exported_link.wrench_in;
        }
        else if (exported_link.lastWrenchCommandTime.isNull())
            continue;
        else if (time - exported_link.lastWrenchCommandTime >=
                 _wrench_command_timeout.get())
            continue;
        else
            exported_link.wrench_in = exported_link.lastWrenchCommand;

        Link link(exported_link.source_link_ptr);
        Eigen::Quaterniond source2world_q =
            gz2Eigen(link.WorldPose(*m_ecm).value().Rot());
        Eigen::Quaterniond world2source_q = source2world_q.inverse();

        Eigen::Vector3d force_in_source = exported_link.wrench_in.force;
        Eigen::Vector3d torque_in_source = exported_link.wrench_in.torque;
        Eigen::Vector3d force_in_world = world2source_q * force_in_source;
        Eigen::Vector3d torque_in_world = world2source_q * torque_in_source;

        link.AddWorldWrench(*m_ecm, eigen2Gz(force_in_world), eigen2Gz(torque_in_world));
    }
}

bool ModelTask::configureHook()
{
    if (!ModelTaskBase::configureHook()) {
        return false;
    }

    // Test if setGazeboModel() has been called -> if world/model are NULL
    if (m_model == kNullEntity) {
        throw std::logic_error(
            "must call setGazebo before configuring a gz_rock::ModelTask");
    }

    setupLinks();
    setupJoints();

    return true;
}

void ModelTask::cleanupHook()
{
    ModelTaskBase::cleanupHook();
    releaseLinks();
    releaseJoints();
}

void ModelTask::releaseJoints()
{
    for (auto& export_setup : joint_export_setup) {
        if (!export_setup.permanent) {
            ports()->removePort(export_setup.in_port->getName());
            delete export_setup.in_port;
            export_setup.in_port = nullptr;

            ports()->removePort(export_setup.out_port->getName());
            delete export_setup.out_port;
            export_setup.out_port = nullptr;
        }
    }
    joint_export_setup.clear();
}

void ModelTask::releaseLinks()
{
    for (auto& exported_link : link_export_setup) {
        if (exported_link.wrench_port != NULL) {
            ports()->removePort(exported_link.wrench_port->getName());
            delete exported_link.wrench_port;
            exported_link.wrench_port = NULL;
        }
        if (exported_link.port) {
            ports()->removePort(exported_link.port->getName());
            delete exported_link.port;
            exported_link.port = NULL;
        }
        if (exported_link.rba_port != NULL) {
            ports()->removePort(exported_link.rba_port->getName());
            delete exported_link.rba_port;
            exported_link.rba_port = NULL;
        }
    }
    link_export_setup.clear();
}

static list<string> splitScopedName(std::string const& scopedName) {
    list<string> result;
    string::size_type delim = scopedName.find("::"), current = 0;
    while(delim != string::npos) {
        result.push_back(scopedName.substr(current, delim));
        current = delim + 2;
        delim = scopedName.find("::", current);
    }
    result.push_back(scopedName.substr(current));
    return result;
}

static Entity resolveLinkRecursive(Entity const& root, std::string const& scopedName, EntityComponentManager& ecm) {
    auto names = splitScopedName(scopedName);

    auto linkName = names.back();
    names.pop_back();

    auto context = root;
    for (auto const& n: names) {
        auto child = Model(context).ModelByName(ecm, n);
        if (child == kNullEntity) {
            throw std::invalid_argument(
                "could not find child model " + n + " of " +
                gz::sim::scopedName(context, ecm, "::")
            );
        }

        context = child;
    }

    auto link = Model(context).LinkByName(ecm, linkName);
    if (link == kNullEntity) {
        throw std::invalid_argument(
            "could not find child link " + linkName + " of " +
            gz::sim::scopedName(context, ecm, "::")
        );
    }

    return link;
}

pair<Entity, string> ModelTask::resolveSelectedLink(std::string const& key,
    std::string const& user_value)
{
    auto default_value = _world_frame.get();
    auto value = optionOrDefault(key, user_value, default_value);
    if (value != default_value) {
        auto link = resolveLinkRecursive(m_model, value, *m_ecm);
        Link(link).EnableAccelerationChecks(*m_ecm, true);
        Link(link).EnableVelocityChecks(*m_ecm, true);
        return make_pair(link, value);
    }

    return make_pair(kNullEntity, value);
}

void ModelTask::validateExportRequestPortName(std::set<std::string> const& used_names,
    std::string const& port_name)
{
    if (port_name.empty()) {
        throw std::invalid_argument("ModelTask: no port name given in link export");
    }
    else if (ports()->getPort(port_name)) {
        throw std::invalid_argument("ModelTask: provided port name " + port_name +
                                    " already used on the task interface");
    }
    else if (used_names.find(port_name) != used_names.end()) {
        throw std::invalid_argument("ModelTask: provided port name " + port_name +
                                    " already used by another exported link");
    }
}

string ModelTask::optionOrDefault(string const& key,
    string const& value,
    string const& default_value)
{
    auto model_name = scopedName(m_model, *m_ecm, "::");

    if (value.empty()) {
        gzmsg << "ModelTask: " << model_name << " " << key << " not set, using "
              << default_value << endl;
        return default_value;
    }
    else {
        gzmsg << "ModelTask: " << model_name << " " << key << ": " << value << endl;
        return value;
    }
}
