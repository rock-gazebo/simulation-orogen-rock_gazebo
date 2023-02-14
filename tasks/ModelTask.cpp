/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */
//======================================================================================
// Brazilian Institute of Robotics
// Authors: Thomio Watanabe
// Date: December 2014
//======================================================================================

#include "ModelTask.hpp"
#include "Gazebo7Shims.hpp"
#include <gazebo/common/Exception.hh>
#include <base-logging/Logging.hpp>

using namespace std;
using namespace gazebo;
using namespace rock_gazebo;
using ignition::math::Vector3d;
using ignition::math::Pose3d;
using ignition::math::Quaterniond;

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

void ModelTask::setGazeboModel(WorldPtr _world,  ModelPtr _model)
{
    string name = "gazebo::" + GzGet((*_world), Name, ()) + "::" + _model->GetName();
    provides()->setName(name);
    _name.set(name);

    BaseTask::setGazeboWorld(_world);
    model = _model;

    if (_model_frame.get().empty()) {
        _model_frame.set(_model->GetName());
    }
    if (_world_frame.get().empty()) {
        _world_frame.set(GzGet((*_world), Name, ()));
    }
}

void ModelTask::InternalJointExport::addJoint(JointPtr joint, std::string name)
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

    // Setup a joint export for the "main" interface
    InternalJointExport main_joint_export;
    main_joint_export.permanent = true;
    main_joint_export.ignore_joint_names = _ignore_joint_names.get();
    main_joint_export.in_port = &_joints_cmd;
    main_joint_export.out_port = &_joints_samples;
    for (auto const& joint : gazebo_joints)
    {
#if GAZEBO_MAJOR_VERSION >= 6
        if (joint->HasType(physics::Base::FIXED_JOINT))
        {
            gzmsg << "ModelTask: ignore fixed joint: "
                  << GzGet((*world), Name, ())
                  << "/" << model->GetName()
                  << "/" << joint->GetName() << endl;
            continue;
        }
#endif
        gzmsg << "ModelTask: found joint (in/out): "
              << GzGet((*world), Name, ())
              << "/" << model->GetName()
              << "/" << joint->GetName() << endl;
        main_joint_export.addJoint(joint, joint->GetScopedName());
    }
    main_joint_export.position_offsets.resize(
        main_joint_export.gazebo_joints.size(), 0
    );
    exported_joints.push_back(main_joint_export);

    std::vector<JointExport> requested_exports =
        _exported_joints.get();

    for (auto const& export_request : requested_exports)
    {
        string prefix = export_request.prefix;
        size_t export_size = export_request.joints.size();

        if (! export_request.position_offsets.empty()) {
            if (export_request.position_offsets.size() != export_size) {
                gzthrow("ModelTask: joint export position_offsets field must either be "
                        "empty, or of the same size of the joints");
            }
        }

        InternalJointExport export_setup;
        for (auto const& gz_joint_name : export_request.joints)
        {
            if (gz_joint_name.substr(0, prefix.size()) != prefix) {
                gzthrow("ModelTask: the name of the exported joint " << gz_joint_name
                        << " does not start with the expected prefix '" + prefix + "'");
            }
            string joint_name = gz_joint_name.substr(prefix.size(), std::string::npos);

            auto gz_joint = model->GetJoint(gz_joint_name);
            if (!gz_joint) {
                gzthrow("ModelTask: cannot find joint " << gz_joint_name
                                                        << " requested in export");
            }
            else if (gz_joint->HasType(physics::Base::FIXED_JOINT)) {
                gzthrow("ModelTask: requesting to export joint "
                        << gz_joint_name << " which is a fixed joint");
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
            export_setup.position_offsets.resize(
                export_setup.gazebo_joints.size(), 0
            );
        }
        else {
            export_setup.position_offsets = export_request.position_offsets;
        }
        exported_joints.push_back(export_setup);
    }

    this->joint_export_setup = exported_joints;
}

void ModelTask::setupLinks()
{

    // The robot configuration YAML file must define the exported links.
    vector<LinkExport> export_conf = _exported_links.get();

    set<string> used_names;
    for (auto const& export_request : export_conf)
    {
        InternalLinkExport exported_link(export_request);

        exported_link.source_link =
            checkExportedLinkElements("source_link", export_request.source_link, _world_frame.get());
        exported_link.target_link =
            checkExportedLinkElements("target_link", export_request.target_link, _world_frame.get());
        exported_link.source_frame =
            checkExportedLinkElements("source_frame", export_request.source_frame, exported_link.source_link);
        exported_link.target_frame =
            checkExportedLinkElements("target_frame", export_request.target_frame, exported_link.target_link);

        if (export_request.source_link != _world_frame.get())
            exported_link.source_link_ptr = model->GetLink( export_request.source_link );
        if (export_request.target_link != _world_frame.get())
            exported_link.target_link_ptr = model->GetLink( export_request.target_link );
        exported_link.port_name = export_request.port_name;
        exported_link.rba_port_name = export_request.port_name + "_acceleration";
        exported_link.wrench_port_name = export_request.port_name + "_wrench";
        exported_link.port_period = export_request.port_period;

        if (exported_link.source_link != _world_frame.get() && !exported_link.source_link_ptr)
        {
            physics::Link_V const& links = model->GetLinks();
            string link_names = std::accumulate(links.begin(), links.end(), string(),
                    [](string s, physics::LinkPtr l) { return s + ", " + l->GetName(); });
            gzthrow("ModelTask: cannot find exported source link " << export_request.source_link << " in model, known links: " << link_names);
        }
        else if (exported_link.target_link != _world_frame.get() && !exported_link.target_link_ptr)
        {
            physics::Link_V const& links = model->GetLinks();
            string link_names = std::accumulate(links.begin(), links.end(), string(),
                    [](string s, physics::LinkPtr l) { return s + ", " + l->GetName(); });
            gzthrow("ModelTask: cannot find exported target link " << export_request.target_link << " in model, known links: " << link_names);
        }
        else if (export_request.port_name.empty())
        { gzthrow("ModelTask: no port name given in link export"); }
        else if (ports()->getPort(export_request.port_name))
        { gzthrow("ModelTask: provided port name " << export_request.port_name << " already used on the task interface"); }
        else if (used_names.find(export_request.port_name) != used_names.end())
        { gzthrow("ModelTask: provided port name " << export_request.port_name << " already used by another exported link"); }

        used_names.insert(exported_link.port_name);
        link_export_setup.push_back(exported_link);
    }

    for (auto& export_setup : link_export_setup)
    {
        // Create the ports dynamicaly
        gzmsg << "ModelTask: exporting link "
            << GzGet((*world), Name, ()) + "/" + model->GetName() + "/" + export_setup.source_link << "2" << export_setup.target_link
            << " through wrench port " << export_setup.wrench_port_name << ", rbs port " << export_setup.port_name << " and rba port " << export_setup.rba_port_name
            << " updated every " << export_setup.port_period.toSeconds() << " seconds."
            << endl;

        export_setup.wrench_port = new WrenchInPort( export_setup.wrench_port_name );
        export_setup.port = new RBSOutPort( export_setup.port_name );
        export_setup.rba_port = new RBAOutPort( export_setup.rba_port_name );
        ports()->addPort(*export_setup.wrench_port);
        ports()->addPort(*export_setup.port);
        ports()->addPort(*export_setup.rba_port);
    }
}

bool ModelTask::startHook()
{
    if (! ModelTaskBase::startHook())
        return false;

    for(auto& exported_joint : joint_export_setup)
    {
        exported_joint.last_command = base::Time();
        exported_joint.joints_out.time = base::Time();
    }

    for(auto& exported_link : link_export_setup)
    {
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
    for (auto& exported_joint : joint_export_setup)
    {
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
    model->SetWorldPose(model2world);
}

void ModelTask::updateModelPose(base::Time const& time)
{
    Pose3d model2world = GzGetIgn((*model), WorldPose, ());
    Vector3d model2world_angular_vel = GzGetIgn((*model), RelativeAngularVel, ());
    Vector3d model2world_vel = GzGetIgn((*model), WorldLinearVel, ());

    RigidBodyState rbs;
    rbs.invalidate();
    rbs.time = time;
    rbs.sourceFrame = _model_frame.get();
    rbs.targetFrame = _world_frame.get();
    rbs.position = base::Vector3d(
        model2world.Pos().X(),
        model2world.Pos().Y(),
        model2world.Pos().Z()
    );
    rbs.cov_position = _cov_position.get();
    rbs.orientation = base::Quaterniond(
        model2world.Rot().W(),
        model2world.Rot().X(),
        model2world.Rot().Y(),
        model2world.Rot().Z()
    );
    rbs.cov_orientation = _cov_orientation.get();
    rbs.velocity = base::Vector3d(
        model2world_vel.X(), model2world_vel.Y(), model2world_vel.Z());
    rbs.cov_velocity = _cov_velocity.get();
    rbs.angular_velocity = base::Vector3d(
        model2world_angular_vel.X(),
        model2world_angular_vel.Y(),
        model2world_angular_vel.Z()
    );
    rbs.cov_angular_velocity = _cov_angular_velocity.get();
    _pose_samples.write(rbs);
}

void ModelTask::writeExportedJointSamples(base::Time const& time, InternalJointExport& exported_joint)
{
    if (time - exported_joint.joints_out.time < exported_joint.port_period) {
        return;
    }

    size_t size = exported_joint.gazebo_joints.size();
    for (unsigned int i = 0; i < size; ++i)
    {
        base::JointState& state = exported_joint.joints_out.elements[i];
        gazebo::physics::Joint& joint = *exported_joint.gazebo_joints[i];

        state.speed = joint.GetVelocity(0);
#if GAZEBO_MAJOR_VERSION >= 8
        state.position = joint.Position(0);
#else
        state.position = joint.GetAngle(0).Radian();
#endif
        state.position += exported_joint.position_offsets[i];
    }
    exported_joint.joints_out.time = time;
    exported_joint.out_port->write(exported_joint.joints_out);
}

void ModelTask::readExportedJointCmd(base::Time const& time, InternalJointExport& exported_joint)
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
        gazebo::physics::Joint& joint = *exported_joint.gazebo_joints[i];
        double position_offset = exported_joint.position_offsets[i];

        // Apply effort to joint
        if (cmd.isEffort()) {
            joint.SetForce(0, cmd.effort);
        }
        else if (cmd.isPosition()) {
            joint.SetPosition(0, cmd.position - position_offset);
        }
        else if (cmd.isSpeed()) {
            joint.SetVelocity(0, cmd.speed);
        }
        else {
            LOG_ERROR_S
                << "Received command that is neither a pure effort, "
                << "position or speed" << std::endl;
            LOG_ERROR_S
                << "p=" << cmd.position
                << " s=" << cmd.speed
                << " e=" << cmd.effort
                << " r=" << cmd.raw
                << " a=" << cmd.acceleration << std::endl;
            return exception(INVALID_JOINT_COMMAND);
        }
    }
}

bool ModelTask::validateExportedJointCmd(InternalJointExport const& exported_joint) const {
    size_t size = exported_joint.gazebo_joints.size();
    if (exported_joint.joints_in.elements.size() != size) {
        LOG_ERROR_S
            << "Received command with size "
            << exported_joint.joints_in.elements.size()
            << " expected " << size << std::endl;
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
        LOG_ERROR_S
            << "Received command with "
            << exported_joint.joints_in.names.size()
            << " names, expected " << size << ":" << joint_names << std::endl;
        return false;
    }

    for (unsigned int i = 0; i < size; ++i) {
        std::string const& name = exported_joint.joints_in.names[i];
        std::string const& expected_name = exported_joint.expected_names[i];
        if (name != expected_name) {
            LOG_ERROR_S
                << "Expected " << i << "th joint to be "
                << expected_name << " but it is " << name << std::endl;
            return false;
        }
    }

    return true;
}

void ModelTask::updateLinks(base::Time const& time)
{
    auto simTime = getSimTime();
    for(auto& exported_link : link_export_setup)
    {
        //do not update the link if the last port writing happened
        //in less then link_period.
        if (!(exported_link.last_update.isNull()))
        {
            if ((simTime - exported_link.last_update) < exported_link.port_period)
                return;
        }

        Pose3d source2world = Pose3d::Zero;
        Vector3d sourceInWorld_linear_vel  = Vector3d::Zero;
        Vector3d sourceRelative_angular_vel = Vector3d::Zero;
        Vector3d sourceRelative_linear_acc  = Vector3d::Zero;
        Vector3d sourceRelative_angular_acc = Vector3d::Zero;
        if (exported_link.source_link_ptr)
        {
            source2world = GzGetIgn((*(exported_link.source_link_ptr)), WorldPose, ());
            sourceInWorld_linear_vel   = GzGetIgn((*(exported_link.source_link_ptr)), WorldLinearVel, ());
            sourceRelative_angular_vel = GzGetIgn((*(exported_link.source_link_ptr)), RelativeAngularVel, ());
            sourceRelative_linear_acc   = GzGetIgn((*(exported_link.source_link_ptr)), RelativeLinearAccel, ());
            sourceRelative_angular_acc  = GzGetIgn((*(exported_link.source_link_ptr)), RelativeAngularAccel, ());
        }

        Pose3d target2world = Pose3d::Zero;
        if (exported_link.target_link_ptr)
            target2world        = GzGetIgn((*(exported_link.target_link_ptr)), WorldPose, ());

        Pose3d source2target( Pose3d(source2world - target2world) );
        Vector3d sourceInTarget_linear_vel (target2world.Rot().RotateVectorReverse(sourceInWorld_linear_vel));

        RigidBodyState rbs;
        rbs.sourceFrame = exported_link.source_frame;
        rbs.targetFrame = exported_link.target_frame;
        rbs.position = base::Vector3d(
            source2target.Pos().X(),source2target.Pos().Y(),source2target.Pos().Z());
        rbs.cov_position = exported_link.cov_position;
        rbs.orientation = base::Quaterniond(
            source2target.Rot().W(),source2target.Rot().X(),source2target.Rot().Y(),source2target.Rot().Z() );
        rbs.cov_orientation = exported_link.cov_orientation;
        rbs.velocity = base::Vector3d(
            sourceInTarget_linear_vel.X(),sourceInTarget_linear_vel.Y(),sourceInTarget_linear_vel.Z());
        rbs.cov_velocity = exported_link.cov_velocity;
        rbs.angular_velocity = base::Vector3d(
            sourceRelative_angular_vel.X(),sourceRelative_angular_vel.Y(),sourceRelative_angular_vel.Z());
        rbs.cov_angular_velocity = exported_link.cov_angular_velocity;
        rbs.time = time;
        exported_link.port->write(rbs);

        base::samples::RigidBodyAcceleration rba;
        rba.cov_acceleration = exported_link.cov_acceleration;
        rba.acceleration = base::Vector3d(
            sourceRelative_linear_acc.X(),sourceRelative_linear_acc.Y(),sourceRelative_linear_acc.Z());
        rba.angular_acceleration = base::Vector3d(
            sourceRelative_angular_acc.X(),sourceRelative_angular_acc.Y(),sourceRelative_angular_acc.Z());
        rba.cov_angular_acceleration = exported_link.cov_angular_acceleration;
        rba.time = time;
        exported_link.rba_port->write(rba);

        exported_link.last_update = simTime;
    }

    for(auto& exported_link : link_export_setup)
    {
        RTT::FlowStatus flow = exported_link.wrench_port->readNewest( exported_link.wrench_in );
        if (flow == RTT::NewData)
        {
            exported_link.lastWrenchCommandTime = time;
            exported_link.lastWrenchCommand = exported_link.wrench_in;
        }
        else if (exported_link.lastWrenchCommandTime.isNull())
            continue;
        else if (time - exported_link.lastWrenchCommandTime >= _wrench_command_timeout.get())
            continue;
        else
            exported_link.wrench_in = exported_link.lastWrenchCommand;

        Pose3d source2world = GzGetIgn((*exported_link.source_link_ptr), WorldPose, ());

        exported_link.source_link_ptr->SetForce(source2world.Rot().RotateVector(
                    Vector3d(exported_link.wrench_in.force[0], exported_link.wrench_in.force[1], exported_link.wrench_in.force[2])));
        exported_link.source_link_ptr->SetTorque(source2world.Rot().RotateVector(
                    Vector3d(exported_link.wrench_in.torque[0], exported_link.wrench_in.torque[1], exported_link.wrench_in.torque[2])));
    }
}

bool ModelTask::configureHook()
{
    if( ! ModelTaskBase::configureHook() )
        return false;

    // Test if setGazeboModel() has been called -> if world/model are NULL
    if( (!world) && (!model) )
        return false;

    gazebo_joints = model->GetJoints();
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
    for (auto& export_setup : joint_export_setup)
    {
        if (!export_setup.permanent)
        {
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
    for(auto& exported_link : link_export_setup)
    {
        if (exported_link.wrench_port != NULL) {
            ports()->removePort(exported_link.wrench_port->getName());
            delete exported_link.wrench_port;
            exported_link.wrench_port = NULL;
        }
        if (exported_link.port)
        {
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

string ModelTask::checkExportedLinkElements(string element_name, string test, string option)
{
    // when not defined, source_link and target_link will recieve "world".
    // when not defined, source_frame and target_frame will receive source_link and target_link content
    if( test.empty() )
    {
        gzmsg << "ModelTask: " << model->GetName() << " " << element_name << " not defined, using "<< option << endl;
        return option;
    }else {
        gzmsg << "ModelTask: " << model->GetName() << " " << element_name << ": " << test << endl;
        return test;
    }
}

