/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */
//======================================================================================
// Brazilian Institute of Robotics
// Authors: Thomio Watanabe
// Date: December 2014
//======================================================================================

#include "ModelTask.hpp"
#include "Gazebo7Shims.hpp"
#include <gazebo/common/Exception.hh>

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
    string name = "gazebo:" + GzGet((*_world), Name, ()) + ":" + _model->GetName();
    provides()->setName(name);
    _name.set(name);

    BaseTask::setGazeboWorld(_world);
    model = _model;

    if (_model_frame.get().empty())
        _model_frame.set(_model->GetName());
    if (_world_frame.get().empty())
        _world_frame.set(GzGet((*_world), Name, ()));
} 

void ModelTask::setupJoints()
{
    // Get all joints from a model and set Rock Input/Output Ports
    for(Joint_V::iterator joint = gazebo_joints.begin(); joint != gazebo_joints.end(); ++joint)
    {
#if GAZEBO_MAJOR_VERSION >= 6
        if((*joint)->HasType(physics::Base::FIXED_JOINT))
        {
            gzmsg << "ModelTask: ignore fixed joint: " << GzGet((*world), Name, ()) + "/" + model->GetName() +
                "/" + (*joint)->GetName() << endl;
            continue;
        }
#endif
        gzmsg << "ModelTask: found joint: " << GzGet((*world), Name, ()) + "/" + model->GetName() +
                "/" + (*joint)->GetName() << endl;
        joints_in.names.push_back( (*joint)->GetName() );
        joints_in.elements.push_back( base::JointState::Effort(0.0) );
    }
}

void ModelTask::setupLinks()
{

    // The robot configuration YAML file must define the exported links.
    vector<LinkExport> export_conf = _exported_links.get();

    for(vector<LinkExport>::iterator it = export_conf.begin();
            it != export_conf.end(); ++it)
    {
        ExportedLink exported_link(*it);

        exported_link.source_link =
            checkExportedLinkElements("source_link", it->source_link, _world_frame.get());
        exported_link.target_link =
            checkExportedLinkElements("target_link", it->target_link, _world_frame.get());
        exported_link.source_frame =
            checkExportedLinkElements("source_frame", it->source_frame, exported_link.source_link);
        exported_link.target_frame =
            checkExportedLinkElements("target_frame", it->target_frame, exported_link.target_link);

        if (it->source_link != _world_frame.get())
            exported_link.source_link_ptr = model->GetLink( it->source_link );
        if (it->target_link != _world_frame.get())
            exported_link.target_link_ptr = model->GetLink( it->target_link );
        exported_link.port_name = it->port_name;
        exported_link.rba_port_name = it->port_name + "_acceleration";
        exported_link.wrench_port_name = it->port_name + "_wrench";
        exported_link.port_period = it->port_period;

        if (exported_link.source_link != _world_frame.get() && !exported_link.source_link_ptr)
        {
            physics::Link_V const& links = model->GetLinks();
            string link_names = std::accumulate(links.begin(), links.end(), string(),
                    [](string s, physics::LinkPtr l) { return s + ", " + l->GetName(); });
            gzthrow("ModelTask: cannot find exported source link " << it->source_link << " in model, known links: " << link_names);
        }
        else if (exported_link.target_link != _world_frame.get() && !exported_link.target_link_ptr)
        {
            physics::Link_V const& links = model->GetLinks();
            string link_names = std::accumulate(links.begin(), links.end(), string(),
                    [](string s, physics::LinkPtr l) { return s + ", " + l->GetName(); });
            gzthrow("ModelTask: cannot find exported target link " << it->target_link << " in model, known links: " << link_names);
        }
        else if (it->port_name.empty())
        { gzthrow("ModelTask: no port name given in link export"); }
        else if (ports()->getPort(it->port_name))
        { gzthrow("ModelTask: provided port name " << it->port_name << " already used on the task interface"); }
        else // if (exported_links.find(it->port_name) != exported_links.end())
        {
            for (vector<ExportedLink>::iterator it_el = exported_links.begin(); it_el != exported_links.end(); it_el++)
            {
                if (it_el->port_name == exported_link.port_name)
                { gzthrow("ModelTask: provided port name " << exported_link.port_name << " already used by another exported link"); }
                else if (it_el->rba_port_name == exported_link.rba_port_name)
                { gzthrow("ModelTask: provided rba port name " << exported_link.rba_port_name << " already used by another exported link"); }
                else if (it_el->wrench_port_name == exported_link.wrench_port_name)
                { gzthrow("ModelTask: provided wrench port name " << exported_link.wrench_port_name << " already used by another exported link"); }
            }
        }

        exported_links.push_back(exported_link);
    }

    for (ExportedLinks::iterator it = exported_links.begin(); it != exported_links.end(); ++it)
    {
        // Create the ports dynamicaly
        gzmsg << "ModelTask: exporting link "
            << GzGet((*world), Name, ()) + "/" + model->GetName() + "/" + it->source_link << "2" << it->target_link
            << " through wrench port " << it->wrench_port_name << ", rbs port " << it->port_name << " and rba port " << it->rba_port_name
            << " updated every " << it->port_period.toSeconds() << " seconds."
            << endl;

        it->wrench_port = new WrenchInPort( it->wrench_port_name );
        it->port = new RBSOutPort( it->port_name );
        it->rba_port = new RBAOutPort( it->rba_port_name );
        ports()->addPort(*it->wrench_port);
        ports()->addPort(*it->port);
        ports()->addPort(*it->rba_port);
    }
}

bool ModelTask::startHook()
{
    if (! ModelTaskBase::startHook())
        return false;

    lastJointCommandTime = base::Time();

    for(ExportedLinks::iterator it = exported_links.begin(); it != exported_links.end(); ++it)
    {
        it->last_update.fromSeconds(0);
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
    updateJoints(time);
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
        model2world.Pos().X(),model2world.Pos().Y(),model2world.Pos().Z());
    rbs.cov_position = _cov_position.get();
    rbs.orientation = base::Quaterniond(
        model2world.Rot().W(),model2world.Rot().X(),model2world.Rot().Y(),model2world.Rot().Z() );
    rbs.cov_orientation = _cov_orientation.get();
    rbs.velocity = base::Vector3d(
        model2world_vel.X(), model2world_vel.Y(), model2world_vel.Z());
    rbs.cov_velocity = _cov_velocity.get();
    rbs.angular_velocity = base::Vector3d(
        model2world_angular_vel.X(), model2world_angular_vel.Y(), model2world_angular_vel.Z());
    _pose_samples.write(rbs);
}

void ModelTask::updateJoints(base::Time const& time)
{
    // Read joint pos and speed from gazebo link
    vector<string> names;
    vector<double> positions;
    vector<float> speeds;
    for(Joint_V::iterator it = gazebo_joints.begin(); it != gazebo_joints.end(); ++it )
    {
#if GAZEBO_MAJOR_VERSION >= 6
        // Do not export fixed joints
        if((*it)->HasType(physics::Base::FIXED_JOINT))
            continue;
#endif

        // Read joint angle from gazebo link
        names.push_back( (*it)->GetScopedName() );
#if GAZEBO_MAJOR_VERSION >= 8
        positions.push_back( (*it)->Position(0) );
        speeds.push_back( (*it)->GetVelocity(0) );
#else
        positions.push_back( (*it)->GetAngle(0).Radian() );
        speeds.push_back( (*it)->GetVelocity(0) );
#endif
    }
    // fill the Joints samples and write
    base::samples::Joints joints = base::samples::Joints::Positions(positions,names);
    vector<float>::iterator speeds_it = speeds.begin();
    for(std::vector<base::JointState>::iterator elements_it = joints.elements.begin(); elements_it != joints.elements.end(); ++elements_it )
    {
        (*elements_it).speed = *speeds_it;
        ++speeds_it;
    }
    joints.time = time;
    _joints_samples.write( joints );

    RTT::FlowStatus flow = _joints_cmd.readNewest( joints_in );
    if (flow == RTT::NewData)
    {
        lastJointCommandTime = time;
        lastJointCommand = joints_in;
    }
    else if (flow == RTT::NoData)
        return;
    else if (time - lastJointCommandTime >= _joint_command_timeout.get())
        return;
    else
        joints_in = lastJointCommand;

    const std::vector<std::string> &cmd_names= joints_in.names;
    for(Joint_V::iterator it = gazebo_joints.begin(); it != gazebo_joints.end(); ++it )
    {
#if GAZEBO_MAJOR_VERSION >= 6
        // Do not set fixed joints
        if((*it)->HasType(physics::Base::FIXED_JOINT))
            continue;
#endif

        // Do not set joints which are not part of the command
        auto result = find(cmd_names.begin(),cmd_names.end(),(*it)->GetScopedName());
        if(result == cmd_names.end())
            continue;

        // Apply effort to joint
        base::JointState j_cmd(joints_in[result-cmd_names.begin()]);
        if( j_cmd.isEffort() )
            (*it)->SetForce(0, j_cmd.effort );
        else if( j_cmd.isPosition() )
            (*it)->SetPosition(0, j_cmd.position );
        else if( j_cmd.isSpeed() )
            (*it)->SetVelocity(0, j_cmd.speed );
    }
}

void ModelTask::updateLinks(base::Time const& time)
{
    for(ExportedLinks::iterator it = exported_links.begin(); it != exported_links.end(); ++it)
    {
        //do not update the link if the last port writing happened
        //in less then link_period.
        if (!(it->last_update.isNull()))
        {
            if ((time - it->last_update) <= it->port_period)
                break;
        }

        Pose3d source2world = Pose3d::Zero;
        Vector3d sourceInWorld_linear_vel  = Vector3d::Zero;
        Vector3d source_angular_vel = Vector3d::Zero;
        Vector3d sourceInWorld_linear_acc  = Vector3d::Zero;
        Vector3d source_angular_acc = Vector3d::Zero;
        if (it->source_link_ptr)
        {
            source2world = GzGetIgn((*(it->source_link_ptr)), WorldPose, ());
            sourceInWorld_linear_vel  = GzGetIgn((*(it->source_link_ptr)), WorldLinearVel, ());
            source_angular_vel        = GzGetIgn((*(it->source_link_ptr)), RelativeAngularVel, ());
            sourceInWorld_linear_acc  = GzGetIgn((*(it->source_link_ptr)), WorldLinearAccel, ());
            source_angular_acc        = GzGetIgn((*(it->source_link_ptr)), RelativeAngularAccel, ());
        }

        Pose3d target2world = Pose3d::Zero;
        if (it->target_link_ptr)
            target2world        = GzGetIgn((*(it->target_link_ptr)), WorldPose, ());

        Pose3d source2target( Pose3d(source2world - target2world) );
        Vector3d sourceInTarget_linear_vel (target2world.Rot().RotateVectorReverse(sourceInWorld_linear_vel));
        Vector3d sourceInTarget_linear_acc (target2world.Rot().RotateVectorReverse(sourceInWorld_linear_acc));

        RigidBodyState rbs;
        rbs.sourceFrame = it->source_frame;
        rbs.targetFrame = it->target_frame;
        rbs.position = base::Vector3d(
            source2target.Pos().X(),source2target.Pos().Y(),source2target.Pos().Z());
        rbs.cov_position = it->cov_position;
        rbs.orientation = base::Quaterniond(
            source2target.Rot().W(),source2target.Rot().X(),source2target.Rot().Y(),source2target.Rot().Z() );
        rbs.cov_orientation = it->cov_orientation;
        rbs.velocity = base::Vector3d(
            sourceInTarget_linear_vel.X(),sourceInTarget_linear_vel.Y(),sourceInTarget_linear_vel.Z());
        rbs.cov_velocity = it->cov_velocity;
        rbs.angular_velocity = base::Vector3d(
            source_angular_vel.X(),source_angular_vel.Y(),source_angular_vel.Z());
        rbs.cov_angular_velocity = it->cov_angular_velocity;
        rbs.time = time;
        it->port->write(rbs);

        base::samples::RigidBodyAcceleration rba;
        rba.cov_acceleration = it->cov_acceleration;
        rba.acceleration = base::Vector3d(
            sourceInTarget_linear_acc.X(),sourceInTarget_linear_acc.Y(),sourceInTarget_linear_acc.Z());
        rba.angular_acceleration = base::Vector3d(
            source_angular_acc.X(),source_angular_acc.Y(),source_angular_acc.Z());
        rba.cov_angular_acceleration = it->cov_angular_acceleration;
        rba.time = time;
        it->rba_port->write(rba);

        it->last_update = time;
    }

    for(ExportedLinks::iterator it = exported_links.begin(); it != exported_links.end(); ++it)
    {
        RTT::FlowStatus flow = it->wrench_port->readNewest( it->wrench_in );
        if (flow == RTT::NewData)
        {
            it->lastWrenchCommandTime = time;
            it->lastWrenchCommand = it->wrench_in;
        }
        else if (flow == RTT::NoData)
            continue;
        else if (time - it->lastWrenchCommandTime >= _wrench_command_timeout.get()) // create _wrench_command_timeout to replace this
            continue;
        else
            it->wrench_in = it->lastWrenchCommand;

        Pose3d source2world = GzGetIgn((*it->source_link_ptr), WorldPose, ());

        it->source_link_ptr->SetForce(source2world.Rot().RotateVector(
                                          Vector3d(it->wrench_in.force[0], it->wrench_in.force[1], it->wrench_in.force[2])));
        it->source_link_ptr->SetTorque(source2world.Rot().RotateVector(
                                           Vector3d(it->wrench_in.torque[0], it->wrench_in.torque[1], it->wrench_in.torque[2])));
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
}

void ModelTask::releaseLinks()
{
    for(ExportedLinks::iterator it = exported_links.begin(); it != exported_links.end(); ++it) {
        if (it->port != NULL) {
            ports()->removePort(it->port->getName());
            delete it->port;
            it->port = NULL;
        }
        if (it->rba_port != NULL) {
            ports()->removePort(it->rba_port->getName());
            delete it->rba_port;
            it->rba_port = NULL;
        }
        if (it->wrench_port != NULL) {
            ports()->removePort(it->wrench_port->getName());
            delete it->wrench_port;
            it->wrench_port = NULL;
        }
    }
    exported_links.clear();
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

