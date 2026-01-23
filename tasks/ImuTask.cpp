/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ImuTask.hpp"

#include <gz/sim/Link.hh>

using namespace std;
using namespace gz_rock;

typedef gz::math::Pose3d IgnPose3d;
typedef gz::math::Vector3d IgnVector3d;
typedef gz::math::Quaterniond IgnQuaterniond;

ImuTask::ImuTask(std::string const& name)
    : ImuTaskBase(name)
{
    _cov_orientation.set(base::Matrix3d::Zero() * base::unknown<double>());
    _cov_angular_velocity.set(base::Matrix3d::Zero() * base::unknown<double>());
}

ImuTask::ImuTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ImuTaskBase(name, engine)
{
    _cov_orientation.set(base::Matrix3d::Zero() * base::unknown<double>());
    _cov_angular_velocity.set(base::Matrix3d::Zero() * base::unknown<double>());
}

ImuTask::~ImuTask()
{
}

void ImuTask::setGazebo(
    std::string const& pluginName,
    gz::sim::Entity const& sensor,
    std::shared_ptr<sdf::Element> const& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_manager
) {
    ImuTaskBase::setGazebo(pluginName, sensor, sdf, ecm, event_manager);

    m_initial_orientation = gz::sim::Link(m_gazebo_link).WorldPose(ecm)->Rot();
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ImuTask.hpp for more detailed
// documentation about them.

bool ImuTask::configureHook()
{
    if (! ImuTaskBase::configureHook()) {
        return false;
    }

    orientation.sourceFrame = _imu_frame.value();
    orientation.targetFrame = _world_frame.value();
    orientation.cov_orientation = _cov_orientation.value();
    orientation.cov_angular_velocity = _cov_angular_velocity.value();

    topicSubscribe(&ImuTask::readInput, m_base_topic_name + "/imu");
    return true;
}

bool ImuTask::startHook()
{
    if (! ImuTaskBase::startHook()) {
        return false;
    }
    return true;
}
void ImuTask::updateHook()
{
    ImuTaskBase::updateHook();
}
void ImuTask::errorHook()
{
    ImuTaskBase::errorHook();
}
void ImuTask::stopHook()
{
    ImuTaskBase::stopHook();
}
void ImuTask::cleanupHook()
{
    ImuTaskBase::cleanupHook();
}

void ImuTask::readInput(gz::msgs::IMU const& imuMsg) {
    if (state() != RUNNING) {
        return;
    }

    const gz::msgs::Quaternion &quat = imuMsg.orientation();
    const gz::msgs::Vector3d& avel = imuMsg.angular_velocity();
    const gz::msgs::Vector3d& linacc =  imuMsg.linear_acceleration();

    base::Time stamp = getCurrentTime(imuMsg.header().stamp());

    orientation.time = stamp;
    orientation.orientation =
        base::Orientation(quat.w(), quat.x(), quat.y(), quat.z());
    orientation.angular_velocity = base::Vector3d(avel.x(), avel.y(), avel.z());

    imuSensors.time = stamp;
    imuSensors.mag  = base::getEuler(orientation.orientation);
    imuSensors.gyro = base::Vector3d(avel.x(), avel.y(), avel.z());
    imuSensors.acc  = base::Vector3d(linacc.x(), linacc.y(), linacc.z());

    _orientation_samples.write(orientation);
    _imu_samples.write(imuSensors);
}
