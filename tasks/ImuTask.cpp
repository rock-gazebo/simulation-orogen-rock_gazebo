/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ImuTask.hpp"
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/SensorsIface.hh>
#include "Gazebo7Shims.hpp"

using namespace std;
using namespace gazebo;
using namespace rock_gazebo;

typedef ignition::math::Pose3d IgnPose3d;
typedef ignition::math::Vector3d IgnVector3d;
typedef ignition::math::Quaterniond IgnQuaterniond;

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

void ImuTask::setGazeboModel(ModelPtr model, sdf::ElementPtr sdfSensor)
{
    ImuTaskBase::setGazeboModel(model, sdfSensor);
    initialOrientation = GzGetIgn((*(gazeboLink)), WorldPose, ()).Rot();
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

    gazebo::sensors::SensorPtr sensor = gazebo::sensors::get_sensor(sensorFullName);
    gazebo::sensors::ImuSensor* imu =
        dynamic_cast<gazebo::sensors::ImuSensor*>(sensor.get());

#if GAZEBO_MAJOR_VERSION >= 8
    if (_reference.get() == REFERENCE_HORIZONTAL_PLANE) {
        IgnVector3d euler = initialOrientation.Euler();
        IgnQuaterniond q  = IgnQuaterniond::EulerToQuaternion(0, 0, euler.Z());
        imu->SetWorldToReferenceOrientation(q);
    }
    else if (_reference.get() == REFERENCE_ABSOLUTE) {
        imu->SetWorldToReferenceOrientation(IgnQuaterniond::Identity);
    }
#endif

    topicSubscribe(&ImuTask::readInput, baseTopicName + "/imu");
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

void ImuTask::readInput(ConstIMUPtr & imuMsg) {
    if (state() != RUNNING) {
        return;
    }

    const gazebo::msgs::Quaternion &quat = imuMsg->orientation();
    const gazebo::msgs::Vector3d& avel = imuMsg->angular_velocity();
    const gazebo::msgs::Vector3d& linacc =  imuMsg->linear_acceleration();

    base::Time stamp = getCurrentTime(imuMsg->stamp());

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
