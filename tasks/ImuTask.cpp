/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ImuTask.hpp"
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/SensorsIface.hh>

using namespace std;
using namespace gazebo;
using namespace rock_gazebo;

typedef ignition::math::Pose3d IgnPose3d;
typedef ignition::math::Vector3d IgnVector3d;
typedef ignition::math::Quaterniond IgnQuaterniond;

ImuTask::ImuTask(std::string const& name)
    : ImuTaskBase(name)
{
}

ImuTask::ImuTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ImuTaskBase(name, engine)
{
}

ImuTask::~ImuTask()
{
}

void ImuTask::setGazeboModel(ModelPtr model, sdf::ElementPtr sdfSensor)
{
    ImuTaskBase::setGazeboModel(model, sdfSensor);
    initialOrientation = gazeboLink->WorldPose().Rot();
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ImuTask.hpp for more detailed
// documentation about them.

bool ImuTask::configureHook()
{
    if (! ImuTaskBase::configureHook())
        return false;

    topicSubscribe(&ImuTask::readInput, baseTopicName + "/imu");
    return true;
}
bool ImuTask::startHook()
{
    if (! ImuTaskBase::startHook())
        return false;

    samples.clear();
    gazebo::sensors::SensorPtr sensor = gazebo::sensors::get_sensor(sensorFullName);
    gazebo::sensors::ImuSensor* imu = dynamic_cast<gazebo::sensors::ImuSensor*>(sensor.get());
    if (_reference.get() == REFERENCE_HORIZONTAL_PLANE) {
        IgnVector3d euler = initialOrientation.Euler();
        IgnQuaterniond q  = IgnQuaterniond::EulerToQuaternion(0, 0, euler.Z());
        imu->SetWorldToReferenceOrientation(q);
    }
    else if (_reference.get() == REFERENCE_ABSOLUTE) {
        imu->SetWorldToReferenceOrientation(IgnQuaterniond::Identity);
    }
    return true;
}
void ImuTask::updateHook()
{
    ImuTaskBase::updateHook();

    Samples samples;
    { lock_guard<mutex> readGuard(readMutex);
        samples = move(this->samples);
    }

    for (auto const& sample : samples)
    {
        _orientation_samples.write(sample.first);
        _imu_samples.write(sample.second);
    }
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

void ImuTask::readInput( ConstIMUPtr & imuMsg)
{ lock_guard<mutex> readGuard(readMutex);
    const gazebo::msgs::Quaternion &quat = imuMsg->orientation();
    const gazebo::msgs::Vector3d& avel =  imuMsg->angular_velocity();
    const gazebo::msgs::Vector3d& linacc =  imuMsg->linear_acceleration();

    base::Time stamp = getCurrentTime(imuMsg->stamp());

    base::samples::IMUSensors imu_sensors;
    base::samples::RigidBodyState orientation;
    orientation.time = stamp;
    orientation.sourceFrame = _imu_frame.value();
    orientation.targetFrame = _world_frame.value();
    orientation.orientation = base::Orientation(quat.w(),quat.x(),quat.y(),quat.z());
    orientation.angular_velocity = base::Vector3d(avel.x(),avel.y(),avel.z());

    imu_sensors.time = stamp;
    imu_sensors.mag  = base::getEuler(orientation.orientation);
    imu_sensors.gyro = base::Vector3d(avel.x(),avel.y(),avel.z());
    imu_sensors.acc  = base::Vector3d(linacc.x(), linacc.y(), linacc.z());

    samples.push_back(make_pair(orientation, imu_sensors));
}

