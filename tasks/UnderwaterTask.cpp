/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "UnderwaterTask.hpp"
#include "Gazebo7Shims.hpp"

using namespace std;
using namespace gazebo;
using namespace rock_gazebo;

UnderwaterTask::UnderwaterTask(std::string const& name)
    : UnderwaterTaskBase(name)
{
}

UnderwaterTask::~UnderwaterTask()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See UnderwaterTask.hpp for more detailed
// documentation about them.

bool UnderwaterTask::configureHook()
{
    if (!UnderwaterTaskBase::configureHook())
        return false;

    // Set gazebo topic to advertise
    node = transport::NodePtr(new transport::Node());
    node->Init();
    fluid_velocityPublisher = node->Advertise<gazebo::msgs::Vector3d>("/" + topicName);
    gzmsg << "UnderwaterTask: advertising to gazebo topic ~/" + topicName << endl;
    return true;
}

void UnderwaterTask::setGazeboModel(std::string const& pluginName, ModelPtr model)
{
    string worldName = GzGet((*(model->GetWorld())), Name, ());

    string taskName = std::regex_replace(pluginName, std::regex("__"), "::");
    provides()->setName(taskName);
    _name.set(taskName);
    string topicName = std::regex_replace(pluginName, std::regex("__"), "/") + "/fluid_velocity";
}

bool UnderwaterTask::startHook()
{
    if (!UnderwaterTaskBase::startHook())
        return false;
    return true;
}
void UnderwaterTask::updateHook()
{
    base::Vector3d velocity;
    while (_fluid_velocity.read(velocity) == RTT::NewData) {
        gazebo::msgs::Vector3d v3;
        v3.set_x(velocity.x());
        v3.set_y(velocity.y());
        v3.set_z(velocity.z());
        fluid_velocityPublisher->Publish(v3);
    }
    UnderwaterTaskBase::updateHook();
}
void UnderwaterTask::errorHook()
{
    UnderwaterTaskBase::errorHook();
}
void UnderwaterTask::stopHook()
{
    gazebo::msgs::Vector3d v3;
    v3.set_x(0);
    v3.set_y(0);
    v3.set_z(0);
    fluid_velocityPublisher->Publish(v3);
    UnderwaterTaskBase::stopHook();
}
void UnderwaterTask::cleanupHook()
{
    UnderwaterTaskBase::cleanupHook();
}
