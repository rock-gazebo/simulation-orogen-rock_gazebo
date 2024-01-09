/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PointCloudTask.hpp"

using namespace rock_gazebo;

PointCloudTask::PointCloudTask(std::string const& name)
    : PointCloudTaskBase(name)
{
}

PointCloudTask::PointCloudTask(std::string const& name, RTT::ExecutionEngine* engine)
    : PointCloudTaskBase(name, engine)
{
}

PointCloudTask::~PointCloudTask()
{
}

bool PointCloudTask::configureHook()
{
    if (!PointCloudTaskBase::configureHook())
        return false;
    return true;
}
bool PointCloudTask::startHook()
{
    if (!PointCloudTaskBase::startHook())
        return false;
    return true;
}
void PointCloudTask::updateHook()
{
    PointCloudTaskBase::updateHook();
}
void PointCloudTask::errorHook()
{
    PointCloudTaskBase::errorHook();
}
void PointCloudTask::stopHook()
{
    PointCloudTaskBase::stopHook();
}
void PointCloudTask::cleanupHook()
{
    PointCloudTaskBase::cleanupHook();
}
