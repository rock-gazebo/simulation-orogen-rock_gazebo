/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ModelPluginTask.hpp"
#include <regex>

using namespace rock_gazebo;

ModelPluginTask::ModelPluginTask(std::string const& name)
    : ModelPluginTaskBase(name)
{
}

ModelPluginTask::~ModelPluginTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ModelPluginTask.hpp for more detailed
// documentation about them.

bool ModelPluginTask::configureHook()
{
    if (! ModelPluginTaskBase::configureHook())
        return false;
    return true;
}
bool ModelPluginTask::startHook()
{
    if (! ModelPluginTaskBase::startHook())
        return false;
    return true;
}
void ModelPluginTask::updateHook()
{
    ModelPluginTaskBase::updateHook();
}
void ModelPluginTask::errorHook()
{
    ModelPluginTaskBase::errorHook();
}
void ModelPluginTask::stopHook()
{
    ModelPluginTaskBase::stopHook();
}
void ModelPluginTask::cleanupHook()
{
    ModelPluginTaskBase::cleanupHook();
}

std::string ModelPluginTask::getNamespaceFromPluginName(std::string const& plugin_name)
{
    return std::regex_replace(plugin_name, std::regex("__"), "/");
}

void ModelPluginTask::setGazeboPluginTaskName(std::string const& plugin_task_name)
{
    provides()->setName(plugin_task_name);
    _name.set(plugin_task_name);
}
