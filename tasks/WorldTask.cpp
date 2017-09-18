/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WorldTask.hpp"
#include "Gazebo7Shims.hpp"

using namespace rock_gazebo;
using namespace gazebo;

WorldTask::WorldTask(std::string const& name)
    : WorldTaskBase(name)
{
}

WorldTask::WorldTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WorldTaskBase(name, engine)
{
}

WorldTask::~WorldTask()
{
}


void WorldTask::setGazeboWorld(physics::WorldPtr _world)
{
    provides()->setName("gazebo:" + GzGet((*_world), Name, ()));
    world = _world;
    _name.set(getWorldName());
}

std::string WorldTask::getWorldName() const
{
    return GzGet((*world), Name, ());
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See WorldTask.hpp for more detailed
// documentation about them.

bool WorldTask::configureHook()
{
    if (! WorldTaskBase::configureHook())
        return false;
    return true;
}
bool WorldTask::startHook()
{
    if (! WorldTaskBase::startHook())
        return false;
    return true;
}
void WorldTask::updateHook()
{
    common::Time sim_time = GzGet((*world), SimTime, ());
    _time.write(
            base::Time::fromSeconds(sim_time.sec) +
            base::Time::fromMicroseconds(sim_time.nsec / 1000));
    WorldTaskBase::updateHook();
}
void WorldTask::errorHook()
{
    WorldTaskBase::errorHook();
}
void WorldTask::stopHook()
{
    WorldTaskBase::stopHook();
}
void WorldTask::cleanupHook()
{
    WorldTaskBase::cleanupHook();
}
