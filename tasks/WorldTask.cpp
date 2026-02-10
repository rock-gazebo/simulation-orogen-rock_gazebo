/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WorldTask.hpp"
#include <gz/sim/World.hh>

using namespace gz_rock;
using namespace gz::sim;

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

void WorldTask::setGazebo(Entity const& worldEntity,
    std::shared_ptr<const sdf::Element> const& sdf,
    EntityComponentManager& ecm,
    EventManager& event_manager)
{
    m_world = worldEntity;
    m_ecm = &ecm;

    provides()->setName("gazebo::" + getWorldName());
    _name.set(getWorldName());
}

void WorldTask::setSimTime(base::Time const& time)
{
    m_sim_time = time;
}

std::string WorldTask::getWorldName() const
{
    return World(m_world).Name(*m_ecm).value_or("world");
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See WorldTask.hpp for more detailed
// documentation about them.

bool WorldTask::configureHook()
{
    if (!WorldTaskBase::configureHook())
        return false;
    return true;
}
bool WorldTask::startHook()
{
    if (!WorldTaskBase::startHook())
        return false;
    return true;
}
void WorldTask::updateHook()
{
    _time.write(m_sim_time);

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
