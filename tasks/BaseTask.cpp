/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "BaseTask.hpp"

#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <stdexcept>

using namespace gz_rock;
using namespace gz;
using namespace gz::sim;
using namespace gz::sim::systems;

BaseTask::BaseTask(std::string const& name)
    : BaseTaskBase(name)
{
}

BaseTask::BaseTask(std::string const& name, RTT::ExecutionEngine* engine)
    : BaseTaskBase(name, engine)
{
}

BaseTask::~BaseTask()
{
}

void BaseTask::setGazebo(EntityComponentManager & ecm, Entity world)
{
    m_world = world;
    m_ecm.reset(&ecm);
}

std::string BaseTask::getWorldName() const
{
    return *World(m_world).Name(*m_ecm);
}

void BaseTask::setSimTime(base::Time const& sim_time)
{
    m_sim_time = sim_time;
}

base::Time BaseTask::getSimTime() const
{
    return m_sim_time;
}

base::Time BaseTask::getCurrentTime(gz::msgs::Time const& sim_timestamp) const
{
    return getCurrentTime(base::Time::fromSeconds(sim_timestamp.sec())+
            base::Time::fromMicroseconds(sim_timestamp.nsec() / 1000));
}

base::Time BaseTask::getCurrentTime(base::Time sim_timestamp) const
{
    if (_use_sim_time)
        return sim_timestamp;
    else
        return base::Time::now() - (getSimTime() - sim_timestamp);
}

base::Time BaseTask::getCurrentTime() const
{
    if (_use_sim_time)
        return getSimTime();
    else
        return base::Time::now();
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See BaseTask.hpp for more detailed
// documentation about them.

bool BaseTask::configureHook()
{
    if (! BaseTaskBase::configureHook())
        return false;

    if (m_world == kNullEntity) {
        throw std::logic_error(
            "must call setGazebo before configuring a gz_rock::BaseTask task"
        );
    }

    return true;
}
bool BaseTask::startHook()
{
    if (! BaseTaskBase::startHook())
        return false;
    return true;
}
void BaseTask::updateHook()
{
    BaseTaskBase::updateHook();
}
void BaseTask::errorHook()
{
    BaseTaskBase::errorHook();
}
void BaseTask::stopHook()
{
    BaseTaskBase::stopHook();
}
void BaseTask::cleanupHook()
{
    BaseTaskBase::cleanupHook();
}

std::optional<gz::sim::Entity> BaseTask::findParentOfType(
    gz::sim::Entity entity, gz::sim::EntityComponentManager& ecm,
    ComponentTypeId const& typeId
) {
    auto search = entity;
    while (!ecm.EntityHasComponentType(search, typeId)) {
        search = ecm.ParentEntity(search);
        if (search == gz::sim::kNullEntity) {
            return std::make_optional<gz::sim::Entity>();
        }
    }
    return std::make_optional(search);
}
