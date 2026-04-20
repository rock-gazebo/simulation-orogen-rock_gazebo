/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "BaseTask.hpp"

#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <mutex>
#include <regex>

using namespace rock_gazebo;
using namespace gz;
using namespace gz::sim;
using namespace gz::sim::systems;
using namespace std;

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
    return getCurrentTime(base::Time::fromSeconds(sim_timestamp.sec()) +
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
    if (!BaseTaskBase::configureHook())
        return false;

    return true;
}
bool BaseTask::startHook()
{
    if (!BaseTaskBase::startHook())
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

std::optional<gz::sim::Entity> BaseTask::findParentOfType(gz::sim::Entity entity,
    gz::sim::EntityComponentManager& ecm,
    ComponentTypeId const& typeId)
{
    auto search = entity;
    while (!ecm.EntityHasComponentType(search, typeId)) {
        search = ecm.ParentEntity(search);
        if (search == gz::sim::kNullEntity) {
            return std::make_optional<gz::sim::Entity>();
        }
    }
    return std::make_optional(search);
}

void BaseTask::setGazebo(gz::sim::Entity const& entity,
    sdf::ElementConstPtr const& task_sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_manager)
{
}

void BaseTask::gazeboCriticalZone()
{
    unique_lock<mutex> guard(m_gazebo_critical_mutex);

    m_gazebo_critical_zone = true;

    while (m_gazebo_critical_zone_request) {
        m_gazebo_critical_signal.notify_one();
        m_gazebo_critical_signal.wait(guard);
    }

    m_gazebo_critical_zone = false;
}

void BaseTask::enterGazeboCriticalZone()
{
    unique_lock<mutex> guard(m_gazebo_critical_mutex);

    while (m_gazebo_critical_zone_request) {
        m_gazebo_critical_signal.wait(guard);
    }

    m_gazebo_critical_zone_request = true;
    while (!m_gazebo_critical_zone) {
        m_gazebo_critical_signal.wait(guard);
    }
}

void BaseTask::leaveGazeboCriticalZone()
{
    m_gazebo_critical_zone_request = false;
    m_gazebo_critical_signal.notify_one();
}
