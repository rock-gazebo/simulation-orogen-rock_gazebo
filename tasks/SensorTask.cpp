/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SensorTask.hpp"

#include <gz/transport.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Sensor.hh>
#include <sdf/sdf.hh>

#include <base-logging/Logging.hpp>
#include <thread>
#include "Helpers.hpp"

using namespace gz_rock;
using namespace std;

using namespace gz::sim;

SensorTask::SensorTask(std::string const& name)
    : SensorTaskBase(name)
{
}

SensorTask::SensorTask(std::string const& name, RTT::ExecutionEngine* engine)
    : SensorTaskBase(name, engine)
{
}

SensorTask::~SensorTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See SensorTask.hpp for more detailed
// documentation about them.

bool SensorTask::configureHook()
{
    if (! SensorTaskBase::configureHook())
        return false;

    // Initialize communication node and subscribe to gazebo topic
    optional<string> topic;
    {
        GazeboSync sync(*this);
        m_node.reset(new gz::transport::Node());
        topic = gz::sim::Sensor(m_sensor_entity).Topic(*m_ecm);
    }

    base::Time deadline = base::Time::now() + base::Time::fromSeconds(5);
    while (!topic.has_value() && base::Time::now() < deadline)
    {
        this_thread::sleep_for(chrono::milliseconds(100));

        GazeboSync sync(*this);
        topic = gz::sim::Sensor(m_sensor_entity).Topic(*m_ecm);
    }

    if (!topic.has_value()) {
        LOG_ERROR_S << "imu sensor " << scopedName(m_sensor_entity, *m_ecm) << " does not have a topic name";
        return false;
    }
    m_base_topic_name = topic.value();

    return true;
}
bool SensorTask::startHook()
{
    if (! SensorTaskBase::startHook())
        return false;
    return true;
}
void SensorTask::updateHook()
{
    SensorTaskBase::updateHook();
}
void SensorTask::errorHook()
{
    SensorTaskBase::errorHook();
}
void SensorTask::stopHook()
{
    SensorTaskBase::stopHook();
}
void SensorTask::cleanupHook()
{
    m_node.reset();
    SensorTaskBase::cleanupHook();
}
void SensorTask::setGazebo(
    std::string const& plugin_name,
    gz::sim::Entity const& sensor,
    std::shared_ptr<const sdf::Element> const& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_manager
)
{
    SensorTaskBase::setGazebo(plugin_name, sensor, sdf, ecm, event_manager);
    m_ecm = &ecm;
    m_sensor_entity = sensor;

    string taskName = "gazebo::" + scopedName(sensor, ecm, "::", false);
    if (!provides()) {
        throw std::runtime_error("SensorTask::provides returned NULL");
    }

    provides()->setName(taskName);
    _name.set(taskName);
}

