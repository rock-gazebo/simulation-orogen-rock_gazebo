/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SensorTask.hpp"

#include <gz/transport.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Model.hh>
#include <sdf/sdf.hh>

#include <base-logging/Logging.hpp>

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
    m_node.reset(new gz::transport::Node());

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
    std::string const& pluginName,
    gz::sim::Entity const& sensor,
    std::shared_ptr<sdf::Element> const& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_manager
)
{
    auto world = findParentOfType(sensor, ecm, components::World::typeId);
    if (!world.has_value()) {
        throw std::runtime_error(
            "expected the sensor parent's to be included in a world"
        );
    }

    setGazeboWorld(ecm, *world);

    auto model = findParentOfType(sensor, ecm, components::Model::typeId);
    if (!model.has_value()) {
        throw std::runtime_error("expected the sensor parent's to be a model");
    }

    sdf::ElementPtr sdfLink = sdf->GetParent();
    m_sdf = sdf;
    m_gazebo_link = Model(*model).LinkByName(ecm, sdfLink->Get<string>("name"));

    m_sensor_full_name =
        scopedName(m_gazebo_link, ecm, "::", true) + "::" + m_sdf->Get<string>("name");
    m_base_topic_name =
        "~/" + scopedName(m_gazebo_link, ecm, "/") + "/" + m_sdf->Get<string>("name");

    string taskName = "gazebo::" + m_sensor_full_name;
    if (!provides())
        throw std::runtime_error("SensorTask::provides returned NULL");
    provides()->setName(taskName);
    _name.set(taskName);
}

