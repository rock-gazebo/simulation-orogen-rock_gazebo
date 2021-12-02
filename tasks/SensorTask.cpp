/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SensorTask.hpp"
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>
#include <regex>
#include <base-logging/Logging.hpp>

using namespace rock_gazebo;
using namespace std;

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
    node = gazebo::transport::NodePtr( new gazebo::transport::Node() );
    node->Init();

    int attempt = 0;
    while (!world->SensorsInitialized()) {
        LOG_WARN_S << "waiting for sensors to initialize" << std::endl;
        usleep(100000);
        if (++attempt > 100) {
            LOG_ERROR_S << "sensors did not initialize" << std::endl;
            return false;
        }
    }

    mSensor = gazebo::sensors::get_sensor(sensorFullName);
    if (!mSensor) {
        LOG_ERROR_S << "no sensor named " << sensorFullName << " can be found"
                    << std::endl;
        return false;
    }
    mSensor->SetActive(false);

    return true;
}
bool SensorTask::startHook()
{
    if (! SensorTaskBase::startHook())
        return false;
    mSensor->SetActive(true);
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
    mSensor->SetActive(false);
    SensorTaskBase::stopHook();
}
void SensorTask::cleanupHook()
{
    node->Fini();
    SensorTaskBase::cleanupHook();
}
void SensorTask::setGazeboModel(ModelPtr model, sdf::ElementPtr sdfSensor)
{
    BaseTask::setGazeboWorld( model->GetWorld() );

    sdf::ElementPtr sdfLink = sdfSensor->GetParent();
    this->gazeboModel = model;
    this->sdfSensor   = sdfSensor;
    this->gazeboLink  = model->GetChildLink(sdfLink->Get<string>("name"));

    sensorFullName =
        getWorldName() + "::" +
        model->GetScopedName() + "::" +
        sdfLink->Get<string>("name") + "::" +
        sdfSensor->Get<string>("name");
    baseTopicName = "~/" + model->GetName() + "/" + sdfLink->Get<string>("name") + "/" + sdfSensor->Get<string>("name");
    std::regex gz_namespace_separator("::");
    baseTopicName = std::regex_replace(baseTopicName, gz_namespace_separator, "/");

    string taskName = "gazebo:" + getWorldName() + ":" + model->GetName() + ":" + sdfSensor->Get<string>("name");
    if(!provides())
        throw std::runtime_error("SensorTask::provides returned NULL");
    provides()->setName(taskName);
    _name.set(taskName);
}

