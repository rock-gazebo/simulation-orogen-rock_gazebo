/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ThrusterTask.hpp"
#include "Gazebo7Shims.hpp"

using namespace std;
using namespace gazebo;
using namespace rock_gazebo;

ThrusterTask::ThrusterTask(string const& name)
    : ThrusterTaskBase(name)
{
}

ThrusterTask::ThrusterTask(string const& name, RTT::ExecutionEngine* engine)
    : ThrusterTaskBase(name, engine)
{
}

ThrusterTask::~ThrusterTask()
{
}

bool ThrusterTask::configureHook()
{
    if (! ThrusterTaskBase::configureHook())
        return false;

    // Set gazebo topic to advertise
    node = transport::NodePtr( new transport::Node() );
    node->Init();
    thrusterPublisher = node->Advertise<ThrustersMSG>("/" + topicName);
    gzmsg << "ThrusterTask: advertising to gazebo topic /" + topicName << endl;
    return true;
}


bool ThrusterTask::startHook()
{
    if (! ThrusterTaskBase::startHook())
        return false;

    return true;
}

void ThrusterTask::updateHook()
{
    ThrusterTaskBase::updateHook();

    // Read Rock input port and update the message
    if (_thrusters_cmd.read( jointsCMD ) != RTT::NewData)
        return;

    if (jointsCMD.names.empty()) {
        exception(NO_JOINT_NAMES);
        return;
    }

    ThrustersMSG thrustersMSG;
    for(vector<string>::iterator jointName = jointsCMD.names.begin();
            jointName != jointsCMD.names.end(); ++jointName)
    {
        base::JointState jointState = jointsCMD.getElementByName(*jointName);
        gazebo_thruster::msgs::Thruster* thruster = thrustersMSG.add_thrusters();
        thruster->set_name( *jointName );
        if( jointState.isEffort() )
            thruster->set_effort( jointState.effort );
    }

    // Write in gazebo topic
    if(thrusterPublisher->HasConnections())
    {
        thrusterPublisher->Publish(thrustersMSG);
    }else{
        gzmsg << "ThrusterTask: publisher has no connections. Going into exception" << endl;
        exception(NO_TOPIC_CONNECTION);
    }
    jointsCMD.time = base::Time::now();
    _joint_samples.write( jointsCMD );
}

void ThrusterTask::errorHook()
{
    ThrusterTaskBase::errorHook();
}

void ThrusterTask::stopHook()
{
    ThrusterTaskBase::stopHook();
}

void ThrusterTask::cleanupHook()
{
    ThrusterTaskBase::cleanupHook();

    node->Fini();
}

void ThrusterTask::setGazeboModel( std::string const& pluginName, ModelPtr model )
{
    topicName = getNamespaceFromPluginName(pluginName) + "/thrusters";
}



