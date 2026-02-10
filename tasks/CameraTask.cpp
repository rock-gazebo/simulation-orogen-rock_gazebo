/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CameraTask.hpp"
#include "Helpers.hpp"
#include <base/samples/Frame.hpp>
#include <stdexcept>

#include <base-logging/Logging.hpp>

using namespace std;
using namespace gz_rock;

CameraTask::CameraTask(std::string const& name)
    : CameraTaskBase(name)
{
}

CameraTask::CameraTask(std::string const& name, RTT::ExecutionEngine* engine)
    : CameraTaskBase(name, engine)
{
}

CameraTask::~CameraTask()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See CameraTask.hpp for more detailed
// documentation about them.

bool CameraTask::configureHook()
{
    if (!CameraTaskBase::configureHook())
        return false;

    return true;
}
bool CameraTask::startHook()
{
    if (!CameraTaskBase::startHook()) {
        return false;
    }

    topicSubscribe(&CameraTask::readInput, m_base_topic_name);
    return true;
}
void CameraTask::updateHook()
{
    CameraTaskBase::updateHook();
}
void CameraTask::errorHook()
{
    CameraTaskBase::errorHook();
}
void CameraTask::stopHook()
{
    CameraTaskBase::stopHook();

    m_node->Unsubscribe(m_base_topic_name);
}
void CameraTask::cleanupHook()
{
    CameraTaskBase::cleanupHook();
}

void CameraTask::readInput(gz::msgs::Image const& image)
{
    unique_ptr<base::samples::frame::Frame> pframe;
    bool ready =
        sensor_stop_guard([&] { pframe.reset(output_frame.try_write_access()); });
    if (!ready) {
        return;
    }

    if (!pframe) {
        pframe.reset(new base::samples::frame::Frame());
    }
    auto pixel_format = gzToRock(image.pixel_format_type());
    pframe->init(image.width(),
        image.height(),
        pixel_format.first,
        pixel_format.second,
        -1);

    size_t gz_size = image.step() * image.height();
    if (gz_size != pframe->image.size()) {
        LOG_ERROR_S << "CameraTask does not support having line padding. "
                    << "Rock expects " << pframe->image.size() << " but Gazebo reports "
                    << image.ByteSizeLong();
        throw std::runtime_error("gz_rock::CameraTask image size mismatch");
    }
    memcpy((void*)&(pframe->image.front()), (void*)image.data().data(), gz_size);
    pframe->time = getCurrentTime();
    pframe->received_time = base::Time::now();
    pframe->frame_status = base::samples::frame::STATUS_VALID;

    sensor_stop_guard([&] {
        output_frame.reset(pframe.release());
        _frame.write(output_frame);
    });
}
