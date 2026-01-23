/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CameraTask.hpp"
#include <base/samples/Frame.hpp>
#include "Helpers.hpp"

using namespace std;
using namespace gz_rock;

CameraTask::CameraTask(std::string const& name)
    : CameraTaskBase(name),
    output_frame(new base::samples::frame::Frame())
{
}

CameraTask::CameraTask(std::string const& name, RTT::ExecutionEngine* engine)
    : CameraTaskBase(name, engine),
    output_frame(new base::samples::frame::Frame())
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
    if (! CameraTaskBase::configureHook())
        return false;

    topicSubscribe(&CameraTask::readInput, m_base_topic_name + "/image");
    return true;
}
bool CameraTask::startHook()
{
    if (! CameraTaskBase::startHook()) {
        return false;
    }

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
}
void CameraTask::cleanupHook()
{
    CameraTaskBase::cleanupHook();
}

void CameraTask::readInput(gz::msgs::Image const& image)
{
    if (state() != RUNNING) {
        return;
    }

    base::samples::frame::Frame *pframe = output_frame.write_access();
    auto pixel_format = gzToRock(image.pixel_format_type());
    pframe->init(
        image.width(), image.height(),
        pixel_format.first, pixel_format.second
    );

    if (image.ByteSizeLong() != pframe->image.size()) {
        throw std::runtime_error("gz_rock::CameraTask image size mismatch");
    }
    memcpy((void*)&(pframe->image.front()), (void*)image.data().data(), image.ByteSizeLong());
    pframe->time = getCurrentTime();
    pframe->frame_status = base::samples::frame::STATUS_VALID;
    output_frame.reset(pframe);

    _frame.write(pframe);
}
