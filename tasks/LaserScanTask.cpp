/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "LaserScanTask.hpp"

using namespace std;
using namespace gazebo;
using namespace rock_gazebo;
using namespace base;
using namespace samples;

#define meters_to_milimeters(x) (x) * 1000

LaserScanTask::LaserScanTask(std::string const& name)
    : LaserScanTaskBase(name)
{
}

LaserScanTask::LaserScanTask(std::string const& name, RTT::ExecutionEngine* engine)
    : LaserScanTaskBase(name, engine)
{
}

LaserScanTask::~LaserScanTask()
{
}

bool LaserScanTask::configureHook()
{
    if (!LaserScanTaskBase::configureHook()) {
        return false;
    }
    m_depth_map.vertical_projection = DepthMap::PROJECTION_TYPE::POLAR;
    m_depth_map.horizontal_projection = DepthMap::PROJECTION_TYPE::POLAR;
    m_depth_map.horizontal_interval.resize(2);
    m_depth_map.vertical_interval.resize(2);
    m_depth_map.timestamps.resize(1);
    topicSubscribe(&LaserScanTask::readInput, baseTopicName + "/scan");
    return true;
}

bool LaserScanTask::startHook()
{
    if (!LaserScanTaskBase::startHook()) {
        return false;
    }

    hasNewSample = false;
    return true;
}

void LaserScanTask::updateHook()
{
    LaserScanTaskBase::updateHook();

    lock_guard<mutex> readGuard(readMutex);
    if (!hasNewSample) {
        return;
    }
    hasNewSample = false;
}

void LaserScanTask::errorHook()
{
    LaserScanTaskBase::errorHook();
}

void LaserScanTask::stopHook()
{
    LaserScanTaskBase::stopHook();
}

void LaserScanTask::cleanupHook()
{
    LaserScanTaskBase::cleanupHook();
}

void LaserScanTask::readInput(ConstLaserScanStampedPtr& laserScanMSG)
{
    if (state() != RUNNING) {
        return;
    }
    lock_guard<mutex> readGuard(readMutex);
    if (laserScanMSG->scan().has_vertical_count()) {

        outputDepthMap(laserScanMSG);
    }
    else {

        outputLaserScan(laserScanMSG);
    }
    hasNewSample = true;
}

void LaserScanTask::outputLaserScan(ConstLaserScanStampedPtr& laserScanMSG)
{
    unsigned int scan_size = laserScanMSG->scan().ranges_size();
    double range_min = laserScanMSG->scan().range_min();
    double range_max = laserScanMSG->scan().range_max();
    scan.time = getCurrentTime(laserScanMSG->time());
    scan.minRange = meters_to_milimeters(range_min);
    scan.maxRange = meters_to_milimeters(range_max);
    scan.angular_resolution = laserScanMSG->scan().angle_step();
    scan.start_angle = laserScanMSG->scan().angle_min();
    scan.ranges.resize(scan_size);
    for (unsigned int i = 0; i < scan_size; ++i) {
        double range = laserScanMSG->scan().ranges(i);
        if (range >= range_max) {
            scan.ranges[i] = base::samples::TOO_FAR;
        }
        else if (range <= range_min) {
            scan.ranges[i] = base::samples::TOO_NEAR;
        }
        else {
            scan.ranges[i] = meters_to_milimeters(range);
        }
    }
    _laser_scan_samples.write(scan);
}

void LaserScanTask::outputDepthMap(ConstLaserScanStampedPtr& laserScanMSG)
{
    unsigned int scan_size = laserScanMSG->scan().ranges_size();
    m_depth_map.vertical_size = laserScanMSG->scan().vertical_count();
    m_depth_map.horizontal_size = laserScanMSG->scan().count();
    m_depth_map.vertical_interval[0] = laserScanMSG->scan().vertical_angle_max();
    m_depth_map.vertical_interval[1] = laserScanMSG->scan().vertical_angle_min();
    m_depth_map.horizontal_interval[0] = laserScanMSG->scan().angle_min();
    m_depth_map.horizontal_interval[1] = laserScanMSG->scan().angle_max();
    m_depth_map.distances.resize(scan_size);
    m_depth_map.time = getCurrentTime(laserScanMSG->time());
    m_depth_map.timestamps[0] = m_depth_map.time;
    for (unsigned int i = 0; i < scan_size; ++i) {
        m_depth_map.distances[i] = laserScanMSG->scan().ranges(i);
    }
    _depth_map_samples.write(m_depth_map);
}
