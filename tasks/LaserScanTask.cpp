/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "LaserScanTask.hpp"

using namespace std;
using namespace gz_rock;
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

    topicSubscribe(&LaserScanTask::readInput, m_base_topic_name + "/scan");
    return true;
}

bool LaserScanTask::startHook()
{
    if (!LaserScanTaskBase::startHook()) {
        return false;
    }

    return true;
}

void LaserScanTask::updateHook()
{
    LaserScanTaskBase::updateHook();
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

void LaserScanTask::readInput(gz::msgs::LaserScan const& scan)
{
    if (state() != RUNNING) {
        return;
    }

    if (scan.vertical_count() > 1) {

        outputDepthMap(scan);
    }
    else {

        outputLaserScan(scan);
    }
}

void LaserScanTask::outputLaserScan(gz::msgs::LaserScan const& gz_scan)
{
    unsigned int scan_size = gz_scan.ranges_size();
    double range_min = gz_scan.range_min();
    double range_max = gz_scan.range_max();
    scan.time = getCurrentTime(gz_scan.header().stamp());
    scan.minRange = meters_to_milimeters(range_min);
    scan.maxRange = meters_to_milimeters(range_max);
    scan.angular_resolution = gz_scan.angle_step();
    scan.start_angle = gz_scan.angle_min();
    scan.ranges.resize(scan_size);
    for (unsigned int i = 0; i < scan_size; ++i) {
        double range = gz_scan.ranges(i);
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

void LaserScanTask::outputDepthMap(gz::msgs::LaserScan const& scan)
{
    unsigned int scan_size = scan.ranges_size();
    m_depth_map.vertical_size = scan.vertical_count();
    m_depth_map.horizontal_size = scan.count();
    m_depth_map.vertical_interval[0] = -scan.vertical_angle_min();
    m_depth_map.vertical_interval[1] = -scan.vertical_angle_max();
    m_depth_map.horizontal_interval[0] = scan.angle_min();
    m_depth_map.horizontal_interval[1] = scan.angle_max();
    m_depth_map.distances.resize(scan_size);
    m_depth_map.time = getCurrentTime(scan.header().stamp());
    m_depth_map.timestamps[0] = m_depth_map.time;
    for (unsigned int i = 0; i < scan_size; ++i) {
        m_depth_map.distances[i] = scan.ranges(i);
    }
    _depth_map_samples.write(m_depth_map);
}
