/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROCK_GAZEBO_LASERSCANTASK_TASK_HPP
#define ROCK_GAZEBO_LASERSCANTASK_TASK_HPP

#include "rock_gazebo/LaserScanTaskBase.hpp"
#include <base/samples/DepthMap.hpp>
#include <gz/msgs/laserscan.pb.h>
#include <gz/sim/System.hh>
#include <gz/transport.hh>

namespace rock_gazebo {
    class LaserScanTask : public LaserScanTaskBase {
        friend class LaserScanTaskBase;

    public:
        LaserScanTask(std::string const& name = "rock_gazebo::LaserScanTask");
        LaserScanTask(std::string const& name, RTT::ExecutionEngine* engine);
        ~LaserScanTask();

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();

    private:
        void outputLaserScan(gz::msgs::LaserScan const& scan);
        void outputDepthMap(gz::msgs::LaserScan const& scan);
        void readInput(gz::msgs::LaserScan const& scan);
        bool hasNewSample;
        base::samples::LaserScan scan;
        base::samples::DepthMap m_depth_map;
    };
}

#endif
