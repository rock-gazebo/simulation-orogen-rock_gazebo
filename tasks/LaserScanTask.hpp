/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROCK_GAZEBO_LASERSCANTASK_TASK_HPP
#define ROCK_GAZEBO_LASERSCANTASK_TASK_HPP

#include "gz_rock/LaserScanTaskBase.hpp"
#include <base/samples/DepthMap.hpp>
#include <gz/msgs/laserscan.pb.h>
#include <gz/sim/System.hh>
#include <gz/transport.hh>

namespace gz_rock {
    class LaserScanTask : public LaserScanTaskBase {
        friend class LaserScanTaskBase;

    public:
        LaserScanTask(std::string const& name = "gz_rock::LaserScanTask");
        LaserScanTask(std::string const& name, RTT::ExecutionEngine* engine);
        ~LaserScanTask();

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();

    private:
        void outputLaserScan(gz::msgs::ConstLaserScanSharedPtr& laserScanMSG);
        void outputDepthMap(gz::msgs::ConstLaserScanSharedPtr& laserScanMSG);
        void readInput(gz::msgs::ConstLaserScanSharedPtr& laserScanMSG);
        bool hasNewSample;
        base::samples::LaserScan scan;
        base::samples::DepthMap m_depth_map;
    };
}

#endif
