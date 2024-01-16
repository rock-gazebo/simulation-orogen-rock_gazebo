/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROCK_GAZEBO_LASERSCANTASK_TASK_HPP
#define ROCK_GAZEBO_LASERSCANTASK_TASK_HPP

#include "rock_gazebo/LaserScanTaskBase.hpp"
#include <base/samples/DepthMap.hpp>
#include <gazebo/msgs/laserscan_stamped.pb.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

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
        void outputLaserScan(ConstLaserScanStampedPtr& laserScanMSG);
        void outputDepthMap(ConstLaserScanStampedPtr& laserScanMSG);
        void readInput(ConstLaserScanStampedPtr& laserScanMSG);
        bool hasNewSample;
        base::samples::LaserScan scan;
        base::samples::DepthMap m_depth_map;
    };
}

#endif
