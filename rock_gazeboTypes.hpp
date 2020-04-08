#ifndef rock_gazebo_TYPES_HPP
#define rock_gazebo_TYPES_HPP

#include <iostream>
#include <base/Time.hpp>
#include <base/Eigen.hpp>
#include <base/Float.hpp>
#include <gazebo/gazebo_config.h>
#include <vector>

namespace rock_gazebo
{
    enum IMUReference {
        REFERENCE_INITIAL_POSE,
#if GAZEBO_MAJOR_VERSION >= 8
        REFERENCE_HORIZONTAL_PLANE,
        REFERENCE_ABSOLUTE
#endif
    };

    struct LinkExport {
        // The port name
        std::string port_name;
        // The RBS sourceFrame, leave empty to use source_link
        std::string source_frame;
        // The RBS targetFrame, leave empty to use target_link
        std::string target_frame;
        // The source gazebo link, leave empty for "world"
        std::string source_link;
        // The target gazebo link, leave empty for "world"
        std::string target_link;
        // The period of update the output port
        base::Time port_period;
        // The position covariance
        base::Matrix3d cov_position;
        // The orientation covariance
        base::Matrix3d cov_orientation;
        // The velocity covariance
        base::Matrix3d cov_velocity;
        // The angular velocity covariance
        base::Matrix3d cov_angular_velocity;
        // The acceleration covariance
        base::Matrix3d cov_acceleration;
        // The angular acceleration covariance
        base::Matrix3d cov_angular_acceleration;

        LinkExport()
            : cov_position(base::Matrix3d::Ones() * base::unset<double>())
            , cov_orientation(base::Matrix3d::Ones() * base::unset<double>())
            , cov_velocity(base::Matrix3d::Ones() * base::unset<double>()) {}
    };

    struct JointExport {
        /** Basename for the in/out ports. The actual port names is this name
         * with either _cmd and _samples appended.
         */
        std::string port_name;
        /** If given, remove this prefix from the joint names at the port
         * interface
         */
        std::string prefix;
        /** The joints that will be exported
         */
        std::vector<std::string> joints;
        /** Position offsets for each of the exported joint
         *
         * It either can be empty, or has to have the same size than \c joints
         */
        std::vector<double> position_offsets;
        /** The period of update of the output port */
        base::Time port_period;
        /** Whether to validate the names of the incoming joint commands,
         * or just assume they are provided in the same order than \c joints
         */
        bool ignore_joint_names;

        JointExport()
            : ignore_joint_names(false) {}
    };
}

#endif
