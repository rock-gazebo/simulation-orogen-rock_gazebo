#ifndef rock_gazebo_TYPES_HPP
#define rock_gazebo_TYPES_HPP

#include <iostream>
#include <base/Time.hpp>
#include <base/Eigen.hpp>
#include <base/Float.hpp>

namespace rock_gazebo
{
    struct LinkExport {
        // The RBS port name
        std::string port_name;
        // The RBA port name
        std::string rba_port_name;
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
}

#endif
