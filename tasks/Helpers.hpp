#ifndef OROGEN_GZ_ROCK_HELPERS_HPP
#define OROGEN_GZ_ROCK_HELPERS_HPP

#include <Eigen/Geometry>
#include <gz/math.hh>
#include <base/Float.hpp>

inline Eigen::Vector3d gz2Eigen(gz::math::Vector3d const& gz) {
    return Eigen::Vector3d(gz.X(), gz.Y(), gz.Z());
}

inline Eigen::Vector3d gz2Eigen(std::optional<gz::math::Vector3d> const& gz) {
    if (gz.has_value()) {
        return gz2Eigen(gz.value());
    }

    return Eigen::Vector3d(
        base::unknown<double>(),
        base::unknown<double>(),
        base::unknown<double>()
    );
}

inline Eigen::Quaterniond gz2Eigen(gz::math::Quaterniond const& gz) {
    return Eigen::Quaterniond(gz.Z(), gz.X(), gz.Y(), gz.Z());
}

inline Eigen::Quaterniond gz2Eigen(std::optional<gz::math::Quaterniond> const& gz) {
    if (gz.has_value()) {
        return gz2Eigen(gz.value());
    }

    return Eigen::Quaterniond(
        base::unknown<double>(),
        base::unknown<double>(),
        base::unknown<double>(),
        base::unknown<double>()
    );
}

inline Eigen::Isometry3d gz2Eigen(gz::math::Pose3d const& gz) {
    Eigen::Vector3d pos = gz2Eigen(gz.Pos());
    Eigen::Quaterniond rot = gz2Eigen(gz.Rot());
    Eigen::Isometry3d pose;
    pose.setIdentity();
    pose.rotate(rot);
    pose.translate(pos);
    return pose;
}

inline Eigen::Isometry3d gz2Eigen(std::optional<gz::math::Pose3d> const& gz) {
    if (gz.has_value()) {
        return gz2Eigen(gz.value());
    }

    Eigen::Isometry3d pose;
    pose.matrix() *= base::unknown<double>();
    return pose;
}

inline gz::math::Vector3d eigen2Gz(Eigen::Vector3d const& gz) {
    return gz::math::Vector3d(gz.x(), gz.y(), gz.z());
}

#endif