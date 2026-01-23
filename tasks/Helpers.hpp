#ifndef OROGEN_GZ_ROCK_HELPERS_HPP
#define OROGEN_GZ_ROCK_HELPERS_HPP

#include <Eigen/Geometry>
#include <gz/math.hh>
#include <gz/msgs/details/image.pb.h>
#include <gz/msgs/image.pb.h>
#include <base/Float.hpp>
#include <base/samples/Frame.hpp>

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

inline std::pair<int, base::samples::frame::frame_mode_t> gzToRock(
    gz::msgs::PixelFormatType gz
) {
    using namespace base::samples::frame;
    using std::make_pair;

    switch (gz) {
        case gz::msgs::RGB_INT8:
            return make_pair(8, MODE_RGB);
        case gz::msgs::RGB_INT16:
            return make_pair(16, MODE_RGB);
        case gz::msgs::RGB_INT32:
            return make_pair(32, MODE_RGB);
        case gz::msgs::RGBA_INT8:
            return make_pair(8, MODE_RGB32);
        case gz::msgs::BGR_INT8:
            return make_pair(8, MODE_BGR);
        case gz::msgs::BGR_INT16:
            return make_pair(16, MODE_BGR);
        case gz::msgs::BGR_INT32:
            return make_pair(32, MODE_BGR);
        case gz::msgs::BAYER_RGGB8:
            return make_pair(8, MODE_BAYER_RGGB);
        case gz::msgs::BAYER_BGGR8:
            return make_pair(8, MODE_BAYER_BGGR);
        case gz::msgs::BAYER_GBRG8:
            return make_pair(8, MODE_BAYER_GBRG);
        case gz::msgs::BAYER_GRBG8:
            return make_pair(8, MODE_BAYER_GRBG);
        default:
            throw std::invalid_argument(
                "received image that cannot represented in Rock"
            );
    }

}

#endif