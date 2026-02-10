/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "GPSTask.hpp"

#include <gz/math/CoordinateVector3.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Sensor.hh>
#include <base-logging/Logging.hpp>
#include <sdf/Element.hh>
#include <sdf/World.hh>

#include "Helpers.hpp"

using namespace std;
using namespace gz_rock;
using gz::math::CoordinateVector3;

typedef gz::math::Angle GzAngle;

GPSTask::GPSTask(std::string const& name)
    : GPSTaskBase(name)
    , deviationHorizontal(base::unknown<double>())
    , deviationVertical(base::unknown<double>())
{
    _nwu_origin.set(base::Position::Zero());
}

GPSTask::GPSTask(std::string const& name, RTT::ExecutionEngine* engine)
    : GPSTaskBase(name, engine)
    , deviationHorizontal(base::unknown<double>())
    , deviationVertical(base::unknown<double>())
{
    _nwu_origin.set(base::Position::Zero());
}

GPSTask::~GPSTask()
{
}

void GPSTask::setGazebo(
    std::string const& pluginName,
    gz::sim::Entity const& entity,
    sdf::ElementConstPtr const& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_manager
)
{
    GPSTaskBase::setGazebo(pluginName, entity, sdf, ecm, event_manager);
}


bool GPSTask::configureHook()
{
    if (! GPSTaskBase::configureHook()) {
        return false;
    }

    m_deviation_horizontal = _deviation_horizontal.get();
    m_deviation_vertical = _deviation_vertical.get();

    GazeboSync sync(*this);
    topicSubscribe(&GPSTask::readInput, m_base_topic_name);
    return true;
}
bool GPSTask::startHook()
{
    if (! GPSTaskBase::startHook()) {
        return false;
    }

    if (_use_proper_utm_conversion.get())
    {
        utm_converter.setUTMZone(_utm_zone.value());
        utm_converter.setUTMNorth(_utm_north.value());
        utm_converter.setNWUOrigin(_nwu_origin.value());
    }
    else
    {
        utm_converter.setNWUOrigin(Eigen::Vector3d::Zero());
        gazeboSpherical.SetLatitudeReference(
            GzAngle(_latitude_origin.value().getRad())
        );
        gazeboSpherical.SetLongitudeReference(
            GzAngle(_longitude_origin.value().getRad())
        );
    }
    return true;
}

void GPSTask::updateHook()
{
    GPSTaskBase::updateHook();

}
void GPSTask::errorHook()
{
    GPSTaskBase::errorHook();
}
void GPSTask::stopHook()
{
    GPSTaskBase::stopHook();
}
void GPSTask::cleanupHook()
{
    GPSTaskBase::cleanupHook();
}

void GPSTask::readInput(gz::msgs::NavSat const& msg) {
    if (state() != RUNNING) {
        return;
    }

    solution.time = getCurrentTime(msg.header().stamp());
    solution.latitude = msg.latitude_deg();
    solution.longitude = msg.longitude_deg();
    solution.altitude = msg.altitude();
    solution.positionType = gps_base::AUTONOMOUS;
    solution.noOfSatellites = 5;
    solution.geoidalSeparation = base::unknown<double>();
    solution.ageOfDifferentialCorrections = 0;
    solution.deviationAltitude = m_deviation_vertical;
    solution.deviationLatitude = m_deviation_horizontal;
    solution.deviationLongitude = m_deviation_horizontal;

    base::samples::RigidBodyState utm, position;
    if (_use_proper_utm_conversion.get())
    {
        utm = utm_converter.convertToUTM(solution);
        position = utm_converter.convertToNWU(utm);
    }
    else
    {
        auto global = CoordinateVector3::Spherical(
            GzAngle(solution.latitude * M_PI / 180),
            GzAngle(solution.longitude * M_PI / 180),
            solution.altitude
        );
        gz::math::CoordinateVector3 local =
            gazeboSpherical.LocalFromSphericalPosition(global).value();

        Eigen::Vector3d local_xyz(
            local.X().value(), local.Y().value(), local.Z().value()
        );

        utm.position = local_xyz;
        utm.cov_position = 1.0 * base::Matrix3d::Identity();
        utm.cov_position(0, 0) =
            solution.deviationLongitude * solution.deviationLongitude;
        utm.cov_position(1, 1) =
            solution.deviationLatitude * solution.deviationLatitude;
        utm.cov_position(2, 2) =
            solution.deviationAltitude * solution.deviationAltitude;

        position.position = Eigen::Vector3d(local_xyz.y(), -local_xyz.x(), local_xyz.z());
        position.cov_position = utm.cov_position;
        std::swap(position.cov_position(0, 0), position.cov_position(1, 1));
    }

    utm.time = solution.time;
    utm.sourceFrame = _gps_frame.value();
    utm.targetFrame = _utm_frame.value();

    position.time = solution.time;
    position.sourceFrame = _gps_frame.value();
    position.targetFrame = _nwu_frame.value();

    sensor_stop_guard([&]{
        _gps_solution.write(solution);
        _utm_samples.write(utm);
        _position_samples.write(position);
    });
}
