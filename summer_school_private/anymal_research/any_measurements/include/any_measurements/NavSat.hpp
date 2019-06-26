/*!
 * @file    NavSat.hpp
 * @author  Philipp Leemann
 * @date    Mar 03, 2017
 * @version 0.0
 *
 */
#pragma once

#include <Eigen/Core>

#include "any_measurements/Time.hpp"

#include <string>


namespace any_measurements {

struct NavSat
{
 public:
    using CovarianceMatrix = Eigen::Matrix<double, 3, 3>;

    // Whether to output an augmented fix is determined by both the fix
    // type and the last time differential corrections were received.  A
    // fix is valid when status >= STATUS_FIX.
    enum Status : int8_t {
        // same as in sensor_msgs/NavSatStatus.msg
       STATUS_NO_FIX=-1,
       STATUS_FIX=0,
       STATUS_SBAS_FIX=1,
       STATUS_GBAS_FIX=2
    };

    // Bits defining which Global Navigation Satellite System signals were
    // used by the receiver.
    enum Service : uint16_t {
        // same as in sensor_msgs/NavSatStatus.msg
        SERVICE_NONE =    0,
        SERVICE_GPS =     1,
        SERVICE_GLONASS = 2,
        SERVICE_COMPASS = 4,      // includes BeiDou.
        SERVICE_GALILEO = 8
    };

    enum CovarianceType : uint8_t {
        // same as in sensor_msgs/NavSatFix.msg
        COVARIANCE_TYPE_UNKNOWN=0,
        COVARIANCE_TYPE_APPROXIMATED,
        COVARIANCE_TYPE_DIAGONAL_KNOWN,
        COVARIANCE_TYPE_KNOWN
    };

    NavSat():
        time_(),
        status_(STATUS_NO_FIX),
        service_(SERVICE_GPS),
        latitude_(0.0),
        longitude_(0.0),
        altitude_(0.0),
        positionCovarianceType_(COVARIANCE_TYPE_UNKNOWN),
        positionCovariance_(CovarianceMatrix::Zero())
    {
    }

    NavSat(const Time& time,
        const Status status,
        const Service service,
        const double latitude,
        const double longitude,
        const double altitude,
        const CovarianceType positionCovarianceType,
        const CovarianceMatrix& positionCovariance
    ):
        time_(time),
        status_(status),
        service_(service),
        latitude_(latitude),
        longitude_(longitude),
        altitude_(altitude),
        positionCovarianceType_(positionCovarianceType),
        positionCovariance_(positionCovariance)
    {
    }

    virtual ~NavSat() = default;

 public:
    Time time_;

    Status status_;
    Service service_;

    double latitude_;
    double longitude_;
    double altitude_;

    CovarianceType positionCovarianceType_;
    CovarianceMatrix positionCovariance_;

};

inline std::ostream& operator<<(std::ostream& os, const NavSat& navSat)
{
    return os << "NavSat (Time: " << navSat.time_ << ")"
              << "\n Status: " << navSat.status_
              << "\n Service: " << static_cast<uint16_t>(navSat.service_)
              << "\n Latitude: " << navSat.latitude_
              << "\n Longitude: " << navSat.longitude_
              << "\n Altitude: " << navSat.altitude_
              << "\n PositionCovarianceType: " << navSat.positionCovarianceType_
              << "\n PositionCovariance: " << navSat.positionCovariance_;
}


} /* namespace any_measurements */
