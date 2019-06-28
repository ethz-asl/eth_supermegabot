/*!
 * @file    ConvertRosMessages.hpp
 * @author  Philipp Leemann
 * @date    Sep 19, 2016
 * @version 0.0
 *
 */
#pragma once

#include "any_measurements_ros/ConversionTraits.hpp"

// Functions in this header are deprecated, use conversion traits ConversionTraits.hpp instead!

namespace any_measurements_ros {

/// Time
inline ros::Time toRos(const any_measurements::Time& time) {
    return ros::Time(time.seconds(), time.nanoSeconds());
}

inline any_measurements::Time fromRos(const ros::Time& time) {
    return any_measurements::Time(time.sec, time.nsec);
}

/// Imu Covariance matrix
inline sensor_msgs::Imu::_linear_acceleration_covariance_type toRos(const any_measurements::ImuWithCovariance::CovarianceMatrix& covariance) {
    return ConversionTraits<any_measurements::ImuWithCovariance::CovarianceMatrix, sensor_msgs::Imu::_linear_acceleration_covariance_type>::convert(covariance);
}

inline any_measurements::ImuWithCovariance::CovarianceMatrix fromRos(const sensor_msgs::Imu::_linear_acceleration_covariance_type& msg) {
    return ConversionTraits<any_measurements::ImuWithCovariance::CovarianceMatrix, sensor_msgs::Imu::_linear_acceleration_covariance_type>::convert(msg);
}

/// Imu
inline sensor_msgs::Imu toRos(const any_measurements::Imu& imu) {
    return ConversionTraits<any_measurements::Imu, sensor_msgs::Imu>::convert(imu);
}

inline any_measurements::Imu fromRos(const sensor_msgs::Imu& msg) {
    return ConversionTraits<any_measurements::Imu, sensor_msgs::Imu>::convert(msg);
}

/// Imu with trigger
inline any_msgs::ImuWithTrigger toRos(const any_measurements::ImuWithTrigger& imu) {
    return ConversionTraits<any_measurements::ImuWithTrigger, any_msgs::ImuWithTrigger>::convert(imu);
}

inline any_measurements::ImuWithTrigger fromRos(const any_msgs::ImuWithTrigger& msg) {
    return ConversionTraits<any_measurements::ImuWithTrigger, any_msgs::ImuWithTrigger>::convert(msg);
}

/// ImuWithCovariance
inline sensor_msgs::Imu toRos(const any_measurements::ImuWithCovariance& imu) {
    return ConversionTraits<any_measurements::ImuWithCovariance, sensor_msgs::Imu>::convert(imu);
}

// no fromRos for ImuWithCovariance available, would be amigous with Imu conversion

/// NavSat
inline sensor_msgs::NavSatFix toRos(const any_measurements::NavSat& navSat) {
    return ConversionTraits<any_measurements::NavSat, sensor_msgs::NavSatFix>::convert(navSat);
}

inline any_measurements::NavSat fromRos(const sensor_msgs::NavSatFix& msg) {
    return ConversionTraits<any_measurements::NavSat, sensor_msgs::NavSatFix>::convert(msg);
}

/// PoseStamped
inline geometry_msgs::PoseStamped toRos(const any_measurements::Pose& pose) {
    return ConversionTraits<any_measurements::Pose, geometry_msgs::PoseStamped>::convert(pose);
}

inline any_measurements::Pose fromRos(const geometry_msgs::PoseStamped& msg) {
    return ConversionTraits<any_measurements::Pose, geometry_msgs::PoseStamped>::convert(msg);
}

/// Wrench

// no toRos(..) provided, would be ambigous with the toRos of WrenchStamped

inline any_measurements::Wrench fromRos(const geometry_msgs::Wrench& msg) {
    return ConversionTraits<any_measurements::Wrench, geometry_msgs::Wrench>::convert(msg);
}


/// Wrench stamped
inline geometry_msgs::WrenchStamped toRos(const any_measurements::Wrench& wrench) {
    return ConversionTraits<any_measurements::Wrench, geometry_msgs::WrenchStamped>::convert(wrench);
}

inline any_measurements::Wrench fromRos(const geometry_msgs::WrenchStamped& msg) {
    return ConversionTraits<any_measurements::Wrench, geometry_msgs::WrenchStamped>::convert(msg);
}

/// Pose covariance matrix

inline geometry_msgs::PoseWithCovariance::_covariance_type toRos(const any_measurements::PoseWithCovariance::CovarianceMatrix& covariance) {
    return ConversionTraits<any_measurements::PoseWithCovariance::CovarianceMatrix, geometry_msgs::PoseWithCovariance::_covariance_type>
            ::convert(covariance);
}

inline any_measurements::PoseWithCovariance::CovarianceMatrix fromRos(const geometry_msgs::PoseWithCovariance::_covariance_type& msg) {
    return ConversionTraits<any_measurements::PoseWithCovariance::CovarianceMatrix, geometry_msgs::PoseWithCovariance::_covariance_type>::convert(msg);
}

// Pose with covariance stamped
inline geometry_msgs::PoseWithCovarianceStamped toRos(const any_measurements::PoseWithCovariance& poseWithCov) {
    return ConversionTraits<any_measurements::PoseWithCovariance, geometry_msgs::PoseWithCovarianceStamped>::convert(poseWithCov);
}

inline any_measurements::PoseWithCovariance fromRos(const geometry_msgs::PoseWithCovarianceStamped& msg) {
    return ConversionTraits<any_measurements::PoseWithCovariance, geometry_msgs::PoseWithCovarianceStamped>::convert(msg);
}

// Twist with covariance stamped
inline geometry_msgs::TwistWithCovarianceStamped toRos(const any_measurements::TwistWithCovariance& twistWithCov) {
    return ConversionTraits<any_measurements::TwistWithCovariance, geometry_msgs::TwistWithCovarianceStamped>::convert(twistWithCov);
}

inline any_measurements::TwistWithCovariance fromRos(const geometry_msgs::TwistWithCovarianceStamped& msg) {
    return ConversionTraits<any_measurements::TwistWithCovariance, geometry_msgs::TwistWithCovarianceStamped>::convert(msg);
}

} /* namespace any_measurements */
