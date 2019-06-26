/*!
 * @file    ConversionTraits.hpp
 * @author  Philipp Leemann
 * @date    Nov, 2017
 * @version 0.0
 *
 */

#pragma once

#include <Eigen/Core>

#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/JointState.h>
#include <any_msgs/ExtendedJointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <any_msgs/ImuWithTrigger.h>
#include <any_msgs/PointContact.h>

#include "any_measurements/Time.hpp"
#include "any_measurements/Imu.hpp"
#include "any_measurements/ImuWithCovariance.hpp"
#include "any_measurements/NavSat.hpp"
#include "any_measurements/JointState.hpp"
#include "any_measurements/ExtendedJointState.hpp"
#include "any_measurements/Pose.hpp"
#include "any_measurements/Wrench.hpp"
#include "any_measurements/PoseWithCovariance.hpp"
#include "any_measurements/Twist.hpp"
#include "any_measurements/TwistWithCovariance.hpp"
#include "any_measurements/ImuWithTrigger.hpp"
#include "any_measurements/PointContact.hpp"

#include "kindr_ros/kindr_ros.hpp"

#define DEFINE_CONVERT_FUNCTIONS_WITH_RETURN        \
inline static MsgRos convert(const Msg& msg) {      \
    MsgRos msgRos;                                  \
    convert(msg, msgRos);                           \
    return msgRos;                                  \
}                                                   \
inline static Msg convert(const MsgRos& msgRos) {   \
    Msg msg;                                        \
    convert(msgRos, msg);                           \
    return msg;                                     \
}

namespace any_measurements_ros {

// Forward declaration
template<typename Msg_, typename MsgRos_>
class ConversionTraits;

/// Generic templates
template <typename StructType, typename RosType>
inline void toRos(const StructType& in, RosType& out)
{
	ConversionTraits<StructType, RosType>::convert(in, out);
}

template <typename StructType, typename RosType>
inline RosType toRos(const StructType& in)
{
    return ConversionTraits<StructType, RosType>::convert(in);
}

template <typename StructType, typename RosType>
inline void fromRos(const RosType& in, StructType& out)
{
	ConversionTraits<StructType, RosType>::convert(in, out);
}

template <typename StructType, typename RosType>
inline StructType fromRos(const RosType& in)
{
    return ConversionTraits<StructType, RosType>::convert(in);
}

/// Time
template<>
class ConversionTraits<any_measurements::Time, ros::Time> {
public:
	using Msg = any_measurements::Time;
	using MsgRos = ros::Time;

    inline static void convert(const Msg& msg, MsgRos& msgRos) {
        msgRos.sec = msg.seconds();
        msgRos.nsec = msg.nanoSeconds();
    }

	inline static MsgRos convert(const Msg& msg) {
		return MsgRos(msg.seconds(), msg.nanoSeconds());
	}

    inline static void convert(const MsgRos& msgRos, Msg& msg) {
        msg.setSeconds(msgRos.sec);
        msg.setNanoSeconds(msgRos.nsec);
    }

	inline static Msg convert(const MsgRos& msgRos) {
		return Msg(msgRos.sec, msgRos.nsec);
	}
};


/// Wrench
template<>
class ConversionTraits<any_measurements::Wrench, geometry_msgs::Wrench> {
public:
	using Msg = any_measurements::Wrench;
	using MsgRos = geometry_msgs::Wrench;

    inline static void convert(const Msg& msg, MsgRos& msgRos) {
        kindr_ros::convertToRosGeometryMsg(msg.wrench_.getForce(), msgRos.force);
        kindr_ros::convertToRosGeometryMsg(msg.wrench_.getTorque(), msgRos.torque);
    }

    inline static void convert(const MsgRos& msgRos, Msg& msg) {
        kindr_ros::convertFromRosGeometryMsg(msgRos.force, msg.wrench_.getForce());
        kindr_ros::convertFromRosGeometryMsg(msgRos.torque, msg.wrench_.getTorque());
    }

    DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

/// Wrench stamped
template<>
class ConversionTraits<any_measurements::Wrench, geometry_msgs::WrenchStamped> {
  public:
	using Msg = any_measurements::Wrench;
	using MsgRos = geometry_msgs::WrenchStamped;

    inline static void convert(const Msg& msg, MsgRos& msgRos) {
        toRos(msg, msgRos.wrench);
        toRos(msg.time_, msgRos.header.stamp);
    }

    inline static void convert(const MsgRos& msgRos, Msg& msg) {
        fromRos(msgRos.wrench, msg);
        fromRos(msgRos.header.stamp, msg.time_);
    }

    DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

// Imu
template<>
class ConversionTraits<any_measurements::Imu, sensor_msgs::Imu> {
public:
	using Msg = any_measurements::Imu;
	using MsgRos = sensor_msgs::Imu;

    inline static void convert(const Msg& msg, MsgRos& msgRos) {
        toRos(msg.time_, msgRos.header.stamp);
        kindr_ros::convertToRosGeometryMsg(msg.orientation_, msgRos.orientation);
        kindr_ros::convertToRosGeometryMsg(msg.angularVelocity_, msgRos.angular_velocity);
        kindr_ros::convertToRosGeometryMsg(msg.linearAcceleration_, msgRos.linear_acceleration);
        // Covariances are set to 0 (unknown)
        msgRos.orientation_covariance.fill(0.0);
        msgRos.angular_velocity_covariance.fill(0.0);
        msgRos.linear_acceleration_covariance.fill(0.0);
    }

    inline static void convert(const MsgRos& msgRos, Msg& msg) {
        fromRos(msgRos.header.stamp, msg.time_);
        kindr_ros::convertFromRosGeometryMsg(msgRos.orientation, msg.orientation_);
        kindr_ros::convertFromRosGeometryMsg(msgRos.angular_velocity, msg.angularVelocity_);
        kindr_ros::convertFromRosGeometryMsg(msgRos.linear_acceleration, msg.linearAcceleration_);
    }

    DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

/// IMU Covariance matrix
template<>
class ConversionTraits<any_measurements::ImuWithCovariance::CovarianceMatrix, sensor_msgs::Imu::_linear_acceleration_covariance_type> {
public:
	using Msg = any_measurements::ImuWithCovariance::CovarianceMatrix;
	using MsgRos = sensor_msgs::Imu::_linear_acceleration_covariance_type;

	inline static void convert(const Msg& msg, MsgRos& msgRos) {
		for(unsigned int i=0; i<9; i++) {
			msgRos[i] = msg(i/3, i%3);
		}
	}

	inline static void convert(const MsgRos& msgRos, Msg& msg) {
		for(unsigned int i=0; i<9; i++) {
			msg(i/3, i%3) = msgRos[i];
		}
	}

    DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

/// ImuWithCovariance
template<>
class ConversionTraits<any_measurements::ImuWithCovariance, sensor_msgs::Imu> {
  public:
	using Msg = any_measurements::ImuWithCovariance;
	using MsgRos = sensor_msgs::Imu;

	inline static void convert(const Msg& msg, MsgRos& msgRos) {
        // explicit call to ConverstionTraits required, since msg has type Imu, not ImuMinimal
		ConversionTraits<any_measurements::Imu, sensor_msgs::Imu>::convert(msg, msgRos);

		toRos(msg.orientationCovariance_, msgRos.orientation_covariance);
		toRos(msg.angularVelocityCovariance_, msgRos.angular_velocity_covariance);
		toRos(msg.linearAccelerationCovariance_, msgRos.linear_acceleration_covariance);
	}

	inline static void convert(const MsgRos& msgRos, Msg& msg) {
        // explicit call to ConverstionTraits required, since msg has type Imu, not ImuMinimal
        ConversionTraits<any_measurements::Imu, sensor_msgs::Imu>::convert(msgRos, msg);

		fromRos(msgRos.orientation_covariance, msg.orientationCovariance_);
		fromRos(msgRos.angular_velocity_covariance, msg.angularVelocityCovariance_);
		fromRos(msgRos.linear_acceleration_covariance, msg.linearAccelerationCovariance_);
	}

    DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

/// JointState
template<typename Msg>
class ConversionTraits<Msg, sensor_msgs::JointState> {
public:
    using MsgRos = sensor_msgs::JointState;

    inline static void convert(const Msg& msg, MsgRos& msgRos) {
        const auto msgSize = msg.size();
        msgRos.position.resize(msgSize);
        msgRos.velocity.resize(msgSize);
        msgRos.effort.resize(msgSize);
        for(unsigned int i=0; i<msgSize; ++i) {
            msgRos.position[i] = msg[i].position_;
            msgRos.velocity[i] = msg[i].velocity_;
            msgRos.effort[i] = msg[i].effort_;
        }
        toRos(msg[0].time_, msgRos.header.stamp);
    }

    inline static void convert(const MsgRos& msgRos, Msg& msg) {
        for(unsigned int i=0; i<msg.size(); ++i) {
            fromRos(msgRos.header.stamp, msg[i].time_);
            msg[i].position_ = msgRos.position[i];
            msg[i].velocity_ = msgRos.velocity[i];
            msg[i].effort_ = msgRos.effort[i];
        }
    }

    DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

/// ExtendedJointState
template<typename Msg>
class ConversionTraits<Msg, any_msgs::ExtendedJointState> {
public:
    using MsgRos = any_msgs::ExtendedJointState;

    inline static void convert(const Msg& msg, MsgRos& msgRos) {
        const auto msgSize = msg.size();
        msgRos.position.resize(msgSize);
        msgRos.velocity.resize(msgSize);
        msgRos.acceleration.resize(msgSize);
        msgRos.effort.resize(msgSize);
        unsigned int i = 0;
        for(const auto& message : msg) {
            msgRos.position[i] = message.position_;
            msgRos.velocity[i] = message.velocity_;
            msgRos.acceleration[i] = message.acceleration_;
            msgRos.effort[i] = message.effort_;           
            ++i;
        }

        toRos(msg.front().time_, msgRos.header.stamp);
    }

    inline static void convert(const MsgRos& msgRos, Msg& msg) {
        if(msgRos.position.size() != msg.size()) {
            std::cout << "Trying to convert extended joint state message with length " << msgRos.position.size() << ", should be " << msg.size() << "!" << std::endl;
            return;
        }

        unsigned int i = 0;
        for(auto& message : msg) {
            fromRos(msgRos.header.stamp, message.time_);
            message.position_ = msgRos.position[i];
            message.velocity_ = msgRos.velocity[i];
            message.acceleration_ = msgRos.acceleration[i];
            message.effort_ = msgRos.effort[i];
            ++i;          
        }
    }

    DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

template<>
class ConversionTraits<any_measurements::Pose, geometry_msgs::PoseStamped> {
  public:
	using Msg = any_measurements::Pose;
	using MsgRos = geometry_msgs::PoseStamped;

	inline static void convert(const Msg& msg, MsgRos& msgRos) {
		toRos(msg.time_, msgRos.header.stamp);
		kindr_ros::convertToRosGeometryMsg(msg.pose_, msgRos.pose);
	}

	inline static void convert(const MsgRos& msgRos, Msg& msg) {
		fromRos(msgRos.header.stamp, msg.time_);
		kindr_ros::convertFromRosGeometryMsg(msgRos.pose, msg.pose_);
	}

	DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

template<>
class ConversionTraits<any_measurements::NavSat, sensor_msgs::NavSatFix> {
public:
    using Msg = any_measurements::NavSat;
    using MsgRos = sensor_msgs::NavSatFix;

    inline static void convert(const Msg& msg, MsgRos& msgRos) {
		toRos(msg.time_, msgRos.header.stamp);
		msgRos.status.status = static_cast<int8_t>(msg.status_);
		msgRos.status.service = static_cast<uint16_t>(msg.service_);
		msgRos.latitude = msg.latitude_;
		msgRos.longitude = msg.longitude_;
		msgRos.altitude = msg.altitude_;
		msgRos.position_covariance_type = msg.positionCovarianceType_;
		toRos(msg.positionCovariance_, msgRos.position_covariance);
    }

    inline static void convert(const MsgRos& msgRos, Msg& msg) {
		fromRos(msgRos.header.stamp, msg.time_);
		msg.status_ = static_cast<any_measurements::NavSat::Status>(msgRos.status.status);
		msg.service_ = static_cast<any_measurements::NavSat::Service>(msgRos.status.service);
		msg.latitude_ = msgRos.latitude;
		msg.longitude_ = msgRos.longitude;
		msg.altitude_ = msgRos.altitude;
		msg.positionCovarianceType_ = static_cast<any_measurements::NavSat::CovarianceType>(msgRos.position_covariance_type);
		fromRos(msgRos.position_covariance, msg.positionCovariance_);
    }

	DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

/// Pose Covariance matrix
template<>
class ConversionTraits<any_measurements::PoseWithCovariance::CovarianceMatrix, geometry_msgs::PoseWithCovariance::_covariance_type> {
public:
    using Msg = any_measurements::PoseWithCovariance::CovarianceMatrix;
    using MsgRos = geometry_msgs::PoseWithCovariance::_covariance_type;

    inline static void convert(const Msg& msg, MsgRos& msgRos) {
        for(unsigned int i=0; i<36; i++) {
            msgRos[i] = msg(i/6, i%6);
        }
    }

    inline static void convert(const MsgRos& msgRos, Msg& msg) {
        for(unsigned int i=0; i<36; i++) {
            msg(i/6, i%6) = msgRos[i];
        }
    }

    DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

template<>
class ConversionTraits<any_measurements::PoseWithCovariance, geometry_msgs::PoseWithCovarianceStamped> {
public:
    using Msg = any_measurements::PoseWithCovariance;
    using MsgRos = geometry_msgs::PoseWithCovarianceStamped;

    inline static void convert(const Msg& msg, MsgRos& msgRos) {
        // cannot use converstion traits of Pose here, the ros message has the header saved differently
        toRos(msg.time_, msgRos.header.stamp);
        kindr_ros::convertToRosGeometryMsg(msg.pose_, msgRos.pose.pose);
        toRos(msg.covariance_, msgRos.pose.covariance);
    }

    inline static void convert(const MsgRos& msgRos, Msg& msg) {
        fromRos(msgRos.header.stamp, msg.time_);
        kindr_ros::convertFromRosGeometryMsg(msgRos.pose.pose, msg.pose_);
        fromRos(msgRos.pose.covariance, msg.covariance_);
    }

    DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

template<>
class ConversionTraits<any_measurements::Twist, geometry_msgs::TwistStamped> {
  public:
    using Msg = any_measurements::Twist;
    using MsgRos = geometry_msgs::TwistStamped;

    inline static void convert(const Msg& msg, MsgRos& msgRos) {
        toRos(msg.time_, msgRos.header.stamp);
        kindr_ros::convertToRosGeometryMsg(msg.twist_, msgRos.twist);
    }

    inline static void convert(const MsgRos& msgRos, Msg& msg) {
        fromRos(msgRos.header.stamp, msg.time_);
        kindr_ros::convertFromRosGeometryMsg(msgRos.twist, msg.twist_);
    }

    DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

template<>
class ConversionTraits<any_measurements::TwistWithCovariance, geometry_msgs::TwistWithCovarianceStamped> {
public:
    using Msg = any_measurements::TwistWithCovariance;
    using MsgRos = geometry_msgs::TwistWithCovarianceStamped;

    inline static void convert(const Msg& msg, MsgRos& msgRos) {
        // cannot use converstion traits of Twist here, the ros message has the header saved differently
        toRos(msg.time_, msgRos.header.stamp);
        kindr_ros::convertToRosGeometryMsg(msg.twist_, msgRos.twist.twist);
        toRos(msg.covariance_, msgRos.twist.covariance);
    }

    inline static void convert(const MsgRos& msgRos, Msg& msg) {
        fromRos(msgRos.header.stamp, msg.time_);
        kindr_ros::convertFromRosGeometryMsg(msgRos.twist.twist, msg.twist_);
        fromRos(msgRos.twist.covariance, msg.covariance_);
    }

    DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

// ImuWithTrigger
template<>
class ConversionTraits<any_measurements::ImuWithTrigger, any_msgs::ImuWithTrigger> {
public:
	using Msg = any_measurements::ImuWithTrigger;
	using MsgRos = any_msgs::ImuWithTrigger;

    inline static void convert(const Msg& msg, MsgRos& msgRos) {
        toRos(msg.time_, msgRos.imu.header.stamp);
        kindr_ros::convertToRosGeometryMsg(msg.orientation_, msgRos.imu.orientation);
        kindr_ros::convertToRosGeometryMsg(msg.angularVelocity_, msgRos.imu.angular_velocity);
        kindr_ros::convertToRosGeometryMsg(msg.linearAcceleration_, msgRos.imu.linear_acceleration);
        msgRos.triggerIndicator = msg.triggerIndicator_;
    }

    inline static void convert(const MsgRos& msgRos, Msg& msg) {
        fromRos(msgRos.imu.header.stamp, msg.time_);
        kindr_ros::convertFromRosGeometryMsg(msgRos.imu.orientation, msg.orientation_);
        kindr_ros::convertFromRosGeometryMsg(msgRos.imu.angular_velocity, msg.angularVelocity_);
        kindr_ros::convertFromRosGeometryMsg(msgRos.imu.linear_acceleration, msg.linearAcceleration_);
        msg.triggerIndicator_ = msgRos.triggerIndicator;
    }

    DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

/// Wrench
template<>
class ConversionTraits<any_measurements::PointContact, any_msgs::PointContact> {
public:
    using Msg = any_measurements::PointContact;
    using MsgRos = any_msgs::PointContact;

    inline static void convert(const Msg& msg, MsgRos& msgRos) {
        toRos(msg.time_, msgRos.header.stamp);
        kindr_ros::convertToRosGeometryMsg(msg.wrench_.wrench_.getForce(), msgRos.wrench.force);
        kindr_ros::convertToRosGeometryMsg(msg.wrench_.wrench_.getTorque(), msgRos.wrench.torque);
        kindr_ros::convertToRosGeometryMsg(msg.position_, msgRos.position);
        kindr_ros::convertToRosGeometryMsg(msg.twist_, msgRos.twist);
        kindr_ros::convertToRosGeometryMsg(msg.normal_, msgRos.normal);
        msgRos.state = msg.state_;
    }

    inline static void convert(const MsgRos& msgRos, Msg& msg) {
        fromRos(msgRos.header.stamp, msg.time_);
        kindr_ros::convertFromRosGeometryMsg(msgRos.wrench.force, msg.wrench_.wrench_.getForce());
        kindr_ros::convertFromRosGeometryMsg(msgRos.wrench.torque, msg.wrench_.wrench_.getTorque());
        kindr_ros::convertFromRosGeometryMsg(msgRos.position, msg.position_);
        kindr_ros::convertFromRosGeometryMsg(msgRos.twist, msg.twist_);
        kindr_ros::convertFromRosGeometryMsg(msgRos.normal, msg.normal_);
        msg.state_ = msgRos.state;
    }

    DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

} // namespace
