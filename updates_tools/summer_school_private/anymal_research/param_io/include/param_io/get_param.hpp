/*
 * get_param.hpp
 *
 *  Created on: Nov 23, 2015
 *      Author: Remo Diethelm
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once


// ros
#include <ros/ros.h>
#include <XmlRpc.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <Eigen/Dense>

namespace param_io {

namespace internal {

inline std::string getAbsoluteKey(const ros::NodeHandle& nh, const std::string& key)
{
  if (key.empty() || key[0] == '/')
    return key;
  else
    return nh.getNamespace() + std::string("/") + key;
}

} // internal

/*!
 * Ostream overloads.
 */

template <typename T>
inline std::ostream& operator<<(std::ostream& ostream, const std::vector<T>& vector)
{
  ostream << "[";
  for (typename std::vector<T>::const_iterator it = vector.begin(); it != vector.end(); ++it)
  {
    ostream << " " << *it;
  }
  ostream << "]";
  return ostream;
}

template <typename T1, typename T2>
inline std::ostream& operator<<(std::ostream& ostream, const std::map<T1, T2>& map)
{
  for (typename std::map<T1, T2>::const_iterator it = map.begin(); it != map.end(); ++it)
  {
    ostream << it->first << " --> " << it->second << std::endl;
  }
  return ostream;
}

inline std::ostream& operator<<(std::ostream& ostream, const XmlRpc::XmlRpcValue& xmlRpcValue)
{
  xmlRpcValue.write(ostream);
  return ostream;
}

/*
 * Interfaces:
 *
 * 1) bool getParam(const ros::NodeHandle& nh, const std::string& key, ParamT& parameter);
 *
 * 2) ParamT param(const ros::NodeHandle& nh, const std::string& key, const ParamT& defaultParameter);
 *
 *
 *
 * Examples:
 *
 * 1a) double myParam = 0;
 *     bool success = getParam(nh, "my_param", myParam);
 *
 * 1b) double myParam1 = 0;
 *     double myParam2 = 0;
 *     bool success = true;
 *     success &= getParam(nh, "my_param1", myParam1);
 *     success &= getParam(nh, "my_param2", myParam2);
 *
 * 2)  double myParamDefault = 1.0;
 *     double myParam = param<double>(nh, "my_param", myParamDefault);
 */



/*!
 * Interface 1:
 */
template <typename ParamT>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, ParamT& parameter)
{
  const ParamT defaultParameter = parameter;
  if (!nh.getParam(key, parameter))
  {
    parameter = defaultParameter; // Make sure NodeHandle::getParam() does not modify parameter.
    ROS_WARN_STREAM("Could not acquire parameter '" << internal::getAbsoluteKey(nh, key) <<
        "' from server. Parameter still contains '" << parameter << "'.");
    return false;
  }
  return true;
}

/*!
 * Interface 2:
 */
template <typename ParamT>
inline ParamT param(const ros::NodeHandle& nh, const std::string& key, const ParamT& defaultParameter)
{
  ParamT parameter = defaultParameter;
  getParam(nh, key, parameter);
  return parameter;
}

/*!
 * Template specializations.
 */

// primitive types
template <>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, uint32_t& parameter)
{
  int32_t value = parameter;
  bool success = getParam(nh, key, value);
  if (value < 0)
  {
    ROS_ERROR_STREAM("Parameter '" << internal::getAbsoluteKey(nh, key) <<
        "' is smaller than 0, cannot be stored inside an unsigned int.");
    return false;
  }
  parameter = value;
  return success;
}

// ros
template <>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, ros::Time& parameter)
{
  bool success = true;
  success = success && getParam(nh, key + "/sec", parameter.sec);
  success = success && getParam(nh, key + "/nsec", parameter.nsec);
  return success;
}

// std_msgs
template <>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, std_msgs::Header& parameter)
{
  bool success = true;
  success = success && getParam(nh, key + "/stamp", parameter.stamp);
  success = success && getParam(nh, key + "/seq", parameter.seq);
  success = success && getParam(nh, key + "/frame_id", parameter.frame_id);
  return success;
}

// geometry_msgs
template <>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::Vector3& parameter)
{
  bool success = true;
  success = success && getParam(nh, key + "/x", parameter.x);
  success = success && getParam(nh, key + "/y", parameter.y);
  success = success && getParam(nh, key + "/z", parameter.z);
  return success;
}

template <>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::Point& parameter)
{
  bool success = true;
  success = success && getParam(nh, key + "/x", parameter.x);
  success = success && getParam(nh, key + "/y", parameter.y);
  success = success && getParam(nh, key + "/z", parameter.z);
  return success;
}

template <>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::Quaternion& parameter)
{
  bool success = true;
  success = success && getParam(nh, key + "/w", parameter.w);
  success = success && getParam(nh, key + "/x", parameter.x);
  success = success && getParam(nh, key + "/y", parameter.y);
  success = success && getParam(nh, key + "/z", parameter.z);
  return success;
}

template <>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::Pose& parameter)
{
  bool success = true;
  success = success && getParam(nh, key + "/position", parameter.position);
  success = success && getParam(nh, key + "/orientation", parameter.orientation);
  return success;
}

template <>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::PoseStamped& parameter)
{
  bool success = true;
  success = success && getParam(nh, key + "/header", parameter.header);
  success = success && getParam(nh, key + "/pose", parameter.pose);
  return success;
}

template <>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::Twist& parameter)
{
  bool success = true;
  success = success && getParam(nh, key + "/linear", parameter.linear);
  success = success && getParam(nh, key + "/angular", parameter.angular);
  return success;
}

template <>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::TwistStamped& parameter)
{
  bool success = true;
  success = success && getParam(nh, key + "/header", parameter.header);
  success = success && getParam(nh, key + "/twist", parameter.twist);
  return success;
}

// Eigen
template <>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, Eigen::Matrix<double,3,1>& parameter)
{
  bool success = true;
  success = success && getParam(nh, key + "/x", parameter(0));
  success = success && getParam(nh, key + "/y", parameter(1));
  success = success && getParam(nh, key + "/z", parameter(2));
  return success;
}

template <>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, Eigen::Quaterniond& parameter)
{
  bool success = true;
  success = success && getParam(nh, key + "/x", parameter.vec()(0));
  success = success && getParam(nh, key + "/y", parameter.vec()(1));
  success = success && getParam(nh, key + "/z", parameter.vec()(2));
  success = success && getParam(nh, key + "/w", parameter.w());
  return success;
}

template <>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, Eigen::Matrix<double,2,1>& parameter)
{
  bool success = true;
  success = success && getParam(nh, key + "/x", parameter(0));
  success = success && getParam(nh, key + "/y", parameter(1));
  return success;
}

template <typename T>
T getMember(XmlRpc::XmlRpcValue parameter, const std::string& key)
{
  try
  {
    if (!parameter.hasMember(key))
    {
      ROS_ERROR_STREAM("XmlRpcValue does not contain member '" << key << "'.");
      return T();
    }
    return static_cast<T>(parameter[key]);
  }
  catch (const XmlRpc::XmlRpcException& exception)
  {
    ROS_ERROR_STREAM("Caught an XmlRpc exception while getting member '" << key << "'.");
    return T();
  }
}

} // param_io

