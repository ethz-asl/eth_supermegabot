/*
 * set_param.hpp
 *
 *  Created on: Nov 23, 2015
 *      Author: Remo Diethelm
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once


// ros
#include <ros/ros.h>
#include <std_msgs/Header.h>


namespace param_io {


template<typename ParamT>
inline void setParam(const ros::NodeHandle& nh, const std::string& key, const ParamT& param)
{
  nh.setParam(key, param);
}


} // param_io
