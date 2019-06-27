/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, C. Dario Bellicoso, Christian Gehring
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * ParameterHandlerRos.hpp
 *
 *  Created on: Sep 24, 2015
 *      Author: Christian Gehring, Gabriel Hottiger
 */

#pragma once


#include <ros/ros.h>
#include <parameter_handler_msgs/SetIntegralParameter.h>
#include <parameter_handler_msgs/GetIntegralParameter.h>
#include <parameter_handler_msgs/SetFloatingPointParameter.h>
#include <parameter_handler_msgs/GetFloatingPointParameter.h>
#include <parameter_handler_msgs/GetParameterList.h>

#include <parameter_handler/type_macros.hpp>
#include <parameter_handler_std/ParameterHandlerStd.hpp>
#include <parameter_handler_ros/helper_methods.hpp>

#include <mutex>

namespace parameter_handler_ros {

class ParameterHandlerRos : public parameter_handler_std::ParameterHandlerStd
{
 public:
  ParameterHandlerRos();
  ~ParameterHandlerRos() override = default;
  void initializeServices();
  void shutdown();

  void setNodeHandle(ros::NodeHandle* nodeHandle);

  bool cleanup() override;

  bool getParameterList(parameter_handler_msgs::GetParameterList::Request &req,
                        parameter_handler_msgs::GetParameterList::Response &res);

  bool setIntegralParameter(parameter_handler_msgs::SetIntegralParameterRequest &req,
                            parameter_handler_msgs::SetIntegralParameterResponse &res);

  bool getIntegralParameter(parameter_handler_msgs::GetIntegralParameterRequest &req,
                            parameter_handler_msgs::GetIntegralParameterResponse &res);

  bool setFloatingPointParameter(parameter_handler_msgs::SetFloatingPointParameterRequest &req,
                                 parameter_handler_msgs::SetFloatingPointParameterResponse &res);

  bool getFloatingPointParameter(parameter_handler_msgs::GetFloatingPointParameterRequest &req,
                                 parameter_handler_msgs::GetFloatingPointParameterResponse &res);

  void parameterChanged(const parameter_handler::ParameterInterface & param) override;

 protected:
  ros::NodeHandle* nodeHandle_;
  ros::ServiceServer getParameterListService_;
  ros::ServiceServer getIntegralParameterService_;
  ros::ServiceServer setIntegralParameterService_;
  ros::ServiceServer getFloatingPointParameterService_;
  ros::ServiceServer setFloatingPointParameterService_;
  ros::Publisher     notifyIntegralParameterChange_;
  ros::Publisher     notifyFloatingPointParameterChange_;
};

} /* namespace parameter_handler */
