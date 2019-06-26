#pragma once

#include "parameter_handler/parameter_handler.hpp"
#include "parameter_handler_ros/ParameterHandlerRos.hpp"
#include "parameter_handler_ros/helper_methods.hpp"

namespace parameter_handler_ros {

  void setParameterHandlerRos(ros::NodeHandle * nh) {
    parameter_handler::handler.reset(new parameter_handler_ros::ParameterHandlerRos());
    parameter_handler_ros::ParameterHandlerRos* parameterHandlerRos = static_cast<parameter_handler_ros::ParameterHandlerRos*>(parameter_handler::handler.get());
    parameterHandlerRos->setNodeHandle(nh);
    parameterHandlerRos->initializeServices();
  }

}
