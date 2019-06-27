#pragma once

#include "parameter_handler/parameter_handler.hpp"
#include "parameter_handler_std/ParameterHandlerStd.hpp"


namespace parameter_handler_std {

  void setParameterHandlerStd() {
    parameter_handler::handler.reset(new parameter_handler_std::ParameterHandlerStd());
  }

}
