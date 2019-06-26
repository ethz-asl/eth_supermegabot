/*!
 * @file     ExampleContainers.hpp
 * @author   Gabriel Hottiger
 * @date     Nov, 2017
 */

#pragma once

// std_utils
#include <std_utils/std_utils.hpp>

// romo
#include <romo/common/RobotContainers.hpp>

// Stl
#include <iostream>

namespace ext {

// Some exmaple container types
struct ExampleRobotState {
  int state = 0;
  void update() { std::cout << "Update robot state " << ++state << "!" << std::endl; }
};

struct ExampleActuatorReadings {
  int reading = 0;
  void read() { std::cout << "Read actuator reading of " << ++reading << "!" << std::endl; }
};

struct ExampleActuatorCommands {
  int command = 0;
  void read() { std::cout << "Read actuator command of: " << ++command << "!" << std::endl; }
};

}

namespace romo_test {

struct ExampleConcreteContainers {
  //! Expose Container Types
  struct RobotState {
    using type = ext::ExampleRobotState;
  };
  struct ActuatorReadings {
    using type = ext::ExampleActuatorReadings;
  };

  struct ActuatorCommands {
    using type = ext::ExampleActuatorCommands;
  };

  // Example robot has no imu
  using Imu = void;
};

using ExampleContainers = romo::RobotContainers<ExampleConcreteContainers>;

}