/*!
 * @file     ExampleContainersRos.hpp
 * @author   Gabriel Hottiger
 * @date     Mar, 2018
 */

#pragma once

// romo
#include <romo/common/RobotContainersRos.hpp>

// romo_test
#include <romo_test/ExampleContainers.hpp>

// std utils
#include <std_utils/std_utils.hpp>

// STL
#include <string>
#include <type_traits>

namespace ext {

// Some exmaple container types
struct ExampleRobotStateRos {
  int state = 0;
  void send() { std::cout << "Send robot state " << state << "!" << std::endl; }
};

struct ExampleActuatorReadingsRos {
  int reading = 0;
  void send() { std::cout << "Send actuator readings " << reading << "!" << std::endl; }
};

struct ExampleActuatorCommandsRos {
  int command = 0;
  void send() { std::cout << "Send actuator commands" << command << "!" << std::endl; }
};

template <typename Msg_, typename MsgRos_>
struct ConversionTraits;

template <>
struct ConversionTraits<ExampleRobotState, ExampleRobotStateRos> {
  inline static ExampleRobotState convert(const ExampleRobotStateRos& msgRos) {
    ExampleRobotState msg;
    msg.state = msgRos.state;
    return msg;
  }
  inline static ExampleRobotStateRos convert(const ExampleRobotState& msg) {
    ExampleRobotStateRos msgRos;
    msgRos.state = msg.state;
    return msgRos;
  }
};

template <>
struct ConversionTraits<ExampleActuatorReadings, ExampleActuatorReadingsRos> {
  inline static ExampleActuatorReadings convert(const ExampleActuatorReadingsRos& msgRos) {
    ExampleActuatorReadings msg;
    msg.reading = msgRos.reading;
    return msg;
  }
  inline static ExampleActuatorReadingsRos convert(const ExampleActuatorReadings& msg) {
    ExampleActuatorReadingsRos msgRos;
    msgRos.reading = msg.reading;
    return msgRos;
  }
};

template <>
struct ConversionTraits<ExampleActuatorCommands, ExampleActuatorCommandsRos> {
  inline static ExampleActuatorCommands convert(const ExampleActuatorCommandsRos& msgRos) {
    ExampleActuatorCommands msg;
    msg.command = msgRos.command;
    return msg;
  }
  inline static ExampleActuatorCommandsRos convert(const ExampleActuatorCommands& msg) {
    ExampleActuatorCommandsRos msgRos;
    msgRos.command = msg.command;
    return msgRos;
  }
};


}  // namespace ext

namespace romo_test {

/**
 * @brief Static base class for all robot descriptions:
 */
struct ExampleConcreteContainersRos {
  //! Delete constructor, class consists only of static members
  ExampleConcreteContainersRos() = delete;

  //! Expose Msg Types
  struct RobotStateRos {
    using msgType = ext::ExampleRobotStateRos;
    template <typename RobotState_, typename RobotStateRos_>
    using ConversionTrait = ext::ConversionTraits<RobotState_, RobotStateRos_>;
  };

  struct ActuatorReadingsRos {
    using msgType = ext::ExampleActuatorReadingsRos;
    template <typename ActuatorReadings_, typename ActuatorReadingsRos_>
    using ConversionTrait = ext::ConversionTraits<ActuatorReadings_, ActuatorReadingsRos_>;
  };

  struct ActuatorCommandsRos {
    using msgType = ext::ExampleActuatorCommandsRos;
    template <typename ActuatorCommands_, typename ActuatorCommandsRos_>
    using ConversionTrait = ext::ConversionTraits<ActuatorCommands_, ActuatorCommandsRos_>;
  };

  //! Not existing imu
  using ImuRos = void;
};

using ExampleContainersRos = romo::RobotContainersRos<ExampleConcreteContainers, ExampleConcreteContainersRos>;

}  // namespace romo_test
