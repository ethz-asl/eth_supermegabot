/*!
 * @file     RobotContainers.hpp
 * @author   Gabriel Hottiger, Dario Bellicoso
 * @date     Oct, 2017
 */

#pragma once

// std utils
#include <std_utils/std_utils.hpp>

// STL
#include <type_traits>

namespace romo {

template < typename ConcreteContainers_ >
struct RobotContainers {

  //! Delete constructor, class consists only of static members
  RobotContainers() = delete;

  //! Expose Types
  using ConcreteContainers = ConcreteContainers_;

  //! Expose Container Types
  struct RobotState {
    static_assert(!std::is_void<typename ConcreteContainers::RobotState>::value,
                  "ConcreteContainers defines RobotState as non-existing (void).");
    using type = typename ConcreteContainers::RobotState::type;
  };
  struct ActuatorReadings {
    static_assert(!std::is_void<typename ConcreteContainers::ActuatorReadings>::value,
                  "ConcreteContainers defines ActuatorReadings as non-existing (void).");
    using type = typename ConcreteContainers::ActuatorReadings::type;
  };
  struct ActuatorCommands {
    static_assert(!std::is_void<typename ConcreteContainers::ActuatorCommands>::value,
                  "ConcreteContainers defines ActuatorCommands as non-existing (void).");
    using type = typename ConcreteContainers::ActuatorCommands::type;
  };

  struct Imu {
    static_assert(!std::is_void<typename ConcreteContainers::Imu>::value,
                  "ConcreteContainers defines Imu as non-existing (void).");
    using type = typename ConcreteContainers::Imu::type;
  };

};

}  // namespace romo
