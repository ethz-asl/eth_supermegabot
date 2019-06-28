/*!
 * @file     RobotContainersRos.hpp
 * @author   Gabriel Hottiger
 * @date     Mar, 2018
 */

#pragma once

// romo
#include <romo/common/RobotContainers.hpp>

// std utils
#include <std_utils/std_utils.hpp>

// STL
#include <type_traits>

namespace romo {

/**
 * @brief Static base class for all robot descriptions:
 */
template <typename ConcreteContainers_, typename ConcreteContainersRos_>
struct RobotContainersRos {
  //! Delete constructor, class consists only of static members
  RobotContainersRos() = delete;

  //! Expose Types
  using ConcreteContainers = ConcreteContainers_;
  using ConcreteContainersRos = ConcreteContainersRos_;

  //! Expose Ros Container Types
  struct RobotStateRos {
    static_assert(!std::is_void<typename ConcreteContainersRos::RobotStateRos>::value,
                  "ConcreteContainersRos defines RobotStateRos as non-existing (void).");
    using type = typename ConcreteContainers::RobotState::type;
    using msgType = typename ConcreteContainersRos::RobotStateRos::msgType;
    template <typename RobotState_, typename RobotStateRos_>
    using ConversionTrait =
        typename ConcreteContainersRos::RobotStateRos::template ConversionTrait<RobotState_, RobotStateRos_>;
  };

  struct ActuatorReadingsRos {
    static_assert(!std::is_void<typename ConcreteContainersRos::ActuatorReadingsRos>::value,
                  "ConcreteContainersRos defines ActuatorReadingsRos as non-existing (void).");
    using type = typename ConcreteContainers::ActuatorReadings::type;
    using msgType = typename ConcreteContainersRos::ActuatorReadingsRos::msgType;
    template <typename ActuatorReadings_, typename ActuatorReadingsRos_>
    using ConversionTrait =
        typename ConcreteContainersRos::ActuatorReadingsRos::template ConversionTrait<ActuatorReadings_,
                                                                                      ActuatorReadingsRos_>;
  };

  struct ActuatorCommandsRos {
    static_assert(!std::is_void<typename ConcreteContainersRos::ActuatorCommandsRos>::value,
                  "ConcreteContainersRos defines ActuatorCommandsRos as non-existing (void).");
    using type = typename ConcreteContainers::ActuatorCommands::type;
    using msgType = typename ConcreteContainersRos::ActuatorCommandsRos::msgType;
    template <typename ActuatorCommands_, typename ActuatorCommandsRos_>
    using ConversionTrait =
        typename ConcreteContainersRos::ActuatorCommandsRos::template ConversionTrait<ActuatorCommands_,
                                                                                      ActuatorCommandsRos_>;
  };  

  struct ImuRos {
    static_assert(!std::is_void<typename ConcreteContainersRos::ImuRos>::value,
                  "ConcreteContainersRos defines ImuRos as non-existing (void).");
    using type = typename ConcreteContainers::Imu::type;
    using msgType = typename ConcreteContainersRos::ImuRos::msgType;
    template <typename Imu_, typename ImuRos_>
    using ConversionTrait = typename ConcreteContainersRos::ImuRos::template ConversionTrait<Imu_, ImuRos_>;
  };
};

}  // namespace romo
