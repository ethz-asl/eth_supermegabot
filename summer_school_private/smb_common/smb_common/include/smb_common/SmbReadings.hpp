//
// Created by Koen Kramer on 23.08.18.
//

#pragma once

#include "smb_common/SmbWheelReading.hpp"
#include <std_utils/containers/EnumArray.hpp>

namespace smb_common {

    template<typename ConcreteDescription_>
    struct SmbReadings {
        std_utils::EnumArray<typename ConcreteDescription_::ConcreteTopology::SmbActuatorEnum, smb_common::SmbWheelReading> wheelReadings_;
    };

} /* namespace smb_common */
