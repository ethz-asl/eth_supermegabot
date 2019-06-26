//
// Created by johannes on 29.04.19.
//

#pragma once

#include <cstddef>

namespace smb_path_following {

enum SmbPathFollowingDefinitions : size_t {
  STATE_DIM_ = 3,  // [I_x_b I_y_b \theta_b \phi^T]^T
  INPUT_DIM_ = 2,  // \dot{ [B_x_b \theta_b \phi^T]^T }
  REFERENCE_STATE_DIM_ = 3,
  REFERENCE_INPUT_DIM_ = 2
};

}  // namespace smb_path_following
