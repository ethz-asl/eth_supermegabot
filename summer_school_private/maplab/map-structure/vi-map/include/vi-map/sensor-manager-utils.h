#ifndef VI_MAP_SENSOR_MANAGER_UTILS_H_
#define VI_MAP_SENSOR_MANAGER_UTILS_H_

#include <aslam/cameras/ncamera.h>
#include <aslam/common/memory.h>
#include <glog/logging.h>

#include "vi-map/sensor-manager.h"

namespace vi_map {
// Convenience function that returns a pointer to a camera if called with a
// valid optional camera id, or a nullptr if called with a sensor id.
template <typename SensorOrCameraId>
aslam::Camera::Ptr getOptionalCameraIfCameraId(
    const vi_map::SensorManager& sensor_manager,
    const SensorOrCameraId& sensor_or_camera_id);

}  // namespace vi_map

#endif  // VI_MAP_SENSOR_MANAGER_UTILS_H_
