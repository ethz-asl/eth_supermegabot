#include "vi-map/sensor-manager-utils.h"

namespace vi_map {

template <>
aslam::Camera::Ptr getOptionalCameraIfCameraId(
    const vi_map::SensorManager& /*sensor_manager*/,
    const vi_map::SensorId& /*sensor_id*/) {
  return aslam::Camera::Ptr();
}

template <>
aslam::Camera::Ptr getOptionalCameraIfCameraId(
    const vi_map::SensorManager& sensor_manager,
    const aslam::CameraId& camera_id) {
  return sensor_manager.getOptionalCameraWithExtrinsics(camera_id).second;
}
}  // namespace vi_map
