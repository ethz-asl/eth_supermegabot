/*
 * camera_mt9v034.cpp
 *
 *  Created on: Apr 18, 2012
 *      Author: burrimi
 */

#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

#include "sensors/led_flasher.hpp"
#include "visensor_impl.hpp" // needed by forward declaration

namespace visensor {
LedFlasher::LedFlasher(SensorId::SensorId sensor_id,
                           ConfigConnection::WeakPtr config_connection)
    : Sensor(SensorSettings(sensor_id,
                         SensorType::SensorType::LIGHT_CONTROL,
                         0,
                         LedFlasherDefaults::NUM_OF_MSGS_IN_PACKAGE,
                         LedFlasherDefaults::USE_CONST_PACKAGE_SIZE,
                         LedFlasherDefaults::CALIBRATION_SIZE),
          config_connection),
      config_(sensor_id) {
}

LedFlasher::~LedFlasher() {
}

void LedFlasher::init() {
}

ViConfigMsg LedFlasher::getConfigParam(std::string cmd, uint32_t value) {
  return config_.getConfigParam(cmd, value);
}

}  //namespace visensor
