/*
 * camera_mt9v034.hpp
 *
 *  Created on: Apr 18, 2012
 *      Author: burrimi
 */

#ifndef LED_FLASHER_HPP_
#define LED_FLASHER_HPP_

#include <config/config.hpp>
#include <boost/smart_ptr.hpp>

#include "networking/param_server.hpp"
#include "sensors/camera.hpp"

namespace visensor {

namespace LedFlasherDefaults {
const int I2C_ADRESS = 0x48,
    RATE = CAMERA_FREQUENCY,
    NUM_OF_MSGS_IN_PACKAGE = 1,
    CALIBRATION_SIZE = 0;
const uint8_t COM_TYPE = ViComType::FPGA_32;

const bool USE_CONST_PACKAGE_SIZE = true;
}

class LedFlasherConfig {
 public:
  LedFlasherConfig(const SensorId::SensorId sensorId)
      : sensor_id_(sensorId) {
    // name, type, register, default, mask, min, max
    param_server_.addParam("strobe", param_server::UINT_T, 0X04, 2, 0, 0, 12287);
    param_server_.addParam("strobe_mode", param_server::BOOL_T, 0X00, true, 0x01);
  }

  ViConfigMsg getConfigParam(const std::string paramName, uint16_t value) {
    ViConfigMsg msg;
    msg.valChanged = param_server_.getConfigParam(paramName, value, msg.reg,
                                                  msg.val);
    msg.sensorId = sensor_id_;
    msg.devAdress = LedFlasherDefaults::I2C_ADRESS;
    msg.comType = LedFlasherDefaults::COM_TYPE;
    return msg;
  }

  ViConfigMsg getConfigParam(const std::string paramName) {
    ViConfigMsg msg;
    msg.valChanged = param_server_.getConfigParam(paramName, msg.reg, msg.val);
    msg.sensorId = sensor_id_;
    msg.devAdress = LedFlasherDefaults::I2C_ADRESS;
    msg.comType = LedFlasherDefaults::COM_TYPE;
    return msg;
  }

  const SensorId::SensorId sensor_id_;

 private:
  param_server::ParamServer param_server_;
};

class LedFlasher : public Sensor {
 public:
  typedef boost::shared_ptr<LedFlasher> Ptr;

  LedFlasher(SensorId::SensorId sensor_id,
               ConfigConnection::WeakPtr config_connection);
  virtual ~LedFlasher();

  virtual void init();
  virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value);

 private:
  LedFlasherConfig config_;
};

}  //namespace visensor

#endif /* LED_FLASHER_HPP_ */
