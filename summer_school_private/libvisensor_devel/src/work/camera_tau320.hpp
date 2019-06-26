/*
 * camera_tau320.hpp
 *
 *  Created on: Apr 18, 2012
 *      Author: burrimi
 */

#ifndef CAMERA_TAU320_HPP_
#define CAMERA_TAU320_HPP_

#include "camera.hpp"
#include <math.h>
#include <map>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "param_server.hpp"

namespace visensor {

namespace CameraTau320Defaults {
const int I2C_ADRESS = 0x48, WIDTH = 324, HEIGHT = 256, RATE = CAMERA_FREQUENCY,
    NUM_OF_MSGS_IN_PACKAGE = 1, CALIBRATION_SIZE = 0;
const ViImageType TYPE = MONO16;
const bool USE_CONST_PACKAGE_SIZE = true;
}

class CameraTau320Config {
 public:
  CameraTau320Config(const uint8_t sensorId)
      : sensor_id_(sensorId) {
  }

  ViConfigMsg getConfigParam(const std::string paramName, uint16_t value) {
    ViConfigMsg msg;
    msg.valChanged = param_server_.getConfigParam(paramName, msg.reg, msg.val);
    msg.sensorId = sensor_id_;
    msg.devAdress = CameraTau320Defaults::I2C_ADRESS;
    msg.comType = ViComType::I2C_16;
    return msg;
  }

  ViConfigMsg getConfigParam(const std::string paramName) {
    ViConfigMsg msg;
    msg.valChanged = param_server_.getConfigParam(paramName, msg.reg, msg.val);
    msg.sensorId = sensor_id_;
    msg.devAdress = CameraTau320Defaults::I2C_ADRESS;
    msg.comType = ViComType::I2C_16;
    return msg;
  }

 private:
  param_server::ParamServer param_server_;
  const uint8_t sensor_id_;
};

class CameraTau320 : public Camera {
 public:
  CameraTau320(SensorId::SensorId sensor_id, int stream_id,
               IpConnection::WeakPtr config_connection);
  virtual ~CameraTau320();

  virtual ViConfigMsg getConfigParam(std::string cmd, uint16_t value);
  //the threaded function that works on the queue of finished measurements
  void processMeasurements();
  virtual bool init();
  virtual bool setCalibration(CalibBase::Ptr calibration);
  static uint32_t calculateBufferSize();

 private:
  void allocateBuffer();
  void checkTimestamp(uint64_t timestamp);
  void bitReshuffle(uint8_t *in /*10 bit values*/,
                    uint8_t *out /*8 bit values*/,
                    int size_in /*number of 10 bit values*/, int offset);
  void imageReshuffle(uint8_t *data_in /*10 bit raw data*/, uint8_t *image_out,
                      int image_height, int image_width);

  uint32_t frame_size_; /* size of one frame */
  int width_;
  int height_;
  CameraTau320Config config_;
};

}  //namespace visensor

#endif /* CAMERA_TAU320_HPP_ */
