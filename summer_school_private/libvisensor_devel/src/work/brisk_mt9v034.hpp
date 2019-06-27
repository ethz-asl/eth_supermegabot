/*
 * corner_mt9v034.hpp
 *
 *  Created on: Apr 18, 2012
 *      Author: burrimi
 */

#ifndef BRISK_MT9V034_HPP_
#define BRISK_MT9V034_HPP_

#include "camera.hpp"
#include <math.h>
#include <map>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "param_server.hpp"

namespace visensor {

namespace BriskMt9v034Defaults {
const int I2C_ADRESS = 0x48, WIDTH = 752, HEIGHT = 480, MAX_FEATURES = 30000,
    RATE = CAMERA_FREQUENCY, SCORE_BYTES = 4, BRISK_BYTES = 48,
    NUM_OF_MSGS_IN_PACKAGE = 1, CALIBRATION_SIZE = 0;
const uint8_t COM_TYPE = ViComType::I2C_16;
const bool USE_CONST_PACKAGE_SIZE = false;
}

class BriskMt9v034Config {
 private:
  param_server::ParamServer _paramServer;
  const uint8_t _sensorId;
 public:
  BriskMt9v034Config(const uint8_t sensorId)
      : _sensorId(sensorId) {
    // name, type, register, default, mask, min, max

    //_paramServer.addParam("coarse_shutter_width",param_server::UINT_T,0x0B, 480, 0x7FFF,2,32765);		//page 27
  }

  ViConfigMsg getConfigParam(const std::string paramName, uint16_t value) {
    ViConfigMsg msg;
    msg.valChanged = _paramServer.getConfigParam(paramName, value, msg.reg,
                                                 msg.val);
    msg.sensorId = _sensorId;
    msg.devAdress = BriskMt9v034Defaults::I2C_ADRESS;
    msg.comType = BriskMt9v034Defaults::COM_TYPE;
    return msg;
  }

  ViConfigMsg getConfigParam(const std::string paramName) {
    ViConfigMsg msg;
    msg.valChanged = _paramServer.getConfigParam(paramName, msg.reg, msg.val);
    msg.sensorId = _sensorId;
    msg.devAdress = BriskMt9v034Defaults::I2C_ADRESS;
    msg.comType = BriskMt9v034Defaults::COM_TYPE;
    return msg;
  }

};

class BriskMt9v034 : public Camera {
 public:
  BriskMt9v034(int sensor_id, int camera_id,
               ViSensorDriver::Impl* ViSensorDriver);
  virtual ~BriskMt9v034();
//	virtual int processData(uint8_t const * data,int nBytesReceived); //this function will get called by the ASLCamDriver
//	virtual uint8_t* getBufferPtr(); //return raw pointer to the buffer
//	virtual int getBytesInBuffer(); //return the number of bytes in the buffer
//	virtual int getBufferSize(); // return the buffer size

  virtual ViConfigMsg getConfigParam(std::string cmd, uint16_t value);

  //the threaded function that works on the queue of finished measurements
  void processMeasurements();

  bool getBriskElem(uint8_t* buf, ViCornerElem& corner);

  virtual bool init();

  // set calibration
  virtual bool setCalibration(CalibBase::Ptr calibration);

  static uint32_t calculateBufferSize();

 private:
  //ViCameraConfig	config;               	/* the camera configuration */
//	uint8_t	* _buffer;            				/* camera frame buffer */
//	uint32_t _buffer_size;						/* size of the frame buffer */
//	uint32_t _bytes_in_buffer;					/* number of bytes stored in the frame buffer */
//	bool _active;								/* sets if camera is active or not */
  uint32_t _frame_size; /* size of one frame */

  int _width;
  int _height;

  BriskMt9v034Config _config;

};

}  //namespace visensor

#endif /* BRISK_MT9V034_HPP_ */
