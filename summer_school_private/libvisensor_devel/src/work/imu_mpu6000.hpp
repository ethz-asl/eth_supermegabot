#ifndef IMU_MPU6000_HPP_
#define IMU_MPU6000_HPP_

#include "imu.hpp"
#include "param_server.hpp"

namespace visensor {

namespace ImuMpu6000Defaults {
const int I2C_ADRESS = 0x68, RATE = IMU_FREQUENCY, MSG_SIZE = (6 * 2 + 6 * 2),
    NUM_OF_MSGS_IN_PACKAGE = 50, CALIBRATION_SIZE = 0;
const uint8_t COM_TYPE = ViComType::I2C_8;
const bool USE_CONST_PACKAGE_SIZE = true;
}

class ImuMpu6000Config {
 public:
  ImuMpu6000Config(const uint8_t sensorId)
      : sensor_id_(sensorId) {
    // name, type, register, default, mask, min, max

    param_server_.addParam("enable", param_server::CONST_T, 0x6B, 0x0000,
                           0x00FF);  // enable mpu 6000 page 41
    param_server_.addParam("digital_low_pass_filter_config",
                           param_server::UINT_T, 0x1A, 0x0002, 0x000F, 0, 6);  // config parameters page 13
    // settings: 0 Bandwith: 260 Hz, Delay 0 ms
    // settings: 1 Bandwith: 184 Hz, Delay 2.0 ms
    // settings: 2 Bandwith: 94 Hz, Delay 3.0 ms
    // settings: 3 Bandwith: 44 Hz, Delay 4.9 ms
    // settings: 4 Bandwith: 21 Hz, Delay 8.5 ms
    // settings: 5 Bandwith: 10 Hz, Delay 13.8 ms
    // settings: 6 Bandwith: 5 Hz, Delay 19.0 ms

  }

  ViConfigMsg getConfigParam(const std::string paramName, uint16_t value) {
    ViConfigMsg msg;
    msg.valChanged = param_server_.getConfigParam(paramName, value, msg.reg,
                                                  msg.val);
    msg.sensorId = sensor_id_;
    msg.devAdress = ImuMpu6000Defaults::I2C_ADRESS;
    msg.comType = ImuMpu6000Defaults::COM_TYPE;
    return msg;
  }

  ViConfigMsg getConfigParam(const std::string paramName) {
    ViConfigMsg msg;
    msg.valChanged = param_server_.getConfigParam(paramName, msg.reg, msg.val);
    msg.sensorId = sensor_id_;
    msg.devAdress = ImuMpu6000Defaults::I2C_ADRESS;
    msg.comType = ImuMpu6000Defaults::COM_TYPE;
    return msg;
  }

 private:
  param_server::ParamServer param_server_;
  const uint8_t sensor_id_;
};

class ImuMpu6000 : public Imu {
 public:
  ImuMpu6000(int sensor_id, int imu_id,
             ViSensorDriver::Impl* ViSensorDriver);
  virtual ~ImuMpu6000() {
  }
  ;

  virtual ViConfigMsg getConfigParam(std::string cmd, uint16_t value);
  //the threaded function that works on the queue of finished measurements
  void processMeasurements();
  virtual bool init();
  virtual bool setCalibration(CalibBase::Ptr calibration);
  static uint32_t calculateBufferSize();

 private:
  void allocateBuffer();
  void getGyro(uint8_t* buffer, double * gyro);
  void getAcc(uint8_t* buffer, double * acc);
  void getGyro2(uint8_t* buffer, double * gyro);
  void getAcc2(uint8_t* buffer, double * acc);
//	void getMag(double * mag);
//	void getBaro(double * baro);
  void incrementMsgCounter();
  void newImuMsgs();

  ImuMpu6000Config config_;
  CalibImuMpu6000 calibration_;
};
}  //namespace visensor

#endif /* IMU_MPU6000_HPP_ */
