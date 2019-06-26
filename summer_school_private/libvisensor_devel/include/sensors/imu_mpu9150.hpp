/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2014, Autonomous Systems Lab, ETH Zurich, Switzerland
 *
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef IMU_MPU9150_HPP_
#define IMU_MPU9150_HPP_

#include <config/config.hpp>
#include <boost/smart_ptr.hpp>

#include "networking/connection.hpp"
#include "networking/param_server.hpp"
#include "sensors/imu.hpp"

namespace visensor {

namespace ImuMpu9150Defaults {
const int I2C_ADRESS = 0x68,
    RATE = IMU_FREQUENCY,
    MSG_SIZE = (3 * 2 /*Acc*/ + 2 /*temperature*/ + 3 * 2 /*gyro*/),
    NUM_OF_MSGS_IN_PACKAGE = 10,
    CALIBRATION_SIZE = 0;
const bool USE_CONST_PACKAGE_SIZE = true;
const uint8_t COM_TYPE = ViComType::I2C_8;
}

class ImuMpu9150Config {
 public:
  ImuMpu9150Config(const SensorId::SensorId sensorId)
      : sensor_id_(sensorId) {
    // name, type, register, default, mask, min, max
    param_server_.addParam("enable", param_server::CONST_T, 0x6B, 0x0000,
                           0x00FF);  // enable mpu 9150 page 41
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
    msg.devAdress = ImuMpu9150Defaults::I2C_ADRESS;
    msg.comType = ImuMpu9150Defaults::COM_TYPE;
    return msg;
  }

  ViConfigMsg getConfigParam(const std::string paramName) {
    ViConfigMsg msg;
    msg.valChanged = param_server_.getConfigParam(paramName, msg.reg, msg.val);
    msg.sensorId = sensor_id_;
    msg.devAdress = ImuMpu9150Defaults::I2C_ADRESS;
    msg.comType = ImuMpu9150Defaults::COM_TYPE;
    return msg;
  }

 private:
  param_server::ParamServer param_server_;
  const SensorId::SensorId sensor_id_;
};

class ImuMpu9150 : public Imu {
 public:
  typedef boost::shared_ptr<ImuMpu9150> Ptr;

  ImuMpu9150(SensorId::SensorId sensor_id, IpConnection::WeakPtr config_connection);
  virtual ~ImuMpu9150() {};

  virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value);
  //the threaded function that works on the queue of finished measurements
  void processMeasurements();
  virtual void init();
  static uint32_t calculateBufferSize();

 private:
  void getGyro(uint8_t* buffer, double * gyro);
  void getAcc(uint8_t* buffer, double * acc);
  void getTemperature(uint8_t* buffer, double * acc);
//	void getMag(double * mag);
//	void getBaro(double * baro);

  ImuMpu9150Config config_;
};
}  //namespace visensor

#endif /* IMU_MPU9150_HPP_ */
