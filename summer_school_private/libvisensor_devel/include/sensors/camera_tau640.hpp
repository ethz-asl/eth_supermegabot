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

#ifndef CAMERA_TAU640_HPP_
#define CAMERA_TAU640_HPP_

#include <config/config.hpp>
#include <boost/smart_ptr.hpp>

#include "networking/connection.hpp"
#include "sensors/camera.hpp"
#include "networking/param_server.hpp"

namespace visensor {

namespace CameraTau640Defaults {
const int I2C_ADRESS = 0x48,
    WIDTH = 640,
    HEIGHT = 512,
    RATE = CAMERA_FREQUENCY,
    NUM_OF_MSGS_IN_PACKAGE = 1,
    CALIBRATION_SIZE = 0;

const ViImageType TYPE = ViImageType::MONO16;
const bool USE_CONST_PACKAGE_SIZE = true;
}

class CameraTau640Config {
 private:
  param_server::ParamServer _paramServer;
  const SensorId::SensorId _sensorId;
 public:
  CameraTau640Config(const SensorId::SensorId sensorId)
      : _sensorId(sensorId) {
  }

  ViConfigMsg getConfigParam(const std::string paramName, uint16_t value) {
    ViConfigMsg msg;
    msg.valChanged = _paramServer.getConfigParam(paramName, msg.reg, msg.val);
    msg.sensorId = _sensorId;
    msg.devAdress = CameraTau640Defaults::I2C_ADRESS;
    msg.comType = ViComType::I2C_16;
    return msg;
  }

  ViConfigMsg getConfigParam(const std::string paramName) {
    ViConfigMsg msg;
    msg.valChanged = _paramServer.getConfigParam(paramName, msg.reg, msg.val);
    msg.sensorId = _sensorId;
    msg.devAdress = CameraTau640Defaults::I2C_ADRESS;
    msg.comType = ViComType::I2C_16;
    return msg;
  }
};

class CameraTau640 : public Camera {
 public:
  CameraTau640(SensorId::SensorId sensor_id, IpConnection::WeakPtr config_connection);
  virtual ~CameraTau640();

  virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value);
  void processMeasurements();
  virtual void init();
  static uint32_t calculateBufferSize();
  //  virtual bool setCalibration(CalibBase::Ptr calibration);

  typedef boost::shared_ptr<CameraTau640> Ptr;

 private:
  void allocateBuffer();
  void bitReshuffle(uint8_t *in /*10 bit values*/,
                    uint8_t *out /*8 bit values*/,
                    int size_in /*number of 10 bit values*/, int offset);
  void imageReshuffle(uint8_t *data_in /*10 bit raw data*/, uint8_t *image_out,
                      int image_height, int image_width);

 private:
  uint32_t frame_size_; /* size of one frame */
  int _width;
  int _height;
  CameraTau640Config _config;
  uint64_t _prevTimestamp;
  uint32_t _allowedTimediff;
};

}  //namespace visensor

#endif /* CAMERA_TAU640_HPP_ */
