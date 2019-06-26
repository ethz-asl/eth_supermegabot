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

#ifndef DENSE_MATCHER_HPP_
#define DENSE_MATCHER_HPP_

#include <config/config.hpp>
#include <boost/smart_ptr.hpp>

#include "networking/param_server.hpp"
#include "sensors/camera.hpp"

namespace visensor {

namespace DenseMatcherDefaults {
const int I2C_ADRESS = 0x48,
    WIDTH = 752,
    HEIGHT = 480,
    RATE = CAMERA_FREQUENCY,
    NUM_OF_MSGS_IN_PACKAGE = 1,
    CALIBRATION_SIZE = 0;
const uint8_t COM_TYPE = ViComType::FPGA_32;

const ViImageType IMAGE_TYPE = ViImageType::MONO8;
const bool USE_CONST_PACKAGE_SIZE = true;
}

class DenseMatcherConfig {
 public:
  DenseMatcherConfig(const SensorId::SensorId sensorId)
      : sensor_id_(sensorId) {
    // name, type, register, default, mask, min, max
    param_server_.addParam("penalty_1", param_server::UINT_T, 0X04 * 40, 0, 0xFFFF, 0, 255);
    param_server_.addParam("penalty_2", param_server::UINT_T, 0X04 * 41, 0, 0xFFFF, 0, 255);
    param_server_.addParam("threshold", param_server::UINT_T, 0X04 * 42, 0, 0xFFFF, 0, 255);
    param_server_.addParam("lr_check", param_server::UINT_T, 0X04 * 43, 0, 0xFFFF, 0, 32);
  }

  ViConfigMsg getConfigParam(const std::string paramName, uint16_t value) {
    ViConfigMsg msg;
    msg.valChanged = param_server_.getConfigParam(paramName, value, msg.reg,
                                                  msg.val);
    msg.sensorId = sensor_id_;
    msg.devAdress = DenseMatcherDefaults::I2C_ADRESS;
    msg.comType = DenseMatcherDefaults::COM_TYPE;
    return msg;
  }

  ViConfigMsg getConfigParam(const std::string paramName) {
    ViConfigMsg msg;
    msg.valChanged = param_server_.getConfigParam(paramName, msg.reg, msg.val);
    msg.sensorId = sensor_id_;
    msg.devAdress = DenseMatcherDefaults::I2C_ADRESS;
    msg.comType = DenseMatcherDefaults::COM_TYPE;
    return msg;
  }

  const SensorId::SensorId sensor_id_;

 private:
  param_server::ParamServer param_server_;
};

class DenseMatcher : public Camera {
 public:
  typedef boost::shared_ptr<DenseMatcher> Ptr;

  DenseMatcher(SensorId::SensorId sensor_id,
               ConfigConnection::WeakPtr config_connection);
  virtual ~DenseMatcher();

  virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value);
  void setCalibration(ViCameraCalibration& calib_cam0, ViCameraCalibration& calib_cam1);

  //the threaded function that works on the queue of finished measurements
  void processMeasurements();
  virtual void init();
  static uint32_t calculateBufferSize();

 private:
  void bitReshuffle(uint8_t *in /*10 bit values*/,
                    uint8_t *out /*8 bit values*/,
                    int size_in /*number of 10 bit values*/, int offset);
  void imageReshuffle(uint8_t *data_in /*10 bit raw data*/, uint8_t *image_out,
                      int image_height, int image_width);

 private:
  uint32_t frame_size_; /* size of one frame */
  int width_;
  int height_;
  DenseMatcherConfig config_;
};

}  //namespace visensor

#endif /* CAMERA_MT9V034_HPP_ */
