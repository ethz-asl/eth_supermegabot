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

#ifndef CAMERA_MT9V034_HPP_
#define CAMERA_MT9V034_HPP_

#include <config/config.hpp>
#include <boost/smart_ptr.hpp>

#include "networking/connection.hpp"
#include "networking/param_server.hpp"
#include "sensors/camera.hpp"

namespace visensor {

namespace CameraMt9v034Defaults {
const int I2C_ADRESS = 0x48,
    WIDTH = 752,
    HEIGHT = 480,
    RATE = CAMERA_FREQUENCY,
    NUM_OF_MSGS_IN_PACKAGE = 1,
    CALIBRATION_SIZE = 0;

const uint8_t COM_TYPE = ViComType::I2C_16;
const ViImageType IMAGE_TYPE = ViImageType::MONO8;
const bool USE_CONST_PACKAGE_SIZE = true;
}

class CameraMt9v034Config {
 public:
  CameraMt9v034Config(const SensorId::SensorId sensorId)
      : sensor_id_(sensorId) {
    // name, type, register, default, mask, min, max

    param_server_.addParam("coarse_shutter_width", param_server::UINT_T, 0x0B,
                           480, 0x7FFF, 2, 32765);		//page 27

    param_server_.addParam("row_flip", param_server::BOOL_T, 0x0D, 0, 0x0010);	//page 29
    param_server_.addParam("column_flip", param_server::BOOL_T, 0x0D, 0,
                           0x0020);  //page 29

    param_server_.addParam("adc_mode", param_server::UINT_T, 0x1C, 2, 0x0003, 2,
                           3);  //page 30

    param_server_.addParam("vref_adc_voltage_level", param_server::UINT_T, 0x2C,
                           4, 0x0007);  // page 31

    param_server_.addParam("global_analog_gain", param_server::UINT_T, 0x35,
                           0x10, 0x007F, 16, 64);  //page 32
    param_server_.addParam("global_analog_gain_attenuation",
                           param_server::BOOL_T, 0x35, 0, 0x8000);  //page 32

    param_server_.addParam("black_level_calibration_manual_override",
                           param_server::BOOL_T, 0x47, 0x10, 0x0001);  //page 34
    param_server_.addParam("black_level_calibration_value", param_server::INT_T,
                           0x48, 0, 0x00FF);  //page 34

    param_server_.addParam("digital_gain_tile_0", param_server::UINT_T, 0x80, 4,
                           0x000F, 1, 15);	//page 36
    param_server_.addParam("digital_gain_tile_1", param_server::UINT_T, 0x81, 4,
                           0x000F, 1, 15);	//page 36
    param_server_.addParam("digital_gain_tile_2", param_server::UINT_T, 0x82, 4,
                           0x000F, 1, 15);	//page 36
    param_server_.addParam("digital_gain_tile_3", param_server::UINT_T, 0x83, 4,
                           0x000F, 1, 15);	//page 36
    param_server_.addParam("digital_gain_tile_4", param_server::UINT_T, 0x84, 4,
                           0x000F, 1, 15);	//page 36
    param_server_.addParam("digital_gain_tile_5", param_server::UINT_T, 0x85, 4,
                           0x000F, 1, 15);	//page 36
    param_server_.addParam("digital_gain_tile_6", param_server::UINT_T, 0x86, 4,
                           0x000F, 1, 15);	//page 36
    param_server_.addParam("digital_gain_tile_7", param_server::UINT_T, 0x87, 4,
                           0x000F, 1, 15);	//page 36
    param_server_.addParam("digital_gain_tile_8", param_server::UINT_T, 0x88, 4,
                           0x000F, 1, 15);	//page 36
    param_server_.addParam("digital_gain_tile_9", param_server::UINT_T, 0x89, 4,
                           0x000F, 1, 15);	//page 36
    param_server_.addParam("digital_gain_tile_10", param_server::UINT_T, 0x8A,
                           4, 0x000F, 1, 15);  //page 36
    param_server_.addParam("digital_gain_tile_11", param_server::UINT_T, 0x8B,
                           4, 0x000F, 1, 15);  //page 36
    param_server_.addParam("digital_gain_tile_12", param_server::UINT_T, 0x8C,
                           4, 0x000F, 1, 15);  //page 36
    param_server_.addParam("digital_gain_tile_13", param_server::UINT_T, 0x8D,
                           4, 0x000F, 1, 15);  //page 36
    param_server_.addParam("digital_gain_tile_14", param_server::UINT_T, 0x8E,
                           4, 0x000F, 1, 15);  //page 36
    param_server_.addParam("digital_gain_tile_15", param_server::UINT_T, 0x8F,
                           4, 0x000F, 1, 15);  //page 36
    param_server_.addParam("digital_gain_tile_16", param_server::UINT_T, 0x90,
                           4, 0x000F, 1, 15);  //page 36
    param_server_.addParam("digital_gain_tile_17", param_server::UINT_T, 0x91,
                           4, 0x000F, 1, 15);  //page 36
    param_server_.addParam("digital_gain_tile_18", param_server::UINT_T, 0x92,
                           4, 0x000F, 1, 15);  //page 36
    param_server_.addParam("digital_gain_tile_19", param_server::UINT_T, 0x93,
                           4, 0x000F, 1, 15);  //page 36
    param_server_.addParam("digital_gain_tile_20", param_server::UINT_T, 0x94,
                           4, 0x000F, 1, 15);  //page 36
    param_server_.addParam("digital_gain_tile_21", param_server::UINT_T, 0x95,
                           4, 0x000F, 1, 15);  //page 36
    param_server_.addParam("digital_gain_tile_22", param_server::UINT_T, 0x96,
                           4, 0x000F, 1, 15);  //page 36
    param_server_.addParam("digital_gain_tile_23", param_server::UINT_T, 0x97,
                           4, 0x000F, 1, 15);  //page 36
    param_server_.addParam("digital_gain_tile_24", param_server::UINT_T, 0x98,
                           4, 0x000F, 1, 15);  //page 36

    param_server_.addParam("digital_gain_sample_weight_tile_0",
                           param_server::UINT_T, 0x80, 15, 0x00F0, 1, 15);  //page 36
    param_server_.addParam("digital_gain_sample_weight_tile_1",
                           param_server::UINT_T, 0x81, 15, 0x00F0, 1, 15);  //page 36
    param_server_.addParam("digital_gain_sample_weight_tile_2",
                           param_server::UINT_T, 0x82, 15, 0x00F0, 1, 15);  //page 36
    param_server_.addParam("digital_gain_sample_weight_tile_3",
                           param_server::UINT_T, 0x83, 15, 0x00F0, 1, 15);  //page 36
    param_server_.addParam("digital_gain_sample_weight_tile_4",
                           param_server::UINT_T, 0x84, 15, 0x00F0, 1, 15);  //page 36
    param_server_.addParam("digital_gain_sample_weight_tile_5",
                           param_server::UINT_T, 0x85, 15, 0x00F0, 1, 15);  //page 36
    param_server_.addParam("digital_gain_sample_weight_tile_6",
                           param_server::UINT_T, 0x86, 15, 0x00F0, 1, 15);  //page 36
    param_server_.addParam("digital_gain_sample_weight_tile_7",
                           param_server::UINT_T, 0x87, 15, 0x00F0, 1, 15);  //page 36
    param_server_.addParam("digital_gain_sample_weight_tile_8",
                           param_server::UINT_T, 0x88, 15, 0x00F0, 1, 15);  //page 36
    param_server_.addParam("digital_gain_sample_weight_tile_9",
                           param_server::UINT_T, 0x89, 15, 0x00F0, 1, 15);  //page 36
    param_server_.addParam("digital_gain_sample_weight_tile_10",
                           param_server::UINT_T, 0x8A, 15, 0x00F0, 1, 15);	//page 36
    param_server_.addParam("digital_gain_sample_weight_tile_11",
                           param_server::UINT_T, 0x8B, 15, 0x00F0, 1, 15);	//page 36
    param_server_.addParam("digital_gain_sample_weight_tile_12",
                           param_server::UINT_T, 0x8C, 15, 0x00F0, 1, 15);	//page 36
    param_server_.addParam("digital_gain_sample_weight_tile_13",
                           param_server::UINT_T, 0x8D, 15, 0x00F0, 1, 15);	//page 36
    param_server_.addParam("digital_gain_sample_weight_tile_14",
                           param_server::UINT_T, 0x8E, 15, 0x00F0, 1, 15);	//page 36
    param_server_.addParam("digital_gain_sample_weight_tile_15",
                           param_server::UINT_T, 0x8F, 15, 0x00F0, 1, 15);	//page 36
    param_server_.addParam("digital_gain_sample_weight_tile_16",
                           param_server::UINT_T, 0x90, 15, 0x00F0, 1, 15);	//page 36
    param_server_.addParam("digital_gain_sample_weight_tile_17",
                           param_server::UINT_T, 0x91, 15, 0x00F0, 1, 15);	//page 36
    param_server_.addParam("digital_gain_sample_weight_tile_18",
                           param_server::UINT_T, 0x92, 15, 0x00F0, 1, 15);	//page 36
    param_server_.addParam("digital_gain_sample_weight_tile_19",
                           param_server::UINT_T, 0x93, 15, 0x00F0, 1, 15);	//page 36
    param_server_.addParam("digital_gain_sample_weight_tile_20",
                           param_server::UINT_T, 0x94, 15, 0x00F0, 1, 15);	//page 36
    param_server_.addParam("digital_gain_sample_weight_tile_21",
                           param_server::UINT_T, 0x95, 15, 0x00F0, 1, 15);	//page 36
    param_server_.addParam("digital_gain_sample_weight_tile_22",
                           param_server::UINT_T, 0x96, 15, 0x00F0, 1, 15);	//page 36
    param_server_.addParam("digital_gain_sample_weight_tile_23",
                           param_server::UINT_T, 0x97, 15, 0x00F0, 1, 15);	//page 36
    param_server_.addParam("digital_gain_sample_weight_tile_24",
                           param_server::UINT_T, 0x98, 15, 0x00F0, 1, 15);	//page 36

    param_server_.addParam("max_analog_gain", param_server::UINT_T, 0xAB, 64,
                           0x007F, 16, 64);  //page 39

    param_server_.addParam("min_coarse_shutter_width", param_server::UINT_T,
                           0xAC, 2, 0xFFFF, 2, 32765);	//page 39
    param_server_.addParam("max_coarse_shutter_width", param_server::UINT_T,
                           0xAD, 100, 0xFFFF, 2, 32765);	//page 39

    param_server_.addParam("aec_enable", param_server::BOOL_T, 0xAF, 1, 0x0001);  //page 39
    param_server_.addParam("agc_enable", param_server::BOOL_T, 0xAF, 1, 0x0002);  //page 39

    param_server_.addParam("fine_shutter_width", param_server::UINT_T, 0xD5, 0,
                           0x07FF, 0, 1774);  //page 44

    param_server_.addParam("chip_control", param_server::CONST_T, 0x07, 0x0118,
                           0xFFFF);  //page 26 - chip control
    param_server_.addParam("lvds_shift_clk_ctrl", param_server::CONST_T, 0xB2,
                           0x0000, 0xFFFF);  // enable lvds clock
    param_server_.addParam("lvds_payload_control", param_server::CONST_T, 0xB6,
                           0x0001, 0xFFFF);  // use 10 bit per pixel
    param_server_.addParam("lvds_master_control", param_server::CONST_T, 0xB1,
                           0x0000, 0xFFFF);  // lvds control
    param_server_.addParam("soft_reset", param_server::CONST_T, 0x0C, 0x0001,
                           0x0001);  // soft reset
    param_server_.addParam("dangerous_reserved_register", param_server::CONST_T,
                           0x20, 0x03C7, 0xFFFF);  // needed to avoid fixed patern noise with short shutter times

    param_server_.addParam("aec_low_pass_filter", param_server::CONST_T, 0xA8,
                           0, 0xFFFF);  // This value plays a role in determining the increment
                                             // decrement size of exposure value from frame to frame.
    param_server_.addParam("agc_pixel_count", param_server::CONST_T, 0xB0,
                           0xABE0, 0xFFFF);  // needed to avoid fixed patern noise with short shutter times
    param_server_.addParam("agc_current_bin", param_server::CONST_T, 0xBC, 0x0,
                           0xFFFF);  // needed to avoid fixed patern noise with short shutter times
    param_server_.addParam("agc_desired_bin", param_server::CONST_T, 0xA5, 0x3A,
                           0xFFFF);  // needed to avoid fixed patern noise with short shutter times
  }

  ViConfigMsg getConfigParam(const std::string paramName, uint32_t value) {
    ViConfigMsg msg;
    msg.valChanged = param_server_.getConfigParam(paramName, value, msg.reg,
                                                  msg.val);
    msg.sensorId = sensor_id_;
    msg.devAdress = CameraMt9v034Defaults::I2C_ADRESS;
    msg.comType = CameraMt9v034Defaults::COM_TYPE;
    return msg;
  }

  ViConfigMsg getConfigParam(const std::string paramName) {
    ViConfigMsg msg;
    msg.valChanged = param_server_.getConfigParam(paramName, msg.reg, msg.val);
    msg.sensorId = sensor_id_;
    msg.devAdress = CameraMt9v034Defaults::I2C_ADRESS;
    msg.comType = CameraMt9v034Defaults::COM_TYPE;
    return msg;
  }

 private:
  param_server::ParamServer param_server_;
  const SensorId::SensorId sensor_id_;
};

class CameraMt9v034 : public Camera {
 public:
  typedef boost::shared_ptr<CameraMt9v034> Ptr;

  CameraMt9v034(SensorId::SensorId sensor_id, IpConnection::WeakPtr config_connection);
  virtual ~CameraMt9v034();

  virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value);
  //the threaded function that works on the queue of finished measurements
  void processMeasurements();
  virtual void init();
  static uint32_t calculateBufferSize();

 private:
  uint32_t frame_size_; /* size of one frame */
  int width_;
  int height_;
  CameraMt9v034Config config_;
  uint32_t exposure_update_frequency_;
  uint32_t exposure_;
  uint32_t gain_;

  void controlExposure();
};

}  //namespace visensor

#endif /* CAMERA_MT9V034_HPP_ */
