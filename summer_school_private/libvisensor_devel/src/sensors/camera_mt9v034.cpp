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

#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

#include "sensors/camera_mt9v034.hpp"
#include "visensor_impl.hpp" // needed by forward declaration

namespace visensor {
CameraMt9v034::CameraMt9v034(SensorId::SensorId sensor_id, IpConnection::WeakPtr config_connection)
    : Camera(
        sensor_id,
          ViCameraConfig(CameraMt9v034Defaults::WIDTH,
                             CameraMt9v034Defaults::HEIGHT,
                             CameraMt9v034Defaults::RATE),
          SensorSettings(sensor_id, SensorType::SensorType::CAMERA_MT9V034,
                         calculateBufferSize(),
                         CameraMt9v034Defaults::NUM_OF_MSGS_IN_PACKAGE,
                         CameraMt9v034Defaults::USE_CONST_PACKAGE_SIZE,
                         CameraMt9v034Defaults::CALIBRATION_SIZE),
          config_connection),
      config_(sensor_id) {

  exposure_ = 80;
  gain_ = 16;

  //allocateBuffer();
}

void CameraMt9v034::init() {
  writeRequest(config_.getConfigParam("chip_control"));
  writeRequest(config_.getConfigParam("min_coarse_shutter_width"));
  writeRequest(config_.getConfigParam("max_coarse_shutter_width"));
  writeRequest(config_.getConfigParam("aec_low_pass_filter", 2));
  writeRequest(config_.getConfigParam("lvds_shift_clk_ctrl"));
  writeRequest(config_.getConfigParam("lvds_payload_control"));
  writeRequest(config_.getConfigParam("lvds_master_control"));
  writeRequest(config_.getConfigParam("dangerous_reserved_register"));
  writeRequest(config_.getConfigParam("soft_reset"));
  if (camera_id_ == 1) {
    writeRequest(config_.getConfigParam("row_flip", 1));
    writeRequest(config_.getConfigParam("column_flip", 1));
  } else {
    writeRequest(config_.getConfigParam("row_flip", 0));
    writeRequest(config_.getConfigParam("column_flip", 0));
  }
}

uint32_t CameraMt9v034::calculateBufferSize() {
  return (CameraMt9v034Defaults::WIDTH * CameraMt9v034Defaults::HEIGHT);
}

CameraMt9v034::~CameraMt9v034() {

}

void CameraMt9v034::controlExposure() {
//  ++_exposure_update_frequency;
//  if (_sensorId == 4 && _exposure_update_frequency > 1) {
//    _exposure_update_frequency = 0;
//    ViConfigMsg msg;
//    uint32_t desired_bin, current_bin, max_exp;
//    desired_bin = 0x3A;
//    msg = _config.getConfigParam("agc_current_bin");
//    readRequest(msg);
//    current_bin = msg.val;
//    msg = _config.getConfigParam("max_coarse_shutter_width");
//    readRequest(msg);
//    max_exp = msg.val;
//    if (_gain == 16) {
//      if (current_bin < 7)
//        _exposure = _exposure * 2;
//      else if (current_bin == 64)
//        _exposure = _exposure * 3 / 4;
//      else
//        _exposure = (_exposure * desired_bin) / current_bin;
//
//      if (_exposure > max_exp)
//        _exposure = max_exp;
//
//      writeRequest(_config.getConfigParam("coarse_shutter_width", _exposure));
//    } else {
//      _exposure = max_exp;
//      if (current_bin < 7)
//        _gain = _gain * 2;
//      else if (current_bin == 64)
//        _gain = _gain * 3 / 4;
//      else
//        _gain = (_gain * desired_bin) / current_bin;
//
//      if (_gain > 64)
//        _gain = 64;
//
//      if (_gain < 16)
//        _gain = 16;
//
//      writeRequest(_config.getConfigParam("global_analog_gain", _gain));
//    }
//    VISENSOR_DEBUG("exposure_new %X gain_new: %X \n", _exposure, _gain);
//  }
//  VISENSOR_DEBUG("in camera thread %d\n", Camera::_cameraId);
}

//the threaded function that works on the queue of finished measurements
void CameraMt9v034::processMeasurements() {
  while (1) {

    boost::this_thread::interruption_point();

    //get the newest measurement
    boost::shared_ptr<Measurement> meas = Sensor::measurement_queue_.pop();

    //cam exposure control
    controlExposure();

    //check for missed frames
    // TODO(schneith): use a timer to check for missed frames (currently we only detected sequences of MISSED-RECEIVED-MISSED, not MISSED-MISSED,...)
    if( checkTimestamp(meas->timestamp) )
    {
      //publish an empty missed frame
      ViFrame::Ptr missed_frame_ptr = boost::make_shared<ViFrame>();
      missed_frame_ptr->camera_id = Camera::camera_id_;
      publishFrameData(missed_frame_ptr, ViErrorCodes::MEASUREMENT_DROPPED);
    }


    ViFrame::Ptr frame_ptr = boost::make_shared<ViFrame>();

    // Add camera information to the frame
    frame_ptr->width = Camera::config_.width;
    frame_ptr->height = Camera::config_.height;
    frame_ptr->camera_id = Camera::camera_id_;
    frame_ptr->timestamp = meas->timestamp;
    frame_ptr->timestamp_synchronized = meas->timestamp_synchronized;
    frame_ptr->timestamp_fpga_counter = meas->timestamp_fpga_counter;
    frame_ptr->timestamp_host = meas->timestamp_host;

    frame_ptr->image_type = CameraMt9v034Defaults::IMAGE_TYPE;

    frame_ptr->setImageRawPtr(meas->data, meas->bytes_in_buffer);

    publishFrameData(frame_ptr, ViErrorCodes::NO_ERROR);
  }
}

ViConfigMsg CameraMt9v034::getConfigParam(std::string cmd, uint32_t value) {
//	msg->sensorId=Sensor::_sensorId;
  return config_.getConfigParam(cmd, value);
}

}  //namespace visensor
