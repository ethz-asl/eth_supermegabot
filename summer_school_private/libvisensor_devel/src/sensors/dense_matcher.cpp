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

#include "helpers/stereo_homography.hpp"
#include "sensors/dense_matcher.hpp"
#include "visensor_impl.hpp" // needed by forward declaration

// dense control register
#define R_DENSE_CAM0_CCX 0X04*0
#define R_DENSE_CAM0_CCY 0X04*1
#define R_DENSE_CAM0_FCX 0X04*2
#define R_DENSE_CAM0_FCY 0X04*3
#define R_DENSE_CAM0_KC1 0X04*4
#define R_DENSE_CAM0_KC2 0X04*5
#define R_DENSE_CAM0_PC1 0X04*6
#define R_DENSE_CAM0_PC2 0X04*7
#define R_DENSE_CAM0_H11 0X04*8
#define R_DENSE_CAM0_H12 0X04*9
#define R_DENSE_CAM0_H13 0X04*10
#define R_DENSE_CAM0_H21 0X04*11
#define R_DENSE_CAM0_H22 0X04*12
#define R_DENSE_CAM0_H23 0X04*13
#define R_DENSE_CAM0_H31 0X04*14
#define R_DENSE_CAM0_H32 0X04*15
#define R_DENSE_CAM0_H33 0X04*16
#define R_DENSE_CAM1_CCX 0X04*20
#define R_DENSE_CAM1_CCY 0X04*21
#define R_DENSE_CAM1_FCX 0X04*22
#define R_DENSE_CAM1_FCY 0X04*23
#define R_DENSE_CAM1_KC1 0X04*24
#define R_DENSE_CAM1_KC2 0X04*25
#define R_DENSE_CAM1_PC1 0X04*26
#define R_DENSE_CAM1_PC2 0X04*27
#define R_DENSE_CAM1_H11 0X04*28
#define R_DENSE_CAM1_H12 0X04*29
#define R_DENSE_CAM1_H13 0X04*30
#define R_DENSE_CAM1_H21 0X04*31
#define R_DENSE_CAM1_H22 0X04*32
#define R_DENSE_CAM1_H23 0X04*33
#define R_DENSE_CAM1_H31 0X04*34
#define R_DENSE_CAM1_H32 0X04*35
#define R_DENSE_CAM1_H33 0X04*36
#define R_DENSE_FLIP     0X04*44

namespace visensor {
DenseMatcher::DenseMatcher(SensorId::SensorId sensor_id,
                           ConfigConnection::WeakPtr config_connection)
    : Camera(
          sensor_id,
          ViCameraConfig(DenseMatcherDefaults::WIDTH,
                         DenseMatcherDefaults::HEIGHT,
                         DenseMatcherDefaults::RATE),
          SensorSettings(sensor_id,
                         SensorType::SensorType::DENSE_MATCHER,
                         calculateBufferSize(),
                         DenseMatcherDefaults::NUM_OF_MSGS_IN_PACKAGE,
                         DenseMatcherDefaults::USE_CONST_PACKAGE_SIZE,
                         DenseMatcherDefaults::CALIBRATION_SIZE),
          config_connection),
      config_(sensor_id) {
}

void DenseMatcher::init() {
//  setCalibration();
}

// set calibration
void DenseMatcher::setCalibration(ViCameraCalibration& calib_cam0, ViCameraCalibration& calib_cam1)
{
  // transform camera calibration in format used by SGM in FPGA
  Eigen::Matrix3d H0;
  Eigen::Matrix3d H1;
  double f;
  Eigen::Vector2d p0;
  Eigen::Vector2d p1;

  StereoHomography stereo_homography(calib_cam0, calib_cam1);
  stereo_homography.getHomography(H0, H1, f, p0, p1);

  // write to FPGA
  float gain = 5500.0;

  const ConfigConnection::Ptr& con = getConfigConnection();

  ViCameraLensModelRadtan::Ptr cam0_lens_model = calib_cam0.getLensModel<ViCameraLensModelRadtan>();
  ViCameraLensModelRadtan::Ptr cam1_lens_model = calib_cam1.getLensModel<ViCameraLensModelRadtan>();

  // cam0
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_CCX, static_cast<uint32_t>(p0[0]*256.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_CCY, static_cast<uint32_t>(p0[1]*256.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_FCX, static_cast<uint32_t>(1.0/f*1024.0*1024.0*4.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_FCY, static_cast<uint32_t>(1.0/f*1024.0*1024.0*4.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_KC1, static_cast<uint32_t>(cam0_lens_model->k1_*1024.0*32.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_KC2, static_cast<uint32_t>(cam0_lens_model->k2_*1024.0*32.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_PC1, static_cast<uint32_t>(cam0_lens_model->r1_*f*1024.0*32.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_PC2, static_cast<uint32_t>(cam0_lens_model->r2_*f*1024.0*32.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_H11, static_cast<uint32_t>(H0(0,0)*gain*64.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_H12, static_cast<uint32_t>(H0(0,1)*gain*256.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_H13, static_cast<uint32_t>(H0(0,2)*gain*256.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_H21, static_cast<uint32_t>(H0(1,0)*gain*256.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_H22, static_cast<uint32_t>(H0(1,1)*gain*64.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_H23, static_cast<uint32_t>(H0(1,2)*gain*256.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_H31, static_cast<uint32_t>(H0(2,0)*gain*256.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_H32, static_cast<uint32_t>(H0(2,1)*gain*256.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM0_H33, static_cast<uint32_t>(H0(2,2)*gain), ViComType::FPGA_32);

  // cam1
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_CCX, static_cast<uint32_t>(p1[0]*256.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_CCY, static_cast<uint32_t>(p1[1]*256.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_FCX, static_cast<uint32_t>(1.0/f*1024.0*1024.0*4.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_FCY, static_cast<uint32_t>(1.0/f*1024.0*1024.0*4.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_KC1, static_cast<uint32_t>(cam1_lens_model->k1_*1024.0*32.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_KC2, static_cast<uint32_t>(cam1_lens_model->k2_*1024.0*32.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_PC1, static_cast<uint32_t>(cam1_lens_model->r1_*f*1024.0*32.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_PC2, static_cast<uint32_t>(cam1_lens_model->r2_*f*1024.0*32.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_H11, static_cast<uint32_t>(H1(0,0)*gain*64.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_H12, static_cast<uint32_t>(H1(0,1)*gain*256.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_H13, static_cast<uint32_t>(H1(0,2)*gain*256.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_H21, static_cast<uint32_t>(H1(1,0)*gain*256.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_H22, static_cast<uint32_t>(H1(1,1)*gain*64.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_H23, static_cast<uint32_t>(H1(1,2)*gain*256.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_H31, static_cast<uint32_t>(H1(2,0)*gain*256.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_H32, static_cast<uint32_t>(H1(2,1)*gain*256.0), ViComType::FPGA_32);
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_CAM1_H33, static_cast<uint32_t>(H1(2,2)*gain), ViComType::FPGA_32);

  // set flip bit
  con->writeConfig(config_.sensor_id_, 0, R_DENSE_FLIP, static_cast<uint32_t>(calib_cam0.is_flipped_&&calib_cam1.is_flipped_), ViComType::FPGA_32);
}

uint32_t DenseMatcher::calculateBufferSize() {
  //only use const variables here
  //return	ceil( ( (2+DenseMatcherDefaults::WIDTH) * DenseMatcherDefaults::HEIGHT+2+2/* magic 2 */) * 10.0/8.0);
  return (DenseMatcherDefaults::WIDTH * DenseMatcherDefaults::HEIGHT);
}

//void DenseMatcher::allocateBuffer()
//{
////	_frame_size=Camera::_config.width * Camera::_config.height;
////	Sensor::_buffer_size=ceil( ( (2+Camera::_config.width) * Camera::_config.height+2+2/* magic 2 */) * 10.0/8.0)+8; //TODO fix magic 2!!!
////	Sensor::_buffer = new unsigned char[Sensor::_buffer_size];
////	Sensor::_bytes_in_buffer=0;
//
//
//}

DenseMatcher::~DenseMatcher() {

}

//the threaded function that works on the queue of finished measurements
void DenseMatcher::processMeasurements() {
  while (1) {

    boost::this_thread::interruption_point();

    //get the newest measurement
    boost::shared_ptr<Measurement> meas = Sensor::measurement_queue_.pop();

    //check if image size is OK
    if(meas->buffer_size != calculateBufferSize()+4)
    {
      VISENSOR_DEBUG("DenseMatcher buffer size does not match expected image size %d <-> %d\n", meas->buffer_size, calculateBufferSize());
      continue;
    }

    //check for missed frames
    // TODO(schneith): use a timer to check for missed frames (currently we only detected sequences of MISSED-RECEIVED-MISSED, not MISSED-MISSED,...)
    if( checkTimestamp(meas->timestamp) )
    {
      //publish an empty missed frame
      ViFrame::Ptr missed_frame_ptr = boost::make_shared<ViFrame>();
      missed_frame_ptr->camera_id = Camera::camera_id_;
      publishFrameData(missed_frame_ptr, ViErrorCodes::MEASUREMENT_DROPPED);
    }

//		//TODO adapt
//		// create new shared pointer for the frame
    ViFrame::Ptr frame_ptr = boost::make_shared<ViFrame>();

    // Add camera information to the frame
    frame_ptr->width = Camera::config_.width;
    frame_ptr->height = Camera::config_.height;
    frame_ptr->camera_id = Camera::camera_id_;
    frame_ptr->timestamp = meas->timestamp;
    frame_ptr->timestamp_synchronized = meas->timestamp_synchronized;
    frame_ptr->timestamp_fpga_counter = meas->timestamp_fpga_counter;
    frame_ptr->timestamp_host = meas->timestamp_host;

    frame_ptr->image_type = DenseMatcherDefaults::IMAGE_TYPE;

    frame_ptr->setImageRawPtr(meas->data, meas->bytes_in_buffer);

    publishFrameData(frame_ptr, ViErrorCodes::NO_ERROR);
  }
}

ViConfigMsg DenseMatcher::getConfigParam(std::string cmd, uint32_t value) {
//	msg->sensorId=Sensor::_sensorId;
  return config_.getConfigParam(cmd, value);
}

}  //namespace visensor
