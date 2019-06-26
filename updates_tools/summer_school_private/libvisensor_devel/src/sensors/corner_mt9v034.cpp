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

#include "sensors/corner_mt9v034.hpp"

namespace visensor {
CornerMt9v034::CornerMt9v034(SensorId::SensorId sensor_id,
                             IpConnection::WeakPtr config_connection)
    : CornerDetector(
        sensor_id,
          ViCornerConfig(CornerMt9v034Defaults::WIDTH,
                             CornerMt9v034Defaults::HEIGHT,
                             CornerMt9v034Defaults::RATE),
          SensorSettings(sensor_id, SensorType::SensorType::CORNER_MT9V034,
                         calculateBufferSize(),
                         CornerMt9v034Defaults::NUM_OF_MSGS_IN_PACKAGE,
                         CornerMt9v034Defaults::USE_CONST_PACKAGE_SIZE,
                         CornerMt9v034Defaults::CALIBRATION_SIZE),
          config_connection),
      config_(sensor_id) {

}

void CornerMt9v034::init() {

}

uint32_t CornerMt9v034::calculateBufferSize() {
  //only use const variables here
  return (CornerMt9v034Defaults::MAX_FEATURES
      * (CornerMt9v034Defaults::DEPTH / 8 + 4));
}

CornerMt9v034::~CornerMt9v034() {

}

//the threaded function that works on the queue of finished measurements
void CornerMt9v034::processMeasurements() {
  while (1) {

    boost::this_thread::interruption_point();
    //get the newest measurement
    Measurement::Ptr meas = Sensor::measurement_queue_.pop();

//		VISENSOR_DEBUG("in corner thread %d\n", Corner::_cameraId);

    //check for missed frames
    // TODO(schneith): use a timer to check for missed frames (currently we only detected sequences of MISSED-RECEIVED-MISSED, not MISSED-MISSED,...)
    if( checkTimestamp(meas->timestamp) )
    {
      //publish an empty missed frame
      ViCorner::Ptr missed_corner_ptr = boost::make_shared<ViCorner>();
      missed_corner_ptr->camera_id = camera_id_;
      publishCornerData(missed_corner_ptr, ViErrorCodes::MEASUREMENT_DROPPED);
    }

		// create new shared pointer for the corners
    ViCorner::Ptr corner_ptr = boost::make_shared<ViCorner>();

    corner_ptr->camera_id = camera_id_;
    corner_ptr->timestamp = meas->timestamp;
    corner_ptr->timestamp_synchronized = meas->timestamp_synchronized;
    corner_ptr->timestamp_fpga_counter = meas->timestamp_fpga_counter;
    corner_ptr->timestamp_host = meas->timestamp_host;

    // process corners
    uint32_t pos = 0;
    while (pos < meas->buffer_size) {
      ViCornerElem corner;
      if (getCornerElem(meas->data.get() + pos, corner))
        corner_ptr->corners.push_back(corner);
//			else
//				VISENSOR_DEBUG("pos in buffer: %u \n",meas->_bytesInBuffer/8-pos/8);
      pos += CornerMt9v034Defaults::DEPTH / 8 + 4;
    }

		VISENSOR_DEBUG("corners size %lu\n",corner_ptr->corners.size());

    publishCornerData(corner_ptr, ViErrorCodes::NO_ERROR);
  }
}

bool CornerMt9v034::getCornerElem(uint8_t* buf, ViCornerElem& corner) {

  corner.x = ((buf[0] << 8) | (buf[1] << 0));
  corner.y = ((buf[2] << 8) | (buf[3] << 0));
  corner.score = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8)
      | (buf[7] << 0);

  if (corner.x >= CornerDetector::config_.width || corner.y >= CornerDetector::config_.height) {
    VISENSOR_DEBUG(
        "Sensor::Corner::CornerMt9v034 - invalid corner position x:%u y:%u\n",
        corner.x, corner.y);
    return false;
  }

//	if(corner.x==0 && corner.y==0)
//	{
//		VISENSOR_DEBUG("Sensor::Corner::CornerMt9v034 - zero corner received score:%u\n",corner.score);
//		return false;
//	}

  return true;
}

ViConfigMsg CornerMt9v034::getConfigParam(std::string cmd, uint32_t value) {
  return config_.getConfigParam(cmd, value);
}

}  //namespace visensor
