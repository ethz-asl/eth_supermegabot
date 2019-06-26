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

#ifndef Corner_DETECTOR_HPP_
#define Corner_DETECTOR_HPP_

#include <config/config.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/function.hpp>

#include "sensors/sensor.hpp"

namespace visensor {

struct ViCornerConfig {
  ViCornerConfig(uint32_t width_, uint32_t height_, uint32_t frame_rate_)
      : width(width_),
        height(height_),
        frame_rate(frame_rate_) {
  }
  ;
  uint32_t width; /* the image width */
  uint32_t height; /* the image height */
  uint32_t frame_rate; /* the Corner frame rate */
};

class CornerDetector : public Sensor {
 public:
  typedef boost::shared_ptr<CornerDetector> Ptr;

  CornerDetector(SensorId::SensorId sensor_id, ViCornerConfig config,
                 SensorSettings sensorSettings,
                 ConfigConnection::WeakPtr config_connection)
      : Sensor(sensorSettings, config_connection),
        config_(config),
        camera_id_(sensor_id){
  }
  ;

  virtual ~CornerDetector() {
  }
  ;

  virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value) = 0;
  void publishCornerData(ViCorner::Ptr& frame, ViErrorCode error);
  void setCornerCallback(boost::function<void(ViCorner::Ptr, ViErrorCode)> callback);
  //the threaded function that works on the queue of finished measurements
  virtual void processMeasurements() = 0;
  virtual void init() = 0;

 protected:
  boost::function<void(ViCorner::Ptr, ViErrorCode)> user_callback_;
  ViCornerConfig config_; /* the Corner configuration */
  const uint32_t camera_id_;
};
}  //namespace visensor

#endif /* Corner_HPP_ */
