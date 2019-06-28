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

#ifndef IMU_HPP_
#define IMU_HPP_

#include <config/config.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/function.hpp>

#include "sensors/sensor.hpp"

namespace visensor {
#define STANDARD_GRAVITY 9.80665

class Imu : public Sensor {
 public:
  typedef boost::shared_ptr<Imu> Ptr;

  Imu(SensorId::SensorId sensor_id, SensorSettings sensorSettings,
      ConfigConnection::WeakPtr config_connection)
      : Sensor(sensorSettings, config_connection),
        imu_id_(sensor_id) {
  }
  virtual ~Imu() {
  }
  ;
  virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value)=0;

  void publishImuData(ViImuMsg::Ptr imu, ViErrorCode error);
  void setUserCallback(boost::function<void(ViImuMsg::Ptr, ViErrorCode)> callback);
  virtual void init() = 0;

 protected:
  boost::function<void(ViImuMsg::Ptr, ViErrorCode)> user_callback_;
  const SensorId::SensorId imu_id_;
};
}  //namespace visensor

#endif /* IMU_HPP_ */
