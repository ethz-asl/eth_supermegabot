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

#ifndef SENSOR_HPP_
#define SENSOR_HPP_

#include <config/config.hpp>
#include <queue>
#include <boost/smart_ptr.hpp>
#include <boost/function.hpp>

#include "networking/config_connection.hpp"
#include "synchronization/concurrent_queue.hpp"
#include "networking/config_connection.hpp"
#include "visensor/visensor_constants.hpp"
#include "visensor/visensor_datatypes.hpp"

namespace visensor {

// used to clean up memory in shared pointers
template<typename T>
struct array_deleter {
  void operator ()(T const * p) {
    delete[] p;
  }
};

struct Measurement {
  typedef boost::shared_ptr<Measurement> Ptr;

  Measurement()
      : data(),
        buffer_size(0),
        bytes_in_buffer(0),
        timestamp(0),
        timestamp_synchronized(0),
        timestamp_fpga_counter(0),
        timestamp_host(0) {
  }
  ~Measurement() {
  }

  boost::shared_ptr<uint8_t> data;
  uint32_t buffer_size;
  uint32_t bytes_in_buffer;
  uint64_t timestamp; /* FPGA time of the message in nano seconds and corrected with an initial offset */
  uint64_t timestamp_synchronized; /* FPGA time of the message in nano seconds and corrected with a internal time synchronization model*/
  uint32_t timestamp_fpga_counter; /* raw fpga counter timestamp (1e-5 s) */
  uint64_t timestamp_host; /* time when the message was received on host*/
};

struct SensorSettings {
  SensorSettings(SensorId::SensorId id, SensorType::SensorType type,
                 uint32_t measurementBufferSize, uint32_t numOfMsgsInPackage,
                 bool constBufferSize, uint32_t calibrationBufferSize)
      : sensor_id(id),
        type(type),
        measurementBufferSize(measurementBufferSize),
        numOfMsgsInPackage(numOfMsgsInPackage),
        constBufferSize(constBufferSize),
        calibrationBufferSize(calibrationBufferSize),
        active(false),
        rate(0) {
  }
  const SensorId::SensorId sensor_id;
  const SensorType::SensorType type;
  const uint32_t measurementBufferSize;
  const uint32_t numOfMsgsInPackage;
  const bool constBufferSize;
  const uint32_t calibrationBufferSize;
  bool active;
  uint32_t rate;
};

class Sensor {
 public:
  typedef boost::shared_ptr<Sensor> Ptr;
  typedef std::map<SensorId::SensorId, Sensor::Ptr> IdMap;

  Sensor(SensorSettings sensorSettings,
         ConfigConnection::WeakPtr config_connection);
  virtual ~Sensor();

  virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value)=0;
  virtual void init() = 0;
  virtual void setUserCallback(boost::function<void(ViImuMsg::Ptr, ViErrorCode)> callback){};
  virtual void setFrameCallback(boost::function<void(ViFrame::Ptr, ViErrorCode)> callback){};
  virtual void setCornerCallback(boost::function<void(ViCorner::Ptr, ViErrorCode)> callback){};

  virtual void setUserCallback(boost::function<void(ViExternalTriggerMsg::Ptr)> callback){};
  virtual void setUserCallback(boost::function<void (ViFrame::Ptr, ViCorner::Ptr)> callback){};

  bool startSensor(uint32_t rate);
  bool stopSensor();
  uint64_t getTimeBetweenMsgs() const;
  int getNumOfMsgsInPackage() const;
  int getMeasurementBufferSize() const;
  Measurement getEmptyCalibrationMsg() const;
  void addMeasurement(const Measurement::Ptr meas);

  inline bool getSensorActive() const {
    return settings_.active;
  }

  inline bool getIsConstBufferSize() const {
    return settings_.constBufferSize;
  }

  inline SensorType::SensorType type() const {
    return settings_.type;
  }

  inline const SensorId::SensorId id() const {
    return settings_.sensor_id;
  }

  const SensorSettings& settings() const {
    return settings_;
  }

 protected:
  void writeRequest(const ViConfigMsg configMsg);
  void readRequest(ViConfigMsg& configMsg);
  bool checkTimestamp(uint64_t timestamp);
  ConfigConnection::Ptr getConfigConnection();

 public:
  concurrent_queue<std::queue<Measurement::Ptr> > measurement_queue_;

 private:
  //config server conenction
  ConfigConnection::WeakPtr config_connection_;

  SensorSettings settings_;
  uint64_t prev_timestamp_;
  uint32_t allowed_timediff_;
  uint32_t measurement_miss_counter_;
};

}  //namespace visensor

#endif /* SENSOR_HPP_ */
