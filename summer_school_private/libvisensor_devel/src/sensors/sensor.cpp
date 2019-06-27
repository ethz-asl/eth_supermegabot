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

#include <boost/smart_ptr.hpp>

#include "networking/config_connection.hpp"
#include "sensors/sensor.hpp"

namespace visensor
{

Sensor::Sensor(SensorSettings sensor_settings, ConfigConnection::WeakPtr config_connection):
    config_connection_(config_connection),
    settings_(sensor_settings),
    prev_timestamp_(0),
    allowed_timediff_(0),
    measurement_miss_counter_(0)
{
}

Sensor::~Sensor()
{
}

//lock weak pointer for usage
ConfigConnection::Ptr Sensor::getConfigConnection() {
  return config_connection_.lock();
}

void Sensor::writeRequest(const ViConfigMsg configMsg)
{
  getConfigConnection()->writeConfig(configMsg.sensorId,configMsg.devAdress,configMsg.reg,configMsg.val, configMsg.comType);
  return;
}

void Sensor::readRequest(ViConfigMsg& configMsg)
{
  getConfigConnection()->readConfig(configMsg.sensorId,configMsg.devAdress,configMsg.reg, configMsg.val, configMsg.comType);
  return;
}

bool Sensor::checkTimestamp(uint64_t timestamp)
{
  bool frameMissed = false;

  //init with first timestamp
	if(prev_timestamp_==0)
	{
		prev_timestamp_=timestamp;
		return frameMissed;
	}

	if(timestamp-prev_timestamp_>allowed_timediff_)
	{
	  measurement_miss_counter_++;
		VISENSOR_DEBUG("sensor %d frame missed (total misses: %u), timediff: %ld ns\n", id(), measurement_miss_counter_, timestamp-prev_timestamp_);
		frameMissed = true;
	}

	prev_timestamp_=timestamp;
	return frameMissed;
}

int Sensor::getMeasurementBufferSize() const
{
  return settings_.measurementBufferSize;
}

void Sensor::addMeasurement(const Measurement::Ptr meas){
  measurement_queue_.push(meas);
}

//return time between msgs in nanoseconds
uint64_t Sensor::getTimeBetweenMsgs() const
{
	// avoid division by 0, just return 0 if rate is not set yet
	if(settings_.rate==0)
		return 0;

	return 1000000000/settings_.rate;
}

int Sensor::getNumOfMsgsInPackage() const
{
	return settings_.numOfMsgsInPackage;
}

// start sensor with given rate
bool Sensor::startSensor(uint32_t rate)
{
	settings_.rate=rate;
	settings_.active=true;
	allowed_timediff_=1.5*getTimeBetweenMsgs();
	return true;
}

// stop sensor
bool Sensor::stopSensor()
{
	settings_.active=false;
	return true;
}

}//namespace visensor
