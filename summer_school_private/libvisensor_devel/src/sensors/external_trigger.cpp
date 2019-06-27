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
#include <boost/function.hpp>

#include "networking/connection.hpp"
#include "sensors/external_trigger.hpp"

namespace visensor {

ExternalTrigger::ExternalTrigger(SensorId::SensorId sensor_id,
                                 IpConnection::WeakPtr config_connection)
    : Sensor(
          SensorSettings(sensor_id,
                         SensorType::SensorType::EXTERNAL_TRIGGER,
                         calculateBufferSize(),
                         ExternalTriggerDefaults::NUM_OF_MSGS_IN_PACKAGE, 0, 0),
          config_connection),
          config_(sensor_id) {
}

void ExternalTrigger::setUserCallback(
    boost::function<void(ViExternalTriggerMsg::Ptr)> callback) {
  user_callback_ = callback;
}

ViConfigMsg ExternalTrigger::getConfigParam(std::string cmd, uint32_t value) {
  return config_.getConfigParam(cmd, value);
}

void ExternalTrigger::publishExternalTriggerData(
    ViExternalTriggerMsg::Ptr msg) {
  if (user_callback_)
    user_callback_(msg);
}

uint32_t ExternalTrigger::calculateBufferSize() {
  return (ExternalTriggerDefaults::MSG_SIZE);
}

void ExternalTrigger::init() {
}

void ExternalTrigger::processMeasurements() {
  while (1) {

    boost::this_thread::interruption_point();
    //get the newest measurement
    Measurement::Ptr meas = Sensor::measurement_queue_.pop();

    // create new shared pointer for the message
    ViExternalTriggerMsg::Ptr trigger_msg_ptr = boost::make_shared<ViExternalTriggerMsg>();

    uint8_t* buffer = meas->data.get();
    uint8_t id = static_cast<int8_t>(buffer[0]);

    trigger_msg_ptr->trigger_id = id;
    trigger_msg_ptr->timestamp = meas->timestamp;
    trigger_msg_ptr->timestamp_synchronized = meas->timestamp_synchronized;
    trigger_msg_ptr->timestamp_fpga_counter = meas->timestamp_fpga_counter;
    trigger_msg_ptr->timestamp_host = meas->timestamp_host;
    trigger_msg_ptr->event_id = *(uint32_t*)(meas->data.get() + 1); //counts all trigger events (allows for event missed checking)
    publishExternalTriggerData(trigger_msg_ptr);

  }
}
}  //namespace visensor
