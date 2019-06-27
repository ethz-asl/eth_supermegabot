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

#ifndef ExternalTrigger_HPP_
#define ExternalTrigger_HPP_

#include <config/config.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/function.hpp>

#include "networking/param_server.hpp"
#include "sensors/sensor.hpp"

namespace visensor {

namespace ExternalTriggerDefaults {
const int MSG_SIZE = 1, NUM_OF_MSGS_IN_PACKAGE = 1;
const int I2C_ADRESS = 0x48;
const uint8_t COM_TYPE = ViComType::FPGA_32;

const uint8_t R_EX_TRIGGER_CONTROL = 0x00;
const uint8_t R_EX_TRIGGER_DIRECTION = 0x04;
const uint8_t R_EX_TRIGGER_POLARITY = 0x08; // 0=active high, 1=low active
const uint8_t R_EX_TRIGGER_LENGHT = 0x0C; // defines the impulse length in fpga zyklen (125MHz)
const uint8_t R_EX_TRIGGER0_START_OFFSET = 0x10; // 0=OFF, 1=ON
const uint8_t R_EX_TRIGGER0_INTERVAL = 0x14; // 0=out, 1=in
const uint8_t R_EX_TRIGGER1_START_OFFSET = 0x18; // 0=active high, 1=low active
const uint8_t R_EX_TRIGGER1_INTERVAL = 0x1C; // defines the impulse length in fpga zyklen (125MHz)
const uint8_t R_EX_TRIGGER2_START_OFFSET = 0x20;
const uint8_t R_EX_TRIGGER2_INTERVAL = 0x24;
}

class ExternalTriggerConfig {
 public:
  ExternalTriggerConfig(const SensorId::SensorId sensorId)
      : sensor_id_(sensorId) {
    // name, type, register, default, mask, min, max
    param_server_.addParam("enable_trigger_0", param_server::BOOL_T, ExternalTriggerDefaults::R_EX_TRIGGER_CONTROL, 0, 1); // mask=0b1
    param_server_.addParam("enable_trigger_1", param_server::BOOL_T, ExternalTriggerDefaults::R_EX_TRIGGER_CONTROL, 0, 2); // mask=0b10
    param_server_.addParam("enable_trigger_2", param_server::BOOL_T, ExternalTriggerDefaults::R_EX_TRIGGER_CONTROL, 0, 4); // mask=0b100

    param_server_.addParam("trigger_0_rate", param_server::UINT_T, ExternalTriggerDefaults::R_EX_TRIGGER0_INTERVAL, 100, 0, 0, 100000);
    param_server_.addParam("trigger_1_rate", param_server::UINT_T, ExternalTriggerDefaults::R_EX_TRIGGER1_INTERVAL, 100, 0, 0, 100000);
    param_server_.addParam("trigger_2_rate", param_server::UINT_T, ExternalTriggerDefaults::R_EX_TRIGGER2_INTERVAL, 100, 0, 0, 100000);

    param_server_.addParam("trigger_0_offset", param_server::UINT_T, ExternalTriggerDefaults::R_EX_TRIGGER0_START_OFFSET, 0, 0, 0, 100000);
    param_server_.addParam("trigger_1_offset", param_server::UINT_T, ExternalTriggerDefaults::R_EX_TRIGGER1_START_OFFSET, 0, 0, 0, 100000);
    param_server_.addParam("trigger_2_offset", param_server::UINT_T, ExternalTriggerDefaults::R_EX_TRIGGER2_START_OFFSET, 0, 0, 0, 100000);

    param_server_.addParam("trigger_0_direction", param_server::BOOL_T, ExternalTriggerDefaults::R_EX_TRIGGER_DIRECTION, 0, 1); //0 is output, 1 is input; mask=0b1
    param_server_.addParam("trigger_1_direction", param_server::BOOL_T, ExternalTriggerDefaults::R_EX_TRIGGER_DIRECTION, 0, 2); //0 is output, 1 is input; mask=0b10
    param_server_.addParam("trigger_2_direction", param_server::BOOL_T, ExternalTriggerDefaults::R_EX_TRIGGER_DIRECTION, 0, 4); //0 is output, 1 is input; mask=0b100
  }

  ViConfigMsg getConfigParam(const std::string paramName, uint16_t value) {
    ViConfigMsg msg;
    msg.valChanged = param_server_.getConfigParam(paramName, value, msg.reg,
                                                  msg.val);
    msg.sensorId = sensor_id_;
    msg.devAdress = ExternalTriggerDefaults::I2C_ADRESS;
    msg.comType = ExternalTriggerDefaults::COM_TYPE;
    return msg;
  }

  ViConfigMsg getConfigParam(const std::string paramName) {
    ViConfigMsg msg;
    msg.valChanged = param_server_.getConfigParam(paramName, msg.reg, msg.val);
    msg.sensorId = sensor_id_;
    msg.devAdress = ExternalTriggerDefaults::I2C_ADRESS;
    msg.comType = ExternalTriggerDefaults::COM_TYPE;
    return msg;
  }

  const SensorId::SensorId sensor_id_;

 private:
  param_server::ParamServer param_server_;
};

class ExternalTrigger : public Sensor {
 public:
  typedef boost::shared_ptr<ExternalTrigger> Ptr;

  ExternalTrigger(SensorId::SensorId sensor_id,
                  IpConnection::WeakPtr config_connection);
  virtual ~ExternalTrigger() {
  }
  ;

  virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value);
  void publishExternalTriggerData(
      ViExternalTriggerMsg::Ptr ExternalTrigger);
  void setUserCallback(
      boost::function<void(ViExternalTriggerMsg::Ptr)> callback);
  virtual void init();

  void processMeasurements();
  static uint32_t calculateBufferSize();

 protected:
  boost::function<void(ViExternalTriggerMsg::Ptr)> user_callback_;

 private:
  ExternalTriggerConfig config_;
};
}  //namespace visensor

#endif /* ExternalTrigger_HPP_ */
