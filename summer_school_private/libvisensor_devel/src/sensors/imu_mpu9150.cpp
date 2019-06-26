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

#include <sensors/imu_mpu9150.hpp>

namespace visensor {

ImuMpu9150::ImuMpu9150(SensorId::SensorId sensor_id,
                       IpConnection::WeakPtr config_connection)
    : Imu(sensor_id,
          SensorSettings(sensor_id, SensorType::SensorType::MPU_9150,
                         calculateBufferSize(),
                         ImuMpu9150Defaults::NUM_OF_MSGS_IN_PACKAGE,
                         ImuMpu9150Defaults::USE_CONST_PACKAGE_SIZE,
                         ImuMpu9150Defaults::CALIBRATION_SIZE),
          config_connection),
      config_(sensor_id) {
}

uint32_t ImuMpu9150::calculateBufferSize() {
  return (ImuMpu9150Defaults::MSG_SIZE);
}

void ImuMpu9150::init() {
  writeRequest(config_.getConfigParam("enable"));
  writeRequest(config_.getConfigParam("digital_low_pass_filter_config"));
}

void ImuMpu9150::processMeasurements() {
  while (1) {
    boost::this_thread::interruption_point();

    //get the newest measurement
    boost::shared_ptr<Measurement> meas = Sensor::measurement_queue_.pop();

    //checkTimestamp(meas.timestamp);

    ViImuMsg::Ptr imu_msg_ptr = boost::make_shared<ViImuMsg>();

    // Add imu information to the msg
    imu_msg_ptr->imu_id = Imu::imu_id_;
    imu_msg_ptr->timestamp = meas->timestamp;
    imu_msg_ptr->timestamp_synchronized = meas->timestamp_synchronized;
    imu_msg_ptr->timestamp_fpga_counter = meas->timestamp_fpga_counter;
    imu_msg_ptr->timestamp_host = meas->timestamp_host;
    getGyro(meas->data.get(), &imu_msg_ptr->gyro[0]);
    getTemperature(meas->data.get(), &imu_msg_ptr->temperature);
    getAcc(meas->data.get(), &imu_msg_ptr->acc[0]);

    publishImuData(imu_msg_ptr, ViErrorCodes::NO_ERROR);
  }
}

ViConfigMsg ImuMpu9150::getConfigParam(std::string cmd, uint32_t value) {
  return config_.getConfigParam(cmd, value);
}

void ImuMpu9150::getAcc(uint8_t* buffer, double * acc) {
  int i, current_pos;
  int16_t temp;

  for (i = 0; i < 3; i++) {
    current_pos = 2 * i;
    temp = (int) ((buffer[current_pos] << 8) | (buffer[current_pos + 1] << 0));
    acc[i] = (double) temp;
    acc[i] = acc[i] / 16384 * STANDARD_GRAVITY;
  }
}

void ImuMpu9150::getGyro(uint8_t* buffer, double * gyro) {
  int i, current_pos;
  int16_t temp;
  for (i = 0; i < 3; i++) {
    current_pos = 2 * 3 + 2 + 2 * i;
    temp = (int) ((buffer[current_pos] << 8) | (buffer[current_pos + 1] << 0));
    gyro[i] = (double) temp;
    gyro[i] = gyro[i] / 131 * M_PI / 180;
  }
}

void ImuMpu9150::getTemperature(uint8_t* buffer, double * temperature) {
  int current_pos;
  int16_t temp;
  current_pos = 2 * 3;
  temp =
      (int16_t) ((buffer[current_pos] << 8) | (buffer[current_pos + 1] << 0));
  *temperature = (double) temp;
  *temperature = *temperature / 340.0 + 35.0;
}

}  //namespace visensor
