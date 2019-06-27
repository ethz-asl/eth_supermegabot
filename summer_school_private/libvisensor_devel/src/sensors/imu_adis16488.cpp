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

#include <sensors/imu_adis16488.hpp>

namespace visensor {

ImuAdis16488::ImuAdis16488(SensorId::SensorId sensor_id, IpConnection::WeakPtr config_connection)
    : Imu(
        sensor_id,
        SensorSettings(sensor_id, SensorType::SensorType::IMU_ADIS16488, calculateBufferSize(),
                       ImuAdis16488Defaults::NUM_OF_MSGS_IN_PACKAGE, ImuAdis16488Defaults::USE_CONST_PACKAGE_SIZE,
                       ImuAdis16488Defaults::CALIBRATION_SIZE),
        config_connection) {
}

uint32_t ImuAdis16488::calculateBufferSize() {
  return (ImuAdis16488Defaults::MSG_SIZE);
}

void ImuAdis16488::init() {
}

void ImuAdis16488::processMeasurements() {
  while (1) {
    boost::this_thread::interruption_point();
    Measurement::Ptr meas = Sensor::measurement_queue_.pop();

    //check for missed frames
    // TODO(schneith): use a timer to check for missed frames (currently we only detected sequences of MISSED-RECEIVED-MISSED, not MISSED-MISSED,...)
    if (checkTimestamp(meas->timestamp)) {
      //publish an empty missed frame
      ViImuMsg::Ptr missed_imu_ptr = boost::make_shared<ViImuMsg>();
      missed_imu_ptr->imu_id = Imu::imu_id_;
      publishImuData(missed_imu_ptr, ViErrorCodes::MEASUREMENT_DROPPED);
    }

    ViImuMsg::Ptr imu_msg_ptr = boost::make_shared<ViImuMsg>();

    // Add imu information to the msg
    imu_msg_ptr->imu_id = Imu::imu_id_;
    imu_msg_ptr->timestamp = meas->timestamp;
    imu_msg_ptr->timestamp_synchronized = meas->timestamp_synchronized;
    imu_msg_ptr->timestamp_fpga_counter = meas->timestamp_fpga_counter;
    imu_msg_ptr->timestamp_host = meas->timestamp_host;

    // get imu data
    getGyro(meas->data.get(), &imu_msg_ptr->gyro[0]);
    getAcc(meas->data.get(), &imu_msg_ptr->acc[0]);
    getMag(meas->data.get(), &imu_msg_ptr->mag[0]);
    getBaro(meas->data.get(), &imu_msg_ptr->baro);
    getTemp(meas->data.get(), &imu_msg_ptr->temperature);

    publishImuData(imu_msg_ptr, ViErrorCodes::NO_ERROR);
  }
}

ViConfigMsg ImuAdis16488::getConfigParam(std::string cmd, uint32_t value) {
  // TODO: implement this
  ViConfigMsg config;
  config.sensorId = SensorId::SensorId::NOT_SPECIFIED;
  config.devAdress = 0;
  config.reg = 0;
  config.val = 0;
  config.comType = 0;
  config.valChanged = false;
  return config;
}

void ImuAdis16488::getGyro(uint8_t * buffer, double * gyro) {
  int32_t temp;
  int current_pos;
  for (int i = 0; i < 3; i++) {
    current_pos = 4 * i;
    temp = (int32_t) ((buffer[current_pos] << 24) | (buffer[current_pos + 1] << 16)
        | (buffer[current_pos + 2] << 8) | (buffer[current_pos + 3] << 0));
    gyro[i] = (double) temp;
    gyro[i] = gyro[i] * 450.0 / 1474560000.0 / 180.0 * M_PI;
  }
}

void ImuAdis16488::getAcc(uint8_t * buffer, double * acc) {
  int i, current_pos;
  int32_t temp;

  for (i = 0; i < 3; i++) {
    current_pos = 3 * 4 + 4 * i;
    temp = (int32_t) ((buffer[current_pos] << 24) | (buffer[current_pos + 1] << 16)
        | (buffer[current_pos + 2] << 8) | (buffer[current_pos + 3] << 0));

    if ((temp / 81920000.0 * STANDARD_GRAVITY) < -200.0)
      VISENSOR_DEBUG("wrong imu acc value id: %i value raw: %i value: %f \n", i, temp,
                     (temp /81920000*STANDARD_GRAVITY));

    acc[i] = (double) temp;
    acc[i] = acc[i] / 81920000.0 * STANDARD_GRAVITY;	//m^2/s
  }
}

void ImuAdis16488::getMag(uint8_t * buffer, double * mag) {
  for (int i = 0; i < 3; i++) {
    int current_pos = 6 * 4 + 2 * i;
    int temp = (int16_t) ((buffer[current_pos + 0] << 8) | (buffer[current_pos + 1] << 0));

    mag[i] = static_cast<double>(temp) * 3.2767 / 2147418112.0;
  }
}

void ImuAdis16488::getBaro(uint8_t * buffer, double * baro) {
  int current_pos = 6 * 4 + 3 * 2;
  int temp = (uint32_t) ((buffer[current_pos + 0] << 24) | (buffer[current_pos + 1] << 16)
      | (buffer[current_pos + 2] << 8) | (buffer[current_pos + 3] << 0));

  *baro = static_cast<double>(temp) * 1.31068 / 2147418112.0 * 1000.0;  //hPa
}

void ImuAdis16488::getTemp(uint8_t * buffer, double * temp) {
  int current_pos = 7 * 4 + 3 * 2;
  int temp_int = (int) ((buffer[current_pos + 0] << 8) | (buffer[current_pos + 1] << 0));
  *temp = static_cast<double>(temp_int) * 0.00565 + 25;  // C;
}

}  //namespace visensor
