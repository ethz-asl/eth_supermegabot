/*
 * Copyright (c) 2015, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2015, Autonomous Systems Lab, ETH Zurich, Switzerland
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

#ifndef VISENSOR_CONSTANTS_HPP_
#define VISENSOR_CONSTANTS_HPP_

#include <stdint.h>

#include "visensor/visensor_config.hpp"

namespace visensor {
  #define CAMERA_FREQUENCY 20 // in Hz
  #define IMU_FREQUENCY 200 // in Hz
  #define FPGA_FREQUENCY 125000000
  #define FPGA_TIME_COUNTER_FREQUENCY 100000.0 //fpga timestamp counter frequency

  static constexpr uint64_t SECOND_IN_NANOSECOND = 1e9;
  static constexpr uint64_t MILLISECOND_IN_NANOSECOND = 1e6;
  static constexpr uint64_t MUSECOND_IN_NANOSECOND = 1e3;
  static constexpr uint64_t MINUTE_IN_SECOND = 60;

  /** The sensor communication types
   *
   */
  namespace ViComType {
    static const int SPI_8 = 0, SPI_16 = 1, SPI_32 = 2, I2C_8 = 3, I2C_16 = 4, I2C_32 = 5, FPGA_32 = 6;
  }

  namespace SensorType {
  /** The enum specifying the type of sensor
   *
   */
    enum SensorType {
      CAMERA_MT9V034 = 1,   ///< CAMERA_MT9V034
      IMU_ADIS16488 = 2,    ///< IMU_ADIS16488
      IMU_MPU6000 = 3,		  ///< mpu of prototype 0
      IMU_ADIS16375 = 4,    ///< IMU_ADIS16375
      IMU_MPU6000_P1 = 5,   ///< mpu of prototype 1
      CAMERA_TAU320 = 6,    ///< CAMERA_TAU320
      CORNER_MT9V034 = 7,   ///< CORNER_MT9V034
      IMU_ADXRS450 = 8,     ///< IMU_ADXRS450
      BRISK_MT9V034 = 9,    ///< BRISK_MT9V034
      TIMING_BLOCK = 10,    ///< TIMING_BLOCK
      IMU_ADIS16448 = 11,   ///< IMU_ADIS16448
      CAMERA_TAU640 = 12,   ///< CAMERA_TAU640
      LIGHT_CONTROL = 13,   ///< LIGHT_CONTROL
      ZYNQ_STATUS = 14,     ///< ZYNQ_STATUS
      DENSE_MATCHER = 15,   ///< DENSE_MATCHER
      EXTERNAL_TRIGGER = 16,///< EXTERNAL_TRIGGER
      MPU_9150 = 17         ///< MPU_9150
    };
  }  //namespace SensorType

  namespace SensorId {
    /** Enum defining the unique ID for all available sensors for the VI-Sensor
     *
     */
    enum SensorId {
      CAM0 = 0,              ///< CAM0 MT9V034 on port 0
      CAM1 = 1,              ///< CAM1 MT9V034 on port 1
      CAM2 = 2,              ///< CAM2 MT9V034 on port 2
      CAM3 = 3,              ///< CAM3 MT9V034 on port 3
      CAM4 = 30,             ///< CAM0 MT9V034 on port 4
      CAM5 = 31,             ///< CAM1 MT9V034 on port 5
      CAM6 = 32,             ///< CAM2 MT9V034 on port 6
      CAM7 = 33,             ///< CAM3 MT9V034 on port 7
      IMU0 = 4,             ///< IMU0 Primary IMU (ADIS 164xx)
      IMU_CAM0 = 5,          ///< IMU_CAM0 Invensense MPU 9050 on cam0 PCB
      IMU_CAM1 = 6,          ///< IMU_CAM1
      IMU_CAM2 = 7,          ///< IMU_CAM2
      IMU_CAM3 = 8,          ///< IMU_CAM3
      CORNER_CAM0 = 9,       ///< CORNER_CAM0 Experimental FPGA Harris Corners for cam0
      CORNER_CAM1 = 10,      ///< CORNER_CAM1
      CORNER_CAM2 = 11,      ///< CORNER_CAM2
      CORNER_CAM3 = 12,      ///< CORNER_CAM3
      DENSE_MATCHER0 = 13,   ///< DENSE_MATCHER0 Experimental FPGA SGM dense matcher
      EXTERNAL_TRIGGER0 = 14,///< EXTERNAL_TRIGGER0 Trigger/Interrupt line
      SENSOR_STATUS = 15,    ///< SENSOR_STATUS Experimental
      SENSOR_CLOCK = 16,     ///< SENSOR_CLOCK Experimental
      FLIR0 = 17,            ///< FLIR0 Experimental FLIR Tau 640
      FLIR1 = 18,            ///< FLIR1
      FLIR2 = 19,            ///< FLIR2
      FLIR3 = 20,            ///< FLIR3
      LED_FLASHER0 = 21,     ///< LED_FLASHER0 PWM interface to LED Flasher
      NOT_SPECIFIED = -1     ///< NOT_SPECIFIED
    };

    DSO_EXPORT inline SensorId getCamId(SensorId id) {
      switch (id) {
        case (CAM0):
        case (CORNER_CAM0):
        case (DENSE_MATCHER0):
        case (FLIR0):
          return CAM0;
        case (CAM1):
        case (CORNER_CAM1):
        case (FLIR1):
          return CAM1;
        case (CAM2):
        case (CORNER_CAM2):
        case (FLIR2):
          return CAM2;
        case (CAM3):
        case (CORNER_CAM3):
        case (FLIR3):
          return CAM3;
        case (CAM4):
          return CAM4;
        case (CAM5):
          return CAM5;
        case (CAM6):
          return CAM6;
        case (CAM7):
          return CAM7;
        default:
          return NOT_SPECIFIED;
      }
    }
  }  //namespace SensorPort

}  //namespace visensor

#endif /* VISENSOR_CONSTANTS_HPP_ */
