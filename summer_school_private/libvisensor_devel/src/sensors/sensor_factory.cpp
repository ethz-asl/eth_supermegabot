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

#include <vector>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

#include "visensor/visensor_constants.hpp"
#include "sensors/camera_mt9v034.hpp"
#include "sensors/camera_tau640.hpp"
#include "sensors/imu_adis16448.hpp"
#include "sensors/imu_adis16488.hpp"
#include "sensors/imu_mpu9150.hpp"
#include "sensors/corner_mt9v034.hpp"
#include "sensors/dense_matcher.hpp"
#include "sensors/external_trigger.hpp"
#include "sensors/corner_detector.hpp"
#include "sensors/led_flasher.hpp"

#include "sensors/sensor_factory.hpp"

namespace visensor {

const void SensorFactory::createSensors(
    IpConnection::WeakPtr config_connection_weak,
    Sensor::IdMap* sensor_map, boost::thread_group* threads) {

  //lock weak pointer
  IpConnection::Ptr config_connection = config_connection_weak.lock();

  //get sensor list
  std::vector<IpComm::SensorInfo> sensor_list = config_connection->getAttachedSensors();

  //	VISENSOR_DEBUG("in creatSensor \n");
  for (size_t i = 0; i < sensor_list.size(); i++) {
    // get references to the current sensor
    const SensorType::SensorType sensor_type =
        static_cast<SensorType::SensorType>(sensor_list[i].sensor_type);
    SensorId::SensorId sensor_id =
        static_cast<SensorId::SensorId>(sensor_list[i].sensor_id);

    switch (sensor_type) {
      case SensorType::SensorType::CAMERA_MT9V034: {
        CameraMt9v034::Ptr new_camera = boost::make_shared<CameraMt9v034>(
            sensor_id, config_connection_weak);
        VISENSOR_DEBUG("camera MT9V034 created with sensor_id: %d\n",
                       sensor_id);

        boost::thread *t1 = new boost::thread(
            &CameraMt9v034::processMeasurements, new_camera);
        threads->add_thread(t1);

        sensor_map->insert(
            std::pair<SensorId::SensorId, Sensor::Ptr>(sensor_id,
                                                       new_camera));
        break;
      }

      case SensorType::SensorType::CAMERA_TAU640: {
        CameraTau640::Ptr new_camera = boost::make_shared<CameraTau640>(
            sensor_id, config_connection_weak);
        VISENSOR_DEBUG("Tau320 created with sensor_id: %d\n", sensor_id);

        boost::thread *t1 = new boost::thread(
            &CameraTau640::processMeasurements, new_camera);
        threads->add_thread(t1);

        sensor_map->insert(
            std::pair<SensorId::SensorId, Sensor::Ptr>(sensor_id,
                                                       new_camera));
        break;
      }

      case SensorType::SensorType::IMU_ADIS16448: {
        ImuAdis16448::Ptr new_imu = boost::make_shared<ImuAdis16448>(
            sensor_id, config_connection_weak);
        VISENSOR_DEBUG("ADIS16448 created with sensor_id: %d\n", sensor_id);

        boost::thread *t1 = new boost::thread(
            &ImuAdis16448::processMeasurements, new_imu);
        threads->add_thread(t1);

        sensor_map->insert(
            std::pair<SensorId::SensorId, Sensor::Ptr>(sensor_id, new_imu));
        break;
      }

      case SensorType::SensorType::IMU_ADIS16488: {
		  ImuAdis16488::Ptr new_imu = boost::make_shared<ImuAdis16488>(
			  sensor_id, config_connection_weak);
		  VISENSOR_DEBUG("ADIS16488 created with sensor_id: %d\n", sensor_id);

		  boost::thread *t1 = new boost::thread(
			  &ImuAdis16488::processMeasurements, new_imu);
		  threads->add_thread(t1);

		  sensor_map->insert(
			  std::pair<SensorId::SensorId, Sensor::Ptr>(sensor_id, new_imu));
		  break;
		}

      case SensorType::SensorType::CORNER_MT9V034: {
        CornerMt9v034::Ptr new_CornerDetector =
            boost::make_shared<CornerMt9v034>(sensor_id,
                                              config_connection_weak);
        VISENSOR_DEBUG("CornerDetector created with sensor_id: %d\n", sensor_id);

        boost::thread *t1 = new boost::thread(
            &CornerMt9v034::processMeasurements, new_CornerDetector);
        threads->add_thread(t1);

        sensor_map->insert(
            std::pair<SensorId::SensorId, Sensor::Ptr>(sensor_id,
                                                       new_CornerDetector));
        break;
      }
      case SensorType::SensorType::DENSE_MATCHER: {
        DenseMatcher::Ptr new_dense_matcher = boost::make_shared<DenseMatcher>(
            sensor_id, config_connection_weak);
        VISENSOR_DEBUG("DenseMatcher created with sensor_id: %d\n", sensor_id);

        boost::thread *t1 = new boost::thread(
            &DenseMatcher::processMeasurements, new_dense_matcher);
        threads->add_thread(t1);

        sensor_map->insert(
            std::pair<SensorId::SensorId, DenseMatcher::Ptr>(
                sensor_id, new_dense_matcher));
        break;
      }
      case SensorType::SensorType::EXTERNAL_TRIGGER: {
        ExternalTrigger::Ptr new_external_trigger = boost::make_shared<
            ExternalTrigger>(sensor_id, config_connection_weak);
        VISENSOR_DEBUG("EXTERNAL_TRIGGER created with sensor_id: %d\n", sensor_id);

        boost::thread *t1 = new boost::thread(
            &ExternalTrigger::processMeasurements, new_external_trigger);
        threads->add_thread(t1);

        sensor_map->insert(
            std::pair<SensorId::SensorId, Sensor::Ptr>(sensor_id, new_external_trigger));
        break;
      }
      case SensorType::SensorType::MPU_9150: {
        ImuMpu9150::Ptr new_imu = boost::make_shared<ImuMpu9150>(
            sensor_id, config_connection_weak);
        VISENSOR_DEBUG("Mpu9150 created with sensor_id: %d\n", sensor_id);

        boost::thread *t1 = new boost::thread(
            &ImuMpu9150::processMeasurements, new_imu);
        threads->add_thread(t1);

        sensor_map->insert(
            std::pair<SensorId::SensorId, Sensor::Ptr>(sensor_id, new_imu));
        break;
      }
        //    if(sensor_type==SensorType::SensorType::BRISK_MT9V034)
        //    {
        //      int CornerDetector_id=CornerDetector_.size();
        //      BriskMt9v034* new_CornerDetector = new BriskMt9v034(sensor_id,CornerDetector_id,this);
        //      CornerDetector_.push_back(new_CornerDetector);
        //
        //#ifdef USE_CornerDetector_HACK
        //      frame_CornerDetector_synchronizer_.push_back(new FrameCornerDetectorSynchronizer());
        //#endif /*  USE_CornerDetector_HACK */
        //
        //      VISENSOR_DEBUG("CornerDetector detector MT9V034 created\n");
        //      boost::thread *t1 = new boost::thread(&BriskMt9v034::processMeasurements, new_CornerDetector);
        //      sensor_threads_.add_thread(t1);
        //      return new_CornerDetector;
        //    }
        //
        //    if(sensor_type==SensorType::SensorType::CAMERA_TAU320)
        //    {
        //      int camera_id=camera_.size();
        //      CameraTau320* new_camera = new CameraTau320(sensor_id,camera_id,this);
        //      camera_.push_back(new_camera);
        //      VISENSOR_DEBUG("camera Tau320 created\n");
        //
        //
        //      if(user_frame_callback_)
        //        new_camera->setUserCallback(user_frame_callback_);
        //
        //      boost::thread *t1 = new boost::thread(&CameraTau320::processMeasurements, new_camera);
        //      sensor_threads_.add_thread(t1);
        //
        //      return new_camera;
        //    }
        //
        //    if(sensor_type==SensorType::SensorType::IMU_ADIS16448)
        //    {
        //      int imu_id=imu_.size();
        //      ImuAdis16448* new_imu = new ImuAdis16448(sensor_id,imu_id,this);
        //      imu_.push_back(new_imu);
        //      VISENSOR_DEBUG("Imu ADIS16448 created\n");
        //      if(user_imu_callback_)
        //        new_imu->setUserCallback(user_imu_callback_);
        //
        //      boost::thread *t1 = new boost::thread(&ImuAdis16448::processMeasurements, new_imu);
        //      sensor_threads_.add_thread(t1);
        //
        //      return new_imu;
        //    }
        //
        //    if(sensor_type==SensorType::SensorType::IMU_ADIS16488)
        //    {
        //      int imu_id=imu_.size();
        //      ImuAdis16488* new_imu = new ImuAdis16488(sensor_id,imu_id,this);
        //      imu_.push_back(new_imu);
        //      VISENSOR_DEBUG("Imu ADIS16488 created\n");
        //      if(user_imu_callback_)
        //        new_imu->setUserCallback(user_imu_callback_);
        //
        //      boost::thread *t1 = new boost::thread(&ImuAdis16488::processMeasurements, new_imu);
        //      sensor_threads_.add_thread(t1);
        //
        //      return new_imu;
        //    }


      case SensorType::SensorType::LIGHT_CONTROL: {
        LedFlasher::Ptr led = boost::make_shared<LedFlasher>(
            sensor_id, config_connection_weak);
        VISENSOR_DEBUG("LedFlasher created with sensor_id: %d\n", sensor_id);

        sensor_map->insert(
            std::pair<SensorId::SensorId, Sensor::Ptr>(sensor_id, led));
        break;
      }

      default:
        VISENSOR_DEBUG("unknown sensor type!!! type: %d \n", sensor_type);
    }

  }
}

}  //namespace visensor
