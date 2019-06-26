/*******************************************************************************
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
 *******************************************************************************/
#include <imu_mpu6000.hpp>
#include "visensor_impl.hpp" // needed by forward declaration

namespace visensor
{


ImuMpu6000::ImuMpu6000(int sensor_id, int imu_id, ViSensorDriver::Impl* ViSensorDriver):
		Imu(imu_id,ViSensorDriver,
				SensorSettings(sensor_id,SensorType::SensorType::IMU_MPU6000,calculateBufferSize(),
						ImuMpu6000Defaults::NUM_OF_MSGS_IN_PACKAGE,
						ImuMpu6000Defaults::USE_CONST_PACKAGE_SIZE,
						ImuMpu6000Defaults::CALIBRATION_SIZE)),
		config_(sensor_id)
{
}

uint32_t ImuMpu6000::calculateBufferSize(){
	//only use const variables here
	//return	(ImuMpu6000Defaults::NUM_OF_MSGS_IN_PACKAGE*ImuMpu6000Defaults::MSG_SIZE+8);
	return	(ImuMpu6000Defaults::MSG_SIZE);
}

// set calibration
bool ImuMpu6000::setCalibration(CalibBase::Ptr calibration)
{
	boost::shared_ptr<CalibImuMpu6000> tempCalib=boost::dynamic_pointer_cast<CalibImuMpu6000>(calibration);
	if(!tempCalib)
	{
		VISENSOR_DEBUG("dynamic_cast from calibBase to CalibImuMpu6000 failed\n");
		return false;
	}
	calibration_=*tempCalib;
	return true;
}


bool ImuMpu6000::init()
{
	bool status=true;
	status&=writeRequest(config_.getConfigParam("enable"));
	status&=writeRequest(config_.getConfigParam("digital_low_pass_filter_config"));
	return status;
}

void ImuMpu6000::processMeasurements()
{
	while (1) {

		boost::this_thread::interruption_point();
		//get the newest measurement
		Measurement meas = Sensor::measurement_queue_.pop();

		//VISENSOR_DEBUG("in imu_adis thread %d\n", meas.buffersize);

		//checkTimestamp(meas.timestamp);

		// TODO adapt
		// create new shared pointer for the imu message
		ViImuMsg::Ptr imu_msg_ptr(new ViImuMsg);
		ViImuMsg::Ptr imu_msg_ptr2(new ViImuMsg);

		// Add imu information to the msg
		imu_msg_ptr->imu_id = Imu::imu_id_;
		imu_msg_ptr->timestamp=meas.timestamp;
		imu_msg_ptr2->imu_id = Imu::imu_id_ + 1;
		imu_msg_ptr2->timestamp=meas.timestamp;
		// get imu data
		getGyro(meas.data,&imu_msg_ptr->gyro[0]);
		getAcc(meas.data,&imu_msg_ptr->acc[0]);
		getGyro2(meas.data,&imu_msg_ptr2->gyro[0]);
		getAcc2(meas.data,&imu_msg_ptr2->acc[0]);

//		// run the imu callback
//		// only run callback if callback is initialized
//		if (Imu::_user_callback) {
//			Imu::_user_callback(imu_msg_ptr);
//			Imu::_user_callback(imu_msg_ptr2);
//		}

		Sensor::publishImu(imu_msg_ptr);
		Sensor::publishImu(imu_msg_ptr2);

		delete[] meas.data;

	}
}

void ImuMpu6000::allocateBuffer()
{
//	int buffer_size=(ImuMpu6000Defaults::NUM_OF_MSGS_IN_PACKAGE*_msg_size+8);
//	Sensor::_buffer = new unsigned char[buffer_size];
//	Sensor::_buffer_size=buffer_size;
//	Sensor::_bytes_in_buffer=0;
//	Imu::_num_of_completed_msgs=0;
}

//int ImuMpu6000::processData(uint8_t const * data,int nBytesReceived)
//{
//
//	// get data from FPGA
//	memcpy(&Sensor::_buffer[Sensor::_bytes_in_buffer], data, nBytesReceived);
//
//	Sensor::_bytes_in_buffer+=nBytesReceived;
//
//
//	if(getNumOfNewMsgs()>0)
//		newImuMsgs();
//
//	if(Sensor::_bytes_in_buffer==Sensor::_buffer_size)
//		resetBuffer();
//
//	return 0;
//}

ViConfigMsg ImuMpu6000::getConfigParam(std::string cmd, uint16_t value)
{
	return config_.getConfigParam(cmd,value);
}

void ImuMpu6000::newImuMsgs()
{
//	int i;
//
//	int temp=getNumOfNewMsgs();
//
//	for(i=0;i<temp;i++)
//	{
//		// create new shared pointer for the imu message
//		ViImuMsg::Ptr imu_msg_ptr(new ViImuMsg);
//		ViImuMsg::Ptr imu_msg_ptr2(new ViImuMsg);
//
//
//		// Add imu information to the msg
//		imu_msg_ptr->imu_id=Imu::_imu_id;
////		imu_msg_ptr->timestamp=getTimestamp();
//		imu_msg_ptr2->imu_id=Imu::_imu_id+1;
////		imu_msg_ptr2->timestamp=getTimestamp();
//		// get imu data
//		getGyro(&imu_msg_ptr->gyro[0]);
//		getAcc(&imu_msg_ptr->acc[0]);
//		getGyro2(&imu_msg_ptr2->gyro[0]);
//		getAcc2(&imu_msg_ptr2->acc[0]);
//
//		// increment received imu message counter
//		incrementMsgCounter();
//
//		// run the imu callback
//		// only run callback if callback is initialized
//		if(Imu::_user_callback)
//		{
//			Imu::_user_callback(imu_msg_ptr);
//			Imu::_user_callback(imu_msg_ptr2);
//		}
//	}
}

//int ImuMpu6000::getNumOfNewMsgs()
//{
////	int unused_bytes=Sensor::_bytes_in_buffer-(Imu::_num_of_completed_msgs*_msg_size)-8;
////	return floor(unused_bytes/(_msg_size));
//}
//
//
//void ImuMpu6000::resetBuffer()
//{
////	Sensor::_bytes_in_buffer=0;
////	Imu::_num_of_completed_msgs=0;
//}


void ImuMpu6000::getAcc(uint8_t* buffer, double * acc)
{
	int i,current_pos;
	int16_t temp;

	for(i=0;i<3;i++)
	{
		current_pos=2*i;
		temp=(int)((buffer[current_pos]<<8) | (buffer[current_pos+1]<<0));
		acc[i]=(double)temp;
		acc[i] = acc[i]/16384*STANDARD_GRAVITY;
	}
}

void ImuMpu6000::getGyro(uint8_t* buffer, double * gyro)
{
	int i,current_pos;
	int16_t temp;
	for(i=0;i<3;i++)
	{
		current_pos=2*3+2*i;
		temp=(int)((buffer[current_pos]<<8) | (buffer[current_pos+1]<<0));
		gyro[i]=(double)temp;
		gyro[i] = gyro[i]/131*M_PI/180;
	}
}

void ImuMpu6000::getAcc2(uint8_t* buffer, double * acc)
{
	int i,current_pos;
	int16_t temp;

	for(i=0;i<3;i++)
	{
		current_pos=4*3+2*i;
		temp=(int)((buffer[current_pos]<<8) | (buffer[current_pos+1]<<0));
		acc[i]=(double)temp;
		acc[i] = acc[i]/16384*STANDARD_GRAVITY;
	}
}

void ImuMpu6000::getGyro2(uint8_t* buffer, double * gyro)
{
	int i,current_pos;
	int16_t temp;
	for(i=0;i<3;i++)
	{
		current_pos=6*3+2*i;
		temp=(int)((buffer[current_pos]<<8) | (buffer[current_pos+1]<<0));
		gyro[i]=(double)temp;
		gyro[i] = gyro[i]/131*M_PI/180;
	}
}

void ImuMpu6000::incrementMsgCounter()
{
//	Imu::_num_of_completed_msgs++;
}


} //namespace visensor
