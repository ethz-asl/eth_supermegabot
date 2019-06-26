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

#include <sensors/imu_adis16448.hpp>

namespace visensor
{


ImuAdis16448::ImuAdis16448(SensorId::SensorId sensor_id, IpConnection::WeakPtr config_connection):
		Imu(sensor_id,
				SensorSettings(sensor_id, SensorType::SensorType::IMU_ADIS16448,calculateBufferSize(),
						ImuAdis16448Defaults::NUM_OF_MSGS_IN_PACKAGE,
						ImuAdis16448Defaults::USE_CONST_PACKAGE_SIZE,
						ImuAdis16448Defaults::CALIBRATION_SIZE),
						config_connection)
{
}

uint32_t ImuAdis16448::calculateBufferSize(){
	//only use const variables here
	//return	(ImuAdis16448Defaults::NUM_OF_MSGS_IN_PACKAGE*ImuAdis16448Defaults::MSG_SIZE);
	return	(ImuAdis16448Defaults::MSG_SIZE);
}

void ImuAdis16448::init()
{
}

void ImuAdis16448::processMeasurements() {
	while (1) {

		boost::this_thread::interruption_point();
		//get the newest measurement
		Measurement::Ptr meas = Sensor::measurement_queue_.pop();

		//VISENSOR_DEBUG("in imu_adis thread %d\n", meas.buffersize);

    //check for missed frames
    // TODO(schneith): use a timer to check for missed frames (currently we only detected sequences of MISSED-RECEIVED-MISSED, not MISSED-MISSED,...)
    if( checkTimestamp(meas->timestamp) )
    {
      //publish an empty missed frame
      ViImuMsg::Ptr missed_imu_ptr = boost::make_shared<ViImuMsg>();
      missed_imu_ptr->imu_id = Imu::imu_id_;
      publishImuData(missed_imu_ptr, ViErrorCodes::MEASUREMENT_DROPPED);
    }

	//		//TODO adapt
	// create new shared pointer for the imu message
	ViImuMsg::Ptr imu_msg_ptr = boost::make_shared<ViImuMsg>();

	// Add imu information to the msg
	imu_msg_ptr->imu_id = Imu::imu_id_;
	imu_msg_ptr->timestamp=meas->timestamp;
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

//void ImuAdis16448::allocateBuffer()
//{
//	int buffer_size=(ImuAdis16448Defaults::NUM_OF_MSGS_IN_PACKAGE*_msg_size+8);
//	Sensor::_buffer = new unsigned char[buffer_size];
//	Sensor::_buffer_size=buffer_size;
//	Sensor::_bytes_in_buffer=0;
//	Imu::_num_of_completed_msgs=0;
//}
//
//int ImuAdis16448::processData(uint8_t const * data,int nBytesReceived)
//{
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

ViConfigMsg ImuAdis16448::getConfigParam(std::string cmd, uint32_t value)
{
	// TODO: implement this
	ViConfigMsg config;
	config.sensorId=SensorId::SensorId::NOT_SPECIFIED;
	config.devAdress=0;
	config.reg=0;
	config.val=0;
	config.comType=0;
	config.valChanged=false;
	return config;
}

void ImuAdis16448::newImuMsgs()
{
//	int i;
//
//	int temp=getNumOfNewMsgs();
//
//	for(i=0;i<temp;i++)
//	{
//		// create new shared pointer for the imu message
//		ViImuPtr_T imu_msg_ptr(new ViImuMsg);
//
//		// Add imu information to the msg
//		imu_msg_ptr->imu_id=Imu::_imu_id;
////		imu_msg_ptr->timestamp=getTimestamp();
//
//		// get imu data
//		getGyro(&imu_msg_ptr->gyro[0]);
//		getAcc(&imu_msg_ptr->acc[0]);
//		getMag(&imu_msg_ptr->mag[0]);
//		getBaro(&imu_msg_ptr->baro);
//
//		// increment received imu message counter
//		incrementMsgCounter();
//
//		// run the imu callback
//		// only run callback if callback is initialized
//		if(Imu::_user_callback)
//			Imu::_user_callback(imu_msg_ptr);
//	}
}

//int ImuAdis16448::getNumOfNewMsgs()
//{
//	int unused_bytes=Sensor::_bytes_in_buffer-(Imu::_num_of_completed_msgs*_msg_size)-8;
//	return floor(unused_bytes/(_msg_size));
//}


//void ImuAdis16448::resetBuffer()
//{
//	Sensor::_bytes_in_buffer=0;
//	Imu::_num_of_completed_msgs=0;
//}



void ImuAdis16448::getGyro(uint8_t * buffer, double * gyro)
{
	int16_t temp;
	for(int i=0;i<3;i++)
	{
		temp=(int16_t)((buffer[2*i]<<8) | (buffer[2*i+1]<<0));
		gyro[i]=(double)temp;
//		gyro[i] = gyro[i] * 450.0/22500.0/65536.0/180.0*M_PI;		// JN 04052012: Changed to SI units.
    gyro[i] = gyro[i] / 25 / 180.0*M_PI;   // JN 04052012: Changed to SI units. //rad/s
	}
}

void ImuAdis16448::getAcc(uint8_t * buffer, double * acc)
{
	int i,current_pos;
	int16_t temp;

	for(i=0;i<3;i++)
	{
		current_pos=3*2+2*i;
		temp=(int16_t)((buffer[current_pos]<<8) | (buffer[current_pos+1]<<0));

		if((temp /1200.0*STANDARD_GRAVITY)<-200.0)
			VISENSOR_DEBUG("wrong imu acc value id: %i value raw: %i value: %f \n", i, temp,(temp /1200*STANDARD_GRAVITY));

		acc[i]=(double)temp;
		acc[i] = acc[i] / 1200.0 * STANDARD_GRAVITY;	//m^2/s // JN 04052012: Changed to SI units.
	}
}

void ImuAdis16448::getMag(uint8_t * buffer, double * mag)
{
  for (int i = 0; i < 3; i++) {
    int current_pos = 6 * 2 + 2 * i;
    int temp = (int16_t) ((buffer[current_pos + 0] << 8)
        | (buffer[current_pos + 1] << 0));
    mag[i] = static_cast<double>(temp) * 0.001/7.0; //gauss
  }
}

void ImuAdis16448::getBaro(uint8_t * buffer, double * baro)
{
	int current_pos = 6 * 2 + 3 * 2;
  int temp = (uint16_t) ((buffer[current_pos + 0] << 8)
      | (buffer[current_pos + 1] << 0));
  *baro = static_cast<double>(temp) * 0.00002*1000.0; //hPa
}

void ImuAdis16448::getTemp(uint8_t * buffer, double * temp)
{
  int current_pos = 6 * 2 + 4 * 2;
  int temp_int = (int) ((buffer[current_pos + 0] << 8)
      | (buffer[current_pos + 1] << 0));
  *temp = static_cast<double>(temp_int)*0.07386 + 31.0;
}


} //namespace visensor
