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
/*
 * camera_tau320.cpp
 *
 *  Created on: Apr 18, 2012
 *      Author: burrimi
 */

#include "camera_tau320.hpp"

namespace visensor
{
CameraTau320::CameraTau320(SensorId::SensorId sensor_id, int stream_id, IpConnection::WeakPtr config_connection):
				Camera(camera_id, ViCameraConfig(
						CameraTau320Defaults::WIDTH, CameraTau320Defaults::HEIGHT, CameraTau320Defaults::RATE),
						SensorSettings(sensor_id,SensorType::SensorType::CAMERA_TAU320,calculateBufferSize(),
								CameraTau320Defaults::NUM_OF_MSGS_IN_PACKAGE,
								CameraTau320Defaults::USE_CONST_PACKAGE_SIZE,
								CameraTau320Defaults::CALIBRATION_SIZE),
								config_connection),
				config_(sensor_id)
{

	_allowedTimediff=((1000000000*1.5)/Camera::config_.frame_rate);

	_prevTimestamp=0;
}


uint32_t CameraTau320::calculateBufferSize(){
	//only use const variables here
	//return	ceil( ( (2+CameraTau320Defaults::WIDTH) * CameraTau320Defaults::HEIGHT+2+2/* magic 2 */) * 10.0/8.0);
	return	(CameraTau320Defaults::WIDTH * CameraTau320Defaults::HEIGHT * 2);
}

bool CameraTau320::init()
{
	return true;
}
// set calibration
bool CameraTau320::setCalibration(CalibBase::Ptr calibration)
{
	// TODO set calibration
	return true;
}

void CameraTau320::allocateBuffer()
{
//	_frame_size=Camera::_config.width * Camera::_config.height;
//	Sensor::_buffer_size=ceil( ( (2+Camera::_config.width) * Camera::_config.height+2+2/* magic 2 */) * 10.0/8.0)+8; //TODO fix magic 2!!!
//	Sensor::_buffer = new unsigned char[Sensor::_buffer_size];
//	Sensor::_bytes_in_buffer=0;
}

CameraTau320::~CameraTau320()
{

}

//the threaded function that works on the queue of finished measurements
void CameraTau320::processMeasurements(){
	while(1){


		boost::this_thread::interruption_point();
		//get the newest measurement
		Measurement meas = Sensor::measurement_queue_.pop();

//		VISENSOR_DEBUG("in camera thread %d\n", meas.buffersize);

		checkTimestamp(meas.timestamp);

//		//TODO adapt
//		// create new shared pointer for the frame
		ViFrame::Ptr frame_ptr(new ViFrame());

		// Add camera information to the frame
		frame_ptr->width=Camera::config_.width;
		frame_ptr->height=Camera::config_.height;
		frame_ptr->camera_id=Camera::camera_id_;
		frame_ptr->timestamp=meas.timestamp;
		frame_ptr->useCorners=false;

		frame_ptr->image_type=CameraTau320Defaults::TYPE;

		//TODO ask simon if this is correct!!!
		frame_ptr->setImageRawPtr(meas.data,meas.bytes_in_buffer);


		//
//		// Reshuffle the 10 bit receive stream to a 8 bit image
//		imageReshuffle(meas.data, frame_ptr->getImageRawPtr(), Camera::_config.height, Camera::_config.width);
//		delete[] meas.data;


		// run the frame callback
		// only run callback if callback is initialized
//		if(Camera::_user_callback)
//		{
//			Camera::_user_callback(frame_ptr);
//		}

//		VISENSOR_DEBUG("tau received \n");

		publishFrameData(frame_ptr);
	}
}

ViConfigMsg CameraTau320::getConfigParam(std::string cmd, uint16_t value)
{

	return config_.getConfigParam(cmd,value);
}


//void CameraTau320::frameComplete()
//{
//	// create new shared pointer for the frame
//	ViFrame::Ptr frame_ptr(new ViFrame(_frame_size));
//
//	// Add camera information to the frame
//	frame_ptr->width=Camera::_config.width;
//	frame_ptr->height=Camera::_config.height;
//	frame_ptr->camera_id=_camera_id;
//	frame_ptr->timestamp=Sensor::_timestamp;
//
//	// Reshuffle the 10 bit receive stream to a 8 bit image
//	imageReshuffle(&Sensor::_buffer[8], frame_ptr->getImageRawPtr(), Camera::_config.height, Camera::_config.width);
//
//	// reset camera buffer
//	Sensor::_bytes_in_buffer=0;
//
//	checkTimestamp();
//
//	// run the frame callback
//	// only run callback if callback is initialized
//	if(Camera::_user_callback)
//	{
//		Camera::_user_callback(frame_ptr);
//	}
//}

void CameraTau320::checkTimestamp(uint64_t timestamp)
{
//	//VISENSOR_DEBUG("camera %d frame missed, timediff: %ld ns\n",_camera_id,Sensor::_timestamp);
//
	if(_prevTimestamp==0)
	{
		_prevTimestamp=timestamp;
		return;
	}

	if(timestamp-_prevTimestamp>_allowedTimediff)
		VISENSOR_DEBUG("camera %d frame missed, timediff: %ld ns\n",Camera::camera_id_,timestamp-_prevTimestamp);
	_prevTimestamp=timestamp;
}

void CameraTau320::bitReshuffle(uint8_t *in /*10 bit values*/, uint8_t *out /*8 bit values*/, int size_in /*number of 10 bit values*/, int offset)
{
	assert(!(size_in % 4));

	switch (offset) {
	case 0:
		for(int i=0;i<size_in/4; i++)
		{
			out[i*4] = in[i*5];
			out[i*4+1] = ((in[i*5+1] & 0x3F) << 2) + ((in[i*5+2] & 0xC0) >> 6);
			out[i*4+2] = ((in[i*5+2] & 0x0F) << 4) + ((in[i*5+3] & 0xF0) >> 4);
			out[i*4+3] = ((in[i*5+3] & 0x03) << 6) + ((in[i*5+4] & 0xFC) >> 2);
		}
		break;
	case 4:
		for(int i=0;i<size_in/4; i++)
		{
			out[i*4] = ((in[i*5] & 0x0F) << 4) + ((in[i*5+1] & 0xF0) >> 4);
			out[i*4+1] = ((in[i*5+1] & 0x03) << 6) + ((in[i*5+2] & 0xFC) >> 2);
			out[i*4+2] = in[i*5+3];
			out[i*4+3] = ((in[i*5+4] & 0x3F) << 2) + ((in[i*5+5] & 0xC0) >> 6);
		}
		break;
	case 2:
		for(int i=0;i<size_in/4; i++)
		{
			out[i*4] = ((in[i*5] & 0x03) << 6) + ((in[i*5+1] & 0xFC) >> 2);
			out[i*4+1] = in[i*5+2];
			out[i*4+2] =((in[i*5+3] & 0x3F) << 2) + ((in[i*5+4] & 0xC0) >> 6);
			out[i*4+3] = ((in[i*5+4] & 0x0F) << 4) + ((in[i*5+5] & 0xF0) >> 4);
		}
		break;
	case 6:
		for(int i=0;i<size_in/4; i++)
		{
			out[i*4] = ((in[i*5] & 0x3F) << 2) + ((in[i*5+1] & 0xC0) >> 6);
			out[i*4+1] = ((in[i*5+1] & 0x0F) << 4) + ((in[i*5+2] & 0xF0) >> 4);
			out[i*4+2] = ((in[i*5+2] & 0x03) << 6) + ((in[i*5+3] & 0xFC) >> 2);
			out[i*4+3] = in[i*5+4];
		}
		break;
	default:
		break;
	}

}

void CameraTau320::imageReshuffle(uint8_t *data_in /*10 bit raw data*/, uint8_t *image_out, int image_height, int image_width)
{
	int i;
	int offset=4; 	// bit offset at the beginning because of 10 bit frame and line sync
	int start=2;	// start byte after frame and line sync
	for(i=0;i<image_height;i++)
	{
		bitReshuffle(&data_in[start], &image_out[i*image_width],image_width, offset);
		offset+=4;
		if (offset>6) {
			offset&=0x07; // = mod(offset,8)
			start+=image_width*5/4+3; // = ceil((image_width+2)*5/4)
		} else
			start+=image_width*5/4+2; // = floor((image_width+2)*5/4)
	}
}




} //namespace visensor
