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

#include "visensor/visensor_constants.hpp"
#include "networking/config_connection.hpp"
#include "sensors/camera_tau640.hpp"

namespace visensor
{
CameraTau640::CameraTau640(SensorId::SensorId sensor_id, IpConnection::WeakPtr config_connection):
				Camera(sensor_id,ViCameraConfig(
						CameraTau640Defaults::WIDTH, CameraTau640Defaults::HEIGHT, CameraTau640Defaults::RATE),
						SensorSettings(sensor_id, SensorType::SensorType::CAMERA_TAU640,calculateBufferSize(),
								CameraTau640Defaults::NUM_OF_MSGS_IN_PACKAGE,
								CameraTau640Defaults::USE_CONST_PACKAGE_SIZE,
								CameraTau640Defaults::CALIBRATION_SIZE),
								config_connection),
				_config(sensor_id)
{

	_allowedTimediff=((1000000000*1.5)/Camera::config_.frame_rate);

	_prevTimestamp=0;
}


uint32_t CameraTau640::calculateBufferSize(){
  // TODO(gohlp): fix missing 2 bytes in fpga
return	(CameraTau640Defaults::WIDTH * CameraTau640Defaults::HEIGHT * 2-2);
}

void CameraTau640::init()
{
}

//// set calibration
//bool CameraTau640::setCalibration(CalibBase::Ptr calibration)
//{
//	// TODO set calibration
//	return true;
//}

void CameraTau640::allocateBuffer()
{
//	_frame_size=Camera::_config.width * Camera::_config.height;
//	Sensor::_buffer_size=ceil( ( (2+Camera::_config.width) * Camera::_config.height+2+2/* magic 2 */) * 10.0/8.0)+8; //TODO fix magic 2!!!
//	Sensor::_buffer = new unsigned char[Sensor::_buffer_size];
//	Sensor::_bytes_in_buffer=0;
}

CameraTau640::~CameraTau640()
{

}

//the threaded function that works on the queue of finished measurements
void CameraTau640::processMeasurements(){
	while(1){
		boost::this_thread::interruption_point();
		//get the newest measurement
		Measurement::Ptr meas = Sensor::measurement_queue_.pop();

//		VISENSOR_DEBUG("in camera thread %d\n", meas.buffersize);

		//check for missed frames
		// TODO(schneith): use a timer to check for missed frames (currently we only detected sequences of MISSED-RECEIVED-MISSED, not MISSED-MISSED,...)
		if( checkTimestamp(meas->timestamp) )
		{
		  //publish an empty missed frame
		  ViFrame::Ptr missed_frame_ptr = boost::make_shared<ViFrame>();
		  missed_frame_ptr->camera_id = Camera::camera_id_;
		  publishFrameData(missed_frame_ptr, ViErrorCodes::MEASUREMENT_DROPPED);
		}

		ViFrame::Ptr frame_ptr = boost::make_shared<ViFrame>();

		// Add camera information to the frame
		frame_ptr->width=Camera::config_.width;
		frame_ptr->height=Camera::config_.height;
		frame_ptr->camera_id=Camera::camera_id_;
		frame_ptr->timestamp=meas->timestamp;
    frame_ptr->timestamp_synchronized = meas->timestamp_synchronized;
		frame_ptr->timestamp_fpga_counter = meas->timestamp_fpga_counter;
		frame_ptr->timestamp_host = meas->timestamp_host;

		frame_ptr->useCorners=false;

		frame_ptr->image_type=CameraTau640Defaults::TYPE;

		frame_ptr->setImageRawPtr(meas->data, meas->bytes_in_buffer);

		publishFrameData(frame_ptr, ViErrorCodes::NO_ERROR);
	}
}

ViConfigMsg CameraTau640::getConfigParam(std::string cmd, uint32_t value)
{

	return _config.getConfigParam(cmd,value);
}


//void CameraTau640::frameComplete()
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


void CameraTau640::bitReshuffle(uint8_t *in /*10 bit values*/, uint8_t *out /*8 bit values*/, int size_in /*number of 10 bit values*/, int offset)
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

void CameraTau640::imageReshuffle(uint8_t *data_in /*10 bit raw data*/, uint8_t *image_out, int image_height, int image_width)
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
