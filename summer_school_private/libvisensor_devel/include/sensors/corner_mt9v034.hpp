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

#ifndef CORNER_MT9V034_HPP_
#define CORNER_MT9V034_HPP_

#include <config/config.hpp>
#include <boost/smart_ptr.hpp>

#include "networking/connection.hpp"
#include "networking/param_server.hpp"
#include "sensors/corner_detector.hpp"

namespace visensor
{

namespace CornerMt9v034Defaults
{
const int
	I2C_ADRESS=0x48,
	WIDTH=752,
	HEIGHT=480,
	MAX_FEATURES=30000,
	RATE=CAMERA_FREQUENCY,
	DEPTH=32,
	NUM_OF_MSGS_IN_PACKAGE=1,
	CALIBRATION_SIZE=0;

const uint8_t COM_TYPE = ViComType::I2C_16;
const bool USE_CONST_PACKAGE_SIZE=false;
}

class CornerMt9v034Config
{
public:
  CornerMt9v034Config(const SensorId::SensorId sensorId)
      : sensor_id_(sensorId) {
  }

	ViConfigMsg getConfigParam(const std::string paramName, uint16_t value)
	{
		ViConfigMsg  msg;
		msg.valChanged=param_server_.getConfigParam(paramName,value,msg.reg,msg.val);
		msg.sensorId=sensor_id_;
		msg.devAdress=CornerMt9v034Defaults::I2C_ADRESS;
		msg.comType=CornerMt9v034Defaults::COM_TYPE;
		return msg;
	}

	ViConfigMsg getConfigParam(const std::string paramName)
	{
		ViConfigMsg  msg;
		msg.valChanged=param_server_.getConfigParam(paramName,msg.reg,msg.val);
		msg.sensorId=sensor_id_;
		msg.devAdress=CornerMt9v034Defaults::I2C_ADRESS;
		msg.comType=CornerMt9v034Defaults::COM_TYPE;
		return msg;
	}

private:
  param_server::ParamServer param_server_;
  const SensorId::SensorId sensor_id_;
};

class CornerMt9v034 : public CornerDetector{
public:
  typedef boost::shared_ptr<CornerMt9v034> Ptr;

	CornerMt9v034(SensorId::SensorId sensor_id, IpConnection::WeakPtr config_connection);
	virtual ~CornerMt9v034();

	virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value);
	//the threaded function that works on the queue of finished measurements
	void processMeasurements();
	bool getCornerElem(uint8_t* buf, ViCornerElem& corner);
	virtual void init();
	static uint32_t calculateBufferSize();

private:
  void bitReshuffle(uint8_t *in /*10 bit values*/, uint8_t *out /*8 bit values*/, int size_in /*number of 10 bit values*/, int offset);
  void imageReshuffle(uint8_t *data_in /*10 bit raw data*/, uint8_t *image_out, int image_height, int image_width);

	CornerMt9v034Config config_;
};


} //namespace visensor


#endif /* CORNER_MT9V034_HPP_ */
