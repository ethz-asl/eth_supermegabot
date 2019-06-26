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
 * calib_mpu6000.cpp
 *
 *  Created on: Feb 4, 2013
 *      Author: burrimi
 */


#include "sensor_calibration.hpp"


namespace visensor
{
	void CalibImuMpu6000::setScaleA(double a0,double a1, double a2)
	{
		_scaleA.at(0)=a0;
		_scaleA.at(1)=a1;
		_scaleA.at(2)=a2;
	}

	void CalibImuMpu6000::setScaleG(double a0,double a1, double a2)
	{
		_scaleG.at(0)=a0;
		_scaleG.at(1)=a1;
		_scaleG.at(2)=a2;
	}

	void CalibImuMpu6000::setMisalignementA(double a0,double a1, double a2)
	{
		_misalignementA.at(0)=a0;
		_misalignementA.at(1)=a1;
		_misalignementA.at(2)=a2;
	}

	void CalibImuMpu6000::setMisalignementG(double a0,double a1, double a2)
	{
		_misalignementG.at(0)=a0;
		_misalignementG.at(1)=a1;
		_misalignementG.at(2)=a2;
	}

	std::vector<std::vector<double> > CalibImuMpu6000::getScaleMisalignementInverseG()
	{
		std::vector<double> temp(3,0.0);
		std::vector<std::vector<double> > scaleMisalignementInverseG(3,temp);

		//scaleMisalignementInverse = inv(Scale*Misalignement)
		scaleMisalignementInverseG.at(0).at(0)=1/_scaleG[0];
		scaleMisalignementInverseG.at(1).at(0)=-_misalignementG[0]/_scaleG[0];
		scaleMisalignementInverseG.at(1).at(1)=1/_scaleG[1];
		scaleMisalignementInverseG.at(2).at(0)=(_misalignementG[0]*_misalignementG[2]-_misalignementG[1])/_scaleG[0];
		scaleMisalignementInverseG.at(2).at(1)=-_misalignementG[2]/_scaleG[1];
		scaleMisalignementInverseG.at(2).at(2)=1/_scaleG[2];

		return scaleMisalignementInverseG;
	}

	std::vector<std::vector<double> > CalibImuMpu6000::getScaleMisalignementInverseA()
	{

		std::vector<double> temp(3,0.0);
		std::vector<std::vector<double> > scaleMisalignementInverseA(3,temp);

		//scaleMisalignementInverse = inv(Scale*Misalignement)
		scaleMisalignementInverseA.at(0).at(0)=1/_scaleA[0];
		scaleMisalignementInverseA.at(1).at(0)=-_misalignementA[0]/_scaleA[0];
		scaleMisalignementInverseA.at(1).at(1)=1/_scaleA[1];
		scaleMisalignementInverseA.at(2).at(0)=(_misalignementA[0]*_misalignementA[2]-_misalignementA[1])/_scaleA[0];
		scaleMisalignementInverseA.at(2).at(1)=-_misalignementA[2]/_scaleA[1];
		scaleMisalignementInverseA.at(2).at(2)=1/_scaleA[2];

//		std::vector<std::vector<double> > scaleMisalignementInverseG={{1/_scaleA[0],0,0},
//				{-_misalignementA[0]/_scaleA[0],1/_scaleA[1],0},
//				{(_misalignementA[0]*_misalignementA[2]-_misalignementA[1])/_scaleA[0],-_misalignementA[2]/_scaleA[1],1/_scaleA[2]}};
		return scaleMisalignementInverseA;
	}



	// todo implement this
	void CalibImuMpu6000::getCalibratedMeasurement(ViImuMsg::Ptr imu_ptr)
	{

	}


} //namespace visensor
