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

#include <boost/function.hpp>

#include <sensors/imu.hpp>

namespace visensor
{

void Imu::setUserCallback(boost::function<void (ViImuMsg::Ptr, ViErrorCode)> callback)
{
	user_callback_=callback;
}

void Imu::publishImuData(ViImuMsg::Ptr imu, ViErrorCode error)
{
  /////////////////////////////
  //
  // WORKAROUND FOR FPGA BUG
  //
  // some bug in the imu pipeline causes ~4 old imu measurements to be stuck
  // in some buffer. these measurements are released at reconnect.
  //
  // therefore we drop the first 4 messages, until we have an fpga fix
  //
  /////////////////////////////

  //discard the first 4 real imu messages (1 imu message = 10 imu measurements)
  static int drop_counter = 41;
  if(drop_counter > 0)
  {
    //only real frames count (= no error)
    if(error==visensor::ViErrorCodes::NO_ERROR)
      drop_counter--;
    return;
  }

  // END WORKAROUND FOR FPGA BUG
  /////////////////////////////

  //publish
  if(user_callback_)
    user_callback_(imu, error);
}

} //namespace visensor
