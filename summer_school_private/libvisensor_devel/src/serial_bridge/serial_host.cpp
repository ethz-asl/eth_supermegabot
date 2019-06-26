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
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

#include "visensor/visensor_datatypes.hpp"

#include "serial_bridge/serial_host.hpp"

namespace visensor {

SerialHost::SerialHost()
{
  //start processing thread
  worker_thread = boost::make_shared<boost::thread>(&SerialHost::processSerialData, this);
}

SerialHost::~SerialHost() {
  //stop thread
  try {
    if(worker_thread)
      worker_thread->interrupt();
  } catch (const std::exception &ex) {
    VISENSOR_DEBUG("SerialHost exception in destructor: %s\n", ex.what());
  }

  //wait for it to stop
  worker_thread->join();
}

//the threaded function that works on the queue of finished measurements
void SerialHost::processSerialData() {

  while (1) {
    boost::this_thread::interruption_point();

    //get the newest measurement (waits if no new msgs available..)
    ViSerialData::Ptr data_ptr = data_queue_.pop();

    //send out to user application
    publishSerialData(data_ptr);
  }
}

void SerialHost::setSerialDataCallback(boost::function<void(ViSerialData::Ptr)> callback) {
  user_callbacks_.push_back(callback);
}

void SerialHost::publishSerialData(ViSerialData::Ptr& data_ptr) {
  if (user_callbacks_.empty() == 0) {
    BOOST_FOREACH( boost::function<void(ViSerialData::Ptr)> callback, user_callbacks_)
        { callback(data_ptr); }
  }
}

void SerialHost::addDataToPublishQueue(ViSerialData::Ptr data_ptr)
{
  data_queue_.push(data_ptr);
}

} /* namespace visensor */
