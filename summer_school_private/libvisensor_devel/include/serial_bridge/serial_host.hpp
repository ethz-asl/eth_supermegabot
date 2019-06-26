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

#ifndef SERIALHOST_HPP_
#define SERIALHOST_HPP_

#include <config/config.hpp>

#include <queue>
#include <boost/smart_ptr.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

#include "synchronization/concurrent_queue.hpp"
#include "visensor/visensor_datatypes.hpp"

namespace visensor {

class SerialHost {

 private:
  void processSerialData();
  void publishSerialData(ViSerialData::Ptr& data_ptr);

  boost::shared_ptr<boost::thread> worker_thread;

  std::vector<boost::function<void(ViSerialData::Ptr)> > user_callbacks_;
  concurrent_queue<std::queue<ViSerialData::Ptr> > data_queue_;

 public:
  SerialHost();
  virtual ~SerialHost();

  void setSerialDataCallback(boost::function<void(ViSerialData::Ptr)> callback);
  void addDataToPublishQueue(ViSerialData::Ptr data_ptr);

  typedef boost::shared_ptr<SerialHost> Ptr;
  typedef boost::weak_ptr<SerialHost> WeakPtr;
};

} /* namespace visensor */
#endif /* SERIALHOST_HPP_ */
