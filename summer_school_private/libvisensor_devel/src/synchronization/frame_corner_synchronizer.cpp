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

#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/function.hpp>

#include "synchronization/frame_corner_synchronizer.hpp"

FrameCornerSynchronizer::FrameCornerSynchronizer() {
}
FrameCornerSynchronizer::~FrameCornerSynchronizer() {
}

void FrameCornerSynchronizer::addFrame(ViFrame::Ptr frame) {
  boost::mutex::scoped_lock lock(mutex_);
  while (corner_queue_.empty())
    cond_.wait(lock);

  if (frame->timestamp < corner_queue_.front()->timestamp) {
    VISENSOR_DEBUG(
        "no matching corners found, publishing image without corners!!!\n");
    frame->useCorners = false;
    return;
  }

  while (frame->timestamp > corner_queue_.front()->timestamp) {
    corner_queue_.pop();
    if (corner_queue_.empty())
      cond_.wait(lock);
  }

  if (frame->timestamp != corner_queue_.front()->timestamp) {
    VISENSOR_DEBUG("pair NOT found: this should not happen!!!\n");
    return;
  }
  ViCorner::Ptr corner = corner_queue_.front();
  corner_queue_.pop();

  if (user_callback_)
    user_callback_(frame, corner);
}

void FrameCornerSynchronizer::addCorner(ViCorner::Ptr corner) {
  {
    boost::mutex::scoped_lock lock(mutex_);
    corner_queue_.push(corner);
  }
  cond_.notify_one();
}

void FrameCornerSynchronizer::setUserCallback(
    boost::function<
        void(ViFrame::Ptr, ViCorner::Ptr)> callback) {
  user_callback_ = callback;
}
