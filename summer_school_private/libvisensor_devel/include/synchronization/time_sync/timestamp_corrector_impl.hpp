/*
 * Copyright (c) 2015, Skybotix AG, Switzerland (info@skybotix.com)
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



#include "config/config.hpp"

namespace timesync {

template<typename T>
TimestampCorrector<T>::TimestampCorrector()
    : midpoint_segment_index_(0)
{
}

template<typename T>
TimestampCorrector<T>::~TimestampCorrector()
{
}

// Returns the local time
template<typename T>
void TimestampCorrector<T>::update(const time_t& remote_time, const time_t& receive_time)
{
  // Make sure this point is forward in time.
  if (convex_hull_.empty() ||
      (receive_time > convex_hull_[convex_hull_.size() - 1].x)) {
    if (!convex_hull_.empty()) {
      VISENSOR_ASSERT_COND( !(remote_time > convex_hull_[convex_hull_.size() - 1].x),
          "The correction algorithm requires that times are passed in with monotonically increasing remote timestamps. "
          "New point is %ld where the last point was at %ld", remote_time, convex_hull_[convex_hull_.size() - 1].x);
    }

    Point p_upper(remote_time, receive_time);

    // If the point is not above the top line in the stack
    if (!isAboveTopLine(p_upper)) {
      // update convex hull
      // While on the top of the stack points are above a line between two back and the new point...
      while (convex_hull_.size() >= 2
          && isAboveLine(convex_hull_[convex_hull_.size() - 2], p_upper,
                         convex_hull_[convex_hull_.size() - 1])) {
        convex_hull_.pop_back();
      }
    }

    // In either case, push the new point on to the convex hull
    convex_hull_.push_back(p_upper);

    midpoint_segment_index_ = getMidpoint(convex_hull_, remote_time);
  }
}


template<typename T>
size_t TimestampCorrector<T>::getMidpoint(convex_hull_t& convex_hull, const time_t& newest_time) const
{
  size_t midpoint_index = 0;
  // Update the midpoint pointer...
  if (convex_hull.size() >= MINIMAL_HULL_SIZE) {
    T midpoint = (convex_hull[0].x + newest_time) / 2.0;

    typename convex_hull_t::iterator lbit = std::lower_bound(convex_hull.begin(), convex_hull.end(), midpoint);
    midpoint_index = lbit - convex_hull.begin() - 1;
    VISENSOR_ASSERT_COND( !(midpoint_index < convex_hull.size()),
        "The computed midpoint segment is out of bounds. Elements in hull: %lu, Start time: %ld, End time: %ld, "
        "midpoint: %ld", convex_hull.size(), convex_hull[0].x, convex_hull[convex_hull.size() - 1].x, midpoint);

    VISENSOR_ASSERT_COND( !(midpoint >= convex_hull[midpoint_index].x),
                     "The computed midpoint is not within the midpoint segment\n"
                     "Midpoint %ld; Midpoint Segment %ld", midpoint, convex_hull[midpoint_index].x);
    VISENSOR_ASSERT_COND( !(midpoint <= convex_hull[midpoint_index + 1].x),
                     "The computed midpoint is not within the midpoint segment"
                     "Midpoint %ld; Midpoint Segment %ld", midpoint, convex_hull[midpoint_index].x);
  }
  return midpoint_index;
}

template<typename T>
double TimestampCorrector<T>::getSlope() const
{
  VISENSOR_ASSERT_COND( !(convex_hull_.size() >= 2),
      "The timestamp correction requires at least two data points before this function can be called"
      "The convex_hull size is only %lu", convex_hull_.size());

  // Get the line at the time midpoint.
  const Point& l1 = convex_hull_[midpoint_segment_index_];
  const Point& l2 = convex_hull_[midpoint_segment_index_ + 1];

  // look up the local timestamp
  return static_cast<double>(l2.y - l1.y) / static_cast<double>(l2.x - l1.x);
}

template<typename T>
T TimestampCorrector<T>::getOffset() const
{
  VISENSOR_ASSERT_COND( !(convex_hull_.size() >= 2),
      "The timestamp correction requires at least two data points before this function can be called"
      "The convex_hull size is only %lu", convex_hull_.size());
  // Get the line at the time midpoint.
  const Point& l1 = convex_hull_[midpoint_segment_index_];

  // look up the local timestamp
  return static_cast<T>(l1.y + -l1.x * getSlope());
}

template<typename T>
T TimestampCorrector<T>::getSpan() const
{
  VISENSOR_ASSERT_COND( !(convex_hull_.size() >= 2),
      "The timestamp correction requires at least two data points before this function can be called"
      "The convex_hull size is only %lu", convex_hull_.size());
  const Point& first = convex_hull_[0];
  const Point& last = convex_hull_[convex_hull_.size() - 1];
  // return span in FPGA tick cycles
  return last.x - first.x;
}

// Get the local time from the remote time.
template<typename T>
T TimestampCorrector<T>::getLocalTime(
    const time_t& remote_time) const
{
  VISENSOR_ASSERT_COND( !(convex_hull_.size() >= 2),
      "The timestamp correction requires at least two data points before this function can be called"
      "The convex_hull size is only %lu", convex_hull_.size());

  // Get the line at the time midpoint.
  const Point& l1 = convex_hull_[midpoint_segment_index_];

  // look up the local timestamp (midleLocalTime + (currentRemoteTime - middleRemoteTiem)*slope
  return static_cast<T>(l1.y + (static_cast<double>(remote_time - l1.x)) * getSlope());
}

template<typename T>
bool TimestampCorrector<T>::isStable() const
{
  return convex_hull_.size() >= MINIMAL_HULL_SIZE;
}

template<typename T>
void TimestampCorrector<T>::swap(TimestampCorrector<T>& obj)
{
  midpoint_segment_index_ = obj.midpoint_segment_index_;
  convex_hull_.swap(obj.convex_hull_);
}

template<typename T>
void TimestampCorrector<T>::reset()
{
  midpoint_segment_index_ = 0;
  convex_hull_.clear();
}

template<typename T>
bool TimestampCorrector<T>::isAboveTopLine(const Point& p) const
{
  if (convex_hull_.size() < 2) {
    return true;
  }
  return isAboveLine(convex_hull_[convex_hull_.size() - 2], convex_hull_[convex_hull_.size() - 1], p);
}

template<typename T>
bool TimestampCorrector<T>::isAboveLine(const Point& l1, const Point& l2, const Point& p) const
{
  Point v1 = l2 - l1;
  Point v2 = p - l1;

  double determinant = static_cast<double>(v1.x) * static_cast<double>(v2.y) - static_cast<double>(v1.y) * static_cast<double>(v2.x);

  return determinant >= 0.0;
}

}  // namespace timesync
