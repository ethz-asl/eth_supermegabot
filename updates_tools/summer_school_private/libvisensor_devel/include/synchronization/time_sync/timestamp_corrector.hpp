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

#ifndef __TIMESTAMP_CORRECTOR_H
#define __TIMESTAMP_CORRECTOR_H

#include <vector>
#include <iostream>

namespace timesync {
/**
 * \class TimestampCorrector
 *
 * An implementation of the convex hull algorithm for one-way
 * timestamp synchronization from
 *
 * L. Zhang, Z. Liu, and C. Honghui Xia,
 * “Clock synchronization algorithms for network measurements”,
 * in INFOCOM 2002. Twenty-First Annual Joint Conference of the
 * IEEE Computer and Communications Societies., vol. 1. IEEE,
 * 2002, pp. 160–169 vol.1.
 *
 */

template<typename TIME_T>
class TimestampCorrector
{
  // Typedefs and Enums
 public:
  typedef TIME_T time_t;

 private:
  class Point
  {
   public:
    Point(const time_t& x, const time_t& y)
        : x(x),
          y(y)
    {
    }
    // remote time
    time_t x;
    // local time
    time_t y;

    Point operator-(const Point& p) const
    {
      return Point(x - p.x, y - p.y);
    }
    Point operator+(const Point& p) const
    {
      return Point(x + p.x, y + p.y);
    }
    bool operator<(const Point& p) const
    {
      return x < p.x;
    }
    bool operator<(const time_t& t) const
    {
      return x < t;
    }
  };
  typedef std::vector<Point> convex_hull_t;

  // Constants
 private:
  static const unsigned int MINIMAL_HULL_SIZE=2;

  // Methodes
 public:
  TimestampCorrector();
  virtual ~TimestampCorrector();

  /**
   * Get an estimate of the local time of a given measurement from the remote timestamp, the local timestamp, and the previous history of timings.
   *
   * NOTE: this function must be called with monotonically increasing
   *       remote timestamps. If this is not followed, an exception will
   *       be thrown.
   *
   * @param remote_time The time of an event on the remote clock
   * @param receive_time  The timestamp that the event was received locally
   *
   * @return The estimated actual local time of the event
   */
  void update(const time_t& remote_time, const time_t& receive_time);

  /**
   * Using the current best estimate of the relationship between
   * remote and local clocks, get the local time of a remote timestamp.
   *
   * @param remote_time The time of an event on the remote clock.
   *
   * @return The estimated local time of the event.
   */
  time_t getLocalTime(const time_t& remote_time) const;

  /**
   * @return The number of points in the convex hull
   */
  size_t convexHullSize() const
  {
    return convex_hull_.size();
  }

  double getSlope() const;
  time_t getOffset() const;

  /**
   * Calculate the local time span between the first measurement and last one
   * @return The local time span in s
   */
  time_t getSpan() const;

  /**
   * Check if there are enough points in the convex hull to estimate a
   * @return The local time span
   */
  bool isStable() const;

  /**
   * Efficient way of exchanging data with another object of the same type
   * @param obj Object to exchange data with
   */
  void swap(TimestampCorrector<time_t>& obj);

  /**
   * Delete all saved variables and set them to the default values
   */
  void reset();

  void printHullPoints()
  {
    for (unsigned i = 0; i < convex_hull_.size(); ++i) {
      std::cout << i << "\t" << convex_hull_[i].x << "\t" << convex_hull_[i].y;
      if (i == midpoint_segment_index_)
        std::cout << " <<< Midpoint segment start";
      std::cout << std::endl;
    }
  }

 private:
  /**
   * Calculate the new Midpoint
   *
   * @param convex_hull hull which need to update the midpoint
   * @param newest_time point which will be added to the convex hull
   *
   * @return the pointer in the middle
   */
  size_t getMidpoint(convex_hull_t& convex_hull, const time_t& newest_time) const;


  /**
   * Is the point above the line defined by the top two points of
   * the convex hull?
   *
   * @param p the point to check
   *
   * @return true if the point is above the last line.
   */
  bool isAboveTopLine(const Point& p) const;

  /**
   * Is the point, p, above the line defined by the points l1 and l2?
   *
   * @param l1
   * @param l2
   * @param p
   *
   * @return true if the point is above the line.
   */
  bool isAboveLine(const Point& l1, const Point& l2, const Point& p) const;

  /**
   * Check if the current timestamp is newer as the first saved timestamp.
   * Otherwise assume that a overflow happens, correct it.
   *
   */
  uint64_t extendTimestampWithOverflows(const time_t& timestamp);


  // Data Members
 private:
  convex_hull_t convex_hull_;
  size_t midpoint_segment_index_;
};

}  // namespace timesync

#include "synchronization/time_sync/timestamp_corrector_impl.hpp"

#endif /* __TIMESTAMP_CORRECTOR_H */
