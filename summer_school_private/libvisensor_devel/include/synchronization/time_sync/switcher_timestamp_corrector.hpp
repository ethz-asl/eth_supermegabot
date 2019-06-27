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

#ifndef __SwitcherTimestampCorrector_h
#define __SwitcherTimestampCorrector_h

#include "timestamp_corrector.hpp"

namespace timesync {


/**
 * \class SwitcherTimestampCorrector
 *
 * An implementation of the convex hull algorithm for one-way timestamp synchronization from
 *
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2014, Autonomous Systems Lab, ETH Zurich, Switzerland
 *
 * L. Zhang, Z. Liu, and C. Honghui Xia,
 * “Clock synchronization algorithms for network measurements”,
 * in INFOCOM 2002. Twenty-First Annual Joint Conference of the
 * IEEE Computer and Communications Societies., vol. 1. IEEE,
 * 2002, pp. 160–169 vol.1.
 *
 */
template<typename TIME_T>
class SwitcherTimestampCorrector
{
 public:
  typedef TIME_T time_t;

  SwitcherTimestampCorrector(const time_t& switchPeriod, time_t expeced_fpga_tick_time)
      : switchPeriod_(switchPeriod),
        tick_time_(expeced_fpga_tick_time)

  {
  }

  virtual ~SwitcherTimestampCorrector()
  {
  }

  /**
   * Add in a new set of time stamp measurements
   *
   * @param fpga_time Time at which the FPGA sends the timestamp (measured by fpga clock)
   * @param pc_receipt_time Time at which the PC received the timestamp (measured by client clock)
   */
  void update( const time_t& fpga_time, const time_t& pc_receipt_time)
  {
    clock_mapper_active_.update( fpga_time, pc_receipt_time);
    clock_mapper_passive_.update( fpga_time, pc_receipt_time);

    checkSwitchingPeriod();
  }

  /**
   * Set period between clock_mapper replacement
   * @param periodTime
   */
  void setSwitchPeriod(const time_t& periodTime)
  {
    switchPeriod_ = periodTime;
  }

  /**
   * Get period between clock_mapper replacement
   * @return
   */
  time_t getSwitchPeriod()
  {
    return switchPeriod_;
  }

  /**
   * Converts fpga clock times to pc clock times, using the currently estimated
   * linear mapping.
   * @param fpga_time FPGA time
   * @return corrected PC time
   */
  time_t getLocalTime(const time_t& fpga_time) const
  {
    return clock_mapper_active_.getLocalTime(fpga_time);
  }

  /**
   * Reset the filter and throw away all existing data.
   */
  void reset()
  {
    clock_mapper_active_.reset();
    clock_mapper_passive_.reset();
  }

  /**
   * Efficient way of exchanging data with another object of the same type
   * @param obj Object to exchange data with
   */
  void swap(SwitcherTimestampCorrector<int64_t>& obj)
  {
    clock_mapper_active_.swap(obj.clock_mapper_active_);
    clock_mapper_passive_.swap(obj.clock_mapper_passive_);
  }

  /**
   * Difference between latest and earliest measurements in seconds
   * @return
   */
  time_t getSpan() const
  {
    return clock_mapper_active_.getSpan() * tick_time_ / 1e9;
  }

  /**
   * Indicates whether the underlying convex hulls have enough points to make a prediction.
   *
   * @return
   */
  bool isStable() const
  {
    return clock_mapper_active_.isStable();
  }

  /**
   * Returns the estimated skew between the two clocks.
   * @return skew of the offset
   */
  double getSkew() const
  {
    return clock_mapper_active_.getSlope();
  }

  /**
   * Returns the estimated offset (beta acording to paper) between FPGA and PC.
   * @return offset
   */
  time_t getOffset() const
  {
    return clock_mapper_active_.getOffset();
  }

 private:

  void checkSwitchingPeriod()
  {
    if ((clock_mapper_passive_.isStable()) && (clock_mapper_passive_.getSpan() * tick_time_ > switchPeriod_)) {
      clock_mapper_active_.swap(clock_mapper_passive_);
      clock_mapper_passive_.reset();
    }
  }

 private:
  TimestampCorrector<time_t> clock_mapper_active_;
  TimestampCorrector<time_t> clock_mapper_passive_;
  // switching period in ns
  time_t switchPeriod_;
  //tick time of a the FPGA in ns
  time_t tick_time_;
};

}  // namespace timesync

#endif /* __SwitcherTimestampCorrector_h */
