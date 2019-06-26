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
#include <limits>
#include <sys/time.h>

#include "synchronization/time_synchronizer.hpp"

namespace timesync {
TimeSynchronizer::TimeSynchronizer( const unsigned int stream_id, const uint64_t fpga_tick_time )
: transmission_time_(-1),  // max unsigned
  initial_offset_(0),
  stream_id_(stream_id),
  clock_mapper_(MINIMAL_STABLE_DRIFT_TIME_IN_S * visensor::SECOND_IN_NANOSECOND, fpga_tick_time),  // switching time is in ns seconds, tick cycle time has to be given
  num_overflows_(0),
  last_timestamp_(0),
  FPGA_TIC_TIME(fpga_tick_time),
  POSSIBLE_OLDER_TIMESTAMP(500e6/ fpga_tick_time ) // allow up to 500ms older timestamps before assuming its an overflow
{
}

void TimeSynchronizer::updateTime(const uint32_t fpga_time, const uint64_t pc_receipt_time)
{
  uint64_t time_fpga_long = extendTimestampWithOverflows(fpga_time);
  clock_mapper_.update(time_fpga_long, pc_receipt_time);
}

void TimeSynchronizer::initialUpdate(const uint64_t pc_request_time, const uint32_t fpga_time,
                                     const uint64_t pc_receipt_time)
{
  uint64_t time_fpga_long = extendTimestampWithOverflows(fpga_time);
  clock_mapper_.update(time_fpga_long, pc_receipt_time);

  // find and save minimal transmission time (assumes symmetric transmission time)
  if (transmission_time_ > (pc_receipt_time - pc_request_time) / 2) {
    transmission_time_ = (pc_receipt_time - pc_request_time) / 2;
    initial_offset_ = pc_receipt_time - transmission_time_ - fpga_time * FPGA_TIC_TIME;
  }
}


// returns synchronized time in nanoseconds
uint64_t TimeSynchronizer::getSynchronizedTime(const uint32_t time_fpga)
{
  uint64_t estLocalTime;
  uint64_t time_fpga_long = extendTimestampWithOverflows(time_fpga);
  if (clock_mapper_.isStable()){
    estLocalTime = clock_mapper_.getLocalTime(time_fpga_long) - transmission_time_;
  }
  else{
    VISENSOR_DEBUG("clock_mapper_ not ready\n");
    estLocalTime = static_cast<uint64_t>(time_fpga) * FPGA_TIC_TIME - transmission_time_;
  }
  return estLocalTime;
}

// get system time in nanoseconds
uint64_t TimeSynchronizer::getSystemTime()
{
  timeval tv;
  gettimeofday(&tv, NULL);
  return (uint64_t) tv.tv_sec * 1000000000 + (uint64_t) tv.tv_usec * 1000;
}

// get estimated slope
double TimeSynchronizer::getSlope()
{
  return clock_mapper_.getSkew();
}

// get estimated offset
uint64_t TimeSynchronizer::getOffset()
{
  return clock_mapper_.getOffset();
}

uint64_t TimeSynchronizer::extendTimestampWithOverflows(const uint32_t& timestamp)
{

	// THIS IS NOT A FIX TO THE WRAP-AROUND PROBLEM!

	//  uint32_t imax = std::numeric_limits<uint32_t>::max();
	//  int64_t time_diff = static_cast<int64_t>(timestamp) - last_timestamp_;
	//
	//  if (time_diff < -POSSIBLE_OLDER_TIMESTAMP) {
	//    num_overflows_++;
	//    VISENSOR_DEBUG("remote time overflow occurred or the timestamp is much older then the last one.\n");
	//    VISENSOR_DEBUG("nb_overflows; %lu ; timestamp; %u; last_timestamp_; %lu\n",
	//                   num_overflows_, timestamp, last_timestamp_);
	//  }
	//  else if (time_diff > (imax-POSSIBLE_OLDER_TIMESTAMP)) {
	//    num_overflows_--;
	//    VISENSOR_DEBUG("a huge step forward occured or the timestamp is much "
	//        "older then the last one and jumped over the overflow.\n");
	//    VISENSOR_DEBUG("nb_overflows; %lu ; timestamp; %u; last_timestamp_; %lu\n",
	//                   num_overflows_, timestamp, last_timestamp_);
	//
	//    }
	//
	//
	//  uint64_t extended_timestamp = static_cast<uint64_t>(timestamp) + num_overflows_ * static_cast<uint64_t>(imax);
	//  last_timestamp_ = static_cast<int64_t>(timestamp);
	//
	//  return extended_timestamp;
	return static_cast<int64_t>(timestamp);
}

}  // namespace timesync
