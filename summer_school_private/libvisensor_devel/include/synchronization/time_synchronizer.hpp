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

#ifndef TIME_SYNCHRONIZER_HPP_
#define TIME_SYNCHRONIZER_HPP_

#include <config/config.hpp>
#include <synchronization/time_sync/switcher_timestamp_corrector.hpp>
#include <visensor/visensor.hpp>

namespace timesync {
/**
 * \class TimeSynchronizer
 *
 * \brief This class manage a model to map a remote (FPGA) clock to local time.
 *
 * With updateTime a linear model is initialized and learned to estimate the skew and offset
 * of the remote to the local clock.
 *
 * The getSynchronizedTime maps the remote (FPGA) time to the local (PC) time
 *
 * \date $Date: 2015/05/12 $
 *
 *
 */
class DSO_EXPORT TimeSynchronizer {
 public:
  static constexpr uint64_t MINIMAL_STABLE_DRIFT_TIME_IN_S = 15.0 * visensor::MINUTE_IN_SECOND;
  static constexpr uint64_t DEFAULT_FPGA_PERIOD_IN_NANOSECONDS = 10.0 * visensor::MUSECOND_IN_NANOSECOND;


  /**
   * Constructor of the class
   *
   * @param two_way_sync   Defines if the TimeSynchronizer used model need one or two way data to estimate the parameters
   * @param fpga_tick_time The time of one remote tick in ns. This is needed to interpret the clocks.
   * @param stream_id      ID of the object on which the update should listen. This is saved here to make we only get timestamps from one source.
   *
   *
   */
  TimeSynchronizer( const unsigned int stream_id, const uint64_t fpga_tick_time );

  /**
   * This function is used to initialize and update the internal model with actual timestamps.
   *
   * @param pc_request_time It represent the sending time in ns of the Ping Pong.
   * @param fpga_time       The receiving request and sending time where the remote device had sent the response. The values are in local ticks.
   * @param pc_receipt_time The receiving time in ns of the PingPong response.
   */
  void initialUpdate(const uint64_t pc_request_time, const uint32_t fpga_time, const uint64_t pc_receipt_time);

  /**
   * This function is used to initialize and update the internal model with actual timestamps.
   *
   * @param fpga_time       The time where the remote device had sent the timestamp. The values are in local ticks.
   * @param pc_receipt_time The receiving time in ns of the timestamp .
   */
  void updateTime(const uint32_t fpga_time, const uint64_t pc_receipt_time);

  /**
   * returns the mapped remote timestamp in ns to the local time according to the internal model
   *
   * @param time_fpga remote timestamp
   * @return
   */
  uint64_t getSynchronizedTime(const uint32_t time_fpga);

  /**
   * returns local time in ns
   *
   * @return
   */
  uint64_t getSystemTime();

  /**
   *
   * @return the estimated skew between remote and local clock
   */
  double getSlope();

  /**
   *
   * @return the local time at the remote time 0
   */
  uint64_t getOffset();

  inline uint64_t getInitialOffset() const {
    return initial_offset_;
  }

  inline void setInitialOffset(uint64_t offset) {
    initial_offset_ = offset;
  }

  inline uint64_t getTransmissionTime() const {
    return transmission_time_;
  }

  /**
   * Tracks the overflows and extend the timestamps to 64 bit.
   *
   * @return extended timestamp
   */
  uint64_t extendTimestampWithOverflows(const uint32_t& timestamp);

  const unsigned int get_stream_id() const { return stream_id_; }
  void set_stream_id(const unsigned int stream_id) { stream_id_ = stream_id; }

 private:
  uint64_t transmission_time_;          ///< the minimal transmission time. This value has to be estimated with PingPong even the model uses one way updates
  uint64_t initial_offset_;             ///< a initial offset between remote clock and local clock. This is used if no synchronization is wished.
  unsigned int stream_id_;              ///< ID of the object on which the TimeSynchronizer should listen

  SwitcherTimestampCorrector<int64_t> clock_mapper_;  //< Model for the one way synchronization
  uint64_t num_overflows_;               ///< keep track on the overflows
  int64_t last_timestamp_;              ///< use to detect overflows of the remote clock

  const uint64_t FPGA_TIC_TIME;
  const int64_t POSSIBLE_OLDER_TIMESTAMP;
};

}  // namespace timesync

#endif /* TIME_SYNCHRONIZER_HPP_ */
