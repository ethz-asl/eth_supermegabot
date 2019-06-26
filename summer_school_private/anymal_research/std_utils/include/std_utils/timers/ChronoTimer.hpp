/*
 * ChronoTimer.hpp
 *
 *  Created on: Mar 22, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

#include <map>
#include <tuple>
#include <chrono>
#include <string>
#include <sstream>

namespace std_utils {
namespace internal {

template<typename ClockType_>
class ChronoTimer
{
 public:
  typedef ClockType_                                clock;
  typedef typename clock::time_point                time_point;
  typedef std::chrono::duration<double>             seconds_t;
  typedef std::chrono::duration<double, std::milli> milliseconds_t;
  typedef std::chrono::duration<double, std::micro> microseconds_t;
  typedef std::chrono::duration<double, std::nano>  nanoseconds_t;

  explicit ChronoTimer(const std::string& timerNamespace = "")
      : alpha_(0.01),
        timerNamespace_(timerNamespace)
  {
    timerMap_["default"] = std::make_tuple("default", clock::now(), nanoseconds_t(), false);
  }

  virtual ~ChronoTimer() = default;

  /*! Pin the current time for a named timer
   */
  void pinTime(const std::string& timerName = "default") {
    std::get<1>(timerMap_[timerName]) = clock::now();
  }

  /*! Pin the current time for all timers
   */
  void pinAllTimers() {
    time_point now = clock::now();
    for (auto& timer : timerMap_) {
      std::get<1>(timer) = clock::now();
    }
  }

  /*! Store the current split time (time since last pin) for a named timer.
   */
  void splitTime(const std::string& timerName = "default") {
    // get split time
    const microseconds_t duration = clock::now() - std::get<1>(timerMap_.at(timerName));



    // alpha filter the split times
    if (std::get<3>(timerMap_[timerName])) {
      std::get<2>(timerMap_[timerName]) = alpha_*duration + (1.0-alpha_)*std::get<2>(timerMap_.at(timerName));
    } else {
      std::get<2>(timerMap_[timerName]) = duration;
      std::get<3>(timerMap_[timerName]) = true;
    }

  }

  /*! Get duration since last split time for a named timer in nanoseconds.
   */
  inline double getElapsedTimeNanoSec(const std::string& timerName = "default") const {
    return nanoseconds_t{clock::now() - std::get<1>(timerMap_.at(timerName))}.count();
  }

  /*! Get duration since last split time for a named timer in microseconds.
   */
  inline double getElapsedTimeUsec(const std::string& timerName = "default") const {
    return microseconds_t{clock::now() - std::get<1>(timerMap_.at(timerName))}.count();
  }

  /*! Get duration since last split time for a named timer in milliseconds.
   */
  inline double getElapsedTimeMsec(const std::string& timerName = "default") const {
    return milliseconds_t{clock::now() - std::get<1>(timerMap_.at(timerName))}.count();
  }

  /*! Get duration since split time for a named timer in seconds.
   */
  inline double getElapsedTimeSec(const std::string& timerName = "default") const {
    return seconds_t{clock::now() - std::get<1>(timerMap_.at(timerName))}.count();
  }

  /*! Get the average split time for a named timer in nanoseconds.
   */
  inline double getAverageElapsedTimeNSec(const std::string& timerName = "default") const {
    return nanoseconds_t{std::get<2>(timerMap_.at(timerName))}.count();
  }

  /*! Get the average split time for a named timer in milliseconds.
   */
  inline double getAverageElapsedTimeMSec(const std::string& timerName = "default") const {
    return milliseconds_t{std::get<2>(timerMap_.at(timerName))}.count();
  }

  /*! Get the average split time for a named timer in microseconds.
   */
  inline double getAverageElapsedTimeUSec(const std::string& timerName = "default") const {
    return microseconds_t{std::get<2>(timerMap_.at(timerName))}.count();
  }

  /*! Get the average split time for a named timer in seconds.
   */
  inline double getAverageElapsedTimeSec(const std::string& timerName = "default") const {
    return seconds_t{std::get<2>(timerMap_.at(timerName))}.count();
  }

  /*! The average split time is obtained by alpha filtering the incoming split times. Set the alpha parameter of the filter.
   */
  void setAlpha(double alpha) { alpha_ = alpha; }

  /*! The average split time is obtained by alpha filtering the incoming split times. Get the alpha parameter of the filter.
   */
  double getAlpha() const { return alpha_; }

  /*! Set a name to be used when printing the average split times of all timers.
   */
  void setTimerNamespace(const std::string& timerNamespace) { timerNamespace_ = timerNamespace; }

  /*! Get the name used when printing the average split times of all timers.
   */
  const std::string& getTimerNamespace() const { return timerNamespace_; }

  /*! Printout all the average split times currently stored in the timer.
   */
  friend std::ostream& operator<< (std::ostream& stream, const ChronoTimer<ClockType_>& chronoTimer) {
    stream << std::endl << "*********" << std::endl;
    stream << "Timer namespace: '" << chronoTimer.timerNamespace_ << "'" << std::endl;
    for (const auto& timer : chronoTimer.timerMap_) {
      stream << "Average elapsed time for timer '" << std::get<0>(timer) << "' is "
             << chronoTimer.getAverageElapsedTimeUSec(std::get<0>(timer)) << "us. " << std::endl;
    }
    stream << "*********" << std::endl;
    return stream;
  }

  std::string asString() {
    std::ostringstream stream;
    stream << std::endl << "*********" << std::endl;
    stream << "Timer namespace: '" << timerNamespace_ << "'" << std::endl;
    for (const auto& timer : timerMap_) {
      stream << "Average elapsed time for timer '" << std::get<0>(timer) << "' is "
             << getAverageElapsedTimeUSec(std::get<0>(timer)) << "us. " << std::endl;
    }
    stream << "*********" << std::endl;
    return stream.str();
  }


 private:
  /*! A dictionary of objects indexed by name. The value of each entry is a tuple which contains:
   *  1. a name
   *  2. the last pinned time
   *  3. the current average split time
   *  4. flag set to true when split time is called for the first time
   */
  std::map<std::string, std::tuple<std::string, time_point, nanoseconds_t, bool>> timerMap_;

  /*! The parameter used to filter the average split times.
   */
  double alpha_;

  /*! A common name for all stored timers to be used in printouts.
   */
  std::string timerNamespace_;

};

} /* namespace robot_utils::internal */

// Some convenience typedefs
typedef internal::ChronoTimer<std::chrono::steady_clock>          SteadyClockTimer;
typedef internal::ChronoTimer<std::chrono::system_clock>          SystemClockTimer;
typedef internal::ChronoTimer<std::chrono::high_resolution_clock> HighResolutionClockTimer;

} /* namespace robot_utils */
