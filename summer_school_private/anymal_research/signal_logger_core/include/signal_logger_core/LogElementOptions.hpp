/*!
 * @file     LogElementOptions.hpp
 * @author   Gabriel Hottiger
 * @date     May 05, 2017
 * @brief    Log element options class.
 */

#pragma once

// Signal logger
#include "signal_logger_core/typedefs.hpp"

// STL
#include <atomic>

namespace signal_logger {

class LogElementInterface;

class LogElementOptions
{
 public:
  /** Constructor
   *  @param name       name of the log var
   *  @param unit       unit of the log var
   *  @param divider    log_freq = ctrl_freq/divider
   *  @param action     save, publish or save and publish
   */
  LogElementOptions(const std::string & name,
                    const std::string & unit,
                    const std::size_t divider,
                    const LogElementAction action) :
     name_(name),
     unit_(unit),
     divider_(divider),
     action_(action)
  {

  }

  LogElementOptions(const LogElementOptions& other):
    name_(other.name_),
    unit_(other.unit_)
  {
    divider_.store(other.divider_.load());
    action_.store(other.action_.load());
  }

  void transfer(const LogElementOptions& other)
  {
    divider_.store(other.getDivider());
    action_.store(other.getAction());
  }

  std::string getName() const { return name_; }

  std::string getUnit() const { return unit_; }

  std::size_t getDivider() const { return divider_.load(); }
  void setDivider(const std::size_t divider) { divider_.store(divider); }

  LogElementAction getAction() const { return action_.load(); }
  // todo update element on action change
  void setAction(const LogElementAction action) { action_.store(action); }

  //! @return check action for publishing
  bool isPublished() const {
    return (action_ == LogElementAction::SAVE_AND_PUBLISH || action_ == LogElementAction::PUBLISH);
  }

  //! @return check action for saving
  bool isSaved() const {
    return (action_ == LogElementAction::SAVE_AND_PUBLISH || action_ == LogElementAction::SAVE);
  }

 private:
  //! Name of the log element
  const std::string name_;
  //! Unit of the log element
  const std::string unit_;
  //! Defines log element collection frequency = updateFrequency/divider
  std::atomic_size_t divider_;
  //! Action
  std::atomic<LogElementAction> action_;
};

} /* namespace signal_logger */
