/*!
 * @file     SignalLoggerRos.hpp
 * @author   Gabriel Hottiger
 * @date     Sep 23, 2016
 * @brief    Implementation of a ROS signal logger. Provides necessary ROS communication.
 *           Extends std logger with publishing and bag storing functionality.
 */

#pragma once

// signal logger
#include "signal_logger_std/SignalLoggerStd.hpp"
#include "signal_logger_ros/LogElementRos.hpp"
#include "signal_logger_core/typedefs.hpp"

// msgs
#include "signal_logger_msgs/GetLoggerConfiguration.h"
#include "signal_logger_msgs/GetLoggerElement.h"
#include "signal_logger_msgs/SetLoggerElement.h"
#include "signal_logger_msgs/EditLoggerScript.h"
#include "signal_logger_msgs/SaveLoggerData.h"

// rosbag
#include <rosbag/bag.h>

// ros services
#include <std_srvs/Trigger.h>

namespace signal_logger_ros {

class SignalLoggerRos : public signal_logger_std::SignalLoggerStd
{
 public:
  /** Constructor
   *  @param nh             pointer to the ros nodehandle
   */
  explicit SignalLoggerRos(ros::NodeHandle * nh);

  //! Destructor
  ~SignalLoggerRos() override = default;

  /** Add variable to logger. This is a default implementation if no specialization is provided an error is posted.
    * @tparam ValueType_       Data type of the logger element
    * @param  var              Pointer to log variable
    * @param  name             name of the log variable
    * @param  group            logger group the variable belongs to
    * @param  unit             unit of the log variable
    * @param  divider          divider is defining the update frequency of the logger element (ctrl_freq/divider)
    * @param  action           log action of the log variable
    * @param  bufferSize       size of the buffer storing log elements
    * @param  bufferType       determines the buffer type
    */
  template<typename ValueType_>
  void add( const ValueType_ * const var,
            const std::string & name,
            const std::string & group                     = signal_logger::LOG_ELEMENT_DEFAULT_GROUP_NAME,
            const std::string & unit                      = signal_logger::LOG_ELEMENT_DEFAULT_UNIT,
            const std::size_t divider                     = signal_logger::LOG_ELEMENT_DEFAULT_DIVIDER,
            const signal_logger::LogElementAction action  = signal_logger::LOG_ELEMENT_DEFAULT_ACTION,
            const std::size_t bufferSize                  = signal_logger::LOG_ELEMENT_DEFAULT_BUFFER_SIZE,
            const signal_logger::BufferType bufferType    = signal_logger::LOG_ELEMENT_DEFAULT_BUFFER_TYPE)
  {
    std::string elementName = options_.loggerPrefix_ + "/" + group + "/" + name;
    elementName.erase(std::unique(elementName.begin(), elementName.end(), signal_logger::both_slashes()), elementName.end());
    {
      // Lock the logger (blocking!)
      boost::unique_lock<boost::shared_mutex> addLoggerLock(loggerMutex_);
      logElementsToAdd_[elementName].reset(new LogElementRos<ValueType_>(var, bufferType, bufferSize, elementName , unit, divider, action,
                                                                         &textStream_, &binaryStream_, nh_, bagWriter_));
    }
  }

  //! Shutdown ros communication
  virtual bool cleanup() override;

  /** Returns the current time
   * @return current time
   */
  virtual signal_logger::TimestampPair getCurrentTime() override;

  //! Save all the buffered data into a log file
  virtual bool workerSaveData(const std::string & logFileName, const signal_logger::LogFileTypeSet & logfileTypes) override;

  /** Get current logger configuration
   *  @param  req empty request
   *  @param  res logger_namespace, script_filepath, log_element_names, collect_frequency
   *  @return true iff successful
   */
  bool getLoggerConfiguration(signal_logger_msgs::GetLoggerConfigurationRequest& req,
                             signal_logger_msgs::GetLoggerConfigurationResponse& res);

  /** Get logger element
   *  @param  req element name
   *  @param  res logger element
   *  @return true iff successful
   */
  bool getLoggerElement(signal_logger_msgs::GetLoggerElementRequest& req,
                        signal_logger_msgs::GetLoggerElementResponse& res);

  /** Set logger element
   *  @param  req log element
   *  @param  res success status
   *  @return true iff successful
   */
  bool setLoggerElement(signal_logger_msgs::SetLoggerElementRequest& req,
                        signal_logger_msgs::SetLoggerElementResponse& res);

  /** Start Logger
   *  @param  req empty request
   *  @param  res success status
   *  @return true iff successful
   */
  bool startLogger(std_srvs::TriggerRequest& req,
                   std_srvs::TriggerResponse& res);

  /** Stop Logger
   *  @param  req empty request
   *  @param  res success status
   *  @return true iff successful
   */
  bool stopLogger(std_srvs::TriggerRequest& req,
                  std_srvs::TriggerResponse& res);

  /** Save Logger data
   *  @param  req empty request
   *  @param  res success status
   *  @return true iff successful
   */
  bool saveLoggerData(signal_logger_msgs::SaveLoggerDataRequest& req,
                      signal_logger_msgs::SaveLoggerDataResponse& res);

  /** Is logger running
   *  @param  req empty request
   *  @param  res is logger running flag
   *  @return true iff successful
   */
  bool isLoggerRunning(std_srvs::TriggerRequest& req,
                       std_srvs::TriggerResponse& res);

  /** Load logger script
   *  @param  req file path
   *  @param  res success status
   *  @return true iff successful
   */
  bool loadLoggerScript(signal_logger_msgs::EditLoggerScriptRequest& req,
                        signal_logger_msgs::EditLoggerScriptResponse& res);


  /** Save logger script
   *  @param  req file path
   *  @param  res success status
   *  @return true iff successful
   */
  bool saveLoggerScript(signal_logger_msgs::EditLoggerScriptRequest& req,
                        signal_logger_msgs::EditLoggerScriptResponse& res);

  /** Write log element to msg
   *  @param  name log element name
   *  @param  msg  log element message
   *  @return true iff successful
   */
  bool logElementtoMsg(const std::string & name, signal_logger_msgs::LogElement & msg);

  /** Write msg to log element
   *  @param  msg  log element message (contains name of log element to write)
   *  @return true iff successful
   */
  bool msgToLogElement(const signal_logger_msgs::LogElement & msg);

 private:
  //! ROS nodehandle
  ros::NodeHandle* nh_;
  //! Shared ptr to a bag writer object
  std::shared_ptr<rosbag::Bag> bagWriter_;
  //! Get logger configuration service
  ros::ServiceServer getLoggerConfigurationService_;
  //! Get logger element service
  ros::ServiceServer getLoggerElementService_;
  //! Set logger element service
  ros::ServiceServer setLoggerElementService_;
  //! Start logger service
  ros::ServiceServer startLoggerService_;
  //! Stop logger service
  ros::ServiceServer stopLoggerService_;
  //! Save logger data service
  ros::ServiceServer saveLoggerDataService_;
  //! Load logger script service
  ros::ServiceServer loadLoggerScriptService_;
  //! Save logger script service
  ros::ServiceServer saveLoggerScriptService_;
  //! Is logger running service
  ros::ServiceServer isLoggerRunningService_;

};

} /* namespace signal_logger */
