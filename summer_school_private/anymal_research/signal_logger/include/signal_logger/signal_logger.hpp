/*!
 * @file     signal_logger.hpp
 * @author   Christian Gehring, C. Dario Bellicoso, Gabriel Hottiger
 * @date     June 23, 2013
 * @brief    Providing add function depending on current logger type.
 */


#pragma once

#ifdef E
#undef E
#endif

#include "signal_logger_core/typedefs.hpp"
#include "signal_logger_core/SignalLoggerBase.hpp"
#include "signal_logger_std/SignalLoggerStd.hpp"
#include "signal_logger/SignalLoggerNone.hpp"

#ifdef SILO_USE_ROS
  #include "signal_logger_ros/SignalLoggerRos.hpp"
#endif

#include <memory>
#include "assert.h"

namespace signal_logger {

//! Reference to the logger
extern std::shared_ptr<SignalLoggerBase> logger;

//! Get the logger type at runtime
enum class LoggerType: int {
  TypeUnknown = -1,/*!< -1 */
  TypeNone    = 0,/*!< 0 */
  TypeStd     = 1,/*!< 1 */
  TypeRos     = 2/*!< 2 */
};

//! @return the logger type
LoggerType getLoggerType();

void setSignalLoggerNone();

void setSignalLoggerStd();

#ifdef SILO_USE_ROS
void setSignalLoggerRos(ros::NodeHandle* nh);
#endif

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
void add( const ValueType_ & var,
          const std::string & name,
          const std::string & group       = LOG_ELEMENT_DEFAULT_GROUP_NAME,
          const std::string & unit        = LOG_ELEMENT_DEFAULT_UNIT,
          const std::size_t divider       = LOG_ELEMENT_DEFAULT_DIVIDER,
          const LogElementAction action   = LOG_ELEMENT_DEFAULT_ACTION,
          const std::size_t bufferSize    = LOG_ELEMENT_DEFAULT_BUFFER_SIZE,
          const BufferType bufferType     = LOG_ELEMENT_DEFAULT_BUFFER_TYPE)
{
    #ifdef SILO_USE_ROS
      signal_logger_ros::SignalLoggerRos* slRos = dynamic_cast<signal_logger_ros::SignalLoggerRos*>(logger.get());
      if(slRos) {
        slRos->add<ValueType_>(&var, name, group, unit, divider, action, bufferSize, bufferType);
        return;
      }
    #endif

    signal_logger_std::SignalLoggerStd* slStd = dynamic_cast<signal_logger_std::SignalLoggerStd*>(logger.get());
    if(slStd) {
      slStd->add<ValueType_>(&var, name, group, unit, divider, action, bufferSize, bufferType);
      return;
    }

    SignalLoggerNone* slNone = dynamic_cast<SignalLoggerNone*>(logger.get());
    if(slNone) {
      slNone->add<ValueType_>(&var, name, group, unit, divider, action, bufferSize, bufferType);
      return;
    }
}

/** Function implementation to add eigen matrices as their underlying type to the logger.
  *@param var            pointer to log matrix
  *@param names          name of every entry of the matrix
  *@param group          logger group the variable belongs to
  *@param unit           unit of the log variable
  *@param divider        divider is defining the update frequency of the logger element (ctrl_freq/divider)
  *@param action         log action of the log variable
  *@param bufferSize     size of the buffer storing log elements
  *@param bufferType     determines type of buffer
  */
template<typename ValueType_>
typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value>::type
add(const ValueType_ & var,
    Eigen::Ref<MatrixXstring> names,
    const std::string & group       = LOG_ELEMENT_DEFAULT_GROUP_NAME,
    const std::string & unit        = LOG_ELEMENT_DEFAULT_UNIT,
    const std::size_t divider       = LOG_ELEMENT_DEFAULT_DIVIDER,
    const LogElementAction action   = LOG_ELEMENT_DEFAULT_ACTION,
    const std::size_t bufferSize    = LOG_ELEMENT_DEFAULT_BUFFER_SIZE,
    const BufferType bufferType     = LOG_ELEMENT_DEFAULT_BUFFER_TYPE)
{
  assert(names.rows() == var.rows() && "rows() have different size in add");
  assert(names.cols() == var.cols() && "cols() have different size in add");

  for(std::size_t i = 0; i < var.size(); ++i)
  {
    add<typename ValueType_::Scalar>(*static_cast<const typename ValueType_::Scalar * const>(var.data() + i),
                                     static_cast<std::string>(*(names.data() + i)),
                                     group,
                                     unit,
                                     divider,
                                     action,
                                     bufferSize,
                                     bufferType);
  }
}


} // end namespace
