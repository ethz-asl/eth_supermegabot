/*!
 * @file     BufferInterface.hpp
 * @author   Gabriel Hottiger
 * @date     May 05, 2017
 * @brief    Interface for circular buffer
 */

#pragma once

#include "signal_logger_core/typedefs.hpp"

namespace signal_logger {

class BufferInterface
{
 public:
  //! @return size of the buffer
  virtual std::size_t getBufferSize() const = 0;
  //! @param desired size of the buffer
  virtual void setBufferSize(const std::size_t bufferSize) = 0;

  //! @return type of the buffer
  virtual BufferType getBufferType() const  = 0;
  //! @param desired size of the buffer
  virtual void setBufferType(const BufferType bufferType)  = 0;

  //! @return no items in the buffer (unread and read)
  virtual std::size_t noTotalItems() const = 0;

  //! @return no unread items in the buffer
  virtual std::size_t noUnreadItems() const = 0;

  //! @brief clears the buffer
  virtual void clear() = 0;

};

} /* namespace signal_logger */
