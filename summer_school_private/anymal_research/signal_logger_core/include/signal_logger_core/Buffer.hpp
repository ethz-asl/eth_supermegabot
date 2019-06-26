/*!
 * @file     Buffer.hpp
 * @author   Gabriel Hottiger
 * @date     Sep 21, 2016
 * @brief    A class that implements a circular buffer for type ValueType_.
 */

#pragma once

// Signal logger
#include "signal_logger_core/BufferInterface.hpp"
#include "signal_logger_core/LogElementTypes.hpp"
#include "signal_logger_core/typedefs.hpp"
#include "signal_logger_core/signal_logger_traits.hpp"

// Message logger
#include "message_logger/message_logger.hpp"

// Eigen
#include <Eigen/Core>

// Boost
#include <boost/circular_buffer.hpp>

// STL
#include <type_traits>
#include <algorithm>
#include <thread>
#include <mutex>


namespace signal_logger {
/** Buffer that stores all elements in a circular manner. Allows looping, thus overwriting
 *  old entries with newer ones, keeps count of the unread items, push and pop influence this count.
 *  If buffer is empty no elements can be popped, if buffer is full and not looping
 *  no elements can be pushed. Popping does not delete the item, only sets unread item
 *  count . A copy of all the elements stored in the  buffer (read and unread items) can be obtained.
 *  This implementation of a thread-safe circular buffer is adapted from here:
 *  http://www.boost.org/doc/libs/1_54_0/libs/circular_buffer/doc/circular_buffer.html#boundedbuffer
 */
//! Thread-safe circular buffer
template <typename ValueType_>
class Buffer: public BufferInterface
{
 public:
  //! Forward typedef as circular_buffer_type
  typedef boost::circular_buffer<ValueType_> circular_buffer_type;

  //! Forward typedef for size_type
  typedef typename circular_buffer_type::size_type size_type;

  //! Default type trait
  template<typename T, typename Enable_ = void>
  struct is_buffer_default_type : std::false_type {};

  template<typename T>
  struct is_buffer_default_type<T, typename std::enable_if<!traits::is_eigen_matrix<T>::value>::type> : std::true_type {};

 public:
  /** Constructor for non-eigen-types
   *  @param ptr  pointer to the data to be buffered
   */
  template<typename V = ValueType_>
  explicit Buffer(const V * const ptr, const std::string & name, typename std::enable_if<is_buffer_default_type<V>::value>::type* = 0 /* default-type */ ) :
      Buffer(ptr, name, 1, 1)
  {
  }

  /** Constructor for eigen-types (initializes rows_ and cols_ correctly)
   *  @param ptr  pointer to the data to be buffered
   */
  template<typename V = ValueType_>
  explicit Buffer(const V * const ptr,  const std::string & name, typename std::enable_if<traits::is_eigen_matrix<V>::value>::type* = 0 /* eigen-type */ ) :
    Buffer(ptr, name, ptr->rows(), ptr->cols())
  {
  }

  /** transfer to new buffer
   *  @param other  Lvalue to other buffer
   *  @brief basically a copy constructor however, the container entries are moved
   */
  void transfer(Buffer & other)
  {
    container_ = other.container_;
    bufferType_ = other.bufferType_;
    bufferSize_ = other.bufferSize_;
    noUnreadItems_ = other.noUnreadItems_;
    noItems_ = other.noItems_;
    noNewItems_ = other.noNewItems_;
  }

  //! Push data into the buffer (if looping or not full).
  bool collect()
  {
    // Lock the circular buffer
    std::unique_lock<std::mutex> lock(mutex_);

    // If buffer is exponentially increasing and full -> increase size by factor 2
    if( bufferType_ == BufferType::EXPONENTIALLY_GROWING && noItems_ >= bufferSize_)
    {
      bufferSize_ = std::max(size_type(100), 2*bufferSize_);
      container_.set_capacity(bufferSize_);
    }
    else {
      // Non-looping buffers don't allow filling a full buffer
      if( (bufferType_ != BufferType::LOOPING && noItems_ >= bufferSize_) || bufferSize_ == 0)
      {
        return false;
      }
    }

    // Add value to the buffer
    if(pushElementFront(ptr_)) {
      /*  noUnreadItems -> increases on push decreased on pop, max out at capacity
       *  noItems -> increases on push, max out at capacity
       */
      noUnreadItems_ = std::min((noUnreadItems_ + 1), bufferSize_);
      noItems_ = std::min((noItems_ + 1), bufferSize_);
      noNewItems_ = std::min((noNewItems_ + 1), bufferSize_);
    }

    return true;
  }

  /** Read an item from the buffer (if buffer not empty). Decrement no unread items.
   *  @param pItem pointer to value in which popped value is stored
   */
  bool read(ValueType_ * pItem)
  {
    // Lock the circular buffer
    std::unique_lock<std::mutex> lock(mutex_);

    // Check if buffer is empty
    if( noUnreadItems_ == 0 ) { return false; }
    --noUnreadItems_;

    // Get item from buffer, Decrement nr of unread items
    if(noUnreadItems_ < 0 || noUnreadItems_ >= noItems_) {
      throw std::out_of_range("[SILO:Buffer]: Can not read element " + name_ + " at position " + std::to_string(noUnreadItems_));
    }

    *pItem = container_[noUnreadItems_];

    return true;
  }

  /** Read a vector of values collected since the last call to this function from the buffer (if buffer not empty). Decrement no new items.
   *  @return vec vector of all new items since last call to this function
   */
  vector_type<ValueType_> readNewValues()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // Allocate vector
    vector_type<ValueType_> newValues;

    if( noNewItems_ != 0 ) {
      // Lock the circular buffer
      newValues.reserve(noNewItems_);

      for(int i = (noNewItems_ - 1); i >= 0; i=i-1) {
        newValues.push_back(container_[i]);
      }
      noNewItems_ = 0;
    }

    return newValues;
  }

  /** Get copy of an item. Not decrementing unread items.
   *  @param idx index of the stored value (idx'th newest value)
   *  @return val Copy of value at idx
   */
  ValueType_ getElementCopyAtPosition(std::size_t idx)  const {
    // Lock the circular buffer
    std::unique_lock<std::mutex> lock(mutex_);

    // read element
    if(idx < 0 || idx >= noItems_) {
      throw std::out_of_range("[SILO:Buffer]: Can not read element " + name_ + " at position " + std::to_string(idx));
    }

    return container_[idx];
  }

  /** Get const pointer to a const item. Not decrementing unread items.
 *  @param idx index of the stored value (idx'th newest value)
 *  @return val const Pointer to const value at idx
 */
  const ValueType_ * const getPointerAtPosition(std::size_t idx)  const {
    // Lock the circular buffer
    std::unique_lock<std::mutex> lock(mutex_);

    if(idx < 0 || idx >= noItems_) {
      throw std::out_of_range("[SILO:Buffer]: Can not read element " + name_ + " at position " + std::to_string(idx));
    }
    // local var for backward compatibility with (Ubuntu 14.04 / Ros Indigo)
    const ValueType_& value = container_[idx];
    return &value;
  }

  //! Allocate buffer size of memory
  void allocate(bool enabled) {
    std::unique_lock<std::mutex> lock(mutex_);
    std::size_t new_capacity = enabled ? (bufferSize_) : 0;
    container_.set_capacity(new_capacity);
  }

  virtual std::size_t getBufferSize() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return bufferSize_;
  }

  virtual void setBufferSize(std::size_t bufferSize) {
    std::lock_guard<std::mutex> lock(mutex_);
    bufferSize_ = bufferSize;
    container_.set_capacity(bufferSize_);
    container_.clear();
    noUnreadItems_ = size_type(0);
    noItems_ = size_type(0);
    noNewItems_ = size_type(0);
  }

  virtual BufferType getBufferType() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return bufferType_;
  }

  virtual void setBufferType(const BufferType bufferType) {
    std::lock_guard<std::mutex> lock(mutex_);
    bufferType_ = bufferType;
  }

  //! @return number of unread elements
  virtual std::size_t noUnreadItems() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return noUnreadItems_;
  }

  //! @return number of items stored in the buffer (read and unread)
  virtual std::size_t noTotalItems() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return noItems_;
  }

  //! Clear the buffer
  void clear() {
    std::unique_lock<std::mutex> lock(mutex_);
    container_.clear();
    noUnreadItems_ = size_type(0);
    noItems_ = size_type(0);
    noNewItems_ = size_type(0);
  }

  //! Clear the buffer
  void resetUnreadItems() {
    std::unique_lock<std::mutex> lock(mutex_);
    noUnreadItems_ = size_type(0);
  }

 private:
  //! Disabled copy constructor
  Buffer(const Buffer&);
  //! Disabled assign operator
  Buffer& operator = (const Buffer&);

 private:
  Buffer(const ValueType_ * const ptr,  const std::string & name, const std::size_t rows, const std::size_t cols):
      ptr_(ptr),
      name_(name),
      bufferType_(BufferType::LOOPING),
      bufferSize_(0),
      container_(bufferSize_),
      noUnreadItems_(0),
      noItems_(0),
      noNewItems_(0),
      rows_(rows),
      cols_(cols),
      mutex_()
  {
  }

  /** Push an element at front for default types
   *  @param item item to push at front
   *  @return template specialization by return type
   */
  template<typename V = ValueType_>
  bool pushElementFront(const ValueType_ * const item, typename std::enable_if<is_buffer_default_type<V>::value>::type* = 0) {
    container_.push_front(*item);
    return true;
  }

  /** Push an element at front for eigen types
   *  @param item item to push at front of underlying-type buffer
   *  @return template specialization by return type
   */
  template<typename V = ValueType_>
  bool pushElementFront(const ValueType_ * const item, typename std::enable_if<traits::is_eigen_matrix<V>::value>::type * = 0) {
    if(item->rows() != rows_ || item->cols() != cols_) {
      // Error output -> don't push back
      MELO_ERROR_STREAM("[SILO:Buffer]: Matrix size of element " << name_ << " not consistent" << std::endl <<
                        " init_rows = " << rows_ << " item_rows = " << item->rows() << std::endl <<
                        " init_cols = " << cols_ << " item_cols = " << item->cols());
      return false;
    }
    container_.push_front(*item);
    return true;
  }

 private:
  //! Pointer to value
  const ValueType_* const ptr_;
  //! Name
  const std::string name_;
  //! Is the buffer "circulating", refreshing old entries with new ones
  BufferType bufferType_;
  //! Size of the buffer (no elements, can differ from buffer capacity)
  std::size_t bufferSize_;
  //! Circular buffer
  circular_buffer_type container_;
  //! Number of unread items
  size_type noUnreadItems_;
  //! Number of items in the buffer (read and unread)
  size_type noItems_;
  //! Counter for readNewValues function
  size_type noNewItems_;
  //! Eigen specific entries (1 in other cases)
  const Eigen::Index rows_;
  const Eigen::Index cols_;
  //! Mutex protecting accessing this container
  mutable std::mutex mutex_;
};

} /* namespace signal_logger */
