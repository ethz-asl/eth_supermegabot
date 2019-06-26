/*!
 * @file    ObserverPointer.hpp
 * @author  Dario Bellicoso, Gabriel Hottiger
 * @date    Jan 26, 2018
 */

#pragma once

#include <memory>

namespace std_utils {

/**
 * @brief An object which wraps around a pointer without taking ownership.
 * @tparam T The (decayed) type of the object which is wrapped around.
 */
template<typename T>
class observer_ptr {
 public:
  using element_type = T;

  observer_ptr() : ptr_(nullptr) { }
  ~observer_ptr() = default;

  //! Copy construct from unique pointer.
  template<typename custom_deleter, typename OtherT_, typename = typename std::enable_if<std::is_base_of<element_type, OtherT_>::value>::type>
  explicit observer_ptr(const std::unique_ptr<OtherT_, custom_deleter>& other) : ptr_(other.get()) {}

  //! Copy construct from a shared pointer.
  template<typename OtherT_, typename = typename std::enable_if<std::is_base_of<element_type, OtherT_>::value>::type>
  explicit observer_ptr(const std::shared_ptr<OtherT_>& other) : ptr_(other.get()) {}

  //! Copy construct from an observer pointer.
  template<typename OtherT_, typename = typename std::enable_if<std::is_base_of<element_type, OtherT_>::value>::type>
  explicit observer_ptr(const std_utils::observer_ptr<OtherT_>& other) : ptr_(other.get()) {}


  //! Copy assign from unique pointers.
  template<typename custom_deleter, typename OtherT_, typename = typename std::enable_if<std::is_base_of<element_type, OtherT_>::value>::type>
  observer_ptr& operator=(const std::unique_ptr<OtherT_, custom_deleter>& other) {
    this->ptr_ = other.get();
    return *this;
  }

  //! Copy assign from a shared pointer.
  template<typename OtherT_, typename = typename std::enable_if<std::is_base_of<element_type, OtherT_>::value>::type>
  observer_ptr& operator=(const std::shared_ptr<OtherT_>& other) {
    this->ptr_ = other.get();
    return *this;
  }

  //! Copy assign from an observer pointer.
  template<typename OtherT_, typename = typename std::enable_if<std::is_base_of<element_type, OtherT_>::value>::type>
  observer_ptr& operator=(const std_utils::observer_ptr<OtherT_>& other) {
    this->ptr_ = other.get();
    return *this;
  }


  //! Delete move construction from shared pointers.
  observer_ptr(std::shared_ptr<element_type>&& other) = delete;
  //! Delete move assign from shared pointers.
  observer_ptr& operator=(std::shared_ptr<element_type>&& other) = delete;

  //! Delete move construction from unique pointers.
  template<typename custom_deleter>
  observer_ptr(std::unique_ptr<element_type, custom_deleter>&& other) = delete;
  //! Delete move assign from unique pointers.
  template<typename custom_deleter>
  observer_ptr& operator=(std::unique_ptr<element_type, custom_deleter>&& other) = delete;


  T* const get() const noexcept { return ptr_; }
  T* const operator->() const noexcept { return ptr_; }
  T& operator*() const { return *ptr_; }

 private:
  T* ptr_;
};

}
