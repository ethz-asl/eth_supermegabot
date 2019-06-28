/*!
 * @file    SmartPointerIterator.hpp
 * @author  Dario Bellicoso, Gabriel Hottiger
 * @date    Jan 25, 2018
 */

#pragma once

#include <type_traits>
#include <memory>
#include "std_utils/containers/ObserverPointer.hpp"
#include "traits.hpp"

namespace std_utils {

/**
 * @brief An iterator for smart pointers which skips the smart pointer layer by exposing a pointer to the internal type.
 *        Can used in range-based for loops.
 * @tparam base_iterator_ Type of the base iterator
 * @tparam element_type_ This template parameter is necessary to help the IDE's autocomplete
 */
template <typename base_iterator_, typename element_type_>
class smart_pointer_iterator : public base_iterator_ {
 public:
  using base_iterator = base_iterator_;
  using value_type = typename base_iterator::value_type;
  using element_type = element_type_;

  static_assert(std_utils::traits::is_smart_ptr<value_type>::value, "smart_pointer_iterator is only valid for smart pointers");

  using pointer = element_type * const;
  using ppointer = pointer * const;

  explicit smart_pointer_iterator(const base_iterator &other) : base_iterator(other) { }

  //! Overload the dereference operator to access a pointer to element_type.
  pointer operator*() { return (this->base_iterator::operator*()).get(); }

  //! Overload the member access operator to access a pointer to element_type.
  ppointer operator->() { return &((this->base_iterator::operator*()).get()); }
};

template <typename base_iterator_, typename element_type_>
class smart_pointer_pair_iterator : public base_iterator_ {
 public:
  using base_iterator = base_iterator_;
  using value_type = typename base_iterator::value_type;
  using element_type = element_type_;

  using pointer = std::pair<typename value_type::first_type, element_type * const>;
  using ppointer = std::pair<typename value_type::first_type, element_type * const * const>;

  explicit smart_pointer_pair_iterator(const base_iterator &other) : base_iterator(other) { }

  //! Overload the dereference operator to access a pointer to element_type.
  pointer operator*() {
    auto && tmp = this->base_iterator::operator*();
    return pointer(tmp.first, tmp.second.get());
  }

  //! Overload the member access operator to access a pointer to element_type.
  ppointer operator->() {
    auto && tmp = this->base_iterator::operator*();
    return ppointer(tmp.first, &(tmp.second.get()) );
  }
};

//! Get a smart_pointer_iterator from an iterator to an element of a container of smart pointers.
template <typename element_type_, typename iterator_>
smart_pointer_iterator<iterator_, element_type_> dereference_iterator(
    iterator_ t, typename std::enable_if<std_utils::traits::is_smart_ptr<typename iterator_::value_type>::value>::type* = 0) {
  return smart_pointer_iterator<iterator_, element_type_>(t);
}

//! Get a smart_pointer_iterator from an iterator to an element of a container of smart pointers.
template <typename element_type_, typename iterator_>
smart_pointer_pair_iterator<iterator_, element_type_> dereference_iterator(
    iterator_ t, typename std::enable_if<std_utils::traits::is_pair_of_smart_ptr<typename iterator_::value_type>::value>::type* = 0) {

  return smart_pointer_pair_iterator<iterator_, element_type_>(t);
}


} /* namespace std_utils */