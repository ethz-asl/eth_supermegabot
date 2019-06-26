/*!
 * @file    SmartPointerIterator.hpp
 * @author  Dario Bellicoso, Gabriel Hottiger
 * @date    Jan 25, 2018
 */

#pragma once

// std utils
#include <std_utils/containers/SmartPointerIterator.hpp>
#include <std_utils/containers/ObserverPointer.hpp>
#include <std_utils/containers/traits.hpp>

// stl
#include <memory>
#include <vector>
#include <algorithm>

namespace std_utils {

namespace internal {

//! A wrapper type for map-like containers.
template<typename key_type_,
         template<typename...>
         class container_ = std::map>
struct wrapper_map {
  template< typename iterator_, typename element_type_>
  using smart_iterator = std_utils::smart_pointer_pair_iterator<iterator_, element_type_>;

  // Container type without specialized smart pointer
  template< typename element_type_, template <typename...> class smart_pointer_ = std::unique_ptr>
  using container_template = container_<key_type_, smart_pointer_<element_type_>>;
};

//! A wrapper type for vector-like containers.
template<template<typename...>
         class container_ = std::vector>
struct wrapper_vec {
  //
  template< typename iterator_, typename element_type_>
  using smart_iterator = std_utils::smart_pointer_iterator<iterator_, element_type_>;

  // Container type without specialized smart pointer
  template< typename element_type_, template <typename...> class smart_pointer_ = std::unique_ptr>
  using container_template = container_<smart_pointer_<element_type_>>;
};

/**
 * @brief smart_pointer_container wraps around an stl container of smart pointers (e.g. std::unique_ptr,
 *  std::shared_ptr, etc.). It exposed begin() and end() methods using the smart_pointer_iterator class.
 * @tparam container_template_ A wrapper type around the stl container (e.g. std::vector, std::list, std::map, etc.)
 *                            used in the implementation.
 * @tparam element_type_ This template parameter is necessary to help the IDE's autocomplete features.
 */
template< typename element_type_, typename container_wrapper_,
    template <typename...> class smart_pointer_ = std::unique_ptr>
class smart_pointer_container {
 public:
  //! The STL container type.
  using container_type = typename container_wrapper_::template container_template<element_type_, smart_pointer_>;
  //! The smart pointer type.
  using smart_pointer_type = typename container_type::value_type;
  //! The template parameter for the smart pointer.
  using element_type = element_type_;

  //! The iterator to an element of the STL container.
  using iterator = typename container_type::iterator;
  //! The const iterator to an element of the STL container.
  using const_iterator = typename container_type::const_iterator;

  //! The iterator to an element of the STL container.
  using smart_iterator = typename container_wrapper_::template smart_iterator<iterator, element_type>;
  //! The const iterator to an element of the STL container.
  using smart_const_iterator = typename container_wrapper_::template smart_iterator<const_iterator, const element_type>;

  //! The reference to an element of the STL container.
  using reference = typename container_type::reference;
  //! The const reference to an element of the STL container.
  using const_reference = typename container_type::const_reference;

  //! The size_type of the STL container.
  using size_type = typename container_type::size_type;

  //! Define a helper type to make containers of observer pointers.
  template<typename T>
  using observer_ptr_container_t = smart_pointer_container<T, container_wrapper_, std_utils::observer_ptr>;
  using observer_ptr_container = observer_ptr_container_t<element_type_>;

  //! Default constructor.
  smart_pointer_container() = default;

  //! Construct the container from iterators.
  template<typename it>
  smart_pointer_container(it begin, it end) :items_(begin, end) { }

  //! Default destructor.
  virtual ~smart_pointer_container() = default;

  //! Default copy constructor.
  smart_pointer_container(const smart_pointer_container&) = default;

  //! Default move constructor.
  smart_pointer_container(smart_pointer_container&&) noexcept = default;

  //! Default copy assign.
  smart_pointer_container& operator=(const smart_pointer_container&) = default;

  //! Default move assign.
  smart_pointer_container& operator=(smart_pointer_container&&) noexcept = default;

  smart_iterator begin() { return std_utils::dereference_iterator<element_type>(items_.begin()); }
  smart_iterator end()   { return std_utils::dereference_iterator<element_type>(items_.end()); }
  smart_const_iterator begin() const { return std_utils::dereference_iterator<const element_type>(items_.begin()); }
  smart_const_iterator end()   const { return std_utils::dereference_iterator<const element_type>(items_.end()); }

  iterator s_begin() { return items_.begin(); }
  iterator s_end()   { return items_.end(); }
  const_iterator s_begin() const { return items_.begin(); }
  const_iterator s_end()   const { return items_.end(); }

  size_type size() const { return items_.size(); }
  void clear() { items_.clear(); }

  observer_ptr_container make_observer_container() const {
    return observer_ptr_container(this->s_begin(), this->s_end());
  }

  template <typename T, typename = typename std::enable_if<std::is_base_of<T, element_type>::value>::type>
  observer_ptr_container_t<T> make_observer_container_of_type() const {
    return observer_ptr_container_t<T>(this->s_begin(), this->s_end());
  }

  template<typename item_iterator_>
  void insert(iterator position, item_iterator_ first, item_iterator_ last) {
    this->items_.insert(position, first, last);
  }

 protected:
  container_type items_;
};
}

//! A class wrapping an STL container of smart pointers.
template<typename element_type_,
         template<typename...>
         class smart_pointer_ = std::unique_ptr,
         template<typename...>
         class container_ = std::vector>
using smart_pointer_container =
  internal::smart_pointer_container< element_type_, internal::wrapper_vec<container_>, smart_pointer_>;

//! A class wrapping a map or unordered map of smart pointers.
template<typename key_type_,
         typename element_type_,
         template<typename...>
         class smart_pointer_ = std::unique_ptr,
         template<typename...>
         class container_ = std::map>
using smart_pointer_dictionary =
  internal::smart_pointer_container< element_type_, internal::wrapper_map<key_type_, container_>, smart_pointer_>;

// Define some helper types.
//! A std::vector of std::unique_ptrs to element_type.
template<typename element_type>
using VectorOfUniquePtrs = std_utils::smart_pointer_container<element_type, std::unique_ptr, std::vector>;

//! A std::vector of std::shared_ptrs to element_type.
template<typename element_type>
using VectorOfSharedPtrs = std_utils::smart_pointer_container<element_type, std::shared_ptr, std::vector>;

//! A std::vector of std_utils::observer_ptrs to element_type.
template<typename element_type>
using VectorOfObserverPtrs = std_utils::smart_pointer_container<element_type, std_utils::observer_ptr, std::vector>;

//! A std::unordered_map of std::unique_ptrs to element_type.
template<typename key_type, typename element_type>
using UnorderedMapOfUniquePtrs = std_utils::smart_pointer_dictionary<key_type, element_type, std::unique_ptr, std::unordered_map>;


}