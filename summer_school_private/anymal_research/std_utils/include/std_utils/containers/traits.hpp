/*!
 * @file    traits.hpp
 * @author  Dario Bellicoso, Gabriel Hottiger
 * @date    Jan 25, 2018
 */

#pragma once

// stl
#include <type_traits>
#include <map>
#include <unordered_map>
#include <memory>

namespace std_utils {
namespace traits {

//! is_unique_ptr false type
template<typename>
struct is_unique_ptr : std::false_type {};

//! is_unique_ptr true type
template<typename T>
struct is_unique_ptr<std::unique_ptr<T>> : std::true_type {};

//! is_shared_ptr false type
template<typename>
struct is_shared_ptr : std::false_type {};

//! is_shared_ptr true type
template<typename T>
struct is_shared_ptr<std::shared_ptr<T>> : std::true_type {};

//! is_observer_ptr false type
template<typename>
struct is_observer_ptr : std::false_type {};

//! is_observer_ptr true type
template<typename T>
struct is_observer_ptr<std_utils::observer_ptr<T>> : std::true_type {};

//! is_observer_ptr false type
template<typename, typename Enable_ = void>
struct is_smart_ptr : std::false_type {};

//! is_observer_ptr true type
template<typename T>
struct is_smart_ptr<T, typename std::enable_if<is_unique_ptr<T>::value ||
    is_shared_ptr<T>::value || is_observer_ptr<T>::value>::type> : std::true_type {};

//! is_map false type
template<typename>
struct is_map : std::false_type {};

//! is_map true type
template<typename T, typename U>
struct is_map<std::map<T,U>> : std::true_type {};

//! is_map true type
template<typename T, typename U>
struct is_map<std::unordered_map<T,U>> : std::true_type {};

//! is_pair false type
template<typename>
struct is_pair : std::false_type {};

//! is_pair true type
template<typename T, typename U>
struct is_pair<std::pair<T,U>> : std::true_type {};

//! is_pair_of_smart_ptr false type
template<typename, typename Enable_ = void>
struct is_pair_of_smart_ptr : std::false_type {};

//! is_pair_of_smart_ptr true type
template<typename T, typename U>
struct is_pair_of_smart_ptr<std::pair<T,U>, typename std::enable_if<is_smart_ptr<U>::value>::type> : std::true_type {};


} /* namespace traits */
} /* namespace std_utils */