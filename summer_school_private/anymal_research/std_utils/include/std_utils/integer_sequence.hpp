/*!
 * @file    integer_sequence.hpp
 * @author  Gabriel Hottiger
 * @date    Apr, 2018
 */

// From here: https://gist.github.com/ntessore/dc17769676fb3c6daa1f

#pragma once

#include <cstddef>

namespace std_utils {

template<typename T, T... Ints>
struct integer_sequence
{
  typedef T value_type;
  static constexpr std::size_t size() { return sizeof...(Ints); }
};

template<std::size_t... Ints>
using index_sequence = integer_sequence<std::size_t, Ints...>;

template<typename T, std::size_t N, T... Is>
struct make_integer_sequence {
  using type = typename make_integer_sequence<T, N-1, N-1, Is...>::type;
};

template<typename T, T... Is>
struct make_integer_sequence<T, 0, Is...> {
  using type = integer_sequence<T, Is...>;
};

template<typename T, std::size_t N>
using make_integer_sequence_t = typename make_integer_sequence<T, N>::type;

template<std::size_t N>
using make_index_sequence_t = typename make_integer_sequence<std::size_t, N>::type;

template<typename T, std::size_t From, std::size_t To, T... Is>
struct make_integer_sequence_range {
  using type = typename make_integer_sequence_range<T, From, To-1, To-1, Is...>::type;
};

template<typename T, std::size_t From, T... Is>
struct make_integer_sequence_range<T, From, From, Is...> {
  using type = integer_sequence<T, Is...>;
};

template<typename T, std::size_t From, std::size_t To>
using make_integer_sequence_range_t = typename make_integer_sequence_range<T, From, To>::type;

template<std::size_t From, std::size_t To>
using make_index_sequence_range_t = typename make_integer_sequence_range<std::size_t, From, To>::type;

template<typename... T>
using index_sequence_for_t = make_index_sequence_t<sizeof...(T)>;

}