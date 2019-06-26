//
// Created by gabrielhottiger on 11.04.18.
//

#pragma once

#include <type_traits>

#define STD_UTILS_HAS_TYPE(name)                                                           \
template< typename T, typename = void >                                                    \
struct has_##name {                                                                        \
  using type = std::false_type;                                                            \
};                                                                                         \
template< typename C >                                                                     \
struct has_##name<C, typename std_utils::internal::to_void<typename C::name>::type> {      \
  using type = std::true_type;                                                             \
};                                                                                         \
                                                                                           \
template< typename C>                                                                      \
using has_##name##_t = typename has_##name<C>::type

namespace std_utils {

namespace internal {

template< typename >
struct to_void { using type = void; };
}

}