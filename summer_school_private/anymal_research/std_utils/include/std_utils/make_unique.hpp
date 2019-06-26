/*!
 * @file    make_unique.hpp
 * @author  Marco Tranzatto
 * @date    Mar, 2018
 */

#pragma once

#include <memory>

namespace std_utils {

// Version of make_unique usable in C++11, taken from Herb Sutter's blog: 
// https://herbsutter.com/gotw/_102/#scid:C89E2BDB-ADD3-4f7a-9810-1B7EACF446C1:a097b818-32a5-4e09-a6a9-dec770b1a912
template<typename T, typename ...Args>
std::unique_ptr<T> make_unique( Args&& ...args ) {
  return std::unique_ptr<T>( new T( std::forward<Args>(args)... ) );
}

} // end namespace
