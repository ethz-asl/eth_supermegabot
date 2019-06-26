/*!
* @file     statistics.hpp
* @author   Gabriel Hottiger
* @date     Dec 19, 2017
* @brief
*/
#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>

namespace robot_utils {



// Remember to get a proper seed first. (e.g std::srand(std::time(nullptr)); )
template <typename T>
T generateRandomNumber(T min = std::numeric_limits<T>::lowest(), T max = std::numeric_limits<T>::max())
{
  double f = static_cast<double>(std::rand()) / RAND_MAX;
  return (1.0-f) * min + f * max;
}

template< typename Container_ >
inline void applyAbs( Container_ &c ) {
  std::transform(std::begin(c), std::end(c), std::begin(c),
                 [ ]( typename Container_::value_type x ) { return std::abs(x); });
}

template< typename Container_ >
inline Container_ abs( const Container_ &c ) {
  Container_ abs(c);
  applyAbs(abs);
  return abs;
}

template< typename Container_ >
inline double absPeak( const Container_& c ) {
  Container_ abs(c);
  applyAbs(abs);
  return c.size() > 0 ? *std::max_element(abs.begin(), abs.end()) : 0.0;
}

template< typename Container_ >
inline double mean( const Container_& c ) {
  return c.size() > 0 ? std::accumulate(std::begin(c), std::end(c), 0.0) / c.size() : 0;
}

template< typename Container_ >
inline double rms( const Container_& c ) {
  return c.size() > 0 ? std::sqrt( std::inner_product(std::begin(c), std::end(c), std::begin(c), 0.0) / c.size() ) : 0;
}

template< typename Container_ >
inline double var( const Container_& c ) {
  const auto mean = robot_utils::mean(c);
  return c.size() > 1 ? std::accumulate(std::begin(c), std::end(c), 0.0, [mean](double tmp_var, double x)
  { return tmp_var + std::pow(x - mean, 2); }) / (c.size() - 1) : 0.0;
}

template< typename Container_ >
inline double stdev( const Container_& c ) {
  return std::sqrt(var(c));
}

}