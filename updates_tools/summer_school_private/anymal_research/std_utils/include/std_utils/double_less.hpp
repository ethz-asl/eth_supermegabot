/*!
 * @file	 double_less.hpp
 * @author Gabriel Hottiger
 * @date	 Dec 4, 2017
 */
#pragma once

#include <functional>
#include <cmath>

namespace std_utils {

class double_less : public std::binary_function<double, double, bool> {
 public:
  explicit double_less( double arg_ = 1e-10 ) : epsilon(arg_) { }
  bool operator()( const double &left, const double &right ) const {
    // you can choose other way to make decision
    // (The original version is: return left < right;)
    return ( std::fabs(left - right) > epsilon ) && ( left < right );
  }
  double epsilon;
};

}