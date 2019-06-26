/*
 * FilteredVariable.hpp
 *
 *  Created on: Mar 31, 2014
 *      Author: gech
 */

#pragma once

#include <robot_utils/math/math.hpp>

namespace robot_utils {


/**
  this is a class that can be used for second-order filtered variables
*/
class SecondOrderFilteredVariable {
public:
  double x_oldest, x_old;
  double y_oldest, y_old;

  SecondOrderFilteredVariable(){
    x_oldest = x_old = y_oldest = y_old = 0;
  }

  void update(double x_new, double (*filter)(double x_oldest, double x_old, double x_new, double y_oldest, double y_old, double filtereParameter) = NULL, double filterParameter = 0){
    double y_new = x_new;
    if (filter)
      y_new = filter(x_oldest, x_old, x_new, y_oldest, y_old, filterParameter);
    x_oldest = x_old;
    x_old = x_new;
    y_oldest = y_old;
    y_old = y_new;
  }

  double getCurrentValue(){
    return y_old;
  }

  void setCurrentValue(double v){
      y_old = v;
  }
};

/**
  implements a low-pass filter.
*/
template <class T> class FilteredVariable{
public:
  T value;
  double alpha;
  bool initialized;

  T& operator = (const T& other){

  }

public:
  FilteredVariable(void){
    //uninitialized, because I don't know what T is...
    initialized = false;
    alpha = 0.6;
  }

  FilteredVariable(const T& other){
    //require a copy operator...
    this->value = other;
    initialized = true;
    alpha = 0.6;
  }

  FilteredVariable(const T& other, double a){
    //require a copy operator...
    this->value = other;
    initialized = true;
    alpha = a;
  }

  void setAlpha(double a){
    alpha = a;
    robot_utils::boundToRange(&alpha, 0, 1);
  }

  ~FilteredVariable(void){
    //nothing to do...
  }

  void initialize(const T& other){
    //require a copy operator...
    this->value = other;
    initialized = true;
  }

  void reset(){
    initialized = false;
  }

  void update(const T& newVal){
    if (initialized == false)
      initialize(newVal);
    else
      value = newVal * alpha + value * (1-alpha);
  }

  bool isInitialized(){
    return initialized;
  }

  T val(){
    return value;
  }
};


typedef FilteredVariable<double> FilteredDouble;




}
