/*
 * MarkerOptions.cpp
 *
 *  Created on: Aug 6, 2017
 *      Author: Perry Franklin
 */

#include <collisions_visualization/MarkerOptions.hpp>

namespace collisions_visualization {

MarkerOptions::MarkerOptions(){
  color.a = 1.0;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 1.0;
}

MarkerOptions::MarkerOptions(double red, double green, double blue, double alpha){
  setColor(red, green, blue, alpha);
}

MarkerOptions::MarkerOptions(ColorType color_type, double alpha){
  setColor(color_type, alpha);
}

MarkerOptions::MarkerOptions(bool colliding, double alpha){
  setColor(colliding);
  color.a = alpha;
}

void MarkerOptions::setColor(double red, double green, double blue, double alpha){
  color.a = alpha;
  color.r = red;
  color.g = green;
  color.b = blue;
}

void MarkerOptions::setColor(ColorType color_type, double alpha){
  switch(color_type){

  case ColorType::RED:
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
    break;

  case ColorType::GREEN:
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    break;

  case ColorType::BLUE:
    color.r = 0.0;
    color.g = 0.0;
    color.b = 1.0;
    break;

  default:
    color.r = 1.0;
    color.g = 1.0;
    color.b = 1.0;
    break;

  }
  color.a = alpha;
}

void MarkerOptions::setColor(bool colliding){
  if (colliding){
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
  } else {
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
  }
}

void MarkerOptions::applyOptions(visualization_msgs::Marker& marker) const{
  marker.color = color;
}

} // namespace collisions_visualization


