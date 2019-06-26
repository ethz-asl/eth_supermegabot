/*
 * MarkerOptions.hpp
 *
 *  Created on: Aug 6, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <ros/ros.h>

#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace collisions_visualization {

class MarkerOptions{

public:

  enum class ColorType: unsigned int{
    GREEN = 0,
    RED = 1,
    BLUE
  } ;


  MarkerOptions();
  MarkerOptions( double red, double green, double blue, double alpha = 0.5 );
  MarkerOptions( ColorType color_type, double alpha = 0.5 );
  MarkerOptions( bool colliding, double alpha = 0.5 );

  ~MarkerOptions() = default;

  void setColor( double red, double green, double blue, double alpha = 0.5 );
  void setColor( ColorType color_type, double alpha = 0.5 );
  // This sets the color to Red if colliding, Green if not colliding
  void setColor( bool colliding );

  inline const std_msgs::ColorRGBA& getRGBA() const{
    return color;
  }

  void applyOptions(visualization_msgs::Marker& marker) const;

private:

  std_msgs::ColorRGBA color;

};

} // namespace collisions_visualization
