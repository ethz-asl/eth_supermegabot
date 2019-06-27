/*
 * CollisionGeometry.hpp
 *
 *  Created on: May 18, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <kindr/Core>

namespace collisions_geometry {

class CollisionGeometry {
 public:

  enum class GeomType: unsigned int {
    NONE = 0,
    BOX,
    CYLINDER,
    CAPSULE,
    SPHERE,
    MESH,
    COMBO
  };

  typedef kindr::HomTransformMatrixD Transform;
  typedef kindr::HomTransformMatrixD::Position Position;
  typedef kindr::HomTransformMatrixD::Rotation Rotation;
  
 public:

  virtual ~CollisionGeometry() = default;

  virtual GeomType getType() const = 0;

  inline const Position& getPosition() const{
    return transform_.getPosition();
  }

  inline const Rotation& getRotation() const{
    return transform_.getRotation();
  }

  inline const Transform& getTransform() const{
    return transform_;
  }

  inline void setMargin(double margin){
    margin_ = margin;
  }

  inline double getMargin() const{
    return margin_;
  }

  virtual CollisionGeometry* clone() const = 0;


protected:

  explicit CollisionGeometry(const Position& position, const Rotation& rotation):
  transform_(position, rotation),
  margin_(0.0){};

  explicit CollisionGeometry(const Position& position, const Rotation& rotation, const double& margin):
  transform_(position, rotation),
  margin_(margin){};

  // Eigen::Vector3d position_;
  // Eigen::Matrix3d orientation_;
  Transform transform_;

  double margin_;


};

}
