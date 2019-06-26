/*
 * btCollisionObjectContainer.cpp
 *
 *  Created on: Apr 17, 2018
 *      Author: Perry Franklin
 */

#include <collisions_bullet/btCollisionObjectContainer.hpp>

#include <iostream>

namespace collisions_bullet {

btCollisionObjectContainer::btCollisionObjectContainer(const btCollisionObject& collision_object) {
  setCollisionObject(collision_object);
}

btCollisionObjectContainer::btCollisionObjectContainer(const std::shared_ptr<btCollisionObject>& collision_object) {
  setCollisionObject(collision_object);
}

btCollisionObjectContainer::~btCollisionObjectContainer() {
}

void btCollisionObjectContainer::setCollisionObject(const btCollisionObject& collision_object){
  compound_handler_.clear();
  collision_object_ = std::make_shared<btCollisionObject>(collision_object);
  collision_shape_.reset(collision_object_->getCollisionShape());
  if (collision_shape_->isCompound()){
    handleCompoundShape(std::dynamic_pointer_cast<btCompoundShape>(collision_shape_));
  }
}

void btCollisionObjectContainer::setCollisionObject(const std::shared_ptr<btCollisionObject>& collision_object){
  compound_handler_.clear();
  collision_object_ = collision_object;
  collision_shape_.reset(collision_object->getCollisionShape());
  if (collision_shape_->isCompound()){
    handleCompoundShape(std::dynamic_pointer_cast<btCompoundShape>(collision_shape_));
  }
}

std::shared_ptr<btCollisionObject>& btCollisionObjectContainer::getCollisionObject(){
  return collision_object_;
}
const std::shared_ptr<btCollisionObject>& btCollisionObjectContainer::getCollisionObject() const{
  return collision_object_;
}

void btCollisionObjectContainer::handleCompoundShape(const std::shared_ptr<btCompoundShape>& compound_shape){

  // Step through the children of the compound shape and create shared_ptrs for them
  int num_children = compound_shape->getNumChildShapes();
  for (int i = 0; i < num_children; i++){

    btCollisionShape* shape = compound_shape->getChildShape(i);

    compound_handler_.emplace_back(shape);

    if (compound_handler_.back()->isCompound()){
      handleCompoundShape(std::dynamic_pointer_cast<btCompoundShape>(compound_handler_.back()));
    }
  }

}

bool btCollisionObjectContainer::isNull() const{
  return !(collision_object_ && collision_shape_);
}


} /* namespace trajectory_optimizer */
