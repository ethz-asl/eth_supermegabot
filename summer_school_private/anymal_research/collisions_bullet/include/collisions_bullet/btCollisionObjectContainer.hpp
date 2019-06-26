/*
 * btCollisionObjectContainer.hpp
 *
 *  Created on: Apr 17, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <memory>
#include <vector>

#include <btBulletCollisionCommon.h>

namespace collisions_bullet {

// This class helps handle the deletion of the btCollision stuff (since the objects and shapes need to be deleted separately)
class btCollisionObjectContainer {
public:
  btCollisionObjectContainer() = default;
  btCollisionObjectContainer(const btCollisionObject& collision_object);
  btCollisionObjectContainer(const std::shared_ptr<btCollisionObject>& collision_object);
  virtual ~btCollisionObjectContainer();

  void setCollisionObject(const btCollisionObject& collision_object);
  void setCollisionObject(const std::shared_ptr<btCollisionObject>& collision_object);

  std::shared_ptr<btCollisionObject>& getCollisionObject();
  const std::shared_ptr<btCollisionObject>& getCollisionObject() const;

  bool isNull() const;

private:

  void handleCompoundShape(const std::shared_ptr<btCompoundShape>& compound_shape);

  std::shared_ptr<btCollisionObject> collision_object_;

  std::shared_ptr<btCollisionShape> collision_shape_;

  std::vector<std::shared_ptr<btCollisionShape>> compound_handler_;

};

} /* namespace trajectory_optimizer */
