/*
 * test_declarations.cpp
 *
 *  Created on: Jul 22, 2017
 *      Author: Perry Franklin
 */

#include "test_declarations.hpp"

namespace collisions_visualization_test{

void fillTestBodyContainer(TestRigidBodyContainer& rigidBodyContainer)
{
  rigidBodyContainer.clear();

    std::tuple<std::string, unsigned int, BodyEnum> key_tuple0 = std::make_tuple("Body0", 0 ,BodyEnum::BODY0);
    rigidBodyContainer.insert(key_tuple0, std::make_shared<TestRigidBody>("Body0",
                                                                          BodyEnum::BODY0,
                                                                          BodyNodeEnum::NODE0,
                                                                          BranchEnum::ALL));

    std::tuple<std::string, unsigned int, BodyEnum> key_tuple1 = std::make_tuple("Body1", 1 ,BodyEnum::BODY1);
    rigidBodyContainer.insert(key_tuple1, std::make_shared<TestRigidBody>("Body1",
                                                                          BodyEnum::BODY1,
                                                                          BodyNodeEnum::NODE1,
                                                                          BranchEnum::ALL));

    std::tuple<std::string, unsigned int, BodyEnum> key_tuple2 = std::make_tuple("Body2", 2 ,BodyEnum::BODY2);
    rigidBodyContainer.insert(key_tuple2, std::make_shared<TestRigidBody>("Body2",
                                                                          BodyEnum::BODY2,
                                                                          BodyNodeEnum::NODE2,
                                                                          BranchEnum::ALL));

    std::tuple<std::string, unsigned int, BodyEnum> key_tuple3 = std::make_tuple("Body3", 3 ,BodyEnum::BODY3);
    rigidBodyContainer.insert(key_tuple3, std::make_shared<TestRigidBody>("Body3",
                                                                          BodyEnum::BODY3,
                                                                          BodyNodeEnum::NODE3,
                                                                          BranchEnum::ALL));

}

} // namespace collisions_fcl_tests
