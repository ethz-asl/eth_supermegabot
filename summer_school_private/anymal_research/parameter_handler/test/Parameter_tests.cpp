/*
 * Parameter_tests.cpp
 *
 *  Created on: Sep 23, 2015
 *      Author: Christian Gehring
 */


#include <gtest/gtest.h>
#include "parameter_handler/Parameter.hpp"

typedef ::testing::Types<
    double,
    int,
    bool
> ParameterValueTypes;

template <typename ValueType_>
class ParameterTest : public ::testing::Test {
 public:
  typedef parameter_handler::Parameter<ValueType_> Parameter;
  typedef ValueType_ Value;
};

TYPED_TEST_CASE(ParameterTest, ParameterValueTypes);

TYPED_TEST(ParameterTest, constructor) {
  //ParameterTest param;
}


//TEST (ParameterTest, constructor) {
//  using namespace parameter_handler;
//  ParameterDouble param(1.0, 2.0, 3.0);
//  EXPECT_EQ(1.0, param.getCurrentValue());
//  EXPECT_EQ(1.0, param.getDefaultValue());
//  EXPECT_EQ(2.0, param.getMinValue());
//  EXPECT_EQ(3.0, param.getMaxValue());
//}

TEST (ParameterTest, setters) {
  using namespace parameter_handler;
  Parameter<double> param;
  param.setValue(1.0);
  EXPECT_EQ(1.0, param.getValue());

  param.setMinValue(2.0);
  EXPECT_EQ(2.0, param.getMinValue());

  param.setMaxValue(3.0);
  EXPECT_EQ(3.0, param.getMaxValue());

  param.setDefaultValue(4.0);
  EXPECT_EQ(3.0, param.getDefaultValue());

  ParameterInterface& paramInterface = param;
  EXPECT_EQ(paramInterface.getType(), typeid(double));
  EXPECT_NO_THROW(paramInterface.getValue<double>());
  EXPECT_ANY_THROW(paramInterface.getValue<int>());
}


TEST (ParameterTest, testInt) {
  using namespace parameter_handler;

  Parameter<int> paramInt;
  paramInt.setValue(4);
  EXPECT_EQ(4, paramInt.getValue());

}
