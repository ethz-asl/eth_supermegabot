/*!
* @file     MultiKeyContainerTest.cpp
* @author   Christian Gehring
* @date     March 20, 2015
* @brief
*/

#include <gtest/gtest.h>

#include <std_utils/containers/containers.hpp>

enum class TestEnum: unsigned int {
  E_0=10,
  E_1=20,
  E_2=30
};


class TestItem {
 public:
  TestItem(double value):
    value_(value) {

  }
  virtual ~TestItem() {

  }

  double value_;
};


TEST(MultiKeyContainer, container) {
  std_utils::MultiKeyContainer<TestItem, TestEnum> container;


  container.insert(std::make_tuple("zero", 0, TestEnum::E_0), TestItem(0.0));
  container.insert(std::make_tuple("two", 2, TestEnum::E_2), TestItem(2.0));
  container.insert(std::make_tuple("one", 1, TestEnum::E_1), TestItem(1.0));

  EXPECT_EQ(0, container["zero"].value_);
  EXPECT_EQ(1, container["one"].value_);
  EXPECT_EQ(2, container["two"].value_);

  EXPECT_EQ(0, container[0].value_);
  EXPECT_EQ(1, container[1].value_);
  EXPECT_EQ(2, container[2].value_);

  EXPECT_EQ(0, container[TestEnum::E_0].value_);
  EXPECT_EQ(1, container[TestEnum::E_1].value_);
  EXPECT_EQ(2, container[TestEnum::E_2].value_);

  EXPECT_EQ(0, container.at("zero").value_);
  EXPECT_EQ(1, container.at("one").value_);
  EXPECT_EQ(2, container.at("two").value_);

  EXPECT_EQ(0, container.at(0).value_);
  EXPECT_EQ(1, container.at(1).value_);
  EXPECT_EQ(2, container.at(2).value_);

  EXPECT_EQ(0, container.at(TestEnum::E_0).value_);
  EXPECT_EQ(1, container.at(TestEnum::E_1).value_);
  EXPECT_EQ(2, container.at(TestEnum::E_2).value_);


  EXPECT_EQ(TestEnum::E_0, container.getEnum(0));
  EXPECT_EQ(TestEnum::E_1, container.getEnum(1));
  EXPECT_EQ(TestEnum::E_2, container.getEnum(2));

  EXPECT_EQ(TestEnum::E_0, container.getEnum("zero"));
  EXPECT_EQ(TestEnum::E_1, container.getEnum("one"));
  EXPECT_EQ(TestEnum::E_2, container.getEnum("two"));

  EXPECT_EQ("zero", container.getName(0));
  EXPECT_EQ("one", container.getName(1));
  EXPECT_EQ("two", container.getName(2));
  EXPECT_EQ("zero", container.getName(TestEnum::E_0));
  EXPECT_EQ("one", container.getName(TestEnum::E_1));
  EXPECT_EQ("two", container.getName(TestEnum::E_2));

  auto containerIt = container.find(TestEnum::E_0);
  EXPECT_EQ(0, containerIt->value_);

  const std_utils::MultiKeyContainer<TestItem, TestEnum>& constContainer = container;
  auto constContainerIt = constContainer.find(TestEnum::E_0);
  EXPECT_EQ(0, constContainerIt->value_);


  EXPECT_EQ(0, container.getId("zero"));
  EXPECT_EQ(1, container.getId("one"));
  EXPECT_EQ(2, container.getId("two"));

  EXPECT_EQ(TestEnum::E_0, container.getEnum("zero"));
  EXPECT_EQ(TestEnum::E_1, container.getEnum("one"));
  EXPECT_EQ(TestEnum::E_2, container.getEnum("two"));

  EXPECT_EQ(TestEnum::E_0, container.getEnum(0));
  EXPECT_EQ(TestEnum::E_1, container.getEnum(1));
  EXPECT_EQ(TestEnum::E_2, container.getEnum(2));

  EXPECT_EQ(3, container.size());
  ASSERT_TRUE(!container.empty());

  std_utils::MultiKeyContainer<TestItem*, TestEnum> containerPtr;
  containerPtr.insert(std::make_tuple("zero", 0, TestEnum::E_0), &container[TestEnum::E_0]);
  containerPtr.insert(std::make_tuple("two", 2, TestEnum::E_2), &container[TestEnum::E_2]);
  containerPtr.insert(std::make_tuple("one", 1, TestEnum::E_1), &container[TestEnum::E_1]);


  std_utils::MultiKeyContainer<TestItem*, TestEnum>::iterator it = containerPtr.begin();
  for (auto& item : container) {
    EXPECT_EQ(item.value_, (*it)->value_);
    ++it;
  }



  container.clear();
  EXPECT_EQ(0, container.size());
  ASSERT_TRUE(container.empty());
}


